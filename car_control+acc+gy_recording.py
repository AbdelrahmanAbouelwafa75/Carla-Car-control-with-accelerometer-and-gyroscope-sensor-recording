import carla
import cv2
import numpy as np
import time
import pygame
import os
import csv

def normalize(value, min_value, max_value):
    """Normalize a value to the range [0, 1]."""
    return (value - min_value) / (max_value - min_value)



def main():
    try:
        pygame.init()
        screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Manual Driving")

        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]
        spawn_point = world.get_map().get_spawn_points()[0]
        ego_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print("Vehicle spawned!")
        friction_bp = world.get_blueprint_library().find('static.trigger.friction')
        #how much area does the friction function affect 
        extent = carla.Location(200000.0, 200000.0, 2000.0)
        #change the str 'friction' value to change friction in the entire world
        friction_bp.set_attribute('friction', str(100.0))
        friction_bp.set_attribute('extent_x', str(extent.x))
        friction_bp.set_attribute('extent_y', str(extent.y))
        friction_bp.set_attribute('extent_z', str(extent.z))

        # Spawn Trigger Friction
        transform = carla.Transform()
        transform.location = carla.Location(100.0, 0.0, 0.0)
        world.spawn_actor(friction_bp, transform)

        # Optional for visualizing trigger
        world.debug.draw_box(box=carla.BoundingBox(transform.location,extent * 1e-2), rotation=transform.rotation, life_time=100, thickness=0.5, color=carla.Color(r=255,g=0,b=0))

        # Apply high friction to the vehicle

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')

        camera_transform = carla.Transform(
            carla.Location(x=1.5, y=0.0, z=2.4),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        )
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        print("Camera attached!")

        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.0))
        imu_sensor = world.spawn_actor(imu_bp, imu_transform, attach_to=ego_vehicle)
        print("IMU sensor attached!")

        frame = None
        accelerometer = None
        gyroscope = None
        speed = 0.0
        recording = False
        csv_file_path = "recorded_data.csv"
        current_row = []

        # Check if the CSV file exists
        if not os.path.exists(csv_file_path):
            with open(csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "Speed"])
            print(f"Created CSV file: {csv_file_path}")

        def process_image(image):
            nonlocal frame
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))
            frame = array[:, :, :3]

        def process_imu(imu_data):
            nonlocal accelerometer, gyroscope
            accelerometer = imu_data.accelerometer
            gyroscope = imu_data.gyroscope

        camera.listen(process_image)
        imu_sensor.listen(process_imu)
        print("Sensors are capturing data...")

        throttle = 0.0
        steer = 0.0
        brake = 0.0
        reverse = False

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_KP7:
                    recording = not recording
                    if recording:
                        print("Recording started.")
                    else:
                        print("Recording stopped.")
                        # Save the row when stopping recording
                        if current_row:
                            with open(csv_file_path, mode='a', newline='') as file:
                                writer = csv.writer(file)
                                writer.writerow(current_row)
                            print(f"Data saved: {current_row}")
                        current_row = []  # Clear current row after saving

            keys = pygame.key.get_pressed()
            throttle = 1.0 if keys[pygame.K_w] else 0.0
            brake = 1.0 if keys[pygame.K_SPACE] else 0.0
            steer = -0.5 if keys[pygame.K_a] else (0.5 if keys[pygame.K_d] else 0.0)
            reverse = keys[pygame.K_s]

            control = carla.VehicleControl(
                throttle=throttle if not reverse else 0.5,
                steer=steer,
                brake=brake,
                reverse=reverse
            )
            ego_vehicle.apply_control(control)

            velocity = ego_vehicle.get_velocity()
            speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

            if recording and accelerometer and gyroscope:
                # Round the data to 2 decimal places before appending it to current_row
                normalized_acc_z = round(normalize(accelerometer.z, -9.81, 9.81), 2)
                current_row += [
                    round(accelerometer.x, 2), round(accelerometer.y, 2), normalized_acc_z,
                    round(gyroscope.x, 2), round(gyroscope.y, 2), round(gyroscope.z, 2),
                    round(speed, 2)
                ]
                print(f"Recording data: Acc={round(accelerometer.x, 2):.2f}, "
                      f"{round(accelerometer.y, 2):.2f}, {normalized_acc_z:.2f}, "
                      f"Gyro={round(gyroscope.x, 2):.2f}, {round(gyroscope.y, 2):.2f}, "
                      f"{round(gyroscope.z, 2):.2f}, Speed={round(speed, 2):.2f} m/s")

            if frame is not None:
                cv2.imshow("CARLA Camera Feed", frame)

            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            world.tick()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")

    finally:
        print("Destroying actors...")
        if camera is not None:
            camera.destroy()
        if imu_sensor is not None:
            imu_sensor.destroy()
        if ego_vehicle is not None:
            ego_vehicle.destroy()
        cv2.destroyAllWindows()
        pygame.quit()
        print("Actors destroyed. Simulation ended.")

if __name__ == "__main__":
    main()
