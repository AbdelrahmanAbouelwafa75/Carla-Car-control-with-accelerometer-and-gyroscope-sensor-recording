# CARLA Vehicle Control and Sensor Data Recording

This project simulates a vehicle controlled via keyboard inputs in the CARLA Simulator. The program captures data from the vehicle's sensors, including the accelerometer, gyroscope, and speed, and saves it to a CSV file. The data is recorded dynamically, with each press of the "Numpad 7" key toggling the start/stop of data recording, storing the data in consecutive rows of the CSV file.

## Features

- **Manual Vehicle Control**: The vehicle can be controlled using the following keys:
  - `W` for forward throttle
  - `S` for reverse throttle
  - `A` for steering left
  - `D` for steering right
  - `Space` for brake

- **Sensor Data Recording**: The vehicle's accelerometer, gyroscope, and speed data are recorded in a CSV file with each press of the "Numpad 7" key.
  - The data for each recording session is saved in a new row in the CSV file.
  - Each recording session includes the accelerometer and gyroscope values along with the vehicle's speed, rounded to two decimal places.

- **CSV File Storage**: The sensor data is saved to a CSV file named `recorded_data.csv` in the same directory as the code. The file includes the following columns:
  - `AccX`, `AccY`, `AccZ`, `GyroX`, `GyroY`, `GyroZ`, `Speed`

## How to Run

### Prerequisites
1. Install the [CARLA Simulator](https://github.com/carla-simulator/carla) and follow the instructions for setting it up.
2. Install the following Python dependencies:
   - `carla`
   - `pygame`
   - `opencv-python`
   - `numpy`

You can install the dependencies using `pip`:

```bash
pip install carla pygame opencv-python numpy
