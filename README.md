## Overview

Self-rising robot with 2 degrees of freedom.

### Tech Stack

- [MuJoCo](https://mujoco.org/): physics simulation for robot
- Python
- ROS 2

### Architecture

![Node publishers and subscribers](progress_photos/node_planning.png)

Components:

- IMU - gyroscope and accelerometer data 
- **Kalman filter** 
  - Predicts the next angle using the current measured angular velocity and gyroscope uncertainty 
  - Updates the angle prediction using the measured acceleration and the gyroscope and accelerometer uncertainty
- **State estimation node** 
  - Subscribes to `/imu/data_raw` to get the angular velocity and acceleration 
  - Since new data is noisy, it uses the Kalman filter to calculate the next angle
  - Publishes the next angle to `/robot/state/pitch`
- **Proportional integral derivative (PID) controller node**
  - Subscribes to `/robot/state/pitch` for the next angle (`current_pitch`)
  - Every 0.1s (100Hz):
    - Calculates the *next next* angle using the `target_pitch` and `current_pitch`
    - Publishes the new angle to `/motor/pwm`
  - Subscribes to `/pid/reset`, which is part of `test_integration.py` for a reset command
    - Clears the total and previous error
- **Integration test**
  - Uses `unittest.TestCase` for automated test cases
    - Check if there is PWM output when there is acceleration
    - Check if there is angle data when there is acceleration
    - Check if the two motors receive the same PWM value
  - Publishes a reset command to `/pid/reset` every 0.05s
  - Starts the other nodes
- STM32 embedded system (not started)
- Camera input using OpenCV (not started)

### Docs

- [Daily Log](docs/log.md)
- [Progress Photos](images)

## Setup

### Initial Setup

Ensure you change your filepath to match your workspace.
```
sudo apt install ros-jazzy-robot-state-publisher -y
sudo apt install ros-jazzy-foxglove-bridge -y
source /opt/ros/jazzy/setup.bash
colcon build
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo 'alias devbot="cd <filepath-to-the-repo>/balance-bot && source install/setup.bash"' >> ~/.bashrc
source ~/.bashrc
colcon build --symlink-install
chmod +x record_topics.sh
```

### Each Session

#### Every time you open a new terminal

```
devbot
```

#### Every time you add new files 

Because of the --symlink-install flag from the initial setup, you don't need to run `colcon build` every time you modify existing files.

```
colcon build
```

## Usage

### Running the robot

Starting the nodes:

```
ros2 launch edge_robot bringup.launch.py
```

Recording topic data in ROS bags (new terminal):

```
ros2 bag record --topics /imu/data_raw /robot/state/pitch /motor/pwm
```

Replaying ROS bag data:

```
ros2 bag play robot_test_bag
```

### Running tests

Run the test:
```
colcon test
```

View the results:
```
colcon test-result --all --verbose
```

Viewing test logs: log\latest_test

### Foxglove Visualization

Terminal 1:

```
ros2 run foxglove_bridge foxglove_bridge
```

Terminal 2:
```
devbot
ros2 launch edge_robot bringup.launch.py
```

**Foxglove Setup**

1. Go to app.foxglove.dev
2. Open connection > Foxglove WebSocket
3. ws://localhost:8765 > Open

**Plotting Topics**

Add Panel > Plot > Series > Y value > {topic name} 

- /robot/state/pitch
- /motor/pwm
- /imu/data_raw

**Manually Simulate IMU Data**

Add Panel > Publish

```
{
  "header": {
    "stamp": { "sec": 0, "nanosec": 0 },
    "frame_id": "imu_link"
  },
  "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 },
  "angular_velocity": { "x": 0.0, "y": 0.0, "z": 0.0 },
  "linear_acceleration": { "x": 0.0, "y": 0.0, "z": 9.81 }
}
```
