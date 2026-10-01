## Overview

Self-rising robot with 2 degrees of freedom.

### Tech Stack

- [MuJoCo](https://mujoco.org/): physics simulation for robot
- Python

### Concepts Applied
- Gyroscope and accelerometer data (in progress)
- Kalman filter for sensor data (in progress)
- PWM and PID for motor control (in progress)
- STM32 embedded system (in progress)

## Docs

- [Daily Log](docs/log.md)
- [Progress Photos](images)

## WSL Setup

```
python3 -m venv .venv
```

```
source .venv/bin/activate
```


## Run the observability (usage) and video nodes

```
cd src/edge_robot
```
```
python3 observability_node.py
```
```
python3 video_node.py
```

## Joint movements 

### Setup

```
sudo apt install ros-jazzy-robot-state-publisher -y
```

```
sudo apt install ros-jazzy-foxglove-bridge -y
```

### Run the telemetry node

In each new terminal:

```
cd src/edge_robot
```

```
source /opt/ros/jazzy/setup.bash
```

Terminal 1 (WSL):

```
ros2 run foxglove_bridge foxglove_bridge
```

Terminal 2 (WSL):

```
ros2 run robot_state_publisher robot_state_publisher robot.urdf
```

Terminal 3 (WSL):

```
python3 telemetry_node.py
```

### Foxglove Visualization

Setup:

1. Go to app.foxglove.dev
2. Open connection > Foxglove WebSocket
3. ws://localhost:8765 > Open

Panels:

- **Graph of motor positions**: Add Panel > Plot > Series > Y value > /joint_states > position[0]
- **3D robotic arm:** Add Panel > 3D 
  - Frame: base_link
  - Topics: /robot_description 
- **CPU and memory usage:** Add Panel > Raw Messages > /system_metrics.data


## Test PID Node

Terminal 1:
```
python3 pid_node.py
```

Terminal 2:
```
ros2 topic echo /motor/pwm
```

Terminal 3:
```
ros2 topic pub -1 /robot/state/pitch std_msgs/msg/Float32 "{data: 5.0}"
```