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

```
pip install mujoco
```

Run in each new terminal session, not in a venv

```
source /opt/ros/jazzy/setup.bash
```

Open multiple terminals

Start the observability node
```
cd src/edge_robot
python3 observability_node.py
```

Listen to the observability node
```

```


## Set up joint movements

Terminal 1 (WSL):

```
sudo apt install ros-jazzy-foxglove-bridge -y
```

```
source /opt/ros/jazzy/setup.bash
```

```
ros2 run foxglove_bridge foxglove_bridge
```

Terminal 2 (WSL):

```
cd src/edge_robot
```

```
source /opt/ros/jazzy/setup.bash
```

```
python3 telemetry_node.py
```

Web browser:

app.foxglove.dev
Open connection > Foxglove WebSocket
ws://localhost:8765 > Open
Add Panel > Plot > Series > Y value > /joint_states > position[0]