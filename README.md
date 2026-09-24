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

Run in each terminal session

```
source /opt/ros/jazzy/setup.bash
```