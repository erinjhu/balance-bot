## Next session

- Implement Kalman filter

## Sept 24, 2026

- Program and test telemetry node
    - Publish 6 values for the motor position using sine
- Set up Foxglove

## Sept 23, 2026

- Set up ROS2 jazzy
- Program and test observability node; publish the CPU and memory usage
- Program and test video node
    - Camera issues
        - Set up usbipd to connect WSL to the Windows camera
        - Not working; most likely because it doesn't have the uvcvideo driver
        - For other robot projects will run on the actual device (e.g. Raspberry Pi or Jetson) which has Linux instead of WSL from main laptop
    - Decided to use mock video for now

## Sept 22, 2026

- Set up mujoco
    - Was having trouble installing mujoco in the venv since VS Code was mixing up Windows and WSL file systems
- Created robot XML
    - Added the chassis, IMU, wheels, motors, gyroscope (sensor for rotation speed), and accelerometer (sensor for linear acceleration)
- Set up basic simulation
    - Data from gyrocope and accelerometer
    - Data from xquat (magnitude and vector components)
    - Convert xquat to pitch (Euler angle)
    - Set the motors to 0 for now
    - Adjust timestep to sync viewer and simulation

![Screenshot of basic simulation](../images/sim_setup.png)