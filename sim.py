import mujoco
import mujoco.viewer
import time
import numpy as np

# model from XML

model = mujoco.MjModel.from_xml_path('balance_bot.xml')

# data object for current state

data = mujoco.MjData(model)

# launch viewer

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():

        # read sensors

        gyro_data = data.sensor('gyro').data
        accel_data = data.sensor('accel').data

        # later: implement Kalman filter

        # xquat: external orientation quaternion - 4-element array [qw, qx, qy, qz]
        #   qw: magnitude
        #   qx, qy, qz: vector components

        quaternion = data.body('chassis').xquat

        # euler angle for an airplane
        #   roll: tilt wings side to side
        #   pitch: tilt nose up/down 
        #   yaw: turn left/right

        # gimbal lock: when you have the object on 3 axes and rotate it until two of them "become" the same axis
        # convert quaternion to euler pitch

        pitch = np.arcsin(2.0 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]))
        
        # later: calculate the motor output using PID
        # error = 0 - pitch
        # motor_output = (Kp * error) + (Kd * derivative) + (Ki * integral)

        motor_output = 0.0 

        # later: add PWM 
        # control actuation each motor

        data.ctrl[0] = motor_output
        data.ctrl[1] = motor_output
        
        # Step the physics simulation forward by one timestep (0.01s)
        # simulation will calculate the next state using gravity, joints, collisions, etc.

        mujoco.mj_step(model, data)
        
        # sync visualizer with physics math

        viewer.sync()

        # sleep so that the computer's time matches with real world time    

        time.sleep(model.opt.timestep)