

class KalmanFilter():

    def __init__(self):
        self.angle = 0.0        # angle to the vertical
        self.bias = 0.0         # offset of the gyroscope

        # P: matrix for total doubt 
        #   - total doubt that affects the variable that you are 
        #     predicting

        self.p00 = 0.0          # angle uncertainty
        self.p01 = 0.0          # covariance
        self.p10 = 0.0          # covariance
        self.p11 = 0.0          # bias uncertainty

        # Q: matrix for process noise (internal system doubt/drift)
        #   - new uncertainty added to the estimates 
        #     every time that the code calculates an estimate for the
        #     next value of a variable
        #   - doubt accumulates and gets added to the total doubt
        #   - chosen variable: angle (gyroscope)
        #       - easier to track relative change
        #       - don't use the acceleration (accelerometer) because 
        #         it doesn't make sense to predict the acceleration
        #         using kinematics equations. it makes more sense to 
        #         predict the next value of the angular speed and angle

        self.q_angle = 0.001    
        self.q_bias = 0.003     

        # R: matrix for measurement noise (external sensor doubt)
        #   - doubt stays constant
        #   - chosen variable: acceleration (accelerometer)
        #       - use this to check the 
  
        self.r_measure = 0.03   

    def predict_next_angle(self, meas_rotation_speed, dt):
        actual_rotation_speed = meas_rotation_speed - self.bias
        self.angle += (actual_rotation_speed * dt)
        self.p00 += dt * (dt * self.p11 - self.p01 - self.p10 + self.q_angle) #  add the uncertainty due to bias, covariance, and angle vibrations
        self.p01 -= dt * self.p11           # bias uncertainty affects angle uncertainty
        self.p10 -= dt * self.p11           # same as p10; matrix is symmetrical
        self.p11 += self.q_bias * dt        # update the bias uncertainty; over time, it gets more off track
        return 0

    def update(self, accel_angle):
        # accel_angle 
        #   the gyroscope
        error = accel_angle - self.angle
        total_doubt = self.p00 + self.r_measure # system uncertainty
        k0 = self.p00 / total_doubt             # ratio (Kalman gain) to correct angle
        k1 = self.p10 / total_doubt             # ratio (Kalman gain) to correct hardware bias
        self.angle += (k0 * error)              
        self.bias += (k1 * error)
