

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
        #   - doubt accumulates and gets added to the total doubt (the
        #     Q matrix)
        #   - chosen variable: angle (gyroscope)
        #       - easier to track relative change
        #       - don't use the acceleration (accelerometer) because 
        #         it doesn't make sense to predict the acceleration
        #         using kinematics equations. it makes more sense to 
        #         predict the next value of the angular speed and angle
        #   - use a 1D matrix because the angle and bias are 
        #     independent variables

        self.q_angle = 0.001    
        self.q_bias = 0.003     

        # R: matrix for measurement noise (external sensor doubt)
        #   - doubt stays constant
        #   - chosen variable: acceleration (accelerometer)
  
        self.r_measure = 0.03   

    def predict_next_angle(self, meas_rotation_speed, dt):

        # - calculate the angle prediction
        # - the gyroscope measures the angle speed, but it doesn't 
        #   measure the angle
        
        actual_rotation_speed = meas_rotation_speed - self.bias
        self.angle += (actual_rotation_speed * dt)

        # the P matrix is for the doubt
        # p00: total accumulated doubt = new bias uncertainty + new cross uncertainty + angle drift
        # p11: new bias uncertainty = bias uncertainty rate of change * dt + old bias uncertainty
        # p01: new cross uncertainty = old cross uncertainty - current bias uncertainty * dt
        #   - subtract to compensate for the bias. originally add the bias, but if the bias
        #     is too high, the prediction for the angle will be too high. 
        # p10: same as p01

        self.p00 += dt * (dt * self.p11 - self.p01 - self.p10 + self.q_angle) 
        self.p01 -= dt * self.p11          
        self.p10 -= dt * self.p11           
        self.p11 += self.q_bias * dt        

        return 0

    def update(self, accel_angle):

        # - update the prediction for the angle using the acceleration data

        error = accel_angle - self.angle
        total_doubt = self.p00 + self.r_measure # system uncertainty
        k0 = self.p00 / total_doubt             # ratio (Kalman gain) to correct angle
        k1 = self.p10 / total_doubt             # ratio (Kalman gain) to correct hardware bias
        self.angle += (k0 * error)              
        self.bias += (k1 * error)
