

class KalmanFilter():

    def __init__(self):
        self.angle = 0.0        # angle to the vertical
        self.bias = 0.0         # offset of the gyroscope

        # P: matrix for total doubt 
        #   - total doubt that affects the variable that you are 
        #     predicting

        self.p00 = 0.0          # angle uncertainty
        self.p01 = 0.0          # covariance; how bias uncertainty "ruins" the angle
        self.p10 = 0.0          # covariance; how angle uncertainty "ruins" the bias
        self.p11 = 0.0          # bias uncertainty

        # Q: matrix for process noise (internal system doubt/drift)
        #   - new uncertainty added to the estimates 
        #     every time that the code calculates an estimate for the
        #     next value of a variable
        #   - doubt accumulates and gets added to the total doubt (the
        #     P matrix)
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
        #     is too high, the prediction for the angle will be too low (the bias gets 
        #     subtracted from the angle). 
        # p10: same as p01

        self.p00 += dt * (dt * self.p11 - self.p01 - self.p10 + self.q_angle) 
        self.p01 -= dt * self.p11          
        self.p10 -= dt * self.p11           
        self.p11 += self.q_bias * dt        

        return 0

    def update(self, accel_angle):

        # 1. difference between measured angle (from the accelerometer) and 
        #    predicted angle (from the gyroscope angle speed data)
        # 2. add the accelerometer uncertainty to the total uncertainty
        # 3. find the Kalman gain, which is the ratio between the 
        #    prediction uncertainty and the total uncertainty. the
        #    total uncertainty includes the prediction (gyroscope)
        #    and the measurement (accelerometer) uncertainties.
        #       - k0: Kalman gain for angle
        #       - k1: Kalman gain for bias and angle covariance
        # 4. apply the update to the angle while accounting for the 
        #    uncertainty from the accelerometer
        # 5. apply the update to the bias while accounting for the 
        #    covariance between the angle and the bias
        # 6. after using the accelerometer to correct the system,
        #    there will be less uncertainty. use the Kalman gain to
        #    scale the current uncertainty in the P matrix and then 
        #    subtract it from the current P matrix. this step does
        #    P_new = P_old - (K * P_old)

        error = accel_angle - self.angle
        total_doubt = self.p00 + self.r_measure 
        k0 = self.p00 / total_doubt             
        k1 = self.p10 / total_doubt             
        self.angle += (k0 * error)              
        self.bias += (k1 * error)
        p00_temp = self.p00
        p01_temp = self.p01
        self.p00 -= k0 * p00_temp
        self.p01 -= k0 * p01_temp
        self.p10 -= k1 * p00_temp
        self.p11 -= k1 * p01_temp