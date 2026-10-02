

class KalmanFilter():

    def __init__(self):
        self.angle = 0.0
        self.bias = 0.0
        self.p00 = 0.0          # angle uncertainty
        self.p01 = 0.0          # covariance
        self.p10 = 0.0          # covariance
        self.p11 = 0.0          # bias uncertainty
        self.q_angle = 0.001    # uncertainty in angle (gyroscope) due to vibrations
        self.q_bias = 0.003     # uncertainty in rotation speed (gyroscope) due to equipment
        self.r_measure = 0.03   # uncertainty in angle (accelerometer) due to physical bumps

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