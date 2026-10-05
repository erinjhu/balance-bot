from edge_robot.kalman_filter import KalmanFilter

def test_kalman_convergence():
    angle_uncert = 1.0
    bias_uncert = 1.0
    kf = KalmanFilter(angle_uncert, bias_uncert)
    
    # Simulate a robot standing still but tilted at exactly 10 degrees
    for x in range(100):
        kf.predict_next_angle(meas_ang_velo=0.0, dt=0.01)
        kf.update(accel_angle=10.0)
        print(f"Iteration: {x}, Angle: {kf.angle}")
        
    # The filter should converge on 10 degrees
    assert 9.9 < kf.angle < 10.1, f"Filter failed to converge, got {kf.angle}"