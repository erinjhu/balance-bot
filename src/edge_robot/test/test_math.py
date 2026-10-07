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


from edge_robot.pid_node import PIDNode


def make_pid(kp=1.0, ki=1.0, kd=1.0, integral_limit=100.0):
    """Build a PIDNode without running its ROS __init__.

    calculate_output() is pure math over a handful of attributes, so we
    create the instance with object.__new__ (skipping super().__init__,
    which would require a running ROS context) and set just those
    attributes. This keeps the PID math tests fast and deterministic with
    no ROS dependency or timing.
    """
    pid = object.__new__(PIDNode)
    pid.kp = kp
    pid.ki = ki
    pid.kd = kd
    pid.total_error = 0.0
    pid.previous_error = 0.0
    pid.integral_limit = integral_limit
    return pid


def test_pid_zero_error_gives_zero_output():
    # No error, no history -> nothing to correct.
    pid = make_pid()
    out = pid.calculate_output(target_pitch=0.0, current_pitch=0.0, dt=0.01)
    assert out == 0.0, f"Expected 0 output for zero error, got {out}"


def test_pid_output_sign_follows_error():
    # error = target - current. Positive error should give positive output
    # (and vice versa) with positive gains.
    pos = make_pid()
    pos_out = pos.calculate_output(target_pitch=1.0, current_pitch=0.0, dt=0.01)
    assert pos_out > 0, f"Expected positive output for positive error, got {pos_out}"

    neg = make_pid()
    neg_out = neg.calculate_output(target_pitch=-1.0, current_pitch=0.0, dt=0.01)
    assert neg_out < 0, f"Expected negative output for negative error, got {neg_out}"


def test_pid_larger_error_gives_stronger_output():
    # A bigger error should produce a larger-magnitude command. Compare two
    # fresh controllers so neither carries integral/derivative history.
    small = make_pid()
    small_out = small.calculate_output(target_pitch=1.0, current_pitch=0.0, dt=0.01)

    large = make_pid()
    large_out = large.calculate_output(target_pitch=5.0, current_pitch=0.0, dt=0.01)

    assert abs(large_out) > abs(small_out), (
        f"Larger error did not give a stronger command: "
        f"small={small_out}, large={large_out}"
    )


def test_pid_integral_is_clamped():
    # Feed a constant error for many steps; the integral term must not grow
    # past ki * integral_limit no matter how long the error persists.
    pid = make_pid(kp=0.0, ki=1.0, kd=0.0, integral_limit=5.0)
    out = 0.0
    for _ in range(100000):
        out = pid.calculate_output(target_pitch=1.0, current_pitch=0.0, dt=0.01)
    # With kp=kd=0, output is purely the integral term = ki * total_error,
    # and total_error is clamped to integral_limit.
    assert out <= 5.0 + 1e-9, f"Integral not clamped, got {out}"
    assert pid.total_error <= 5.0 + 1e-9, (
        f"total_error exceeded clamp, got {pid.total_error}"
    )
