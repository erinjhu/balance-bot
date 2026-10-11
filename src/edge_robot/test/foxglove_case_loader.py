"""Helpers for loading JSON-based Foxglove IMU test cases.

A "case" lives in foxglove_imu_TCs/<name>/ and is two files:
  <name>          -> input: {"description": str, "samples": [ <imu-sample>, ... ]}
  <name>_expected -> expectations: {"pitch": <check>, "pwm": <check>}

Each <imu-sample> is a full sensor_msgs/Imu as JSON, plus a synthetic
"t" (seconds from the start of the case). The loader turns "t" into a
real header.stamp so the state estimator sees an advancing clock.

This module only loads and converts data. It does no ROS spinning and
makes no assertions, so it can be unit-tested on its own.
"""

import json
from pathlib import Path

from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time


def load_case(input_path):
    """Read a case input file and return (description, samples list)."""
    data = json.loads(Path(input_path).read_text())
    return data.get("description", ""), data["samples"]


def load_expected(expected_path):
    """Read a case expected file and return its dict of checks."""
    return json.loads(Path(expected_path).read_text())


def sample_to_imu(sample):
    """Convert one JSON sample dict into a sensor_msgs/Imu message.

    "t" (seconds from case start) becomes header.stamp. All other fields
    map straight onto the Imu message; anything omitted defaults to zero,
    except orientation.w which defaults to 1.0 (a valid identity quaternion).
    """
    msg = Imu()

    t = float(sample.get("t", 0.0))
    sec = int(t)
    nanosec = int(round((t - sec) * 1e9))
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)
    msg.header.frame_id = sample.get("header", {}).get("frame_id", "")

    ori = sample.get("orientation", {})
    msg.orientation.x = float(ori.get("x", 0.0))
    msg.orientation.y = float(ori.get("y", 0.0))
    msg.orientation.z = float(ori.get("z", 0.0))
    msg.orientation.w = float(ori.get("w", 1.0))  # identity quaternion if omitted

    ang = sample.get("angular_velocity", {})
    msg.angular_velocity.x = float(ang.get("x", 0.0))
    msg.angular_velocity.y = float(ang.get("y", 0.0))
    msg.angular_velocity.z = float(ang.get("z", 0.0))

    acc = sample.get("linear_acceleration", {})
    msg.linear_acceleration.x = float(acc.get("x", 0.0))
    msg.linear_acceleration.y = float(acc.get("y", 0.0))
    msg.linear_acceleration.z = float(acc.get("z", 0.0))

    return msg
