#!/usr/bin/env python

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from lerobot.robots.vla_drone.config_vla_drone import VLADroneConfig
from lerobot.robots.vla_drone.ros2_bridge import DronePose
from lerobot.robots.vla_drone.vla_drone import GRIPPER_LEFT, GRIPPER_RIGHT, VLADrone


@dataclass
class DummyCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


class FakeBus:
    def __init__(self):
        self.is_connected = False
        self.calibration = {}
        self.last_write = None
        self.motors = {GRIPPER_LEFT: object(), GRIPPER_RIGHT: object()}

    def connect(self):
        self.is_connected = True

    def disconnect(self, disable_torque=True):
        self.is_connected = False

    def configure_motors(self):
        pass

    def read_calibration(self):
        return {
            GRIPPER_LEFT: DummyCalibration(1, 0, 0, 1000, 2000),
            GRIPPER_RIGHT: DummyCalibration(2, 0, 0, 1000, 2000),
        }

    def sync_read(self, data_name, motors=None, *, normalize=True, num_retry=0):
        assert data_name == "Present_Position"
        return {GRIPPER_LEFT: 25.0, GRIPPER_RIGHT: 75.0}

    def sync_write(self, data_name, values, *, normalize=True, num_retry=0):
        self.last_write = (data_name, values, normalize)

    def write(self, data_name, motor, value, *, normalize=True, num_retry=0):
        pass


class FakePoseBridge:
    def __init__(self):
        self.is_connected = False
        self.last_setpoint = None
        self.pose = DronePose(1.0, 2.0, 0.5, 0.25, time.monotonic())

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_latest_pose(self, max_age_s):
        return self.pose

    def publish_setpoint(self, x, y, z, yaw):
        self.last_setpoint = (x, y, z, yaw)


class FakeCamera:
    def __init__(self):
        self.is_connected = False
        self.height = 2
        self.width = 3

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def read_latest(self):
        return np.zeros((2, 3, 3), dtype=np.uint8)


def make_robot(tmp_path):
    cfg = VLADroneConfig(
        id="test",
        calibration_dir=tmp_path,
        x_bounds=(-1.0, 1.0),
        y_bounds=(-2.0, 2.0),
        z_bounds=(0.0, 1.0),
        configure_gripper_motors=False,
    )
    bus = FakeBus()
    bridge = FakePoseBridge()
    cameras = {"front": FakeCamera(), "down": FakeCamera()}
    robot = VLADrone(cfg, bus=bus, pose_bridge=bridge, cameras=cameras)
    return robot, bus, bridge


def test_vla_drone_observation_features_and_values(tmp_path):
    robot, _, _ = make_robot(tmp_path)
    robot.connect()

    assert robot.observation_features["front"] == (2, 3, 3)
    assert robot.action_features == {
        "x": float,
        "y": float,
        "z": float,
        "yaw": float,
        "gripper_left.pos": float,
        "gripper_right.pos": float,
    }

    obs = robot.get_observation()
    assert obs["x"] == 1.0
    assert obs["y"] == 2.0
    assert obs["z"] == 0.5
    assert obs["yaw"] == 0.25
    assert obs["gripper_left.pos"] == 25.0
    assert obs["gripper_right.pos"] == 75.0
    assert obs["front"].shape == (2, 3, 3)
    assert obs["down"].shape == (2, 3, 3)


def test_vla_drone_send_action_clamps_and_routes_outputs(tmp_path):
    robot, bus, bridge = make_robot(tmp_path)
    robot.connect()

    sent = robot.send_action(
        {
            "x": 4.0,
            "y": -4.0,
            "z": 4.0,
            "yaw": 0.5,
            "gripper_left.pos": -20.0,
            "gripper_right.pos": 120.0,
        }
    )

    assert sent == {
        "x": 1.0,
        "y": -2.0,
        "z": 1.0,
        "yaw": 0.5,
        "gripper_left.pos": 0.0,
        "gripper_right.pos": 100.0,
    }
    assert bridge.last_setpoint == (1.0, -2.0, 1.0, 0.5)
    assert bus.last_write == (
        "Goal_Position",
        {GRIPPER_LEFT: 0.0, GRIPPER_RIGHT: 100.0},
        True,
    )
