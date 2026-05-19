#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import yaml
import logging
import argparse
import numpy as np
from pathlib import Path
from typing import Tuple
from dataclasses import dataclass
from gello.dynamixel.driver import DynamixelDriver
from gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from ufactory_devices.robot import UFRobotConfig, UFRobot

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('gello_teleop')

@dataclass
class GelloTeleopConfig:
    fps: int = 30
    port: str = None
    joint_ids: Tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7)
    joint_signs: Tuple[int, ...] = (1, 1, 1, 1, 1, 1, 1) # if follow the original open-sourced gello xarm7 setup
    start_joints: Tuple[float, ...] = (0, 0, 0, np.pi/2, 0, np.pi/2, 0)
    gripper_id: int = 8  # -1: no gripper
    torque_joint_ids: Tuple[int, ...] = None  # the joints will activate torque mode.


class UFRobotTeleop(object):
    def __init__(self, config: GelloTeleopConfig, robot_config: UFRobotConfig):
        self.config = config
        assert len(self.config.joint_ids) == len(self.config.joint_signs) == len(self.config.start_joints)
        if len(self.config.joint_signs) != len(self.config.joint_ids):
            raise ValueError("joint_signs and joint_ids length mismatch")
        if len(config.start_joints) != len(config.joint_ids):
            raise ValueError("start_joints and joint_ids length mismatch")
        self._action_dim = len(self.config.joint_ids)
        if robot_config.robot_mode != 6:
            raise ValueError("Gello teleop requires robot_mode=6 joint control mode")
        
        # auto get joint offset from gello
        joint_ids = []
        joint_ids.extend(self.config.joint_ids)
        if self.config.gripper_id >= 0:
            joint_ids.append(self.config.gripper_id)
            self._action_dim += 1
        driver = DynamixelDriver(joint_ids, port=self.config.port, baudrate=57600)
        for _ in range(10):
            driver.get_joints()  # warmup
        curr_joints = driver.get_joints()
        driver.close()
        joint_offsets = []
        for i in range(len(self.config.start_joints)):
            offset = curr_joints[i] - self.config.start_joints[i] / self.config.joint_signs[i]
            joint_offsets.append(offset)
        if self.config.gripper_id >= 0:
            gripper_config = [self.config.gripper_id, np.rad2deg(curr_joints[-1]) - 0.2, np.rad2deg(curr_joints[-1]) - 42]
        else:
            gripper_config = None

        param_dict = {
                "joint_ids": self.config.joint_ids,
                "joint_signs": self.config.joint_signs,
                "joint_offsets": joint_offsets,
                "gripper_config": gripper_config
        }

        if self.config.torque_joint_ids:
            driver = DynamixelDriver(self.config.torque_joint_ids, port=self.config.port, baudrate=57600)
            driver.set_torque_mode(True)
            driver.close()

        self._dynamixel_robo_config = DynamixelRobotConfig(**param_dict)
        print(self._dynamixel_robo_config)
        self.gello_agent = GelloAgent(port=self.config.port, dynamixel_config=self._dynamixel_robo_config)

        self.robot = UFRobot(robot_config)
        
        self.status = 2
    
    def set_status(self, status):
        self.status = status

    def run(self):
        sleep_time = 1 / self.config.fps
        fake_obs = dict({"joint_state": np.array([0.0]*self._action_dim)}) # for agent.act() argument, actually no use

        while self.status > 0:
            time.sleep(sleep_time)

            action = np.asarray(self.gello_agent.act(fake_obs), dtype=float).reshape(-1)
            if len(action) != self._action_dim:
                raise ValueError(f"Joint action length must be {self._action_dim}, got {len(action)}")

            code = self.robot.send_action(action)
            if code != 0:
                break


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='configuration args')
    parser.add_argument('-c', '--config', type=str, required=True, 
                       help='configuration file path, e.g.my_config.yaml')
    args = parser.parse_args()
    try:
        with open(Path(args.config).expanduser(), 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config yaml file: {e}")
        exit(1)
    
    robot_confg = UFRobotConfig(**config['RobotConfig'])
    teleop_confg = GelloTeleopConfig(**config['TeleoperatorConfig'])
    teleop = UFRobotTeleop(teleop_confg, robot_confg)

    time.sleep(1)

    print("\n********** Test Teleop With Robot **********")
    input('Enter to control robot with teleop >>> ')

    print("\n********** Teleop Control Loop Start **********")
    teleop.set_status(1)
    teleop.run()
