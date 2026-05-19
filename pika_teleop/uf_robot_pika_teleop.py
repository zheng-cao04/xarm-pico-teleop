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

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from ufactory_devices.pika import PikaDevice
from ufactory_devices.robot import UFRobotConfig, UFRobot
from ufactory_devices.transformations import Transformations

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('pika_teleop')

@dataclass
class PikaTeleopConfig:
    fps: int = 30
    use_gripper: bool = True
    pika_sense_port: str = None
    vive_tracker_id: str = None
    tracker_to_robot_eef: Tuple[float, ...] = (0, 0, 0, np.pi, -np.pi / 2, 0)


class UFRobotTeleop(object):
    def __init__(self, config: PikaTeleopConfig, robot_config: UFRobotConfig):
        self.config = config
        self.robot = UFRobot(robot_config)
        self.pika_device = PikaDevice(1, pika_sense_port=self.config.pika_sense_port)
        self.pika_sense = self.pika_device.pika_sense
        self.status = 2
    
    def set_status(self, status):
        self.status = status

    def run(self):
        init_state = self.pika_sense.get_command_state()
        curr_state = init_state

        last_gripper_distance = 0

        ctrl_flag = False # 是否开启遥操作
        need_initial = False
        sleep_time = 1 / self.config.fps

        # pika坐标系到机械臂坐标系的变换关系对应的变换矩阵
        pika_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(*self.config.tracker_to_robot_eef)
        # 机械臂初始位置对应的变换矩阵
        robot_base_matrix = None
        # pika初始位置转换到机械臂坐标系后对应的变换矩阵
        pika_begin_robot_matrix = None
        # pika目标位置转换到机械臂坐标系后对应的变换矩阵
        pika_end_robot_matrix = None

        gripper_norm = 0

        vive_tracker_id = self.pika_device.pika_tracker_device if self.config.vive_tracker_id is None else self.config.vive_tracker_id

        while self.status > 0:
            time.sleep(sleep_time)

            state = self.pika_sense.get_command_state()
            if state != curr_state:
                curr_state = state
                if self.status != 1 and curr_state != init_state:
                    self.status = 1
                    need_initial = True
                    self.robot.robot_init()
                    logger.info('开始遥操作')
                    time.sleep(1)
                elif self.status != 2 and curr_state == init_state:
                    self.status = 2
                    logger.info('停止遥操作')
                    continue
            
            if self.status != 1:
                continue

            pose = self.pika_sense.get_pose(vive_tracker_id)
            if not pose:
                continue
            x, y, z = pose.position[0] * 1000, pose.position[1] * 1000, pose.position[2] * 1000

            if need_initial:
                need_initial = False
                _, robot_pos = self.robot.get_position()
                logger.info('[初始] 机械臂位置: {}'.format(robot_pos))

                # 机械臂初始位置对应的变换矩阵
                robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*robot_pos)

                # pika初始位置转换到机械臂坐标系后对应的变换矩阵
                pika_begin_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)
                pika_end_robot_matrix = pika_begin_robot_matrix
            else:
                # pika目标位置转换到机械臂坐标系后对应的变换矩阵
                pika_end_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)

            robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(pika_begin_robot_matrix, pika_end_robot_matrix, robot_base_matrix, is_axis_angle=True)
            action = robot_target_pose
            if self.config.use_gripper:
                distance  = self.pika_sense.get_gripper_distance()
                if distance is not None:
                    if abs(last_gripper_distance - distance) > 2:
                        last_gripper_distance = distance
                        gripper_norm = (100 - distance) / (100 - 0)
                    action.append(gripper_norm)
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
    teleop_confg = PikaTeleopConfig(**config['TeleoperatorConfig'])
    teleop = UFRobotTeleop(teleop_confg, robot_confg)

    time.sleep(1)

    print("\n********** Test Teleop With Robot **********")
    input('Enter to control robot with teleop >>> ')

    print("\n********** Teleop Control Loop Start **********")
    teleop.set_status(2)
    teleop.run()
