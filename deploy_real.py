#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real Robot Deployment Script for Unitree Go2 X
Hurricane Walk Policy Deployment
"""

import torch
import numpy as np
import yaml
import time
import argparse
from collections import deque
from pathlib import Path
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


class Go2RobotInterface:
    """
    Go2 机器人接口类
    """
    def __init__(self):
        """
        初始化机器人接口
        """
        print("Initializing Unitree Go2 robot interface...")

        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._state_callback, 10)
        self.lowcmd = LowCmd_()
        self.lowstate = LowState_()
        self.crc = CRC()

        # 状态数据
        self.joint_pos = np.zeros(12)  # 关节位置 [rad]
        self.joint_vel = np.zeros(12)  # 关节速度 [rad/s]
        self.imu_ang_vel = np.zeros(3)  # 角速度 [rad/s] (roll, pitch, yaw rates)
        self.imu_quat = np.array([0., 0., 0., 1.])  # 四元数 (x, y, z, w)
        self.base_lin_vel = np.zeros(3)  # 基座线速度估计 [m/s]

        # 安全标志
        self.is_initialized = False
        self.emergency_stop = False

        print("Robot interface initialized (placeholder mode)")
        print("请根据您的 SDK 版本取消注释并修改初始化代码")

    def _state_callback(self, msg):
        """
        状态回调函数，接收机器人状态数据
        """
        self.lowstate = msg
        
        # 读取关节状态 (Go2 关节顺序)
        for i in range(12):
            self.joint_pos[i] = msg.motor_state[i].q
            self.joint_vel[i] = msg.motor_state[i].dq
        
        # 读取 IMU 数据
        self.imu_ang_vel = np.array([
            msg.imu_state.gyroscope[0],
            msg.imu_state.gyroscope[1],
            msg.imu_state.gyroscope[2]
        ])
        
        self.imu_quat = np.array([
            msg.imu_state.quaternion[0],
            msg.imu_state.quaternion[1],
            msg.imu_state.quaternion[2],
            msg.imu_state.quaternion[3]
        ])
        pass

    def get_state(self):
        """
        获取当前机器人状态

        Returns:
            dict: 包含 joint_pos, joint_vel, imu_ang_vel, imu_quat, base_lin_vel
        """
        # TODO: 在实际部署时，这里应该从SDK读取实时数据
        # 现在返回当前缓存的状态
        return {
            'joint_pos': self.joint_pos.copy(),
            'joint_vel': self.joint_vel.copy(),
            'imu_ang_vel': self.imu_ang_vel.copy(),
            'imu_quat': self.imu_quat.copy(),
            'base_lin_vel': self.base_lin_vel.copy()
        }

    def send_command(self, joint_torques, joint_pos_targets, kp, kd):
        """
        发送控制命令到机器人

        Args:
            joint_torques: [12] 前馈扭矩 [N*m]
            joint_pos_targets: [12] 目标关节位置 [rad]
            kp: [12] 位置增益
            kd: [12] 速度增益
        """
        # TODO: 使用 unitree_sdk2_python 发送命令示例
        # for i in range(12):
        #     self.lowcmd.motor_cmd[i].q = joint_pos_targets[i]
        #     self.lowcmd.motor_cmd[i].dq = 0.0
        #     self.lowcmd.motor_cmd[i].kp = kp[i]
        #     self.lowcmd.motor_cmd[i].kd = kd[i]
        #     self.lowcmd.motor_cmd[i].tau = joint_torques[i]
        #
        # # 设置 CRC 校验
        # self.lowcmd.crc = self.crc.Crc(self.lowcmd)
        #
        # # 发布命令
        # self.lowcmd_publisher.Write(self.lowcmd)
        pass

    def set_to_damping_mode(self):
        """
        设置机器人为阻尼模式（安全模式）
        """
        print("Setting robot to damping mode...")
        # TODO: 实现阻尼模式
        # zero_torques = np.zeros(12)
        # current_pos = self.joint_pos.copy()
        # kp = np.zeros(12)
        # kd = np.array([5.0] * 12)  # 只有阻尼
        # self.send_command(zero_torques, current_pos, kp, kd)


class PolicyRunner:
    """
    策略运行器类
    负责加载模型、构建观测、推理动作
    """

    def __init__(self, config_path, model_path):
        """
        初始化策略运行器

        Args:
            config_path: 配置文件路径
            model_path: 模型文件路径
        """
        # 加载配置
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        # 提取配置参数
        model_cfg = self.cfg['go2/himloco']
        self.num_obs = model_cfg['num_observations']
        self.num_obs_single = model_cfg['num_observations']
        self.obs_history_length = len(model_cfg['observations_history'])
        self.num_obs_total = self.num_obs_single * self.obs_history_length

        self.clip_obs = model_cfg['clip_obs']
        self.clip_actions_lower = np.array(model_cfg['clip_actions_lower'])
        self.clip_actions_upper = np.array(model_cfg['clip_actions_upper'])

        self.rl_kp = np.array(model_cfg['rl_kp'])
        self.rl_kd = np.array(model_cfg['rl_kd'])

        self.action_scale = np.array(model_cfg['action_scale'])
        self.default_dof_pos = np.array(model_cfg['default_dof_pos'])
        self.joint_mapping = model_cfg['joint_mapping']
        self.torque_limits = np.array(model_cfg['torque_limits'])

        # 缩放因子
        self.lin_vel_scale = model_cfg['lin_vel_scale']
        self.ang_vel_scale = model_cfg['ang_vel_scale']
        self.dof_pos_scale = model_cfg['dof_pos_scale']
        self.dof_vel_scale = model_cfg['dof_vel_scale']
        self.commands_scale = np.array(model_cfg['commands_scale'])

        # 加载模型
        print(f"Loading policy from: {model_path}")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy = torch.jit.load(model_path).to(self.device)
        self.policy.eval()
        print(f"Policy loaded successfully on {self.device}")

        # 初始化观测历史缓冲区 (存储最近 6 帧)
        self.obs_history = deque(maxlen=self.obs_history_length)
        for _ in range(self.obs_history_length):
            self.obs_history.append(np.zeros(self.num_obs_single))

        # 上一次的动作
        self.last_actions = np.zeros(12)

        # 用户命令 (lin_vel_x, lin_vel_y, ang_vel_yaw, heading)
        self.commands = np.array([0.0, 0.0, 0.0, 0.0])

        print(f"Observation shape: single={self.num_obs_single}, history={self.obs_history_length}, total={self.num_obs_total}")

    def quat_to_gravity_vector(self, quat):
        """
        从四元数计算重力向量（机器人坐标系中的重力方向）

        Args:
            quat: 四元数 [x, y, z, w]

        Returns:
            gravity_vec: [3] 重力向量
        """
        x, y, z, w = quat

        # 旋转矩阵的第三列（Z轴在世界坐标系中的表示）
        # 世界坐标系中重力向量为 [0, 0, -1]
        # 旋转到机器人坐标系
        gx = 2 * (x*z - w*y)
        gy = 2 * (y*z + w*x)
        gz = 2 * (0.5 - x*x - y*y)

        return np.array([gx, gy, gz])

    def build_observation(self, state):
        """
        构建观测向量

        观测向量组成 (45维):
        - commands (4): lin_vel_x, lin_vel_y, ang_vel_yaw, heading
        - ang_vel (3): IMU角速度
        - gravity_vec (3): 重力向量
        - dof_pos (12): 关节位置
        - dof_vel (12): 关节速度
        - actions (12): 上一次动作

        Args:
            state: 机器人状态字典

        Returns:
            obs: [45] 观测向量
        """
        # 按照训练顺序映射关节数据
        joint_pos_mapped = state['joint_pos'][self.joint_mapping]
        joint_vel_mapped = state['joint_vel'][self.joint_mapping]

        # 计算相对关节位置（相对于默认位置）
        dof_pos_relative = joint_pos_mapped - self.default_dof_pos

        # 计算重力向量
        gravity_vec = self.quat_to_gravity_vector(state['imu_quat'])

        # 构建观测向量 (顺序要与训练时一致)
        obs = np.concatenate([
            self.commands * self.commands_scale,                  # [4]
            state['imu_ang_vel'] * self.ang_vel_scale,           # [3]
            gravity_vec,                                          # [3]
            dof_pos_relative * self.dof_pos_scale,               # [12]
            joint_vel_mapped * self.dof_vel_scale,               # [12]
            self.last_actions                                     # [12]
        ])

        # Clip 观测
        obs = np.clip(obs, -self.clip_obs, self.clip_obs)

        return obs

    def get_action(self, state):
        """
        通过策略网络获取动作

        Args:
            state: 机器人状态字典

        Returns:
            actions: [12] 动作向量
        """
        # 构建当前观测
        obs = self.build_observation(state)

        # 添加到历史
        self.obs_history.append(obs)

        # 构建历史观测向量 (按时间顺序: [t-5, t-4, ..., t-1, t])
        obs_history_array = np.concatenate(list(self.obs_history))

        # 转换为 PyTorch tensor
        obs_tensor = torch.from_numpy(obs_history_array).float().unsqueeze(0).to(self.device)

        # 推理
        with torch.no_grad():
            actions_tensor = self.policy(obs_tensor)

        # 转换回 numpy
        actions = actions_tensor.cpu().numpy().squeeze()

        # Clip 动作
        actions = np.clip(actions, self.clip_actions_lower, self.clip_actions_upper)

        # 保存动作
        self.last_actions = actions.copy()

        return actions

    def compute_joint_targets(self, actions):
        """
        将动作转换为关节目标位置

        Args:
            actions: [12] 归一化动作

        Returns:
            joint_targets: [12] 关节目标位置 [rad]
        """
        # target_pos = default_pos + action_scale * action
        joint_targets = self.default_dof_pos + self.action_scale * actions

        return joint_targets

    def set_command(self, lin_vel_x=0.0, lin_vel_y=0.0, ang_vel_yaw=0.0, heading=0.0):
        """
        设置运动命令

        Args:
            lin_vel_x: 前进速度 [m/s]
            lin_vel_y: 横向速度 [m/s]
            ang_vel_yaw: 旋转角速度 [rad/s]
            heading: 目标朝向 [rad]
        """
        self.commands = np.array([lin_vel_x, lin_vel_y, ang_vel_yaw, heading])
        print(f"Command updated: vx={lin_vel_x:.2f}, vy={lin_vel_y:.2f}, vyaw={ang_vel_yaw:.2f}, heading={heading:.2f}")


def main():
    parser = argparse.ArgumentParser(description='Deploy policy on Unitree Go2 X')
    parser.add_argument('--config', type=str, default='deploy/deploy_real/config/go2.yaml', help='Path to config file')
    parser.add_argument('--model', type=str, default='PATH/TO/YOUR/MODEL.pt', help='Path to the trained policy model')
    parser.add_argument('--freq', type=int, default=50, help='Control frequency [Hz]')
    parser.add_argument('--duration', type=float, default=60.0, help='Run duration [seconds], -1 for infinite')

    # 运动命令参数
    parser.add_argument('--vx', type=float, default=0.0, help='Forward velocity command [m/s]')
    parser.add_argument('--vy', type=float, default=0.0, help='Lateral velocity command [m/s]')
    parser.add_argument('--vyaw', type=float, default=0.0, help='Yaw rate command [rad/s]')

    args = parser.parse_args()

    # 检查模型路径
    if args.model == 'PATH/TO/YOUR/MODEL.pt':
        print("\n" + "="*60)
        print("错误: 请指定训练好的模型路径!")
        print("使用方法: python deploy_real.py --model /path/to/your/model.pt")
        print("="*60 + "\n")
        sys.exit(1)

    if not Path(args.model).exists():
        print(f"\n错误: 模型文件不存在: {args.model}\n")
        sys.exit(1)

    print("\n" + "="*60)
    print("Hurricane Walk Real Robot Deployment")
    print("="*60)
    print(f"Config: {args.config}")
    print(f"Model: {args.model}")
    print(f"Control Frequency: {args.freq} Hz")
    print(f"Duration: {args.duration} seconds" if args.duration > 0 else "Duration: Infinite")
    print("="*60 + "\n")

    # 初始化机器人接口
    robot = Go2RobotInterface()

    # 初始化策略运行器
    policy_runner = PolicyRunner(args.config, args.model)

    # 设置运动命令
    policy_runner.set_command(lin_vel_x=args.vx, lin_vel_y=args.vy, ang_vel_yaw=args.vyaw)

    # 控制循环参数
    dt = 1.0 / args.freq
    start_time = time.time()
    iteration = 0

    print("\n准备开始控制循环...")
    print("按 Ctrl+C 停止程序\n")

    try:
        # 等待一小段时间让系统稳定
        time.sleep(1.0)

        while True:
            loop_start = time.time()

            # 1. 读取机器人状态
            state = robot.get_state()

            # 2. 通过策略获取动作
            actions = policy_runner.get_action(state)

            # 3. 计算关节目标位置
            joint_targets = policy_runner.compute_joint_targets(actions)

            # 4. 将目标从训练顺序映射回硬件顺序
            joint_targets_hardware = np.zeros(12)
            for train_idx, hardware_idx in enumerate(policy_runner.joint_mapping):
                joint_targets_hardware[hardware_idx] = joint_targets[train_idx]

            # 5. 计算前馈扭矩 (这里简单设为0，也可以添加重力补偿)
            feedforward_torques = np.zeros(12)

            # 6. 发送命令到机器人
            robot.send_command(
                joint_torques=feedforward_torques,
                joint_pos_targets=joint_targets_hardware,
                kp=policy_runner.rl_kp,
                kd=policy_runner.rl_kd
            )

            # 7. 打印信息
            if iteration % (args.freq * 2) == 0:  # 每2秒打印一次
                elapsed = time.time() - start_time
                print(f"[{elapsed:.1f}s] Iter: {iteration}, "
                      f"Joint0: {state['joint_pos'][0]:.3f} rad, "
                      f"Action0: {actions[0]:.3f}, "
                      f"Target0: {joint_targets_hardware[0]:.3f} rad")

            iteration += 1

            # 检查是否到达持续时间
            if args.duration > 0 and (time.time() - start_time) > args.duration:
                print(f"\n达到指定持续时间 {args.duration} 秒，停止运行")
                break

            # 等待到下一个控制周期
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(f"警告: 控制循环超时! 期望: {dt*1000:.1f}ms, 实际: {elapsed*1000:.1f}ms")

    except KeyboardInterrupt:
        print("\n\n检测到 Ctrl+C，正在停止...")

    finally:
        # 清理：将机器人设为阻尼模式
        print("\n正在设置机器人为安全模式...")
        robot.set_to_damping_mode()
        time.sleep(0.5)
        print("程序结束")
        print(f"总迭代次数: {iteration}")
        print(f"总运行时间: {time.time() - start_time:.2f} 秒\n")


if __name__ == "__main__":
    main()
