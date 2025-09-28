# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class HurricaneHighWalkRoughCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env ):
        num_envs = 2048  # 适当减少环境数量，加快训练速度

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'  # 仅使用平地训练，移除复杂地形
        

    class commands( LeggedRobotCfg.commands ):
            curriculum = True  # 关闭课程学习，专注基础步态
            max_curriculum = 2.0  # 降低最大难度
            num_commands = 4  # 简化命令：仅保留x方向线速度和偏航角速度
            resampling_time = 10.0  # 适当缩短命令更新间隔
            heading_command = True  # 关闭航向模式，减少复杂性
            class ranges( LeggedRobotCfg.commands.ranges):
                lin_vel_x = [-0.6, 0.6]  # 低姿态下的适度速度范围
                lin_vel_y = [-0.2, 0.2]  # 严格限制侧向移动
                ang_vel_yaw = [-3.14, 3.14]  # 减小转向范围
                heading = [-3.14, 3.14]

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.30]  # 降低初始高度，实现低姿态
        default_joint_angles = {  # 调整关节角度，确保四肢在身体投影内
            # 髋部关节角度大幅减小，防止四肢过度张开
            'FL_hip_joint': 0.03,   # 显著减少前左腿外旋
            'RL_hip_joint': 0.03,   # 显著减少后左腿外旋
            'FR_hip_joint': -0.03,  # 显著减少前右腿内旋
            'RR_hip_joint': -0.03,  # 显著减少后右腿内旋

            # 大腿关节角度增大，降低整体高度
            'FL_thigh_joint': 1.0,     # 增加弯曲度
            'RL_thigh_joint': 1.2,     # 增加弯曲度
            'FR_thigh_joint': 1.0,     # 增加弯曲度
            'RR_thigh_joint': 1.2,     # 增加弯曲度

            # 小腿关节角度调整，配合大腿实现低姿态
            'FL_calf_joint': -1.4,   # 调整角度
            'RL_calf_joint': -1.4,   # 调整角度
            'FR_calf_joint': -1.4,   # 调整角度
            'RR_calf_joint': -1.4,   # 调整角度
        }

    class control( LeggedRobotCfg.control ):
        control_type = 'P'
        stiffness = {'joint': 80.0}  # 增加刚度，提高低姿态稳定性
        damping = {'joint': 2.0}     # 增加阻尼，减少摆动
        action_scale = 0.15  # 减小动作幅度，限制四肢过度运动
        decimation = 4
        hip_reduction = 1.0  # 增加髋部减速比，限制侧向运动

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go1/urdf/go1.urdf'
        name = "hurricane_low_walk"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf", "base"]  # 仅惩罚机身碰撞，无需考虑复杂地形
        terminate_after_contacts_on = ["base"]
        privileged_contacts_on = ["base", "thigh", "calf"]  # 简化接触检测
        self_collisions = 1  # 禁用自碰撞检查，加快训练
        flip_visual_attachments = False

    class rewards( LeggedRobotCfg.rewards ):
        class scales:
            termination = -0.0
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5
            lin_vel_z = -2.0
            ang_vel_xy = -0.2 # stronger penalty on roll/pitch rates
            orientation = -1.0 # stronger penalty to keep body level
            dof_acc = -2.5e-7
            joint_power = -2e-5
            base_height = -5.0 # stronger penalty to keep base at target low height
            foot_clearance = -0.05 # discourage excessive foot lift that can cause splay
            action_rate = -0.01
            smoothness = -0.01
            feet_air_time = 0.0
            collision = -0.1 # penalize non-foot collisions more
            feet_stumble = -0.0
            stand_still = -0.
            torques = -0.0
            dof_vel = -0.0
            dof_pos_limits = -0.0
            dof_vel_limits = -0.0
            torque_limits = -0.0
            

        only_positive_rewards = False
        tracking_sigma = 0.25
        soft_dof_pos_limit = 1.
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.30 # target low base height
        max_contact_force = 100.
        clearance_height_target = -0.10 # keep feet closer to base

class HurricaneHighWalkRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01  # 稍高熵系数，促进探索

    class runner( LeggedRobotCfgPPO.runner ):
        num_steps_per_env = 100  # 调整每环境步数
        max_iterations = 5000  # 减少总迭代次数，专注基础步态
        
        save_interval = 50
        run_name = ''
        experiment_name = 'rough_hurricane_low_walk'