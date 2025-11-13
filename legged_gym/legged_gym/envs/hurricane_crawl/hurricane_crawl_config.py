# Copyright 2025 Jackson Huang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
    主要修改点说明：
        初始高度降低:从0.42米降到0.15米
        默认关节角度调整：让腿部更加弯曲和收缩，以适应低矮姿态
        命令范围缩小:降低速度范围，因为匍匐时移动速度较慢
        base_height_target降低:从0.2降到0.10, 强制机器人保持低姿态
        增加相关惩罚：
        增加orientation惩罚权重(从-0.2到-1.0)以维持稳定姿态
        增加collision惩罚权重(从-0.0到-1.0)以避免机体触地
        增加base_height惩罚权重(从-1.0到-5.0)以强化低姿态要求
        降低max_contact_force:从100降到50,因为匍匐时接触力会更大
"""

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class HurricaneCrawlRoughCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env ):
        num_envs = 4096

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane' # "heightfield" # none, plane, heightfield or trimesh

    class commands( LeggedRobotCfg.commands ):
            curriculum = True
            max_curriculum = 2.0
            num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
            resampling_time = 10. # time before command are changed[s]
            heading_command = True # if true: compute ang vel command from heading error
            class ranges( LeggedRobotCfg.commands.ranges):
                lin_vel_x = [-0.5, 0.5] # min max [m/s]
                lin_vel_y = [-0.3, 0.3]   # min max [m/s]
                ang_vel_yaw = [-1.5, 1.5]    # min max [rad/s]
                heading = [-3.14, 3.14]

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.15] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.0,   # [rad]
            'RL_hip_joint': 0.0,   # [rad]
            'FR_hip_joint': -0.0 ,  # [rad]
            'RR_hip_joint': -0.0,   # [rad]

            'FL_thigh_joint': 1.2,     # [rad]
            'RL_thigh_joint': 1.2,   # [rad]
            'FR_thigh_joint': 1.2,     # [rad]
            'RR_thigh_joint': 1.2,   # [rad]

            'FL_calf_joint': -2.2,   # [rad]
            'RL_calf_joint': -2.2,    # [rad]
            'FR_calf_joint': -2.2,  # [rad]
            'RR_calf_joint': -2.2,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 40.0}  # [N*m/rad]
        damping = {'joint': 1.0}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        hip_reduction = 1.0

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        name = "hurricane_crawl"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf", "base"]
        terminate_after_contacts_on = ["base"]
        privileged_contacts_on = ["base", "thigh", "calf"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False # Some .obj meshes must be flipped from y-up to z-up
  
    class rewards( LeggedRobotCfg.rewards ):
        class scales:
            termination = -0.0                                   # 回合被终止的一次性惩罚权重
            tracking_lin_vel = 1.0                               # 线速度跟踪奖励权重
            tracking_ang_vel = 0.5                               # 角速度跟踪奖励权重
            lin_vel_z = -1.0                                     # 竖直线速度惩罚权重
            ang_vel_xy = -0.1                                    # 横滚和俯仰角速度惩罚权重
            orientation = -1.0                                   # 姿态偏差惩罚权重
            dof_acc = -2.5e-7                                    # 关节加速度惩罚权重
            joint_power = -2e-5                                  # 关节功率惩罚权重
            base_height = -5.0                                   # 机身高度偏差惩罚权重
            foot_clearance = -0.01                               # 足端高度惩罚权重
            action_rate = -0.01                                  # 动作一阶差分惩罚权重
            smoothness = -0.01                                   # 动作二阶差分惩罚权重
            feet_air_time =  0.0                                 # 足端腾空时长奖励权重
            collision = -1.0                                     # 非足端接触惩罚权重
            feet_stumble = -0.0                                  # 足端绊倒惩罚权重
            stand_still = -0.                                    # 静止奖励权重
            torques = -0.0                                       # 关节扭矩惩罚权重
            dof_vel = -0.0                                       # 关节速度惩罚权重
            dof_pos_limits = -0.0                                # 接近关节位置限制惩罚权重
            dof_vel_limits = -0.0                                # 接近关节速度限制惩罚权重
            torque_limits = -0.0                                 # 接近关节扭矩限制惩罚权重

        only_positive_rewards = False                            # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.25                                    # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 1.                                  # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.1                                 # 机身目标高度
        max_contact_force = 50.                                  # 触底冲击阈值 forces above this value are penalized
        clearance_height_target = -0.25                          # 足端目标高度

class HurricaneCrawlRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        num_steps_per_env = 100 # per iteration
        max_iterations = 10000 # number of policy updates
    
        save_interval = 50 # check for potential saves every this many iterations
        run_name = ''
        experiment_name = 'rough_hurricane_crawl'

