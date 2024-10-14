# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import Articulation, AssetBase, RigidObject
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.sensors import FrameTransformer
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.utils.math import combine_frame_transforms
from .math import quat_error_magnitude_xy


if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def object_is_lifted(env: ManagerBasedRLEnv, 
                     minimal_height: float,

                     distance_threshold: float,
                     command_name: str,
                     
                     robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
                     object_cfg: SceneEntityCfg = SceneEntityCfg("object")) -> torch.Tensor:
    """Reward the agent for lifting the object above the minimal height."""
    object: RigidObject = env.scene[object_cfg.name]

    robot: RigidObject = env.scene[robot_cfg.name]
    command = env.command_manager.get_command(command_name)

    # compute the distance between object and disc_pose
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b)
    distance_xy = torch.norm(des_pos_w[:, :2] - object.data.root_pos_w[:, :2], dim=1)
    condition = (object.data.root_pos_w[:, 2] > minimal_height) | (distance_xy < distance_threshold)

    return torch.where(condition, 1.0, 0.0) 


def object_ee_distance(
    env: ManagerBasedRLEnv,
    std: float,

    distance_threshold: float,
    command_name: str,

    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward the agent for reaching the object using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    object: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    # Target object position: (num_envs, 3)
    cube_pos_w = object.data.root_pos_w
    # End-effector position: (num_envs, 3)
    ee_w = ee_frame.data.target_pos_w[..., 0, :]
    # Distance of the end-effector to the object: (num_envs,)
    object_ee_distance = torch.norm(cube_pos_w - ee_w, dim=1)

    robot: RigidObject = env.scene[robot_cfg.name]
    command = env.command_manager.get_command(command_name)

    # compute the distance between object and disc_pose
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b)
    distance_xy = torch.norm(des_pos_w[:, :2] - object.data.root_pos_w[:, :2], dim=1)
    condition = distance_xy > distance_threshold

    return 1 - torch.tanh(object_ee_distance / std)*condition


def object_goal_distance(
    env: ManagerBasedRLEnv,
    std: float,

    delta_z: float,
    distance_threshold: float,
    minimal_height: float,
    command_name: str,

    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    
) -> torch.Tensor:
    """Reward the agent for tracking the goal pose using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b)
    des_pos_w[:, 2] += delta_z
    # distance of the end-effector to the object: (num_envs,)
    distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)

    # calcualte the distance between object and disc_pose
    distance_xy = torch.norm(des_pos_w[:, :2] - object.data.root_pos_w[:, :2], dim=1)
    condition = (object.data.root_pos_w[:, 2] > minimal_height) | (distance_xy < distance_threshold)

    # print("distance_xy is ", distance_xy)

    # rewarded if the object is lifted above the threshold
    return condition * (1 - torch.tanh(distance / std))



def object_goal_distance_six_joint(
    env: ManagerBasedRLEnv,
    std: float,

    delta_z: float,
    distance_threshold: float,
    minimal_height: float,
    command_name: str,

    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Reward the agent for tracking the goal pose using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    asset: Articulation = env.scene[robot_cfg.name]

    command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b)
    des_pos_w[:, 2] += delta_z

    # distance of the end-effector to the object: (num_envs,)
    distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)

    # the angle difference between the current joint position and the default one
    angle = asset.data.joint_pos[:, robot_cfg.joint_ids] - asset.data.default_joint_pos[:, robot_cfg.joint_ids]

    # calcualte the distance between object and disc_pose
    distance_xy = torch.norm(des_pos_w[:, :2] - object.data.root_pos_w[:, :2], dim=1)
    condition = (object.data.root_pos_w[:, 2] > minimal_height) | (distance_xy < distance_threshold)

    # check if the object has arrived at the goal position. If yes, condition1 is 0
    condition1 = (distance_xy > distance_threshold)
    # print("*"*100)
    
    # print("asset.data.joint_pos[:, robot_cfg.joint_ids] is ", asset.data.joint_pos[:, robot_cfg.joint_ids])
    # print("default_joint_pos is ", asset.data.default_joint_pos[:, robot_cfg.joint_ids])
    # print("angle is ", angle)
    # return torch.sum(torch.abs(angle[:,0:6]), dim=1)
    # print("(object.data.root_pos_w[:, 2] > minimal_height) * ((1 - torch.tanh(distance / std)) - torch.sum(torch.abs(angle[:,0:6]), dim=1)*0.1) is ", (object.data.root_pos_w[:, 2] > minimal_height) * ((1 - torch.tanh(distance / std)) - torch.sum(torch.abs(angle[:,0:6]), dim=1)*0.1))
    # rewarded if the object is lifted above the threshold
    # print("torch.sum(torch.abs(angle[:,0:6]), dim=1)*0.1 is ", torch.sum(torch.abs(angle[:,0:6]), dim=1)*0.1)
    # print("torch.abs(angle[:,6])*0.5 is ", torch.abs(angle[:,5])*0.5)
    #return (object.data.root_pos_w[:, 2] > minimal_height) * ((1 - torch.tanh(distance / std)) - torch.sum(torch.abs(angle[:,0:6]), dim=1)*0.1 - torch.abs(angle[:,5])*1.0)
    return condition  * (1 - torch.tanh(distance / std) *condition1 - torch.sum(torch.abs(angle[:,0:6]), dim=1)*0.1 - torch.abs(angle[:,5])*1.0) 


def last_joint_vel(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint positions that deviate from the default one."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # compute out of limits constraints
    # print("asset_cfg.joint_ids is ", asset_cfg.joint_ids)   # asset_cfg.joint_ids is  slice(None, None, None)
    # print("asset.data.default_joint_pos shape is ", asset.data.default_joint_pos.shape) # asset.data.default_joint_pos shape is  torch.Size([1000, 8])
    # angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    # angle = asset.data.joint_pos[:, 5] - asset.data.default_joint_pos[:, 5]
    vel = asset.data.joint_vel[:, 5]
    # print("last angle 5 is ", asset.data.joint_pos[:, 5])
    # print("last angle 6 is ", asset.data.joint_pos[:, 6])
    # print("last angle 7 is ", asset.data.joint_pos[:, 7])
    # print("asset.data.default_joint_pos is ", asset.data.default_joint_pos[:, 5])
    # print("torch.sum(torch.abs(angle), dim=1) shape is ", torch.sum(torch.abs(angle), dim=1).shape)
    # print("torch.abs(angle) shape is ", torch.abs(angle).shape)
    return torch.abs(asset.data.joint_pos[:, 5] - asset.data.default_joint_pos[:, 5])

def undesired_contacts_id(env: ManagerBasedRLEnv, threshold: float, sensor_cfg: SceneEntityCfg, ID: String) -> torch.Tensor:
    """Penalize undesired contacts as the number of violations that are above a threshold."""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # check if contact force is above threshold
    net_contact_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold

    # print("ID is ", ID)
    # print("torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1) is ", torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1))

    # sum over contacts for each environment
    return torch.sum(is_contact, dim=1)


def joint_deviation_l1_six_joints(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint positions that deviate from the default one."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # compute out of limits constraints
    angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]

    # print("default_joint_pos is ", asset.data.default_joint_pos[:, asset_cfg.joint_ids] )
    return torch.sum(torch.abs(angle[:,0:6]), dim=1)


def object_goal_orientation_diff_rew(env: ManagerBasedRLEnv, 
                                 object_cfg: SceneEntityCfg = SceneEntityCfg("object"),) -> torch.Tensor:
    
    object: RigidObject = env.scene[object_cfg.name]

    cube_quat_w = object.data.root_quat_w
    default_quat_w = object.data.default_root_state[:, 3:7]
    # orientation_diff = quat_mul(cube_quat_w, quat_conjugate(default_quat_w))
    # example_quat_w = object.data.default_root_state[:, 3:7]
    # print("example angle diff is ", quat_error_magnitude_xy(cube_quat_w , default_quat_w))
    # print("orientation_diff is ", orientation_diff)
    return quat_error_magnitude_xy(cube_quat_w, default_quat_w)


def joint_deviation_l1_condition(env: ManagerBasedRLEnv,
                                 distance_threshold: float,
                                 command_name: str,

                                 object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
                                 robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
                                 asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint positions that deviate from the default one."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # compute out of limits constraints
    angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]

    return torch.sum(torch.abs(angle), dim=1)

def last_finger_rate(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize the rate of change of the actions using L2 squared kernel."""
    # print("env.action_manager.action[:,6] is ", env.action_manager.action[:,6])
    # print("env.action_manager.action[:,7] is ", env.action_manager.action[:,7])
    return torch.abs(env.action_manager.action[:, 6] + env.action_manager.prev_action[:, 7]) < 0.02
    # return torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1)
