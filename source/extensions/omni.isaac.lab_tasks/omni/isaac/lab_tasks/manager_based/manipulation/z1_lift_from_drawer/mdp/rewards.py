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

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def object_is_lifted(
    env: ManagerBasedRLEnv, minimal_height: float, object_cfg: SceneEntityCfg = SceneEntityCfg("object")
) -> torch.Tensor:
    """Reward the agent for lifting the object above the minimal height."""
    object: RigidObject = env.scene[object_cfg.name]
    return torch.where(object.data.root_pos_w[:, 2] > minimal_height, 1.0, 0.0)


def object_ee_distance(
    env: ManagerBasedRLEnv,
    std: float,
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

    return 1 - torch.tanh(object_ee_distance / std)


def object_goal_distance(
    env: ManagerBasedRLEnv,
    std: float,
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
    # distance of the end-effector to the object: (num_envs,)
    distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)

    # rewarded if the object is lifted above the threshold
    return (object.data.root_pos_w[:, 2] > minimal_height) * (1 - torch.tanh(distance / std))


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


def last_finger_rate(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize the rate of change of the actions using L2 squared kernel."""
    # print("env.action_manager.action[:,6] is ", env.action_manager.action[:,6])
    # print("env.action_manager.action[:,7] is ", env.action_manager.action[:,7])
    return torch.abs(env.action_manager.action[:, 6] + env.action_manager.prev_action[:, 7]) < 0.02
    # return torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1)



def undesired_contacts_id(env: ManagerBasedRLEnv, threshold: float, sensor_cfg: SceneEntityCfg, ID: String) -> torch.Tensor:
    """Penalize undesired contacts as the number of violations that are above a threshold."""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # check if contact force is above threshold
    net_contact_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold

    print("*"*50)
    print("ID is ", ID)
    # print("net_contact_forces shape is ", net_contact_forces.shape)
    # print("net_contact_forces[:, :, sensor_cfg.body_ids] is ", net_contact_forces[:, :, sensor_cfg.body_ids])  # torch.Size([1, 3, 1, 3])   [num_envs, cfg.history_length, num_bodies, 3]  tensor([[[[-11.5659, -18.3871, -46.4664]],[[-11.7570, -18.1430, -46.7345]],[[-11.9166, -17.6637, -46.6894]]]]
    print("torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1) is ", torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1))   # torch.Size([1, 3, 1])  tensor([[[51.2931],[51.4928],[51.3216]]]
    # print("torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1) shape is ", torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0].shape)
    # print("sensor_cfg.body_ids is ", sensor_cfg.body_ids) 
    print("all is_contact are ", torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0]) 
    # print("is_contact shape is ", is_contact.shape)   
    # print("is_contact is ", is_contact)  
    print("final reward is ", torch.sum(is_contact, dim=1))  


    # ID is  robot    finger_right_link
    # net_contact_forces shape is  torch.Size([1, 3, 11, 3])
    # net_contact_forces[:, :, sensor_cfg.body_ids] shape is  torch.Size([1, 3, 1, 3])
    # torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1) is  torch.Size([1, 3, 1])
    # torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1) shape is  torch.Size([1, 1])
    # sensor_cfg.body_ids is  [9]
    # all is_contact are  tensor([[21.6755]], device='cuda:0')
    # is_contact shape is  torch.Size([1, 1])
    # is_contact is  tensor([[True]], device='cuda:0')
    # final reward is  tensor([1], device='cuda:0')


    # ID is  cabinet   sektion
    # net_contact_forces shape is  torch.Size([1, 3, 9, 3])
    # net_contact_forces[:, :, sensor_cfg.body_ids] shape is  torch.Size([1, 3, 1, 3])
    # torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1) is  torch.Size([1, 3, 1])
    # torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] shape is  torch.Size([1, 1])
    # sensor_cfg.body_ids is  [0]
    # all is_contact are  tensor([[0.]], device='cuda:0')
    # is_contact shape is  torch.Size([1, 1])
    # is_contact is  tensor([[False]], device='cuda:0')
    # final reward is  tensor([0], device='cuda:0')


    # sum over contacts for each environment
    return torch.sum(is_contact, dim=1)