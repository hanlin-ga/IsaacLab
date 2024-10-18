# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import RigidObject
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils.math import subtract_frame_transforms

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def object_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """The position of the object in the robot's root frame."""
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    object_pos_w = object.data.root_pos_w[:, :3]
    object_pos_b, _ = subtract_frame_transforms(
        robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], object_pos_w
    )
    return object_pos_b

def object_pose_obs(env: ManagerBasedRLEnv, 
                    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),) -> torch.Tensor:
    
    object: RigidObject = env.scene[object_cfg.name]

    object_quat_w = object.data.root_quat_w
    # default_quat_w = object.data.default_root_state[:, 3:7]
    # orientation_diff = quat_mul(cube_quat_w, quat_conjugate(default_quat_w))

    return object_quat_w


def disc_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    disc_cfg: SceneEntityCfg = SceneEntityCfg("disc"),
) -> torch.Tensor:
    """The position of the disc in the robot's root frame."""
    robot: RigidObject = env.scene[robot_cfg.name]
    disc: RigidObject = env.scene[disc_cfg.name]
    disc_pos_w = disc.data.root_pos_w[:, :3]
    disc_pos_b, _ = subtract_frame_transforms(
        robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], disc_pos_w
    )
    return disc_pos_b