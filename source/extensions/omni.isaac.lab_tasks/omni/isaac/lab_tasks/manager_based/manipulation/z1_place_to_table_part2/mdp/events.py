# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to enable different events.

Events include anything related to altering the simulation state. This includes changing the physics
materials, applying external forces, and resetting the state of the asset.

The functions can be passed to the :class:`omni.isaac.lab.managers.EventTermCfg` object to enable
the event introduced by the function.
"""

from __future__ import annotations

import numpy as np
import torch
from typing import TYPE_CHECKING, Literal

import carb
import omni.physics.tensors.impl.api as physx

import omni.isaac.lab.sim as sim_utils
import omni.isaac.lab.utils.math as math_utils
from omni.isaac.lab.actuators import ImplicitActuator
from omni.isaac.lab.assets import Articulation, DeformableObject, RigidObject
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.terrains import TerrainImporter
import random

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedEnv

# reset_joints_by_offset

def reset_joints_angle(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    position_range: tuple[float, float],
    velocity_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    disc_cfg: SceneEntityCfg = SceneEntityCfg("disc"),
):
    """Reset the robot joints with offsets around the default position and velocity by the given ranges.

    This function samples random values from the given ranges and biases the default joint positions and velocities
    by these values. The biased values are then set into the physics simulation.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    object: RigidObject = env.scene[object_cfg.name]
    disc: RigidObject = env.scene[disc_cfg.name]

    # get default root state
    object_states = object.data.default_root_state[env_ids].clone()
    disc_states = disc.data.default_root_state[env_ids].clone()

    # get default joint state
    joint_pos = asset.data.default_joint_pos[env_ids].clone()
    joint_vel = asset.data.default_joint_vel[env_ids].clone()

    # Load the recorded data
    file_path = "recorded_data.pt"
    try:
        recorded_data = torch.load(file_path)
    except FileNotFoundError:
        print(f"The file {file_path} does not exist.")
        exit()
    N = len(env_ids)
    num_records = len(recorded_data["joint_angles"])
    if num_records < N:
        print(f"Not enough recorded joint angles to sample {N} sets.")
        exit()
    indices = random.sample(range(num_records), N)
    joint_pos = torch.stack([recorded_data["joint_angles"][i] for i in indices]).to(env_ids.device)

    object_position = torch.stack([recorded_data["object_position"][i] for i in indices]).to(env_ids.device) 
    object_angle = torch.stack([recorded_data["object_angle"][i] for i in indices]).to(env_ids.device)

    disc_position = torch.stack([recorded_data["disc_position"][i] for i in indices]).to(env_ids.device)
    disc_angle = torch.stack([recorded_data["disc_angle"][i] for i in indices]).to(env_ids.device)


    # bias these values randomly
    # joint_pos += math_utils.sample_uniform(*position_range, joint_pos.shape, joint_pos.device)
    joint_vel += math_utils.sample_uniform(*velocity_range, joint_vel.shape, joint_vel.device)
    joint_vel_limits = asset.data.soft_joint_vel_limits[env_ids]
    joint_vel = joint_vel.clamp_(-joint_vel_limits, joint_vel_limits)

    # set into the physics simulation
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
    object.write_root_pose_to_sim(torch.cat([object_position+asset.data.root_pos_w, object_angle], dim=-1), env_ids=env_ids)
    disc.write_root_pose_to_sim(torch.cat([disc_position+asset.data.root_pos_w, disc_angle], dim=-1), env_ids=env_ids)
    # asset.write_root_velocity_to_sim(velocities, env_ids=env_ids)