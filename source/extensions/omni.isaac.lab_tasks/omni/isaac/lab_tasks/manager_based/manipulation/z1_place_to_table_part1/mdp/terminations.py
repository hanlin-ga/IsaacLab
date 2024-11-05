# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`omni.isaac.lab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import Articulation, AssetBase, RigidObject
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils.math import combine_frame_transforms
from .math import quat_error_magnitude_xy
import os


if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def object_reached_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "object_pose",
    threshold: float = 0.02,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Termination condition for the object reaching the goal position.

    Args:
        env: The environment.
        command_name: The name of the command that is used to control the object.
        threshold: The threshold for the object to reach the goal position. Defaults to 0.02.
        robot_cfg: The robot configuration. Defaults to SceneEntityCfg("robot").
        object_cfg: The object configuration. Defaults to SceneEntityCfg("object").

    """
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
    return distance < threshold

def terminate_object_goal_distance_record_data(
    env: ManagerBasedRLEnv,
    distance_threshold: float,
    angle_threshold: float,
    minimal_height: float,
    record_data: str,

    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    disc_cfg: SceneEntityCfg = SceneEntityCfg("disc"),
) -> torch.Tensor:
    """Reward the agent for tracking the goal pose using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    # robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    disc: RigidObject = env.scene[disc_cfg.name]
    asset: Articulation = env.scene[robot_cfg.name]

    des_pos_w = disc.data.root_pos_w[:, :3].clone()
    des_pos_w[:, 2] = minimal_height
    distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)

    condition1 = distance < distance_threshold

    cube_quat_w = object.data.root_quat_w
    default_quat_w = object.data.default_root_state[:, 3:7]
    condition2 = quat_error_magnitude_xy(cube_quat_w, default_quat_w) < angle_threshold


    if record_data == "True":
        # Maximum number of data sets to record
        MAX_RECORDS = 10000

        # Initialize storage dictionary
        if os.path.exists("recorded_data.pt"):
            recorded_data = torch.load("recorded_data.pt")
        else:
            recorded_data = {
                "object_position": [],
                "object_angle": [],
                "disc_position": [],
                "disc_angle": [],
                "joint_angles": []
            }


        # Check current number of records
        current_records = len(recorded_data["object_position"])

        for i in range(condition1.size(0)):  # Loop over each environment
            if current_records >= MAX_RECORDS:
                print("Reached maximum record limit of 10,000. Stopping further recording.")
                break  # Stop recording if limit is reached
            if condition1[i].item() and condition2[i].item():  # Only save if both conditions are True for this environment
                # Append data for each quantity when conditions are met for this specific environment
                recorded_data["object_position"].append(object.data.root_pos_w[i, :].cpu() - asset.data.root_pos_w[i, :].cpu())
                recorded_data["object_angle"].append(object.data.root_quat_w[i, :].cpu())
                recorded_data["disc_position"].append(disc.data.root_pos_w[i, :].cpu() - asset.data.root_pos_w[i, :].cpu())
                recorded_data["disc_angle"].append(disc.data.root_quat_w[i, :].cpu())
                recorded_data["joint_angles"].append(asset.data.joint_pos[i, robot_cfg.joint_ids].cpu())
                current_records += 1

        # At the end of the experiment or after certain conditions, save data if there's any recorded
        if recorded_data["object_position"] and current_records < MAX_RECORDS:  # Check if there’s any data to save
            torch.save(recorded_data, "recorded_data.pt")

    # print("*"*100)
    # print("condition1 & condition2 is ", (condition1 & condition2).shape)
    # print("distance is ", distance)
    # print("angle diff is ", quat_error_magnitude_xy(cube_quat_w, default_quat_w))
    
    # print("object position is ", object.data.root_pos_w[:, :])
    # print("object angle is ", object.data.root_quat_w)
    # print("disc position is ", disc.data.root_pos_w[:, :])
    # print("disc angle is ", disc.data.root_quat_w)
    # print("joint angles are ", asset.data.joint_pos[:, robot_cfg.joint_ids])


    return condition1 & condition2