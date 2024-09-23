# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab_assets import Z1_CFG

from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.manager_based.manipulation.cabinet_lift_z1 import mdp

from omni.isaac.lab_tasks.manager_based.manipulation.cabinet_lift_z1.cabinet_env_cfg import (  # isort: skip
    FRAME_MARKER_SMALL_CFG,
    CabinetEnvCfg,
)

##
# Pre-defined configs
##
from omni.isaac.lab_assets.franka import FRANKA_PANDA_CFG  # isort: skip


@configclass
class Z1LiftCabinetEnvCfg(CabinetEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set z1 as robot
        self.scene.robot = Z1_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot", init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, 0.55))
        )

        # Set Actions for the specific robot type (z1)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["joint.*"],
            scale=1.0,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["finger_.*"],
            open_command_expr={"finger_.*": 0.04},
            close_command_expr={"finger_.*": 0.0},
        )

        # Set the body name for the end effector
        self.commands.object_pose.body_name = "gripper_link"

        # Listens to the required transforms
        # IMPORTANT: The order of the frames in the list is important. The first frame is the tool center point (TCP)
        # the other frames are the fingers
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/z1_description/link00",
            debug_vis=True,
            visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/EndEffectorFrameTransformer"),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/z1_description/gripper_link",
                    name="ee_tcp",
                    offset=OffsetCfg(
                        pos=(0.18, 0.0, 0),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/z1_description/finger_left_link",
                    name="tool_leftfinger",
                    offset=OffsetCfg(
                        pos=(0.18, 0.0, 0),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/z1_description/finger_right_link",
                    name="tool_rightfinger",
                    offset=OffsetCfg(
                        pos=(0.18, 0.0, 0),
                    ),
                ),
            ],
        )

        # override rewards
        # self.rewards.approach_gripper_handle.params["offset"] = 0.04
        # self.rewards.grasp_handle.params["open_joint_pos"] = 0.04
        # self.rewards.grasp_handle.params["asset_cfg"].joint_names = ["finger_.*"]


@configclass
class Z1LiftCabinetEnvCfg_PLAY(Z1LiftCabinetEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
