# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import os
from omni.isaac.lab_assets import Z1_CFG

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg,AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

from omni.isaac.lab_tasks.manager_based.manipulation.lift import mdp
from omni.isaac.lab_tasks.manager_based.manipulation.z1_place_to_table.z1_place_env_cfg import Z1PlaceEnvCfg

##
# Pre-defined configs
##
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG  # isort: skip



@configclass
class Z1CubePlaceEnvCfg(Z1PlaceEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        self.scene.robot = Z1_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0, 0, 0.93),
                joint_pos={
                    "joint1": 0.0,
                    "joint2": 0.8,   #   0.8  1.8
                    "joint3": -0.7,  #   -0.7  -1.22
                    "joint4": 0.2,
                    "joint5": 0.0,
                    "joint6": 0.0,
                    "finger_.*": 0.04,
                },
            ),)

        # Set actions for the specific robot type (Z1)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["joint.*"], scale=0.5, use_default_offset=True
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["finger_.*"],
            open_command_expr={"finger_.*": 0.04},
            close_command_expr={"finger_.*": 0.0},
        )
        # Set the body name for the end effector
        # self.commands.object_pose.body_name = "gripper_link"  # gripper_link or finger_right_link
        # self.commands.disc_pose.body_name = "gripper_link"  # gripper_link or finger_right_link


        # Set 006_mustard_bottleas object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.2878, 0, 1.23], rot=[0.7071068, -0.7071068, 0, 0]
            ),  # rot=[0.7071068, -0.7071068, 0, 0]
            spawn=sim_utils.UsdFileCfg(
                usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/006_mustard_bottle.usd"),
                # usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/005_tomato_soup_can.usd"),
                # usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/004_sugar_box.usd"),
                # usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/003_cracker_box.usd"),
                # usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/021_bleach_cleanser.usd"),
                # usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/008_pudding_box.usd"),
                # usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/035_power_drill.usd"),
                # usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/010_potted_meat_can.usd"),
                
                # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(1, 1, 1),
                activate_contact_sensors=True,
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        self.scene.disc = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Disc",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.5, 0, 0.91], rot=[1, 0, 0, 0]
            ),  # rot=[0.7071068, -0.7071068, 0, 0]
            spawn=sim_utils.UsdFileCfg(
                usd_path=os.path.join(os.path.expanduser("~"), "Downloads/GA_dataset/Red_disc.usd"),
                scale=(2, 2, 2),
                activate_contact_sensors=True,
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )
        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/z1_description/link00",
            debug_vis=True,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/z1_description/gripper_link",
                    name="end_effector",
                    offset=OffsetCfg(pos=[0.2, 0.0, 0.0], rot=[0, 1, 0, 0]),
                ),
            ],
        )


@configclass
class Z1CubePlaceEnvCfg_PLAY(Z1CubePlaceEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
