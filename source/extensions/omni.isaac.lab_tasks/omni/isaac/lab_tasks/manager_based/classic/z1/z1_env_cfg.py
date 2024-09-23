# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from omni.isaac.lab_assets import Z1_CFG

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import CameraCfg
from omni.isaac.lab.sim import PreviewSurfaceCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass

import omni.isaac.lab_tasks.manager_based.classic.z1.mdp as mdp

##
# Pre-defined configs
##
from omni.isaac.lab_assets.cartpole import CARTPOLE_CFG  # isort:skip


##
# Scene definition
##


@configclass
class Z1SceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # terrain - flat terrain plane
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        visual_material=PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
    )

    # z1 robot
    robot = Z1_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
    robot.init_state.pos = (0.0, 0.0, 0.76)

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )
    camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/robot/z1_description/wrist_cam_link/camera",
        update_period=0.0333,
        height=360,
        width=640,
        data_types=["rgb"],
        # data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.0, focus_distance=400.0, horizontal_aperture=2.0, clipping_range=(0.1, 10)
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
    )
    # extras - YCB
    YCB = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/YCB",
        # spawn=sim_utils.UsdFileCfg(usd_path=f"/home/hanlin/Downloads/isaac-sim-assets-1-4.1.0/Assets/Isaac/4.0/Isaac/Environments/Office/Props/SM_TableD.usd"),
        spawn=sim_utils.UsdFileCfg(usd_path=f"/home/hanlin/Learn_isaac_sim/learn_table_ycb2.usd"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.90)),
    )

    # extras - table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"/home/hanlin/Downloads/isaac-sim-assets-1-4.1.0/Assets/Isaac/4.0/Isaac/Environments/Office/Props/SM_TableB.usd"
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    # no commands for this MDP
    null = mdp.NullCommandCfg()


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    finger_left_joint_effort = mdp.JointEffortActionCfg(
        asset_name="robot", joint_names=["finger_left_joint"], scale=100.0
    )
    finger_right_joint_effort = mdp.JointEffortActionCfg(
        asset_name="robot", joint_names=["finger_right_joint"], scale=100.0
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # reset
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-1.2, 1.2),
            "velocity_range": (-0.1, 0.1),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)  # 1.0
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)  # -2.0
    # (3) distance reward
    distance_reward = RewTerm(
        func=mdp.check_distance_to_target,
        weight=1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["finger_left_joint"]), "target": 0.0},
    )

    reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0)

    # distance_reward = RewTerm(func=mdp.root_pos_w,
    #                           weight=1.0,
    #                           params={"asset_cfg": SceneEntityCfg("robot")},)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Cart out of bounds
    cart_out_of_bounds = DoneTerm(
        func=mdp.joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint1"]), "bounds": (-3.0, 3.0)},
    )


@configclass
class CurriculumCfg:
    """Configuration for the curriculum."""

    pass


##
# Environment configuration
##


@configclass
class Z1EnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""

    # Scene settings
    scene: Z1SceneCfg = Z1SceneCfg(num_envs=16, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    curriculum: CurriculumCfg = CurriculumCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    # No command generator
    commands: CommandsCfg = CommandsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
