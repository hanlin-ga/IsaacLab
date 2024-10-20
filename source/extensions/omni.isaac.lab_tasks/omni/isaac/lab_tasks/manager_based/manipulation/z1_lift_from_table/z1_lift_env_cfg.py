# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import os
from dataclasses import MISSING

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import CurriculumTermCfg as CurrTerm
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import CameraCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.sensors import ContactSensorCfg
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg

from . import mdp

##
# Scene definition
##


@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):
    """Configuration for the lift scene with a robot and a object.
    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the target object, robot and end-effector frames
    """

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by agent env cfg
    ee_frame: FrameTransformerCfg = MISSING
    # target object: will be populated by agent env cfg
    object: RigidObjectCfg = MISSING

    # object: AssetBaseCfg = MISSING

    # Table
    # table = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Table",
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, -1.02], rot=[0.707, 0, 0, 0.707]),
    #     spawn=sim_utils.UsdFileCfg(
    #         # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd",
    #         usd_path=os.path.join(
    #             os.path.expanduser("~"), "Downloads/Shop_Table/Shop_Table.usd",
    #         ),
    #         scale=(0.01, 0.01, 0.012),
    #         activate_contact_sensors=True,
    #         ),
    # )

    cabinet = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Cabinet",
        spawn=sim_utils.UsdFileCfg(
            # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd",
            usd_path=os.path.join(
                os.path.expanduser("~"), "Downloads/Sektion_Cabinet/sektion_cabinet_instanceable_top_joint_0.usd"
            ),
            scale=(2, 2, 1),
            activate_contact_sensors=True,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.5, 0, 0.5),       # changed from 0.82 to 0.85
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos={
                "door_left_joint": 0.0,
                "door_right_joint": 0.0,
                "drawer_bottom_joint": 0.0,
                "drawer_top_joint": 0.0,   
            },
        ),
        actuators={
            "drawers": ImplicitActuatorCfg(
                joint_names_expr=[
                    "drawer_top_joint",
                    "drawer_bottom_joint",
                ],  # joint_names_expr=["drawer_top_joint", "drawer_bottom_joint"],
                effort_limit=87.0,
                velocity_limit=100.0,
                stiffness=10.0,
                damping=1.0,
            ),
            "doors": ImplicitActuatorCfg(
                joint_names_expr=["door_left_joint", "door_right_joint"],
                effort_limit=87.0,
                velocity_limit=100.0,
                stiffness=10.0,
                damping=2.5,
            ),
        },
    )

    cabinet_contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Cabinet/.*", history_length=3, track_air_time=True)
    # table_contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Table/.*", history_length=3, track_air_time=True)
    # robot_contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/z1_description/.*", history_length=3, track_air_time=True)

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -0.02]),
        spawn=GroundPlaneCfg(),
    )

    # camera
    # camera = CameraCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/z1_description/wrist_cam_link/camera",
    #     update_period=0.0333,
    #     height=360,
    #     width=640,
    #     data_types=["rgb"],
    #     # data_types=["rgb", "distance_to_image_plane"],
    #     spawn=sim_utils.PinholeCameraCfg(
    #         focal_length=1.0, focus_distance=400.0, horizontal_aperture=2.0, clipping_range=(0.1, 10)
    #     ),
    #     offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
    # )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    object_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,  # will be set by agent env cfg
        resampling_time_range=(5.0, 5.0),
        debug_vis=False,
        ranges=mdp.UniformPoseCommandCfg.Ranges(   #0.3707, -0.0079,  1.2895
            # pos_x=(0.2976, 0.2976), pos_y=(0, 0), pos_z=(0.35, 0.35), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
            pos_x=(0.2878, 0.2878), pos_y=(0, 0), pos_z=(0.27, 0.27), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # will be set by agent env cfg
    arm_action: mdp.JointPositionActionCfg = MISSING
    gripper_action: mdp.BinaryJointPositionActionCfg = MISSING


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.1, 0.1), "y": (-0.25, 0.25), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object", body_names="Object"),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0)
    lifting_object = RewTerm(func=mdp.object_is_lifted, params={"minimal_height": 1.007}, weight=15.0)

    object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance_six_joint,
        params={"std": 0.3, "minimal_height": 1.007, "command_name": "object_pose"},
        weight=16.0,
    )

    object_goal_tracking_fine_grained = RewTerm(
        func=mdp.object_goal_distance_six_joint,
        params={"std": 0.05, "minimal_height": 1.007, "command_name": "object_pose"},
        weight=5.0,
    )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    cabinet_sektion_undesired_contacts = RewTerm(
        func=mdp.undesired_contacts_id,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("cabinet_contact_forces", body_names="sektion"), "threshold": 30, "ID": "cabinet_sektion"},
    )

    # This reward is designed for bleach object out of the camera scene problem
    object_goal_orien_diff = RewTerm(func=mdp.end_effector_orientation_diff_rew, weight=-1, params={"default_quat": [0.0268,  0.9899,  0.0361, -0.1343]})


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("object")}
    )

    # added a new threshold for the object to be considered as arrived
    # object_arrive = DoneTerm(
    #     func=mdp.terminate_object_goal_distance, params={"distance_threshold": 0.005, "command_name": "object_pose"}
    # )

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000}
    )

    # joint_pos = CurrTerm(
    #     func=mdp.modify_reward_weight, params={"term_name": "joint_pos", "weight": -1e-1, "num_steps": 10000}
    # )


    # joint_vel1 = CurrTerm(
    #     func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1, "num_steps": 50000}
    # )

    # final_joint_vel = CurrTerm(
    #     func=mdp.modify_reward_weight, params={"term_name": "final_joint_vel", "weight": -1e-3, "num_steps": 10000}
    # )


##
# Environment configuration
##


@configclass
class Z1LiftEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the lifting environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=4096, env_spacing=2.5)  # 4096
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        # self.sim.disable_contact_processing = True
        # if self.scene.table_contact_forces is not None:
        #     self.scene.table_contact_forces.update_period = self.sim.dt
        #     self.scene.robot_contact_forces.update_period = self.sim.dt

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
