# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
from dataclasses import MISSING

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
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
from omni.isaac.lab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

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
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
    #     spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    # )

    cabinet = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Cabinet",
        spawn=sim_utils.UsdFileCfg(
            # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd",
            usd_path=os.path.join(
                os.path.expanduser("~"), "Downloads/Sektion_Cabinet/sektion_cabinet_instanceable.usd"
            ),
            activate_contact_sensors=True,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.82, 0, 0.4),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos={
                "door_left_joint": 0.0,
                "door_right_joint": 0.0,
                "drawer_bottom_joint": 0.0,
                "drawer_top_joint": 0.4,
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

    robot_contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/z1_description/.*", history_length=3, track_air_time=True)
    cabinet_contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Cabinet/.*", history_length=3, track_air_time=True)

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
    )

    # camera
    camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/z1_description/wrist_cam_link/camera",
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
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.3, 0.4), pos_y=(-0.25, 0.25), pos_z=(0.4, 0.5), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
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
            "pose_range": {"x": (-0.03, 0.03), "y": (-0.15, 0.15), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object", body_names="Object"),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0)
    lifting_object = RewTerm(func=mdp.object_is_lifted, params={"minimal_height": 0.8}, weight=15.0)

    object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.3, "minimal_height": 0.8, "command_name": "object_pose"},
        weight=16.0,
    )

    object_goal_tracking_fine_grained = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.05, "minimal_height": 0.8, "command_name": "object_pose"},
        weight=5.0,
    )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    # calculate the undersired contacts penalty
    cabinet_sektion_undesired_contacts = RewTerm(
        func=mdp.undesired_contacts_id,
        weight=-1.0,
        # params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*link01"), "threshold": 1.0},
        params={"sensor_cfg": SceneEntityCfg("cabinet_contact_forces", body_names="sektion"), "threshold": 1.0, "ID": "cabinet_sektion"},
    )

    cabinet_drawer_top_undesired_contacts = RewTerm(
        func=mdp.undesired_contacts_id,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("cabinet_contact_forces", body_names="drawer_top"), "threshold": 50.0, "ID": "cabinet_drawer_top"},
    )
    # right_finger_undesired_contacts = RewTerm(
    #     func=mdp.undesired_contacts_id,
    #     weight=-1.0,
    #     params={"sensor_cfg": SceneEntityCfg("robot_contact_forces", body_names="finger_right_link"), "threshold": 1.0, "ID": "right_finger"},
    # )

    # left_finger_undesired_contacts = RewTerm(
    #     func=mdp.undesired_contacts_id,
    #     weight=-1.0,
    #     params={"sensor_cfg": SceneEntityCfg("robot_contact_forces", body_names="finger_left_link"), "threshold": 1.0, "ID": "left_finger"},
    # )

    # when mustard bottle is in the gripper, right finger is 51, left finger is 49. The sektion and top drawer are 0
    # When the bottle is in the drawer, cabinet drawer top is 3.23. The other three are all zero
    # At the point of grasping, the cabinet drawer top is 16.88, the left finger is 19.3, the right finger is 0. 

    # When the mustard bottle is in the gripper: 
    # left finger: [12.0281,  4.6416, 46.5609]     [48.3138]
    # right finger: [-19.5480,  -3.3597, -47.4498]   [51.4286]

    # when the gripper is going to grasp the bottle@
    # The history data of the drawer top is: 
    # tensor([[[[ 0.0000e+00,  0.0000e+00,  0.0000e+00]],[[-7.8370e-05, -1.5814e-07, -7.9145e+00]],[[ 0.0000e+00,  0.0000e+00,  0.0000e+00]]]
    # The norm of this history data is tensor([[[0.0000],[7.9145],[0.0000]]]. The maximum is 7.91

    # when the two fingers are closed without object inside
    # left finger: 27.68
    # right finger: 27.68

    # when the robot arm is on the cabinet drawer top:
    # The cabinet drawer top: more than 100, sometimes more than 300
    # Left finger and right finger difference is quite big: for example, right finger force: 120. Left finger: 0

    # if the object is on the top drawer
    # top drawer:  3.28



    # final_joint_vel = RewTerm(func=mdp.last_joint_vel, weight=-1e-4)
    # last_two_finger = RewTerm(func=mdp.last_finger_rate, weight=1)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": 0.55, "asset_cfg": SceneEntityCfg("object")}
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000}
    )

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

        self.sim.disable_contact_processing = True
        if self.scene.cabinet_contact_forces is not None:
            self.scene.cabinet_contact_forces.update_period = self.sim.dt
            self.scene.robot_contact_forces.update_period = self.sim.dt

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
