# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Franka Emika robots.

The following configurations are available:

* :obj:`FRANKA_PANDA_CFG`: Franka Emika Panda robot with Panda hand
* :obj:`FRANKA_PANDA_HIGH_PD_CFG`: Franka Emika Panda robot with Panda hand with stiffer PD control

Reference: https://github.com/frankaemika/franka_ros
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

Z1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/hanlin/IsaacLab/source/extensions/omni.isaac.lab_assets/omni/isaac/lab_assets/Robots/Unitree/Z1/z1_gripper.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.0,
            "joint2": 1.8,
            "joint3": -1.22,
            "joint4": 0.0,
            "joint5": 0.0,
            "joint6": 0.0,
            "finger_left_joint": 0.04,
            "finger_right_joint": -0.04,
        },
    ),
    actuators={
        "joint1": ImplicitActuatorCfg(
            joint_names_expr=["joint[1]"],
            effort_limit=30.0,
            velocity_limit=3.14,
            stiffness=251.0,
            damping=25.8,
        ),
        "joint2": ImplicitActuatorCfg(
            joint_names_expr=["joint[2]"],
            effort_limit=60.0,
            velocity_limit=3.14,
            stiffness=501.0,
            damping=50,
        ),
        "joint3": ImplicitActuatorCfg(
            joint_names_expr=["joint[3]"],
            effort_limit=30.0,
            velocity_limit=3.14,
            stiffness=251.0,
            damping=25.8,
        ),
        "joint4": ImplicitActuatorCfg(
            joint_names_expr=["joint[4]"],
            effort_limit=30.0,
            velocity_limit=3.14,
            stiffness=398.0,
            damping=39.8,
        ),
        "joint5": ImplicitActuatorCfg(
            joint_names_expr=["joint[5]"],
            effort_limit=30.0,
            velocity_limit=3.14,
            stiffness=631.0,
            damping=63.8,
        ),
        "joint6": ImplicitActuatorCfg(
            joint_names_expr=["joint[6]"],
            effort_limit=30.0,
            velocity_limit=3.14,
            stiffness=631.0,
            damping=63.8,
        ),
        "finger_left_joint": ImplicitActuatorCfg(  # range[0, 0.04]
            joint_names_expr=["finger_left_joint"],
            effort_limit=100,
            velocity_limit=100,
            stiffness=1e4,
            damping=1e2,
        ),
        "finger_right_joint": ImplicitActuatorCfg(  # range[-0.04, 0]
            joint_names_expr=["finger_right_joint"],
            effort_limit=100,
            velocity_limit=100,
            stiffness=1e4,
            damping=1e2,
        ),
    },
)
"""Configuration of Z1 robot."""
