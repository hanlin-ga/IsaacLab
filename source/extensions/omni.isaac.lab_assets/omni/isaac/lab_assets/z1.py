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
        usd_path="/home/hanlin/IsaacLab/source/extensions/omni.isaac.lab_assets/omni/isaac/lab_assets/Robots/Unitree/Z1/z1.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.4,
            "joint2": 1.4,
            "joint3": -1.22,
            "joint4": 0.0,
            "joint5": 0.0,
            "joint6": 0.0,
        },
    ),
    actuators={
        "joint1": ImplicitActuatorCfg(
            joint_names_expr=["joint[1]"],
            effort_limit=30.0,
            velocity_limit=3.1415,
            stiffness=80.0,
            damping=1.0,
        ),
        "joint2": ImplicitActuatorCfg(
            joint_names_expr=["joint[2]"],
            effort_limit=60.0,
            velocity_limit=3.1415,
            stiffness=80.0,
            damping=2.0,
        ),
        "joint3": ImplicitActuatorCfg(
            joint_names_expr=["joint[3]"],
            effort_limit=30.0,
            velocity_limit=3.1415,
            stiffness=80.0,
            damping=1.0,
        ),
        "joint4": ImplicitActuatorCfg(
            joint_names_expr=["joint[4]"],
            effort_limit=30.0,
            velocity_limit=3.1415,
            stiffness=80.0,
            damping=1.0,
        ),
        "joint5": ImplicitActuatorCfg(
            joint_names_expr=["joint[5]"],
            effort_limit=30.0,
            velocity_limit=3.1415,
            stiffness=80.0,
            damping=1.0,
        ),
        "joint6": ImplicitActuatorCfg(
            joint_names_expr=["joint[6]"],
            effort_limit=30.0,
            velocity_limit=3.1415,
            stiffness=80.0,
            damping=1.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Z1 robot."""

