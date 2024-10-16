# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import gymnasium as gym
import os

from . import agents, joint_pos_env_cfg  

##
# Register Gym environments.
##

##
# Joint Position Control
##

gym.register(
    id="Isaac-Place-Object-Z1-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.Z1CubePlaceEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PlaceCubePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Place-Object-Z1-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.Z1CubePlaceEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PlaceCubePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
    },
    disable_env_checker=True,
)

# ##
# # Inverse Kinematics - Absolute Pose Control
# ##

# gym.register(
#     id="Isaac-Lift-Object-Z1-IK-Abs-v0",
#     entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
#     kwargs={
#         "env_cfg_entry_point": ik_abs_env_cfg.Z1CubeLiftEnvCfg,
#     },
#     disable_env_checker=True,
# )


# ##
# # Inverse Kinematics - Relative Pose Control
# ##

# gym.register(
#     id="Isaac-Lift-Object-Z1-IK-Rel-v0",
#     entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
#     kwargs={
#         "env_cfg_entry_point": ik_rel_env_cfg.Z1CubeLiftEnvCfg,
#         "robomimic_bc_cfg_entry_point": os.path.join(agents.__path__[0], "robomimic/bc.json"),
#     },
#     disable_env_checker=True,
# )