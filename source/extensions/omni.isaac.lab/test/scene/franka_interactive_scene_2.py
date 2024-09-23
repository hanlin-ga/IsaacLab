# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to use the scene interface to quickly setup a scene with multiple
articulated robots and sensors.
"""

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates how to use the scene interface.")
parser.add_argument("--headless", action="store_true", help="Force display off at all times.")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
parser.add_argument("--enable_cameras", action="store_true", help="Enable camera sensors and rendering.")
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(headless=args_cli.headless, enable_cameras=args_cli.enable_cameras)
simulation_app = app_launcher.app

"""Rest everything follows."""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.sensors.ray_caster import RayCasterCfg, patterns
import omni.isaac.lab.envs.mdp as mdp
from omni.isaac.lab.sensors import CameraCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.timer import Timer
import time
from omni.isaac.lab.assets import DeformableObject, DeformableObjectCfg

##
# Pre-defined configs
##
from omni.isaac.lab_assets.anymal import ANYMAL_C_CFG  # isort: skip
from omni.isaac.lab_assets.franka import FRANKA_PANDA_CFG
from omni.isaac.lab_assets import Z1_CFG
from omni.isaac.lab.sim import PreviewSurfaceCfg



@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Example scene configuration."""

    # terrain - flat terrain plane
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        visual_material=PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0))
    )

    # add cube
    # cube: RigidObjectCfg = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/cube",
    #     spawn=sim_utils.CuboidCfg(
    #         size=(0.2, 0.2, 0.2),
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.0),
    #         mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
    #         physics_material=sim_utils.RigidBodyMaterialCfg(),
    #         visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.0, 0.0)),
    #         collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.0, 0.9)),
    # )

    # articulation - robot 1
    robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    robot.init_state.pos = (0.0, 0.0, 0.76)

    # extras - light
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=5000.0, color=(0.75, 0.75, 0.75)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 100.0), rot=(0.6532815 , 0.2705981, 0.2705981, 0.6532815)),
    )

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
    # extras - YCB
    # YCB = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/YCB",
    #     # spawn=sim_utils.UsdFileCfg(usd_path=f"/home/hanlin/Downloads/isaac-sim-assets-1-4.1.0/Assets/Isaac/4.0/Isaac/Environments/Office/Props/SM_TableD.usd"),
    #     spawn=sim_utils.UsdFileCfg(usd_path=f"/home/hanlin/Learn_isaac_sim/learn_table_ycb2.usd"),
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.90))
    # )

    # # extras - table
    # table = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Table",
    #     spawn=sim_utils.UsdFileCfg(usd_path=f"/home/hanlin/Downloads/isaac-sim-assets-1-4.1.0/Assets/Isaac/4.0/Isaac/Environments/Office/Props/SM_TableB.usd"),
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0))
    # )


def main():
    """Main function."""

    # Load kit helper
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cuda:0")) # or device="cpu"
    # Set main camera
    sim.set_camera_view(eye=[5, 5, 5], target=[0.0, 0.0, 0.0])

    # Spawn things into stage
    with Timer("Setup scene"):
        scene = InteractiveScene(MySceneCfg(num_envs=args_cli.num_envs, env_spacing=4.0, lazy_sensor_update=False))

    # Check that parsing happened as expected
    assert len(scene.env_prim_paths) == args_cli.num_envs, "Number of environments does not match."
    assert scene.terrain is not None, "Terrain not found."
    # assert len(scene.articulations) == 3, "Number of robots does not match."
    # assert len(scene.sensors) == 1, "Number of sensors does not match."
    # assert len(scene.extras) == 1, "Number of extras does not match."

    # Play the simulator
    with Timer("Time taken to play the simulator"):
        sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # default joint targets
    robot_1_actions = scene.articulations["robot"].data.default_joint_pos.clone()
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    gripper_count = 0
    direction = 1


    # Simulate physics
    while simulation_app.is_running():
        # If simulation is stopped, then exit.
        if sim.is_stopped():
            break
        # If simulation is paused, then skip.
        if not sim.is_playing():
            sim.step()
            continue

        # reset
        sim_time = 0.0
        direction = direction * (-1)

        joint_pos_target = scene.articulations["robot"].data.default_joint_pos.clone()
        scene.reset()
        print(">>>>>>>> Reset!")
        gripper_count = 0
        # perform this loop at policy control freq (50 Hz)
        for _ in range(300):
            
            joint_pos_target[:, 0] = joint_pos_target[:, 0] + direction * 0.01
            joint_pos_target = joint_pos_target.clamp_(
                scene.articulations["robot"].data.soft_joint_pos_limits[..., 0], scene.articulations["robot"].data.soft_joint_pos_limits[..., 1]
            )
            
            if gripper_count > 150:
                joint_pos_target[:, 6] = 0.04
                joint_pos_target[:, 7] = -0.04
            else: 
                joint_pos_target[:, 6] = 0
                joint_pos_target[:, 7] = 0

            # set joint targets
            scene.articulations["robot"].set_joint_position_target(joint_pos_target)
            # write data to sim
            scene.write_data_to_sim()
            # perform step
            sim.step()
            # read data from sim
            scene.update(sim_dt)


            # print information from the sensors
            print("-------------------------------")
            # print(scene["camera"])
            # print("Received shape of rgb   image: ", scene["camera"].data.output["rgb"].shape)
            # print("Received shape of depth image: ", scene["camera"].data.output["distance_to_image_plane"].shape)


            # if count==1000, reset the scene
            count +=1
            gripper_count +=1
            if count==1000:
                root_state = scene.articulations["robot"].data.default_root_state.clone()
                root_state[:, :3] += scene.env_origins
                joint_pos = scene.articulations["robot"].data.default_joint_pos
                joint_vel = scene.articulations["robot"].data.default_joint_vel

                
                # -- set root state
                # -- robot 1
                # scene.articulations["robot"].write_root_state_to_sim(root_state)
                scene.articulations["robot"].write_joint_state_to_sim(joint_pos, joint_vel)
                scene.reset()


        # update sim-time
        sim_time += sim_dt * 4
        count += 1


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
