import argparse
import time

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different single-arm manipulators.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import numpy as np
import torch

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf

from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.physics_context import PhysicsContext
from pxr import UsdGeom, Gf, Sdf, Usd
import omni.usd
from omni.usd import get_context
import omni.kit

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from pxr import Usd, UsdGeom, Sdf, PhysxSchema, UsdPhysics, UsdShade

##
# Pre-defined configs
##
# isort: off
from omni.isaac.lab_assets import (
    Z1_CFG
)

def define_origins(num_origins: int, spacing: float) -> list[list[float]]:
    """Defines the origins of the the scene."""
    # create tensor based on number of environments
    env_origins = torch.zeros(num_origins, 3)
    # create a grid of origins
    num_rows = np.floor(np.sqrt(num_origins))
    num_cols = np.ceil(num_origins / num_rows)
    xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols), indexing="xy")
    env_origins[:, 0] = spacing * xx.flatten()[:num_origins] - spacing * (num_rows - 1) / 2
    env_origins[:, 1] = spacing * yy.flatten()[:num_origins] - spacing * (num_cols - 1) / 2
    env_origins[:, 2] = 0.0
    # return the origins
    print('num_rows is :', num_rows )
    print('num_cols is :', num_cols )
    return env_origins.tolist()

def design_scene() -> tuple[dict, list[list[float]]]:

    """Designs the scene."""
    # Ground-plane
    # cfg = sim_utils.GroundPlaneCfg()
    # cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2", "Origin3"
    # Each group will have a mount and a robot on top of it
    origins = define_origins(num_origins=7, spacing=2.0)
    print("origins is: ", origins)

    # Origin 1 with z1 arm
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    # -- Robot
    z1_cfg = Z1_CFG.replace(prim_path="/World/Origin1/Robot")
    z1_cfg.init_state.pos = (1.08, 4.0, 0.76)
    z1 = Articulation(cfg=z1_cfg)

    # Origin 2 with z1 arm
    prim_utils.create_prim("/World/Origin2", "Xform", translation=origins[0])
    # -- Robot
    z1_1_cfg = Z1_CFG.replace(prim_path="/World/Origin2/Robot")
    z1_1_cfg.init_state.pos = (2.28, 4.0, 0.76)
    z1_1 = Articulation(cfg=z1_1_cfg)

    print("origins[0] is ", origins[0])

    # return the scene information
    scene_entities = {
        "z1": z1,
        "z1_1": z1_1
    }
    return scene_entities, origins

def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    direction = 1
    remove_flag = 0
    count1 = 0

    # #  read the current cracker box position
    # # Access the USD stage
    # stage = omni.usd.get_context().get_stage()
    # # Specify the prim path
    # prim_path = "/World/MainStage/_03_cracker_box"
    # # Get the prim at the specified path
    # _03_cracker_box_prim = stage.GetPrimAtPath(prim_path)
    # # Ensure the prim is an Xformable (has a transform)
    # if _03_cracker_box_prim and _03_cracker_box_prim.IsA(UsdGeom.Xformable):
    #     xformable = UsdGeom.Xformable(_03_cracker_box_prim)
    #     # Get the transformation matrix at the default time (usually time 0)
    #     _03_cracker_box_transform_matrix = xformable.GetLocalTransformation()
    #     # Extract the translation component from the matrix
    #     _03_cracker_box_translation = _03_cracker_box_transform_matrix.ExtractTranslation()
    #     # _03_cracker_box_rotation = _03_cracker_box_transform_matrix.ExtractRotation().Decompose()
    #     # _03_cracker_box_scale = _03_cracker_box_transform_matrix.ExtractScale()
    #     print("_03_cracker_box Translation:", _03_cracker_box_translation)
    # else:
    #     print(f"Prim at path {prim_path} is not Xformable or does not exist.")

    # reset the scene entities
    for index, robot in enumerate(entities.values()):
        # root state
        root_state = robot.data.default_root_state.clone()
        # print("Before origin, Robot root_state[:, :3] is ", root_state[:, :3])
        # print("origins[index] is ", origins[index])
        # root_state[:, :3] += origins[index]
        # print("After origin, Robot root_state[:, :3] is ", root_state[:, :3])

        if index==0:
            root_state[:, :3] = torch.tensor([[0.08, 1.0, 0.76]], device='cuda:0')
        else: root_state[:, :3] = torch.tensor([[1.28, 1.0, 0.76]], device='cuda:0')
        
        robot.write_root_state_to_sim(root_state)
        # set joint positions
        joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
        print("Robot joint_pos is ", joint_pos)
        robot.write_joint_state_to_sim(joint_pos, joint_vel)
        # clear internal buffers
        robot.reset()
    print("[INFO]: Resetting robots state...")

    # Simulate physics
    while simulation_app.is_running():

        # reset
        if count % 200 == 0:
            # reset counters
            count = 1
            direction = direction * (-1)
            print("reset direction!!!!!!!!!!")

        # if count1==1000:
        #     sim.reset()
        #     print("Table 2 is reset")
        #     sim.play()


        # apply random actions to the robots
        for robot in entities.values():
            # generate random joint positions
            joint_pos_target = robot.data.default_joint_pos
            joint_pos_target[:, 0] = joint_pos_target[:, 0] + direction * 0.01
            joint_pos_target = joint_pos_target.clamp_(
                robot.data.soft_joint_pos_limits[..., 0], robot.data.soft_joint_pos_limits[..., 1]
            )
            # apply action to the robot
            robot.set_joint_position_target(joint_pos_target)
            # write data to sim
            robot.write_data_to_sim()
        # perform step
        sim.step()
        
        # update sim-time
        sim_time += sim_dt
        count += 1
        count1+= 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg()
    sim = sim_utils.SimulationContext(sim_cfg)
    

    # Set main camera
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)


    # Access the USD stage
    stage = omni.usd.get_context().get_stage()

    usd_file_path = "/home/hanlin/Learn_isaac_sim/learn_table_ycb1.usd"
    prim_path = "/World/MainStage"
    # Create a new prim in the stage to hold the reference
    reference_prim = stage.DefinePrim(prim_path, "Xform")
    # Add a reference to the additional USD file
    reference_prim.GetReferences().AddReference(usd_file_path)


    usd_file_path = "/home/hanlin/Learn_isaac_sim/learn_only_table_ycb.usd"
    prim_path = "/World/SecondStage"
    # Create a new prim in the stage to hold the reference
    reference_prim = stage.DefinePrim(prim_path, "Xform")
    # Add a reference to the additional USD file
    reference_prim.GetReferences().AddReference(usd_file_path)
    # Set the translation using UsdGeom.XformCommonAPI
    translation = Gf.Vec3d(1.2, 0.0, 0.485)  # Example translation vector
    xform_api = UsdGeom.XformCommonAPI(reference_prim)
    xform_api.SetTranslate(translation)


    usd_file_path = "/home/hanlin/Learn_isaac_sim/learn_only_table_ycb.usd"
    prim_path = "/World/ThirdStage"
    # Create a new prim in the stage to hold the reference
    reference_prim = stage.DefinePrim(prim_path, "Xform")
    # Add a reference to the additional USD file
    reference_prim.GetReferences().AddReference(usd_file_path)
    # Set the translation using UsdGeom.XformCommonAPI
    translation = Gf.Vec3d(2.4, 0.0, 0.485)  # Example translation vector
    xform_api = UsdGeom.XformCommonAPI(reference_prim)
    xform_api.SetTranslate(translation)

    # Add another USD file as a reference or substage
    additional_usd_file_path = "/home/hanlin/Downloads/isaac-sim-assets-2-4.1.0/Assets/Isaac/4.0/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"
    reference_prim_path = "/World/AdditionalStage"
    # Create a new prim in the stage to hold the reference
    reference_prim = stage.DefinePrim(reference_prim_path, "Xform")
    # Add a reference to the additional USD file
    reference_prim.GetReferences().AddReference(additional_usd_file_path)
    UsdGeom.XformCommonAPI(reference_prim).SetTranslate((0, 0, 1))
    # Define the prim as a rigid body
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(reference_prim)
    # Add a collision shape (usually automatically derived from the geometry in the USD file)
    collision_api = UsdPhysics.CollisionAPI.Apply(reference_prim)

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

    # # Start the simulation loop
    # while simulation_app.is_running():
    #     simulation_app.update()

    # # Clean up and close the app
    # simulation_app.stop()