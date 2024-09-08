import argparse

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

"""Rest everything follows."""


import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.kit import SimulationApp
from omni.isaac.core.utils.stage import open_stage, save_stage
from omni.isaac.core.utils.prims import create_prim
from pxr import UsdGeom, PhysxSchema
import os
import omni.isaac.core.utils.prims as prims_utils
import re

# Open a new or existing stage
open_stage("/home/hanlin/Learn_isaac_sim/newstage.usd")


# Define the base directories
base_dir = "/home/hanlin/ycb_test"
save_dir = "/home/hanlin/ycb_collision_test"

# Initialize lists to collect obj paths and destination paths
obj_paths = []
dest_paths = []

# Walk through the base directory to find subfolders with google_16k folder and textured.obj file
for subdir, dirs, files in os.walk(base_dir):
    print(f"Checking directory: {subdir}")
    if "google_16k" in dirs:
        obj_file_path = os.path.join(subdir, "google_16k", "textured.obj")
        print(f"Found google_16k directory in: {subdir}")
        
        if os.path.exists(obj_file_path):
            print(f"Found textured.obj: {obj_file_path}")
            # Append the obj file path to the list
            obj_paths.append(obj_file_path)

            # Create the corresponding destination path by modifying the subdir path
            relative_subdir = os.path.relpath(subdir, base_dir)
            dest_file_path = os.path.join(save_dir, f"{relative_subdir}.usd")
            dest_paths.append(dest_file_path)


            cleaned_subdir_name = relative_subdir.replace('/', '').replace('_', '').replace('-', '')
            cleaned_subdir_name = re.sub(r'\d+', '', cleaned_subdir_name)
            prim_path = f"/World/{cleaned_subdir_name}Model"
            obj_path = obj_file_path

            print("prim_path is, ", prim_path)
            print("obj_path is, ", obj_path)
            print("dest_file_path is, ", dest_file_path)

            # Create a mesh prim for the .obj file
            mesh_prim = create_prim(prim_path=prim_path, prim_type="Mesh", usd_path=obj_path)

            UsdGeom.Mesh(mesh_prim).GetPrim().GetReferences().ClearReferences()
            PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)

            # Add collision to the imported .obj model
            # collision_prim_path = f"{prim_path}/collider"
            # mesh_prim = create_prim(prim_path=collision_prim_path, prim_type="Mesh", usd_path=obj_path)
            # collision_api = PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)

            save_stage(dest_file_path)
            prims_utils.delete_prim(prim_path)


            # Ensure the destination directory exists
            dest_dir = os.path.dirname(dest_file_path)
            if not os.path.exists(dest_dir):
                os.makedirs(dest_dir)

# Print the found obj paths and their corresponding destination paths
print("OBJ Paths:", obj_paths)
print("Destination Paths:", dest_paths)


# # Define the prim path and the file path to the .obj model
# prim_path = "/World/MyObjModel"
# obj_path = "/home/hanlin/ycb_test/011_banana/google_16k/textured.obj"

# # Create a mesh prim for the .obj file
# create_prim(prim_path=prim_path, prim_type="Mesh", usd_path=obj_path)

# # Add collision to the imported .obj model
# collision_prim_path = f"{prim_path}/collider"
# mesh_prim = create_prim(prim_path=collision_prim_path, prim_type="Mesh", usd_path=obj_path)
# collision_api = PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)

# # Save the stage as a USD file
# save_stage("/home/hanlin/ycb_collision_test/011_banana.usd")
# prims_utils.delete_prim(prim_path)


# # Repeat for another object
# prim_path = "/World/MyObjModelPlum"
# obj_path = "/home/hanlin/ycb_test/065-d_cups/google_16k/textured.obj"

# mesh_prim = create_prim(prim_path=prim_path, prim_type="Mesh", usd_path=obj_path)
# PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)

# save_stage("/home/hanlin/ycb_collision_test/065-d_cups.usd")
