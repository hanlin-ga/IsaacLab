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


import numpy as np
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.physics_context import PhysicsContext
from pxr import UsdGeom, Gf, Sdf, Usd
import omni.usd
from omni.usd import get_context
from pxr import PhysxSchema, UsdPhysics
import omni.kit
import omni.isaac.lab.sim as sim_utils

# PhysicsContext()
# GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.5, 0.5, 0.5]))
# DynamicCuboid(prim_path="/World/cube",
#     position=np.array([-.5, -.2, 1.0]),
#     scale=np.array([.5, .5, .5]),
#     color=np.array([.2,.3,0.]))


# Load the stage (USD file)
usd_file_path = "/home/hanlin/Learn_isaac_sim/learn_table_ycb1.usd"
usd_context = get_context()
usd_context.open_stage(usd_file_path)



# Access the USD stage
stage = omni.usd.get_context().get_stage()

# Add another USD file as a reference or substage
# additional_usd_file_path = "/home/hanlin/Downloads/isaac-sim-assets-2-4.1.0/Assets/Isaac/4.0/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"
additional_usd_file_path = "/home/hanlin/ycb_result/003_cracker_box/003_cracker_box.usd"
reference_prim_path = "/World/AdditionalStage"

# Create a new prim in the stage to hold the reference
reference_prim = stage.DefinePrim(reference_prim_path, "Xform")

# Add a reference to the additional USD file
reference_prim.GetReferences().AddReference(additional_usd_file_path)

# Set the transformation (translation and rotation) of the reference prim
xform_api = UsdGeom.XformCommonAPI(reference_prim)

# Set translation (e.g., moving the referenced stage to a different position)
translation = Gf.Vec3d(1.0, 0.0, 0.0)  # Example: Move 10 units along the X-axis (use Vec3d for double precision)

# Set rotation (e.g., rotating the referenced stage)
rotation = Gf.Vec3f(0.0, 0.0, 0.0)  # Example: Rotate 45 degrees around the Y-axis

# Apply the transformation
# Apply the transformation at the default time (typically time 0)
xform_api.SetTranslate(translation, Usd.TimeCode.Default())
xform_api.SetRotate(rotation, UsdGeom.XformCommonAPI.RotationOrderXYZ, Usd.TimeCode.Default())



# Start the simulation loop
while simulation_app.is_running():
    simulation_app.update()

# Clean up and close the app
simulation_app.stop()