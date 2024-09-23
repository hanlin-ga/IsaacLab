# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import omni.kit.asset_converter as asset_converter
import omni.kit.commands

# Specify the paths for your OBJ file and the destination USD file
obj_file_path = "~/ycb_test/002_master_chef_can/google_16k/textured.obj"  # Path to your OBJ file
usd_file_path = "~/ycb_test/002_master_chef_can/google_16k/textured.obj"  # Path where you want to save the USD file

# Import the OBJ file into the stage
omni.kit.commands.execute(
    "CreateReferenceCommand",
    path_to="/World/MyModel",  # Where to place the model in the scene
    asset_path=obj_file_path,
)

# Save the stage as a USD file
stage = omni.usd.get_context().get_stage()
stage.GetRootLayer().Export(usd_file_path)

print(f"OBJ file successfully converted to USD and saved at {usd_file_path}")
