
import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Utility to convert a mesh file into USD format.")
parser.add_argument("input", type=str, help="The path to the input mesh file.")
parser.add_argument("output", type=str, help="The path to store the USD file.")
parser.add_argument(
    "--make-instanceable",
    action="store_true",
    default=False,
    help="Make the asset instanceable for efficient cloning.",
)
parser.add_argument(
    "--collision-approximation",
    type=str,
    default="convexDecomposition",
    choices=["convexDecomposition", "convexHull", "none"],
    help=(
        'The method used for approximating collision mesh. Set to "none" '
        "to not add a collision mesh to the converted mesh."
    ),
)
parser.add_argument(
    "--mass",
    type=float,
    default=None,
    help="The mass (in kg) to assign to the converted asset. If not provided, then no mass is added.",
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import contextlib
import os

import carb
import omni.isaac.core.utils.stage as stage_utils
import omni.kit.app

from omni.isaac.lab.sim.converters import MeshConverter, MeshConverterCfg
from omni.isaac.lab.sim.schemas import schemas_cfg
from omni.isaac.lab.utils.assets import check_file_path
from omni.isaac.lab.utils.dict import print_dict

def main():
    # check valid file path
    mesh_path = args_cli.input
    if not os.path.isabs(mesh_path):
        mesh_path = os.path.abspath(mesh_path)
    if not check_file_path(mesh_path):
        raise ValueError(f"Invalid mesh file path: {mesh_path}")

    # create destination path
    dest_path = args_cli.output
    if not os.path.isabs(dest_path):
        dest_path = os.path.abspath(dest_path)

    print(dest_path)
    print(os.path.dirname(dest_path))
    print(os.path.basename(dest_path))

    # Mass properties
    if args_cli.mass is not None:
        mass_props = schemas_cfg.MassPropertiesCfg(mass=args_cli.mass)
        rigid_props = schemas_cfg.RigidBodyPropertiesCfg()
    else:
        mass_props = None
        rigid_props = None

    # Collision properties
    collision_props = schemas_cfg.CollisionPropertiesCfg(collision_enabled=args_cli.collision_approximation != "none")


    # Set the path to the ycb_test directory
    source_base_directory = "/home/hanlin/ycb_test"
    destination_base_directory = "/home/hanlin/ycb_result"

    # Loop through all subdirectories in the base directory
    for root, dirs, files in os.walk(source_base_directory):
        # Check if the current directory contains a folder named 'google_16k'
        if 'google_16k' in dirs:
            # Build the source path to the textured.obj file
            textured_obj_path = os.path.join(root, 'google_16k', 'textured.obj')
            
            # Check if textured.obj exists in the google_16k folder
            if os.path.isfile(textured_obj_path):
                # Print the source path to textured.obj
                print(f"Source path: {textured_obj_path}")
                
                # Extract the parent folder name (e.g., '011_banana')
                parent_folder_name = os.path.basename(root)

                # Set the destination folder and the USD file path
                dest_folder = os.path.join(destination_base_directory, parent_folder_name)
                dest_path = os.path.join(dest_folder, f"{parent_folder_name}.usd")
                
                # Create the destination folder if it does not exist
                if not os.path.exists(dest_folder):
                    os.makedirs(dest_folder)
                    print(f"Created folder: {dest_folder}")
                
                # Print the destination path
                print(f"Destination path: {dest_path}")


                # Create Mesh converter config
                mesh_converter_cfg = MeshConverterCfg(
                    mass_props=mass_props,
                    rigid_props=rigid_props,
                    collision_props=collision_props,
                    asset_path=textured_obj_path,
                    force_usd_conversion=True,
                    usd_dir=os.path.dirname(dest_path),
                    usd_file_name=os.path.basename(dest_path),
                    make_instanceable=args_cli.make_instanceable,
                    collision_approximation=args_cli.collision_approximation,
                )

                # Print info
                print("-" * 80)
                print("-" * 80)
                print(f"Input Mesh file: {mesh_path}")
                print("Mesh importer config:")
                print_dict(mesh_converter_cfg.to_dict(), nesting=0)
                print("-" * 80)
                print("-" * 80)

                # Create Mesh converter and import the file
                mesh_converter = MeshConverter(mesh_converter_cfg)
                # print output
                print("Mesh importer output:")
                print(f"Generated USD file: {mesh_converter.usd_path}")
                print("-" * 80)
                print("-" * 80)

                # Determine if there is a GUI to update:
                # acquire settings interface
                carb_settings_iface = carb.settings.get_settings()
                # read flag for whether a local GUI is enabled
                local_gui = carb_settings_iface.get("/app/window/enabled")
                # read flag for whether livestreaming GUI is enabled
                livestream_gui = carb_settings_iface.get("/app/livestream/enabled")


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()


