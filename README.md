# To Run Z1 Manipulator Pick-and-Place Object Simulation

1. **Download the required assets:**
   - Download the `YCB` and `Sektion_Cabinet` folders from the shared Google Drive link and place them in your `Downloads` folder.
   - Download the `logs` folder and place it inside the `IsaacLab` directory.

2. **Ensure dynamic user directory access:**
   - Ensure that the code accesses the correct user directory dynamically by using environment variables or Python's dynamic directory handling (e.g., `os.path.expanduser("~")`).

3. **To pick up the cube:**
   - Uncomment **line 73** in `joint_pos_env_cfg.py` where the `usd_path` is set to:
     ```python
     usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd"
     ```
   - The `joint_pos_env_cfg.py` file is located in the following directory:
     ```bash
     ~/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/manipulation/z1_lift_from_drawer/config/z1
     ```
   - Then, run the following command to start the simulation:
     ```bash
     ./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Lift-Cube-from-drawer-Z1-v0 --num_envs 16 --load_run 2024-09-20_16-45-10_cube_1500
     ```

4. **To pick up the mustard bottle:**
   - Uncomment **line 72** in `joint_pos_env_cfg.py` where the `usd_path` is set to:
     ```python
     usd_path=os.path.join(os.path.expanduser("~"), "Downloads/YCB/Axis_Aligned/006_mustard_bottle.usd")
     ```
   - The `joint_pos_env_cfg.py` file is located in the following directory:
     ```bash
     ~/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/manipulation/z1_lift_from_drawer/config/z1
     ```
   - Then, run the following command to start the simulation:
     ```bash
     ./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Lift-Cube-from-drawer-Z1-v0 --num_envs 16 --load_run 2024-09-20_18-15-07_mustard
     ```

### Notes:
- Make sure that you uncomment the correct lines corresponding to the object you want the manipulator to pick up.
- The simulation will load with the respective object (cube or mustard bottle) based on the configuration.

