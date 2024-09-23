# To Run Z1 Manipulator Pick-and-Place Object Simulation

1. Download the `YCB` and `Sektion_Cabinet` folders from shared google drive link and place them under your `Downloads` folder.

2. Download and leave the `logs` folder under `IsaacLab` folder.
 
3. Ensure the code dynamically accesses the correct user directory by using environment variables or Python's dynamic directory handling (e.g., `os.path.expanduser("~")`).

4. Then run the following command to start the simulation:

   ```bash
   ./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Lift-Cube-from-drawer-Z1-v0 --num_envs 16
