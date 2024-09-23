
# To Run Z1 manipulator pick-and-place object simulation
You will first need change my user name to be yours under the following three files.
1, Modify z1.py under `/home/hanlin/IsaacLab/source/extensions/omni.isaac.lab_assets/omni/isaac/lab_assets`
2, Modify z1_lift_env_cfg.py under `/home/hanlin/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/manipulation/z1_lift_from_drawer`
3, Modify joint_pos_env_cfg.py under `/home/hanlin/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/manipulation/z1_lift_from_drawer/config/z1`

Then run: `./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Lift-Cube-from-drawer-Z1-v0 --num_envs 16`

