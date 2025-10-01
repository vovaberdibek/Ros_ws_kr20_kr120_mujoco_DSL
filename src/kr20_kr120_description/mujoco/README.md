# MuJoCo + ROS 2 World Template (robot cell)

This folder is a minimal, **non-intrusive** setup to run your existing MoveIt/RViz stack **unchanged**, while MuJoCo acts as the `ros2_control` hardware.

## Folder structure
- `world_mujoco.xml` — MuJoCo world that *includes your robot MJCF* and a simple table/tray environment.
- `robot_mjcf.xml` — placeholder; replace this with the MJCF converted from your URDF (or overwrite its contents).
- `controllers.yaml` — example ros2_control config (names/joints must match your MoveIt config).
- `convert_urdf_to_mjcf.py` — helper script to convert a resolved URDF to MJCF using MuJoCo's Python API.
- `start_mujoco.sh` — a shell script to launch the MuJoCo sim node (edit paths and controller names as needed).

> You **do not** edit your existing MoveIt/RViz files. They keep using **URDF/Xacro**. MuJoCo uses **MJCF** with matching joint names.

---

## Quickstart

1) **Copy this folder to your Desktop** (optional):
```bash
cp -r /mnt/data/mujoco_cell ~/Desktop/mujoco_cell
```

2) **Export a resolved URDF** (example):
```bash
# Adjust package & xacro path
ros2 run xacro xacro   ~/Desktop/ros2_ws/src/YOUR_DESCRIPTION_PKG/urdf/YOUR_ROBOT.xacro   > ~/Desktop/mujoco_cell/robot.urdf
```

3) **Convert URDF → MJCF**:
```bash
cd ~/Desktop/mujoco_cell
python3 convert_urdf_to_mjcf.py robot.urdf robot_mjcf.xml
```
- If you see missing mesh errors, open `robot.urdf` and replace `package://...` URIs with absolute paths to your meshes.

4) **Edit `world_mujoco.xml`** and set the `<include file="...">` to your `robot_mjcf.xml` **absolute path** (or keep it as-is if the file is here).

5) **Launch MuJoCo** (desktop mode; ensure your ROS 2 workspace with `mujoco_ros2_control` is sourced):
```bash
cd ~/Desktop/mujoco_cell
bash start_mujoco.sh
```
- For headless NVIDIA GPUs, change `MUJOCO_GL=egl` in the script.
- For CPU offscreen, use `MUJOCO_GL=osmesa` (no interactive viewer).

6) **In another terminal**, bring up your usual **MoveIt + RViz** (unchanged). Make sure your controllers names match those in `controllers.yaml`. Then Plan & Execute → robot moves in MuJoCo.

---

## Tips
- **Joint names must match** between URDF (used by RViz/MoveIt) and MJCF (used by MuJoCo).
- Keep your **controller names** identical to your existing MoveIt config.
- If inertials are missing in URDF, add them there or in MJCF for stable physics.
