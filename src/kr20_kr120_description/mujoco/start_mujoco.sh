#!/usr/bin/env bash
set -eo pipefail

# --- Config ---
WS="$HOME/Desktop/ros2_ws"
MODEL="${1:-$HOME/Desktop/mujoco_cell/world_mujoco.xml}"   # or robot_mjcf.xml if you prefer
GL_BACKEND="${GL_BACKEND:-glfw}"                     # glfw | egl | osmesa

# --- Env ---
export MUJOCO_GL="$GL_BACKEND"

# Source ROS (for ros2 control node, etc.) – tolerate unset COLCON_TRACE
export COLCON_TRACE=${COLCON_TRACE:-0}
source "$WS/install/setup.bash"

echo "Launching MuJoCo Python viewer with MODEL: $MODEL (GL=$MUJOCO_GL)"
exec "$HOME/venvs/mj/bin/python" -m mujoco.viewer "$MODEL"
