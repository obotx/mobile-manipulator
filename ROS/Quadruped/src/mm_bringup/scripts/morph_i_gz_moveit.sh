#!/bin/bash

set -e

PIDS=()

cleanup() {
    echo "[CLEANUP] Stopping launched processes..."

    # kill tracked PIDs first (safer than pkill -f)
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[CLEANUP] Killing PID $pid"
            kill -9 "$pid" || true
        fi
    done

    # optional fallback (ONLY if you really need it)
    pkill -f "move_group|ros2|gz sim|gzserver" || true

    echo "[CLEANUP] Done."
}

trap cleanup EXIT INT TERM

echo "[INFO] Launching Gazebo simulation..."
ros2 launch mm_gazebo morph_i.gazebo.launch.py use_rviz:=false &
PIDS+=($!)
sleep 10

echo "[INFO] Launching MoveIt..."
ros2 launch mm_moveit_config move_group.launch.py &
PIDS+=($!)


wait