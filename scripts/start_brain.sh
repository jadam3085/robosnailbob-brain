#!/bin/bash
# Start the RoboSnailBob brain — kills any stale instances first.

echo "[brain] Stopping any running brain nodes..."
pkill -f llm_brain_node 2>/dev/null
pkill -f voice_io_node  2>/dev/null
sleep 0.5

# Confirm they're gone
if pgrep -f llm_brain_node > /dev/null || pgrep -f voice_io_node > /dev/null; then
    echo "[brain] Force-killing stubborn processes..."
    pkill -9 -f llm_brain_node 2>/dev/null
    pkill -9 -f voice_io_node  2>/dev/null
    sleep 0.5
fi

echo "[brain] Launching brain..."
exec ros2 launch robosnailbob_brain brain.launch.py
