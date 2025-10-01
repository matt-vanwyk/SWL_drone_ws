#!/bin/bash

# Drone System Multi-Terminal Launch Script (tmux version)
# This script launches each ROS2 node in a separate tmux pane for easier debugging

export ROS_DOMAIN_ID=42
unset RMW_IMPLEMENTATION

# Kill existing session if it exists
tmux kill-session -t drone_system 2>/dev/null

# Create new tmux session with first node
tmux new-session -d -s drone_system -n "NTRIP" "cd ~/SWL_Drone_ws && source /opt/ros/humble/setup.bash && source ~/SWL_Drone_ws/install/setup.bash && echo '================================================' && echo 'NTRIP PUBLISHER NODE' && echo '================================================' && echo '' && ros2 run drone_pkg ntrip_publisher; bash"

# Split window and launch second node
tmux new-window -t drone_system "cd ~/SWL_Drone_ws && source /opt/ros/humble/setup.bash && source ~/SWL_Drone_ws/install/setup.bash && echo '================================================' && echo 'MAVSDK NODE' && echo '================================================' && echo '' && ros2 run drone_pkg mavsdk_node; bash"

tmux new-window -t drone_system "cd ~/SWL_Drone_ws && source /opt/ros/humble/setup.bash && source ~/SWL_Drone_ws/install/setup.bash && echo '================================================' && echo 'DRONE STATE MACHINE' && echo '================================================' && echo '' && ros2 run drone_pkg drone_state_machine; bash"

# Select first windows
tmux select-window -t drone_system:0

if [ -t 0 ]; then
    echo "Attaching to tmux session..."
    tmux attach -t drone_system
else
    echo "Running in background (systemd mode)"
    echo "Use 'tmux attach -t drone_system' to view logs"
fi
