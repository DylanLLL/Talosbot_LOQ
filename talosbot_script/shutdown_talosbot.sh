#!/bin/bash

# Terminate ros2 processes
echo "Stopping robot processes..."
pkill -f "ros2 launch talosbot_gazebo gazebo.launch.py" 2>/dev/null
pkill -f "ros2 run mqtt_to_ros_bridge mqtt_to_ros_bridge" 2>/dev/null

# Terminate firefox session
if pgrep firefox > /dev/null; then
    echo "Closing Firefox..."
    pkill -TERM firefox
    sleep 2  # Give Firefox time to save session and close
fi

# Shutdown PC, with password
echo "Shutting down system..."  
# Use pkexec to request admin privileges for shutdown
pkexec /sbin/shutdown -h now