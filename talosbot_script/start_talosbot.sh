#!/bin/bash

# Terminal tracking file
TRACKING_FILE="/tmp/talosbot_terminals.pid"

# First, check if previous instances exist and close them
if [ -f "$TRACKING_FILE" ]; then
    echo "Found existing TalosBot instances. Shutting them down..."
    
    # Kill running processes
    pkill -f "ros2 launch talosbot actions.launch.py" 2>/dev/null
    pkill -f "ros2 run mqtt_to_ros_bridge mqtt_to_ros_bridge" 2>/dev/null
    # pkill -f "gazebo" 2>/dev/null
    # pkill -f "gzserver" 2>/dev/null
    # pkill -f "gzclient" 2>/dev/null
    
    # Close associated terminal windows using their PIDs
    while read terminal_pid; do
        if ps -p $terminal_pid > /dev/null; then
            echo "Closing terminal with PID: $terminal_pid"
            kill $terminal_pid 2>/dev/null
        fi
    done < "$TRACKING_FILE"
    
    # Wait for processes to terminate
    sleep 2
    
    # Remove tracking file
    rm -f "$TRACKING_FILE"
    echo "Cleanup complete."
fi

echo "Starting TalosBot..."

# Function to get terminal PID
get_terminal_pid() {
    # Launch terminal and save its PID
    gnome-terminal --tab -- bash -c "$1; exec bash" & 
    echo $!
}

# Launch Terminal 1 and save its PID
TERM1_PID=$(get_terminal_pid "cd ros2_ws && source install/setup.bash && ros2 launch talosbot actions.launch.py")
echo "Terminal 1 launched with PID: $TERM1_PID"

# Wait a moment for the first command to start
sleep 2

# Launch Terminal 2 and save its PID
TERM2_PID=$(get_terminal_pid "cd ros2_ws && source install/setup.bash && ros2 run mqtt_to_ros_bridge mqtt_to_ros_bridge")
echo "Terminal 2 launched with PID: $TERM2_PID"

# Save terminal PIDs to tracking file
echo "$TERM1_PID" > "$TRACKING_FILE"
echo "$TERM2_PID" >> "$TRACKING_FILE"

echo "TalosBot started successfully. Run this script again to restart."