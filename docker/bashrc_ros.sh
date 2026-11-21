# Sourced by every interactive shell in the container.
source /opt/ros/humble/setup.bash

# Must match the Jetson and the Create3 (set ROS_DOMAIN_ID before ./run.sh).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Build the workspace (imdl_nav_ws, mounted as /nav_ws) and source it.
nav_build() {
    (cd /nav_ws && colcon build --symlink-install --packages-select imdl_nav "$@") \
        && source /nav_ws/install/setup.bash
}

if [ -f /nav_ws/install/setup.bash ]; then
    source /nav_ws/install/setup.bash
else
    echo "Workspace not built yet: run 'nav_build'"
fi
