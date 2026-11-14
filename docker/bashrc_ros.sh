# Sourced by every interactive shell in the container.
source /opt/ros/humble/setup.bash

# Must match the Jetson and the Create3 (set ROS_DOMAIN_ID before ./run.sh).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Build the workspace (imdl_nav_ws, mounted as /nav_ws) and source it.
nav_build() {
    (cd /nav_ws && colcon build --symlink-install --packages-select imdl_nav explore_lite_msgs explore_lite "$@") \
        && source /nav_ws/install/setup.bash
}

# Notebooks; open the printed http://127.0.0.1:8888/... link in the browser on the PC.
nav_notebooks() {
    jupyter notebook --allow-root --no-browser --ip=127.0.0.1 --notebook-dir=/nav_ws/notebooks "$@"
}

if [ -f /nav_ws/install/setup.bash ]; then
    source /nav_ws/install/setup.bash
else
    echo "Workspace not built yet: run 'nav_build'"
fi
