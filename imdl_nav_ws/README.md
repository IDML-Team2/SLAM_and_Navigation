# imdl_nav_ws – commands

Our setup: namespace `/Robi2`, `ROS_DOMAIN_ID=2` (Create3, Jetson and PC).
Full explanations: [TUTORIAL_SLAM_NAVIGATION.md](../TUTORIAL_SLAM_NAVIGATION.md).

## Start Docker on the PC

Run this in **every** PC terminal. The first one starts the container, the
others open another shell in it. Closing the first terminal stops the container.

```bash
cd docker && ./run.sh

```

The first time, and after adding new files, build inside the container:

```bash
nav_build
```

`ROS_DOMAIN_ID=2` must be set in the PC's `~/.bashrc`, otherwise the container
uses domain 1.

## Connect to the Jetson

```bash
ssh jetson@10.23.64.142
cd imdl_slam_nav/imdl_nav_ws
```

If the robot is in its dock, undock it from a PC terminal:

```bash
ros2 action send_goal /Robi2/undock irobot_create_msgs/action/Undock "{}"
```

## Build a map

```bash
# Jetson, terminal 1
ros2 launch imdl_nav mapping.launch.py
# PC, terminal 1
ros2 launch imdl_nav rviz.launch.py
# PC, terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Robi2/cmd_vel
# PC, terminal 3, when the map is done (SLAM still running!)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --free 0.196 --ros-args -p save_map_timeout:=20.0

```

## Drive to coordinates

```bash
# Jetson (stop mapping.launch.py first!)
ros2 launch imdl_nav robot.launch.py
# PC, terminal 1 (if not open anymore)
ros2 launch imdl_nav rviz.launch.py
# PC, terminal 2 – robot is standing on the start mark
ros2 launch imdl_nav navigation.launch.py map:=$HOME/maps/my_map.yaml
# PC, terminal 3
ros2 run imdl_nav go_to 1.0 0.5 90          # x [m], y [m], heading [degrees]
ros2 run imdl_nav waypoints                 # points from config/waypoints.yaml
```
