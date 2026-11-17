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

## Explore automatically

The robot builds the map on its own: `explore_lite` keeps sending Nav2 goals to
unexplored areas (frontiers) and drives back to its start pose when none are left.
`slam:=true` starts Nav2 without `map_server`/`amcl`, slam_toolbox provides map and pose.

```bash
# Jetson, terminal 1
ros2 launch imdl_nav mapping.launch.py
# PC, terminal 1
ros2 launch imdl_nav rviz.launch.py
# PC, terminal 2
ros2 launch imdl_nav navigation.launch.py slam:=true
# PC, terminal 3 – saves the map when exploration is complete
ros2 run imdl_nav save_on_complete --ros-args -p map:=~/maps/my_map
# PC, terminal 4 – starts exploring right away
ros2 launch explore_lite explore.launch.py use_sim_time:=false
```

Pause / continue exploring:

```bash
ros2 topic pub --once /explore/resume std_msgs/Bool "data: false"   # pause
ros2 topic pub --once /explore/resume std_msgs/Bool "data: true"    # continue
```

Parameters (e.g. `min_frontier_size`, `return_to_init`):
`src/m-explore-ros2/explore/config/params.yaml`.

## Simulation

Everything runs on the PC, no Jetson. The simulator must publish `/clock`,
`/scan` (frame `laser`) and `<namespace>/odom`, and listen on `<namespace>/cmd_vel`.

| Argument | Used by | Why |
| --- | --- | --- |
| `sim:=true` | `robot.launch.py`, `mapping.launch.py` | no lidar driver, the simulator publishes `/scan` |
| `use_sim_time:=true` | all launch files | use the simulator clock |
| `namespace:=/<ns>` | `robot`, `mapping`, `navigation` | only if the simulated robot's namespace is not the one in `robot.yaml` |
| `slam:=true` | `navigation.launch.py` | navigate while mapping (no `map_server`/`amcl`), for exploring |

Build a map:

```bash
# PC, terminal 1 – simulator running
ros2 launch imdl_nav mapping.launch.py sim:=true use_sim_time:=true
# PC, terminal 2
ros2 launch imdl_nav rviz.launch.py use_sim_time:=true
# PC, terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Robi2/cmd_vel
# PC, terminal 4, when the map is done (SLAM still running!)
ros2 run nav2_map_server map_saver_cli -f ~/maps/sim_map --free 0.196 --ros-args -p save_map_timeout:=20.0 -p use_sim_time:=true
```

Drive to coordinates:

```bash
# PC, terminal 1 (stop mapping.launch.py first!)
ros2 launch imdl_nav robot.launch.py sim:=true use_sim_time:=true
# PC, terminal 2
ros2 launch imdl_nav rviz.launch.py use_sim_time:=true
# PC, terminal 3 – robot at its spawn point
ros2 launch imdl_nav navigation.launch.py map:=$HOME/maps/sim_map.yaml use_sim_time:=true
# PC, terminal 4
ros2 run imdl_nav go_to 1.0 0.5 90
```

Explore automatically:

```bash
# PC, terminal 1 – simulator running
ros2 launch imdl_nav mapping.launch.py sim:=true use_sim_time:=true
# PC, terminal 2
ros2 launch imdl_nav rviz.launch.py use_sim_time:=true
# PC, terminal 3
ros2 launch imdl_nav navigation.launch.py slam:=true use_sim_time:=true
# PC, terminal 4 – saves the map when exploration is complete
ros2 run imdl_nav save_on_complete --ros-args -p map:=~/maps/sim_map -p use_sim_time:=true
# PC, terminal 5 – starts exploring right away
ros2 launch explore_lite explore.launch.py use_sim_time:=true
```
