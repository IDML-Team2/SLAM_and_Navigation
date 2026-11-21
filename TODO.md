# TODO

## RealSense camera in the costmap (branch `realsense_camera`)

Done:
- [x] Static TF `base_link -> camera_link` (`robot.launch.py`, values from `robot.yaml`)
- [x] Camera point cloud in its own `camera_layer` (`nav2.yaml`, global + local), so lidar
      beams can't clear low obstacles (below the lidar at 13.5 cm) that only the camera sees

Next, on the real robot:
- [ ] Get the RealSense driver running on the Jetson, with low load over Wi-Fi:
      `ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true depth_module.depth_profile:=424x240x15 decimation_filter.enable:=true`
- [ ] Find the point cloud topic: `ros2 topic list | grep points`
      -> put it in `config/nav2.yaml` (`camera: topic:`, both costmaps)
- [ ] Find the driver's root frame (default `camera_link`): `ros2 run tf2_tools view_frames`
      -> if different, set `camera: frame:` in `config/robot.yaml`
- [ ] Measure the camera position from the robot center
      -> `x_cm`, `y_cm`, `z_cm`, `yaw_deg`, `pitch_deg` in `config/robot.yaml`
- [ ] Check the TF chain: `ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame`
- [ ] RViz: add a PointCloud2 display for the camera topic, check that it lines up with the lidar scan
- [ ] Start navigation, put a box in front of the robot, check that it appears in the local costmap
- [ ] Put a LOW box (< 13 cm) in front: it must stay in the costmap (lidar can't see it)
- [ ] If the floor shows up as obstacles: check `z_cm` / `pitch_deg`, or raise `min_obstacle_height` (e.g. 0.08)
