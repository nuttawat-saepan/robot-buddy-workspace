> SUPERSEDED, kept for reference. Hesai XT16 era. The Livox equivalent is the replay appendix in RUNBOOK_ONSITE.md.
> The sensor is a Livox Mid-360 and the robot is a Go2W; commands
> and topic names below are for neither. See CLAUDE.md.

# Hesai XT16 Fake LiDAR Test

This workflow tests the LiDAR-to-costmap path locally without a real Hesai
LiDAR, Unitree ROS network, or robot motion commands.

Do not run `cmd_vel_node` for this test.

## 1. Build

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
colcon build --packages-select go2_interfaces go2_control
source install/setup.bash
```

## 2. Run Fake Hesai And LaserScan

Terminal 1:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch go2_control hesai_fake_scan.launch.py
```

This starts:

```text
fake_hesai_xt16 -> /lidar_points
static TF base_link -> hesai_lidar
pointcloud_to_laserscan -> /scan
```

Expected checks:

```bash
ros2 topic hz /lidar_points
ros2 topic hz /scan
```

Both should be near 10 Hz.

Verified locally on 2026-08-06:

```text
/lidar_points average rate: 9.980 Hz
/scan average rate: 9.947 Hz
```

This confirms the fake XT16 `PointCloud2` to `LaserScan` path is working.

## 3. Run Costmap-Only Test

Terminal 2:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch go2_control costmap_scan_test.launch.py
```

This starts only:

```text
map_server
standalone costmap
robot_state_publisher
static TF map -> odom
static TF odom -> base_link
rviz2
```

It does not start:

```text
cmd_vel_node
controller_server
planner_server
bt_navigator
go2w_read
camera
april_localizer
```

Expected checks:

```bash
ros2 topic list | grep cmd_vel
ros2 service call /costmap/costmap/get_state lifecycle_msgs/srv/GetState {}
ros2 topic hz /costmap/costmap_raw
```

`grep cmd_vel` should print nothing. The costmap state should be `active`.

## 4. Fake Hesai Mapping

This tests the pointcloud conversion path while keeping simulation geometry tied
to the Gazebo house:

```text
Gazebo /scan -> scan_to_pointcloud -> /lidar_points -> pointcloud_to_laserscan -> /hesai_scan -> slam_toolbox -> /map
```

Terminal 1:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch go2_control hesai_fake_mapping.launch.py
```

This launch starts Gazebo/TurtleBot3, a Gazebo scan to fake Hesai pointcloud
bridge, `pointcloud_to_laserscan`, `slam_toolbox`, and RViz. It does not start
`cmd_vel_node` or publish `/cmd_vel` by itself.

Move only the simulated TurtleBot3 if mapping is intentionally being tested:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Expected checks:

```bash
ros2 topic hz /lidar_points
ros2 topic hz /hesai_scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

Save a map after driving the simulation:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/sys20/projects/systonic-2307/Systronic/src/go2_control/map/fake_hesai_map
```

This is a software pipeline rehearsal. It uses Gazebo scan geometry so the map
can match the Gazebo house, while still exercising the pointcloud conversion
path. It does not prove real Pandar networking, mounting, calibration, or
field-quality SLAM.

## 5. Gazebo 3D LiDAR Mapping

This is the closer simulation path for a Pandar-style source. It spawns a local
TurtleBot3 waffle model with a simulated 16-layer ray sensor that publishes
`sensor_msgs/PointCloud2` directly:

```text
Gazebo 3D ray sensor -> /lidar_points -> pointcloud_to_laserscan -> /hesai_scan -> slam_toolbox -> /map
```

Run:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch go2_control hesai_3d_mapping.launch.py
```

Move only the simulated TurtleBot3:

```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

Expected checks:

```bash
ros2 topic hz /lidar_points
ros2 topic hz /hesai_scan
ros2 topic hz /map
ros2 topic info /lidar_points
```

`/lidar_points` should be `sensor_msgs/msg/PointCloud2` from Gazebo, not a
Python fake node.

For the simulated 3D LiDAR mapping debug pass, `pc2scan_hesai.yaml` currently
uses these self-filter values:

```text
target_frame: base_scan
min_height: -0.06
max_height: 0.06
range_min: 0.7
```

This was tuned after RViz showed a dark ring around the robot instead of a room
outline. It is a simulation debug value, not a final field setting. If the map
still mainly shows the robot body, try increasing `range_min` to `1.0`; if
`/hesai_scan` becomes too sparse, widen the height band carefully.

Current status: `/map` can publish and RViz can show a map in this launch, but
the 3D-to-2D projection still needs tuning for clean room-shaped maps.

## 6. RViz

Use these displays:

```text
Fixed Frame: map
TF
Map: /map
LaserScan: /hesai_scan
PointCloud2: /lidar_points
```

For `/map`, set:

```text
Reliability Policy: Reliable
Durability Policy: Transient Local
```

## 7. Real Hesai Swap

When the real Hesai XT16 is available, stop `hesai_fake_scan.launch.py` and run
the real Hesai driver so it publishes:

```text
/lidar_points
frame_id: hesai_lidar
```

Then run:

```bash
ros2 launch go2_control hesai_scan.launch.py
```

The downstream path remains:

```text
/lidar_points -> pointcloud_to_laserscan -> /scan -> costmap
```

Update the static transform in `hesai_scan.launch.py` to match the real sensor
mounting position before field testing.

## 8. Simulation-Only Drive Smoke Test

This test uses a copied TurtleBot3 Gazebo model whose diff-drive plugin listens
to `/sim_cmd_vel`, not `/cmd_vel`.

Terminal:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch go2_control sim_safe_drive_test.launch.py
```

This starts Gazebo only. It does not publish movement commands by default.

To send a short sim-only command:

```bash
ros2 launch go2_control sim_safe_drive_test.launch.py enable_drive:=true
```

The default command publishes `/sim_cmd_vel` for 3 seconds:

```text
linear_x: 0.08
angular_z: 0.0
duration: 3.0
```

Check that `/cmd_vel` is not present:

```bash
ros2 topic list | grep cmd_vel
```

Expected result:

```text
/sim_cmd_vel
```

There should be no root `/cmd_vel` topic.
