# Local Sim Readiness

This runbook checks the ROS2 Foxy Go2/Go2W workspace locally with no real
Unitree robot connected.

Do not run `cmd_vel_node`. Do not launch with `enable_cmd_vel:=true`. Do not
press RViz `Nav2 Goal` during safety-only checks, because Nav2 can publish
`/cmd_vel`.

## 1. Build

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
```

Expected:

```text
Summary: 4 packages finished
```

## 2. Launch Simulation

Terminal 1:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle

ros2 launch go2_control sim.launch.py \
  enable_unitree_bridge:=true \
  enable_april:=false \
  enable_stream:=false \
  enable_init_pose:=false
```

The front-door launcher is also supported:

```bash
ros2 launch go2_control go2.launch.py \
  mode:=sim \
  enable_unitree_bridge:=true \
  enable_april:=false \
  enable_stream:=false \
  enable_init_pose:=false
```

This starts Gazebo, RViz, Nav2 localization, the fake Unitree bridge, and the
local scan-to-pointcloud bridge.

It does not start `cmd_vel_node`, AprilTag localization, or Stream/WebRTC.

The default simulation map is installed from:

```text
src/go2_control/map/house_map.yaml
src/go2_control/map/house_map.pgm
```

These files were restored from the old workspace map and are expected to match
the TurtleBot3 house world better than the generic TurtleBot3 navigation map.

## 3. Run Smoke Check

Terminal 2:

Run this only after Terminal 1 is still running and Gazebo/RViz/Nav2 have had a
short time to start. `local_check` does not launch simulation; it only inspects
the already-running ROS graph.

```bash
cd /home/sys20/projects/systonic-2307/Systronic
./scripts/local_sim_check.sh
```

Equivalent manual command:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run go2_control local_check
```

Expected final line:

```text
Result: OK
```

Order reminder:

```text
1. Build/source workspace
2. Launch simulation in Terminal 1 and leave it running
3. Run local_check in Terminal 2
```

## 4. Expected Data Path

```text
Gazebo/TurtleBot3
  -> /joint_states /odom /imu /scan
  -> gazebo_convert
  -> /lf/lowstate /lf/sportmodestate
  -> scan_to_pointcloud
  -> /utlidar/cloud
  -> go2w_read
  -> /odom /imu/data /pointcloud /battery /joint_states
```

Expected topics:

```text
/lf/lowstate
/lf/sportmodestate
/utlidar/cloud
/pointcloud
/odom
/imu/data
/battery
/joint_states
/map
```

## 5. TF And Localization

Before setting an initial pose, AMCL may warn:

```text
AMCL cannot publish a pose or update the transform. Please set the initial pose
```

This is expected.

In RViz:

```text
Fixed Frame: map
Tool: 2D Pose Estimate
```

After setting the pose, check:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

Both should print `Translation` and `Rotation` values after a short wait.

## 6. Common Fixes

If `/lf/lowstate` exists but has no data, verify that `sim.launch.py` remaps:

```text
/sim_joint_states -> /joint_states
/sim_odom -> /odom
/sim_imu -> /imu
```

If `/pointcloud` has no data, verify this simulation-only path:

```text
/scan -> scan_to_pointcloud -> /utlidar/cloud -> go2w_read -> /pointcloud
```

Keep `enable_pc2scan:=false` for the normal TurtleBot3 simulation path. It is
for PointCloud-to-LaserScan tests, not the default sim readiness flow.

If RViz `2D Goal Pose` does nothing, check Nav2 lifecycle and BT navigator:

```bash
ros2 lifecycle get planner_server
ros2 lifecycle get controller_server
ros2 lifecycle get bt_navigator
ros2 action list
```

Expected movement-ready states:

```text
planner_server: active
controller_server: active
bt_navigator: active
/navigate_to_pose appears in action list
```

On this Foxy install, `nav2_params.yaml` must use the Foxy-compatible
`bt_navigator.plugin_lib_names` list. Newer Nav2 BT plugins such as
`nav2_compute_path_through_poses_action_bt_node` are not available and can leave
`bt_navigator` unconfigured.
