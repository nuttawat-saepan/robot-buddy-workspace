# Claude Project Context

This workspace is a ROS 2 Foxy project for Unitree Go2/Go2W readiness, local
simulation, Hesai/Pandar XT16 LiDAR integration, and field bringup planning.

## Safety Rules

There may be no real Unitree robot connected during local work.

Do not publish to `/cmd_vel` during local checks or no-motion field checks.
Do not run `cmd_vel_node` unless field movement is explicitly approved.
Do not launch real robot movement with `enable_cmd_vel:=true` unless the operator
has approved movement and provided the exact `robot_ack` gate:

```text
I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
```

For real robot no-motion bringup, use:

```bash
enable_cmd_vel:=false
enable_mqtt:=false
```

Movement tests are the final stage only, after sensor, TF, map/localization, and
operator safety checks pass.

## Workspace

```text
Main workspace: /home/sys20/projects/systonic-2307/Systronic
Hesai workspace: /home/sys20/hesai_ws
ROS distro: Foxy
```

Main packages:

```text
src/go2_control
src/go2_interfaces
```

Important files:

```text
src/go2_control/go2_control/main.py
src/go2_control/go2_control/go2w_read.py
src/go2_control/go2_control/cmd_vel_node.py
src/go2_control/go2_control/april_localizer.py
src/go2_control/go2_control/local_check.py
src/go2_control/launch/go2.launch.py
src/go2_control/launch/sim.launch.py
src/go2_control/launch/real.launch.py
src/go2_control/launch/hesai_fake_scan.launch.py
src/go2_control/launch/hesai_fake_mapping.launch.py
src/go2_control/launch/hesai_3d_mapping.launch.py
src/go2_control/launch/hesai_scan.launch.py
```

Read these docs first:

```text
src/go2_control/FIELD_RUNBOOK_QUICK.md
src/go2_control/HESAI_GO2_FIELD_CHECKLIST.md
src/go2_control/HESAI_GO2_HANDOFF.md
src/go2_control/README_HESAI_FAKE_TEST.md
src/go2_control/REAL_ROBOT_FIELD_CHECKLIST.md
```

## Current Status

Known working local checks:

```text
colcon build --packages-select go2_control passes
ros2 run go2_control local_check has returned Result: OK in simulation
fake Hesai /lidar_points publishes around 10 Hz
converted /scan publishes around 10 Hz
SLAM mapping in simulation can publish /map around 0.5 Hz
map -> odom TF can appear after SLAM starts
```

Normal simulation topics confirmed previously:

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

Important note: an initial `Invalid frame ID "map"` from `tf2_echo map odom` can
be normal while SLAM/Nav2 is starting. It is OK only after transform output
appears.

## Mental Model

Local stage:

```text
Build workspace
Run TurtleBot3/Gazebo/Nav2 simulation
Run read-only local_check
Practice fake/sim LiDAR
Practice map creation
Do not use real robot movement
```

Field + mini PC stage:

```text
Connect real Hesai/Pandar XT16
Confirm /lidar_points
Confirm TF base_link -> hesai_lidar
Convert 3D PointCloud2 to 2D LaserScan
Confirm /scan
Create real site map only after sensor/no-motion checks pass
Still do not move the Unitree unless explicitly approved
```

Field + Unitree board stage:

```text
Deploy only after mini PC no-motion workflow is stable
Check Unitree read-only topics
Check odom, imu, battery, joint states, camera, AprilTag localization
Keep cmd_vel disabled until final movement approval
```

Field movement stage:

```text
Last step only
Requires safe area, operator ready, emergency stop readiness, low speed, robot_ack
```

## Topic Model

Normal sim path:

```text
Gazebo/TurtleBot3 /scan
-> fake Unitree bridge /utlidar/cloud
-> go2w_read /pointcloud
```

3D mapping sim path:

```text
Gazebo 3D PointCloud2 /lidar_points
-> pointcloud_to_laserscan /hesai_scan
-> slam_toolbox /map
```

Real Hesai/Pandar XT16 expectation:

```text
Hesai driver publishes /lidar_points as sensor_msgs/PointCloud2
frame_id should be hesai_lidar
Nav2/SLAM usually consumes a 2D LaserScan slice converted from that pointcloud
```

In `hesai_3d_mapping.launch.py`, `/scan` may be empty or irrelevant. Use
`/lidar_points`, `/hesai_scan`, and `/map`.

## Build

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
colcon build --packages-select go2_control
source install/setup.bash
```

## Local Simulation

Terminal 1:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch go2_control sim.launch.py \
  enable_unitree_bridge:=true \
  enable_april:=false \
  enable_stream:=false \
  enable_init_pose:=false
```

Terminal 2, optional readiness check after sim is running:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 run go2_control local_check
```

`local_check` is for local simulation readiness. It is not the main field check.

## 3D LiDAR Mapping Simulation

Run:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch go2_control hesai_3d_mapping.launch.py
```

Teleop in simulation only:

```bash
source /opt/ros/foxy/setup.bash
source /home/sys20/projects/systonic-2307/Systronic/install/setup.bash
export TURTLEBOT3_MODEL=waffle

ros2 run turtlebot3_teleop teleop_keyboard
```

Checks:

```bash
ros2 topic hz /lidar_points
ros2 topic info /lidar_points
ros2 topic hz /hesai_scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

Save map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/sys20/projects/systonic-2307/Systronic/src/go2_control/map/hesai_3d_sim_map
```

## Field + Mini PC No-Motion Check

Source in every terminal:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source /home/sys20/hesai_ws/install/setup.bash
source install/setup.bash
```

Network:

```bash
ip addr
ping 192.168.1.201
```

Start official Hesai driver:

```bash
source /opt/ros/foxy/setup.bash
source /home/sys20/hesai_ws/install/setup.bash
ros2 launch hesai_ros_driver start.py
```

Check LiDAR:

```bash
ros2 topic list | grep lidar
ros2 topic info /lidar_points -v
ros2 topic hz /lidar_points
ros2 topic echo /lidar_points --no-arr
```

Bridge real PointCloud2 to LaserScan:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source /home/sys20/hesai_ws/install/setup.bash
source install/setup.bash

ros2 launch go2_control hesai_scan.launch.py \
  lidar_topic:=/lidar_points \
  lidar_frame:=hesai_lidar \
  base_frame:=base_link
```

Check converted scan and TF:

```bash
ros2 topic info /scan -v
ros2 topic hz /scan
ros2 topic echo /scan --no-arr
ros2 run tf2_ros tf2_echo base_link hesai_lidar
```

## Real Robot No-Motion Launch

Only after sensor checks pass:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source /home/sys20/hesai_ws/install/setup.bash
source install/setup.bash

ros2 launch go2_control real.launch.py \
  enable_cmd_vel:=false \
  enable_camera:=true \
  enable_april:=true \
  enable_mqtt:=false
```

No-motion checks:

```bash
ros2 topic hz /lidar_points
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu/data
ros2 topic hz /battery
ros2 topic info /cmd_vel
ros2 node list | grep cmd_vel
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

Expected:

```text
No cmd_vel_node for no-motion checks
No unexpected movement bridge
/lidar_points steady
/scan steady
odom/base TF available
map/localization available when localization stack is running
```

## Hardcoded Config To Avoid

Before real field movement, parameterize or explicitly verify:

```text
MQTT broker IP in main.py / launch args
Unitree SDK network interface in cmd_vel_node.py / launch args
map path in real.launch.py
Hesai IP, UDP port, PTC port, and frame id
```

## If Editing

Keep changes scoped to:

```text
src/go2_control
src/go2_interfaces
```

Do not remove safety gates.
Do not make `/cmd_vel` enabled by default.
Prefer launch/config parameters over hardcoded IPs, interfaces, and map paths.
