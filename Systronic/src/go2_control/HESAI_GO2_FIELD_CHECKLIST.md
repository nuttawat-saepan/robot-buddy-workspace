# Hesai XT16 + Go2 Field Checklist

This checklist is for field testing without robot movement first.

Do not run `cmd_vel_node`, do not publish `/cmd_vel`, and do not launch with
`enable_cmd_vel:=true` until the read-only checks pass and movement is approved.

For a short terminal-by-terminal version, use:

```text
src/go2_control/FIELD_RUNBOOK_QUICK.md
```

## 1. Source Overlay

Use the already-built official Hesai ROS 2 driver in:

```text
~/hesai_ws/src/HesaiLidar_ROS_2.0
origin: https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
tag: v2.0.12
```

Do not swap to another fork, including `leggedrobotics/hesai_lidar_ros_driver`,
during the first field bringup unless the official driver is confirmed blocked.

```bash
source /opt/ros/foxy/setup.bash
source ~/hesai_ws/install/setup.bash
source ~/projects/systonic-2307/Systronic/install/setup.bash
```

Check:

```bash
ros2 pkg executables hesai_ros_driver
ros2 launch go2_control hesai_scan.launch.py --show-args
ros2 launch go2_control costmap_scan_test.launch.py --show-args
```

## 2. Network Readiness

Record before connecting:

```text
Laptop Ethernet interface:
Laptop IP:
Hesai IP:
UDP port:
PTC port:
```

Check:

```bash
ip addr
ping <HESAI_IP>
```

If `ping` fails, do not start LiDAR integration debugging in Nav2. Fix network
first.

## 3. Hesai Driver Config

Start from:

```text
src/go2_control/config/hesai_xt16_live.template.yaml
src/go2_control/config/hesai_xt16_field.example.yaml
```

Copy the example to a field-specific config:

```bash
cp src/go2_control/config/hesai_xt16_field.example.yaml src/go2_control/config/hesai_xt16_field.yaml
```

Then verify or replace:

```text
device_ip_address: "192.168.1.201"
udp_port: 2368
ptc_port: 9347
ros_frame_id: hesai_lidar
ros_send_point_cloud_topic: /lidar_points
```

Expected output:

```text
topic: /lidar_points
frame_id: hesai_lidar
source_type: 1
```

## 4. LiDAR Only

Run the Hesai driver with the field config.

Check:

```bash
ros2 topic hz /lidar_points
ros2 topic echo /lidar_points
```

Expected:

```text
/lidar_points publishes steadily
header.frame_id is hesai_lidar
```

Stop here if `/lidar_points` is missing or unstable.

## 5. PointCloud To LaserScan

```bash
ros2 launch go2_control hesai_scan.launch.py
```

Check:

```bash
ros2 topic hz /scan
ros2 topic echo /scan
ros2 run tf2_ros tf2_echo base_link hesai_lidar
```

Expected:

```text
/scan publishes steadily
LaserScan frame is base_link
TF base_link -> hesai_lidar is available
```

If scan is empty or all `inf`, tune:

```text
pc2scan_hesai.yaml min_height / max_height / range_min / range_max
```

## 6. Costmap Only

```bash
ros2 launch go2_control costmap_scan_test.launch.py
```

Check:

```bash
ros2 topic list | grep cmd_vel
ros2 service call /costmap/costmap/get_state lifecycle_msgs/srv/GetState {}
ros2 topic hz /costmap/costmap_raw
```

Expected:

```text
No root /cmd_vel topic
costmap state is active
/costmap/costmap_raw publishes
```

RViz:

```text
Fixed Frame: map
Map: /map
LaserScan: /scan
Costmap: /costmap/costmap_raw
TF
```

For `/map`, set:

```text
Reliability Policy: Reliable
Durability Policy: Transient Local
```

## 7. Go2 Read-Only Gate

This is not required for LiDAR-only testing, but it is required before Go2
integration. `unitree_go` should already be available in this workspace.

Check:

```bash
python3 -c "from unitree_go.msg import LowState, SportModeState; print('unitree_go ok')"
```

Expected:

```text
unitree_go ok
```

If it fails, `go2w_read` and `real.launch.py` are not ready for Go2 read-only
testing.

Import-check `go2w_read` without running the node:

```bash
python3 -c "import go2_control.go2w_read; print('go2w_read import ok')"
```

Expected:

```text
go2w_read import ok
```

## 8. Stop Conditions

Stop and do not proceed toward movement if any of these happen:

```text
/lidar_points missing or unstable
/scan missing or mostly empty
TF base_link -> hesai_lidar wrong or missing
costmap marks the robot body as an obstacle
unexpected /cmd_vel publisher appears
go2w_read crashes
odom -> base_link TF missing
```
