# Go2 + Hesai Field Runbook Quick

Use this when the robot and Hesai XT16 are physically present.

Stop immediately if any no-motion check fails. Do not publish `/cmd_vel`, do not
run `cmd_vel_node`, and do not use `enable_cmd_vel:=true` during this runbook.

## 0. Files

```text
Main workspace: ~/projects/systonic-2307/Systronic
Hesai workspace: ~/hesai_ws
Hesai config example: src/go2_control/config/hesai_xt16_field.example.yaml
Field checklist: src/go2_control/HESAI_GO2_FIELD_CHECKLIST.md
Fake mapping rehearsal: ros2 launch go2_control hesai_fake_mapping.launch.py
Gazebo 3D mapping rehearsal: ros2 launch go2_control hesai_3d_mapping.launch.py
```

## 1. Terminal Setup

Run this in every terminal:

```bash
cd ~/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source ~/hesai_ws/install/setup.bash
source install/setup.bash
```

Check:

```bash
printenv ROS_DISTRO
ros2 pkg executables hesai_ros_driver
ros2 pkg executables go2_control
```

Expected:

```text
foxy
hesai_ros_driver_node is listed
go2_control executables are listed
```

## 2. Network

Record:

```text
Laptop Ethernet interface:
Laptop IP:
Hesai IP:
```

Check:

```bash
ip addr
ping 192.168.1.201
```

If the Hesai IP is different, update the field config before continuing.

## 3. Field Config

Create once:

```bash
cp src/go2_control/config/hesai_xt16_field.example.yaml \
   src/go2_control/config/hesai_xt16_field.yaml
```

Verify these values:

```text
device_ip_address: "192.168.1.201"
udp_port: 2368
ptc_port: 9347
ros_frame_id: hesai_lidar
ros_send_point_cloud_topic: /lidar_points
```

## 4. Hesai Driver

Terminal 1:

```bash
source /opt/ros/foxy/setup.bash
source ~/hesai_ws/install/setup.bash
ros2 launch hesai_ros_driver start.py
```

Terminal 2 checks:

```bash
ros2 topic hz /lidar_points
ros2 topic echo /lidar_points --no-arr
```

Expected:

```text
/lidar_points publishes steadily
header.frame_id: hesai_lidar
```

Stop here if `/lidar_points` is missing or unstable.

## 5. PointCloud To Scan

Local fake validation passed on 2026-08-06:

```text
hesai_fake_scan.launch.py
/lidar_points average rate: 9.980 Hz
/scan average rate: 9.947 Hz
```

Terminal 3:

```bash
cd ~/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source ~/hesai_ws/install/setup.bash
source install/setup.bash

ros2 launch go2_control hesai_scan.launch.py \
  lidar_topic:=/lidar_points \
  lidar_frame:=hesai_lidar \
  base_frame:=base_link
```

Checks:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link hesai_lidar
```

Expected:

```text
/scan publishes steadily
TF base_link -> hesai_lidar is available
```

Stop here if `/scan` is empty or TF is missing.

## 6. Real Robot No-Motion Bringup

Terminal 4:

```bash
cd ~/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source ~/hesai_ws/install/setup.bash
source install/setup.bash

ros2 launch go2_control real.launch.py \
  enable_cmd_vel:=false \
  enable_camera:=true \
  enable_april:=true \
  enable_mqtt:=false
```

Checks:

```bash
ros2 topic list | grep cmd_vel
ros2 topic hz /scan
ros2 topic hz /lidar_points
ros2 run tf2_ros tf2_echo map odom
```

Expected:

```text
No cmd_vel_node
No root /cmd_vel publisher from a movement bridge
LiDAR and scan are steady
map -> odom appears after localization
```

## 7. Hold Before Movement

Do not continue to field movement until these are confirmed:

```text
Emergency stop/operator ready
Hesai /lidar_points OK
/scan OK
TF OK
map/localization OK
AprilTag detection and /initialpose OK
Unitree network interface known
robot_ack gate explicitly approved
```
