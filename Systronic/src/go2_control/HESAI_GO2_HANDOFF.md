# Hesai XT16 + Go2 Handoff

Use this note to resume field integration work safely.

## Safety

- Do not publish `/cmd_vel`.
- Do not run `cmd_vel_node`.
- Do not launch with `enable_cmd_vel:=true`.
- Do not run real robot movement commands.
- Start field testing with LiDAR + costmap only.

## Workspaces

```bash
source /opt/ros/foxy/setup.bash
source ~/hesai_ws/install/setup.bash
source ~/projects/systonic-2307/Systronic/install/setup.bash
```

Main workspace:

```text
~/projects/systonic-2307/Systronic
```

Hesai workspace:

```text
~/hesai_ws
```

## Hesai Driver Source

The installed Hesai driver workspace is based on the official Hesai ROS 2 repo:

```text
~/hesai_ws/src/HesaiLidar_ROS_2.0
origin: https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
branch: master
tag: v2.0.12
commit: e7e112f
```

Do not switch to `leggedrobotics/hesai_lidar_ros_driver` during field bringup
unless there is a specific driver issue to solve. Keep the first real LiDAR test
on the already-built official driver so only one variable changes at a time.

## Verified

- `hesai_ros_driver` builds in `~/hesai_ws`.
- `ros2 pkg executables hesai_ros_driver` shows `hesai_ros_driver_node`.
- Fake Hesai pipeline passed:

```text
/lidar_points -> /scan
```

Verified locally on 2026-08-06 with `hesai_fake_scan.launch.py`:

```text
/lidar_points average rate: 9.980 Hz
/scan average rate: 9.947 Hz
```

- Expected real Hesai output:

```text
topic: /lidar_points
frame_id: hesai_lidar
```

## Key Files

```text
src/go2_control/HESAI_GO2_FIELD_CHECKLIST.md
src/go2_control/config/hesai_xt16_live.template.yaml
src/go2_control/config/hesai_xt16_field.example.yaml
src/go2_control/pc2scan_hesai.yaml
src/go2_control/launch/hesai_scan.launch.py
src/go2_control/launch/costmap_scan_test.launch.py
```

## Unitree Messages

`unitree_go` and `unitree_api` are now copied into the Systronic workspace and
build successfully.

```bash
python3 -c "from unitree_go.msg import LowState, SportModeState; print('unitree_go ok')"
```

Expected result:

```text
unitree_go ok
```

The Unitree packages were patched to make `rosidl_generator_dds_idl` optional,
because it is not installed in this ROS 2 Foxy environment. This keeps normal
ROS 2 message generation available for `go2w_read`.

`go2w_read` has been import-checked only; it has not been run against a real
robot.

## Field Test Order

1. Source overlay.
2. Check Hesai network:

```bash
ip addr
ping <HESAI_IP>
```

3. Copy `hesai_xt16_field.example.yaml` to a field config:

```bash
cp src/go2_control/config/hesai_xt16_field.example.yaml src/go2_control/config/hesai_xt16_field.yaml
```

Then verify or replace:

```text
device_ip_address: "192.168.1.201"
udp_port: 2368
ptc_port: 9347
```

4. Run Hesai driver with `source_type: 1` only after LiDAR/network are connected.
5. Verify:

```bash
ros2 topic hz /lidar_points
ros2 topic echo /lidar_points
```

6. Run downstream scan conversion:

```bash
ros2 launch go2_control hesai_scan.launch.py
```

7. Verify:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link hesai_lidar
```

8. Run costmap-only:

```bash
ros2 launch go2_control costmap_scan_test.launch.py
```

9. Verify safety and costmap:

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

10. Do not proceed to movement until Go2 odom/TF and safety checks pass.
