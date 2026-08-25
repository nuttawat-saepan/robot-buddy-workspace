# Real Robot Field Checklist

Use this checklist only when a real Unitree Go2/Go2W is present and the field
operator has approved movement testing.

Local development and simulation checks must keep movement disabled.

## Safety Gate

Real movement requires both:

```text
enable_cmd_vel:=true
robot_ack:=I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
```

If `robot_ack` is missing or different, movement-capable nodes must refuse to
arm.

The ack is intentionally long. Do not shorten it or replace it with a boolean.

## No-Motion Bringup

Start here before any movement:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch go2_control real.launch.py \
  enable_cmd_vel:=false \
  enable_camera:=false \
  enable_april:=false \
  enable_mqtt:=false \
  enable_unitree_read:=true
```

`go2.launch.py mode:=real` can be used as a front door, but pass the real map
and params explicitly. The default `go2.launch.py` values are optimized for
local simulation.

Expected checks:

```bash
ros2 topic hz /odom
ros2 topic hz /imu/data
ros2 topic echo /battery
ros2 run tf2_ros tf2_echo odom base_link
```

Stop if odom, IMU, battery, or TF is missing.

## LiDAR Checks

Before movement:

```bash
ros2 topic hz /pointcloud
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link radar
```

Verify the LiDAR frame and static transform match the real mounting position.
Tune `pc2scan.yaml` before trusting costmaps.

## AprilTag Checks

Run AprilTag localization in no-motion mode first:

```bash
ros2 launch go2_control real.launch.py \
  enable_cmd_vel:=false \
  enable_camera:=true \
  enable_april:=true \
  enable_mqtt:=false
```

No-motion AprilTag localization may publish `/initialpose`; it must not publish
`/cmd_vel`.

Only enable scan/rotate localization after the movement gate is armed and the
operator approves it.

## Movement Arming

Only after sensor checks, operator approval, clear floor space, and emergency
stop readiness:

```bash
ros2 launch go2_control real.launch.py \
  enable_cmd_vel:=true \
  robot_ack:=I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT \
  unitree_interface:=<real_network_interface>
```

Use low speed and short duration first. Keep a human operator ready to stop the
robot.

Do not run this checklist on a machine connected to an unattended robot.
