# Livox Mid-360 Field Day, 2026-08-19

Record of the first on-site day with the Mid-360. Read
`LIVOX_MID360_MIGRATION.md` alongside this: that file is the standing reference
for how the Livox path is put together, while this one is what happened on the
day and which numbers came from real hardware.

No robot motion took place. Nothing here published `/cmd_vel`, no
`cmd_vel_node` was started, and no `enable_cmd_vel:=true` was used. Any walking
described below was done by the operator on the handheld remote, with the
software subscribing only.

## 1. What The Day Was For

Three things: see a real point cloud from the Mid-360, capture raw data that
can be replayed later away from site, and get a 2D map on screen in RViz.

All three were reached. The sensor ran over a LAN cable straight into the
MiniPC, which is a bench arrangement rather than a deployment - see section 7.

## 2. Data Captured

Bags live in `~/livox_bags_field/`, deliberately outside git: 2.5 GB total,
against 151 MB for thirty seconds.

| bag | duration | `/livox/lidar` | `/livox/imu` | size |
|---|---|---|---|---|
| `01_static` | 29.7 s | 298 | 5946 | 151 MB |
| `02_loop` | 179.7 s | 1797 | 35943 | 909 MB |
| `02_loopfix` | 179.7 s | 1797 | 35945 | 909 MB |
| `04_rotate` | 59.7 s | 598 | 11946 | 303 MB |
| `04_rotateffix` | 59.7 s | 597 | 11945 | 302 MB |

Every count matches 10 Hz and 200 Hz over the recorded duration, so no frames
were dropped in any take.

Only `/livox/lidar` and `/livox/imu` were recorded, and that choice paid for
itself the same day. `/Odometry` is computed from those two, so leaving it out
meant that when the 13 degree problem in section 6 turned up in the afternoon,
the morning's takes were still good and nothing had to be walked again.

Outstanding: `03_site`, a 300 s walk over the whole area, was never recorded.

Cleanup available: `01_static-1` is an empty 24 KB failed take, and the two
`*fix` bags duplicate their originals. Removing them frees about 1.2 GB.

## 3. Getting The Driver To Run

The driver hung at `Init lds lidar success!` and published nothing. This unit
reports `dev_type = 35`, which needs the JSON root key `"Mid360s"` rather than
`"MID360"`, with `host_net_info` as an **array** carrying `host_ip`. The
diagnostic signature is that the log has no `GetFreeIndex` line.

The working file is `~/ws_livox/src/livox_ros_driver2/config/MID360_minipc.json`
and is still outside git.

```text
LiDAR       192.168.123.20
MiniPC      192.168.123.18   enp3s0, NetworkManager profile "livox-direct"
```

The manually added host IP kept disappearing because NetworkManager manages the
interface. A persistent profile with no gateway fixed it, the missing gateway
being deliberate so the LiDAR subnet cannot capture the default route:

```bash
sudo nmcli con add type ethernet ifname enp3s0 con-name livox-direct \
  ipv4.method manual ipv4.addresses 192.168.123.18/24 ipv6.method ignore
```

Note that the board's `eth0` also holds 192.168.123.18. Take the profile down
with `sudo nmcli con down livox-direct` when the LiDAR is not on the MiniPC.

## 4. Why RViz Showed Nothing

Three independent causes, each sufficient on its own:

1. `/livox/lidar` carried two message types and the live one was `CustomMsg`.
   Counting settled it: PointCloud2 0 messages, CustomMsg 1. Fixed by forcing
   `xfer_format = 0`.
2. `/tf_static` was empty, so the frame `livox_frame` did not exist and RViz
   had nowhere to draw. Fixed with a static transform.
3. QoS had to match. The publisher is RELIABLE.

## 5. Two Bugs In Upstream FAST-LIO

Both were found while proving the mapping chain end to end, and both would
otherwise have surfaced only after a walk had already been done - the failure
mode in each case is a run that appears to work and produces no file.

**Ctrl-C never returned from `spin()`.** `main()` installed its own
`signal(SIGINT, SigHandle)`, replacing the handler `rclcpp::init` had set, and
`SigHandle` then called `rclcpp::shutdown()` from inside the signal handler
while the main thread sat in `spin()`. On Foxy that deadlocks. The map save at
the end of `main()` sits directly after `spin()`, so it was never reached and
the node only ended when something escalated to SIGKILL. Shutdown now takes
about two seconds.

**The map accumulation was commented out.** In `publish_frame_world`, the whole
`if (pcd_save_en)` block that appends each registered scan to `pcl_wait_save`
ships commented out on the ROS2 branch. The save is guarded by
`pcl_wait_save->size() > 0`, so `pcd_save_en` could be set and no `.pcd` would
ever appear, with nothing logged to say why.

These are patches 5 and 6 in `LIVOX_MID360_MIGRATION.md` section 7, which is
where the full list lives since `~/ws_fastlio_livox` is not under version
control.

## 6. The 13 Degree Mount Tilt

Unitree publish the Mid-360 mount pose as (0.1870, 0, 0.0803) in the Go2 body
IMU frame, rotated 13 degrees about Y. That tilt caused two separate failures.

**The map came out as a solid black blob.** FAST-LIO's world frame
`camera_init` inherits the sensor's orientation at startup, so the whole world
frame is tilted 13 degrees. A flat height slice through it cuts diagonally
through the floor and paints the floor as wall right across the room. Fixed by
levelling the cloud before slicing, via `pitch_deg` on `livox_grid_map`.

**Nav2 would have been given a tilted odom.** The `odom -> camera_init` bridge
was identity, which is right for the translation and wrong for the rotation:
`base_link` came out pitched 13 degrees out of level, permanently. Everything
downstream assumes odom is gravity-aligned. The bridge now carries pitch
+0.22689 rad, and `base_link` measured **+0.11 degrees** in odom afterwards.

Both fixes assume the robot is standing roughly level when the run starts.
Starting on a ramp would bake that ramp in.

## 7. Verified Numbers

All measured on hardware on the day.

```text
mount pose         (0.1870, 0, 0.0803), pitch 13 deg about Y
body -> base_link  (-0.16414, 0, -0.12031), pitch -0.22689 rad
base_link in odom  +0.11 deg after the bridge fix
LiDAR              10 Hz, 19968 points/frame, frame_id livox_frame
IMU                200 Hz, acc (-0.25, -0.04, 0.98), gyro ~0
scan lines         line field 0..3, confirming scan_line: 4
test map           2 840 484 points, 241 x 457 cells @ 0.05 m, walls straight
```

## 8. Still Open

```text
1. Record 03_site, a 300 s walk over the whole area.
2. Build fast_lio_livox for aarch64. Only an x86 build exists, so on-robot
   LIO is not possible yet.
3. Move the LiDAR onto the Unitree board. The LAN cable to the MiniPC used all
   day is a bench test: the robot has to move, and the point cloud cannot be
   streamed off the robot over Wi-Fi (33% loss, 395-1453 ms RTT measured).
4. Copy MID360_minipc.json into src/go2_control/config/. It is the only thing
   that made the sensor work and it is currently outside git.
5. Delete 01_static-1 and the duplicate *fix bags, freeing ~1.2 GB.
```

## 9. Rules Worth Keeping

**`ros2 bag record` does not fail on a missing topic.** It waits silently and
writes a 16 KB empty file. Three takes were lost to this. Always wait for
`Subscribed to topic '/livox/lidar'` and `'/livox/imu'` before walking away.

**Record raw sensor topics, never derived ones.** Recording `/Odometry` would
have locked in the wrong 13 degree world frame with no way back.

**Kill by PID, not by pattern.** `pkill -f livox_ros_driver2_node` matched the
shell running it and killed the session. Use
`ps -eo pid,comm | awk` and kill the PID.
