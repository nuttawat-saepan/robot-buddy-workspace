# Livox Go2 Deployment Plan

The plan from where the workspace stands on 2026-08-25 to a robot that takes a
mission from the web, drives it on LiDAR, localises with AprilTag, and sends
the photographs back.

Read `LIVOX_AMCL_REPLAY_2026-08-25.md` and `LIVOX_NAV2_REPLAY_2026-08-25.md`
first for the measurements this plan is built on.

Nothing in Parts 1 to 3 moves the robot. Movement is Part 4 only, and every
gate in Part 3 must pass before it.

## 0. The Architecture, And Why

Two machines, split so that nothing large crosses the network:

```text
ON THE ROBOT
  Livox Mid-360 --(LAN, 30 cm, on the robot)--> Unitree board (robot battery)
      livox_ros_driver2        /livox/lidar, /livox/imu
      fastlio_mapping          /Odometry, /cloud_registered_body
      pointcloud_to_laserscan  /scan
      static TF x2             odom->camera_init, body->base_link
      lio_odom_relay           /odom
      cmd_vel_node             /cmd_vel -> Unitree SDK -> legs
      watchdog                 stops the robot when signal goes stale

         Wi-Fi, dedicated AP, about 0.33 Mbps

GROUND STATION (MiniPC or any laptop with Foxy)
      map_server, amcl         /map, map->odom
      Nav2                     planner, controller, bt_navigator, recoveries
      rviz2
      camera consumer, april_localizer, main.py    (see 1.G)
```

The point cloud never crosses the network. That is the whole design
constraint: 20000 points at 10 Hz is 41.6 Mbps fragmented into ~370 UDP
packets per frame, and under RELIABLE QoS one lost fragment resends the whole
sample. That is what failed on 2026-08-19. A projected scan is 3 KB and needs
no fragmentation at all.

Why not run everything on the board: measured on the MiniPC with a goal
active, the navigation stack alone costs 2.2 cores. On a 4-core ARM board that
is 4 to 7 cores before the camera, AprilTag, `main.py` and Unitree's own leg
control are counted.

The ground station is a **role**, not a specific machine. Any laptop running
Ubuntu 20.04 with Foxy will do, and a laptop is better in the field because it
carries its own battery.

## Part 1 - Desk Preparation (no hardware needed)

Ordered so that the cheapest thing that can delete later work comes first.

### 1.A Odometry ownership - decided

Two sources describe `odom -> base_link` on this robot, and both also publish
`/odom`:

```text
go2w_read.py        the Go2's own leg odometry, via the Unitree SDK
lio_odom_relay      FAST-LIO, through odom -> camera_init -> body -> base_link
```

Two publishers of one transform break the TF tree silently. tf2 keeps whichever
message arrived last, so the robot pose flips between the two sources with
nothing logged, and Nav2 and `main.py` receive an interleaved `/odom`.

**Decision: FAST-LIO owns odometry.** `go2w_read` now takes `publish_odom` and
`publish_tf` parameters, both defaulting to true so the existing Hesai and
simulation paths are unchanged, and both set false on the Livox stack. It keeps
publishing what nothing else provides: imu, battery, joint states, motor
temperatures and the Unitree lidar.

Consequences of this decision, which is why it is recorded here rather than
left implicit:

* FAST-LIO must run on the robot, so building it for aarch64 is mandatory and
  1.B below becomes the highest-value preparation task.
* The alternative that was on the table - projecting the raw `/livox/lidar`
  and taking leg odometry, which would have removed the ARM build entirely -
  is no longer being pursued.
* Leg odometry remains available for comparison by re-enabling `publish_odom`
  under a remapped topic name, which is worth doing once on the robot to see
  how the two drift against each other.

### Already solved, not re-litigated

ROS 2 traffic between the board and the ground station over Wi-Fi has already
been made to work in earlier testing: the RMW implementations agree, the
`CYCLONEDDS_URI` interface binding covers the wireless interface, and the
domain configuration is workable. Unitree's `setup.sh` sets
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and pins `CYCLONEDDS_URI` to a single
named interface, which reads like a blocker but is not one in practice.

Note that the Livox stack was developed and measured on the default
`rmw_fastrtps_cpp`. Running it under CycloneDDS alongside the Unitree stack is
expected to work but has not been exercised, so re-run the replay once under
CycloneDDS before the field day.

### 1.B Put the FAST-LIO patches under version control - highest priority

`~/ws_fastlio_livox` exists only on the MiniPC and is not in git. Six hand
edits were needed to get upstream working on Foxy, all recorded in
`LIVOX_MID360_MIGRATION.md` section 7:

```text
1. ikd-Tree submodule is not fetched by a --depth 1 clone
2. package renamed fast_lio -> fast_lio_livox, plus the message namespace
3. Foxy service callback ConstSharedPtr -> SharedPtr
4. field name reflectivity -> intensity
5. remove signal(SIGINT), which deadlocked Ctrl-C so no map was ever saved
6. uncomment the map accumulation block
```

Patches 5 and 6 produce a run that appears to work and writes no file, with
nothing logged. Rediscovering them on the board would cost a day.

**Do:** generate a patch series from the working tree and commit it under
`src/go2_control/patches/fast_lio/`, with a short README giving the exact
clone command and apply order.

**Acceptance:** a fresh clone plus the patch series builds on the MiniPC and
produces the same `/Odometry` on `02_loop` as the existing build.

### 1.C Watchdog

Required before `/cmd_vel` is ever enabled, in either architecture, and
doubly so when the command crosses Wi-Fi: if the link stalls, the robot keeps
executing the last velocity it received.

**Do:** a node on the robot that watches `/cmd_vel`, `/scan` and `/odom` and
publishes zero velocity - and refuses to pass anything else - when any of them
goes stale past a configurable timeout. It sits between Nav2 and
`cmd_vel_node`, so `cmd_vel_node` subscribes to the watchdog's output rather
than to Nav2 directly.

**Acceptance:** replay a bag, then kill the bag mid-run, and confirm the
watchdog publishes zero within its timeout and latches to stopped.

### 1.D Wi-Fi check script - done

`scripts/check_robot_link.sh`. Run the ground station in `listen` mode, then
the robot in `check` mode:

```bash
./scripts/check_robot_link.sh listen                # ground station
./scripts/check_robot_link.sh check <ground-ip>     # robot
```

It reports, in order, and stops at the first hard failure:

```text
local state       ROS_DOMAIN_ID, RMW, and which interfaces CYCLONEDDS_URI binds
1 reachability    distinguishes client isolation from an unreachable network
2 latency         loss and jitter idle - the baseline never taken on 2026-08-19
3 multicast       ros2 multicast send/receive, the thing discovery needs
4 throughput      iperf3 if present; least important, 0.33 Mbps is all that is
                  needed
```

Step 2 is the measurement that has been missing since the field day. The
recorded 33% loss and 395-1453 ms RTT were taken while a point cloud was being
streamed, so they say the cloud collapses the link and nothing about whether
the link is usable idle.

**Do not run the field day on shared or meeting Wi-Fi.** Client isolation and
multicast filtering are both defaults on such networks and neither can be
worked around from the robot. Use the Go2's own access point, or a small
dedicated router that only the two machines join.

### 1.E RViz configuration for the wireless link - done

`rviz/livox_ground.rviz`, and it is what `livox_ground.launch.py` loads by
default. Eight displays, no PointCloud2 of any kind, adding up to about
0.4 Mbps. The file opens with a comment saying why nobody should add one.

The one that was found while writing it: `/scan` has to be subscribed
BEST_EFFORT, because `pointcloud_to_laserscan` publishes with SensorDataQoS. A
Reliable subscription connects to nothing and displays nothing, with no error -
the same mismatch that made `sensor_watchdog` report a dead LiDAR while the
sensor was healthy.

`rviz/livox_mapping.rviz` still has its two cloud displays and is for running
on the same machine as FAST-LIO, never across the network.

### 1.F AprilTag - reviewed and made to work

The review found the AprilTag path was not merely mismatched with the new frame
tree, it could not run at all. Three things were missing, and each failed
quietly:

```text
apriltag_msgs         not in the Foxy apt repos. april_localizer catches the
                      ImportError, logs once, and then never subscribes - so it
                      starts cleanly and detects nothing, forever.
a detector            ros-foxy-apriltag is the C library only, with no ROS node.
                      apriltag_detect.py existed as an empty 0-line placeholder.
base_link ->          camera.py stamps images camera_link and april_localizer
camera_link           looks that transform up per detection. Nothing published
                      it and the URDF has no camera link, so every detection
                      would be discarded with a TF warning.
```

The last one predates the Livox migration; it was already broken.

**What was done**

`apriltag_msgs` is now cloned into `src/` and built, with its one Focal
incompatibility recorded in `patches/apriltag_msgs/` - upstream requires CMake
3.22 and Focal ships 3.16.3, which fails with no useful output. The board is
also Focal, so the patch is needed there too.

`go2_control/apriltag_detect.py` is the detector, written against `cv2.aruco`
rather than building `apriltag_ros`. OpenCV is already a dependency and has
carried the AprilTag dictionaries since 3.4, which leaves three message
definitions as the only thing built from source - a much smaller thing to get
working on aarch64.

`livox_robot.launch.py` gained an `enable_camera_tf` argument publishing
`base_link -> camera_link`. **It is off by default and its values are
placeholders.** Measure the camera on the robot before enabling it: a wrong
camera pose does not disable AprilTag, it makes AprilTag place the robot
confidently in the wrong place, which is worse than not having it.

`april_localizer`'s scan rotation now publishes to a configurable
`cmd_vel_topic`, defaulting to `/cmd_vel_nav_preview`, instead of straight to
`/cmd_vel`. It was the one path that bypassed `sensor_watchdog`, and a slow
rotate-in-place hunting for a tag is exactly when nobody is watching.

**Verified end to end**, with a synthetic tag, no hardware:

```text
tag 36h11 id 7, 240 px wide, focal 500, tag_size 0.16 m
  implied range   500 * 0.16 / 240        = 0.333 m from the camera
  camera offset   base_link + 0.30 m in x
  tag map pose    (5.00, 2.00, yaw 0)
  robot expected  5.00 - 0.333 - 0.30     = 4.367
  /initialpose    x = 4.37, y = 2.00, frame map
```

That agreement exercises the corner ordering, the camera transform convention,
`solvePnP` and the pose composition together. Corner order is the part worth
guarding: `april_localizer` assumes top-left then clockwise, which is what
`cv2.aruco.detectMarkers` returns, and any other order yields a pose that looks
plausible and is wrong.

**Still open:** the tag family must match the tags physically installed (36h11
is the default here), tag poses have to be surveyed into the site map, and the
camera transform has to be measured.

### 1.G Decide where the camera pipeline runs

```text
on the robot        images never cross Wi-Fi, but the board needs its own
                    internet path for MQTT and pays 40-90% more CPU
on the ground       board stays light and the ground station already has
                    internet; images cross Wi-Fi at roughly 4 Mbps, but only
                    in bursts while the robot is stopped at a waypoint
```

Recommended: **ground station**, because the board has only 4 cores and
AprilTag detection is the heaviest single consumer in the system.

### 1.H Split the launch files along the machine boundary

`livox_amcl.launch.py` currently bundles the TF bridges and
`pointcloud_to_laserscan` together with `map_server` and AMCL. The first two
belong on the robot and the last two on the ground station.

**Do:** regroup what already exists into

```text
livox_robot.launch.py    driver, fastlio, projection, TF, odom relay,
                         cmd_vel_node, watchdog
livox_ground.launch.py   map_server, amcl, Nav2, rviz
```

No new nodes; this is regrouping only. Keep `replay` and `cmd_vel_topic`
arguments and their defaults.

### 1.I Mission from MQTT, against the replayed bag - done

`main.py` needs exactly two things from the navigation stack, the
`navigate_to_pose` action and `/odom`, and both now exist on the Livox path.
Driven end to end with a two-waypoint mission:

```text
MQTT /missions/start   ->  main.py accepts, queues 2 waypoints
                       ->  /missions/status  PENDING, then RUNNING
                       ->  /missions/progress  WP-A, 50%
                       ->  Nav2: "Received a goal, begin computing control effort"
/cmd_vel                   Unknown topic; no cmd_vel_node running
```

There is no broker on this machine and mosquitto needs root, so
`test/mini_mqtt_broker.py` is a 180-line MQTT 3.1.1 broker covering only what
`main.py` uses - CONNECT, SUBSCRIBE, PUBLISH at QoS 0, PINGREQ, DISCONNECT and
the `+`/`#` wildcards. It is a test fixture, not something to run in the field.

```bash
python3 src/go2_control/test/mini_mqtt_broker.py &
ros2 run go2_control main --ros-args \
    -p enable_mqtt:=true -p mqtt_broker:=127.0.0.1 -p mqtt_port:=1883 \
    -p use_sim_time:=true
```

**Not covered by this test:** the photograph upload. The capture spin fires on
arrival at an `isCapture` waypoint, and on a replayed bag the robot never
arrives anywhere, so that branch cannot be reached without either the robot or
a simulated base. It is the one part of the mission loop still unproven.

**Worth knowing before the field day.** Counted at the broker over the run,
`main.py` publishes far more than the mission traffic:

```text
odom    505 messages
map      12 messages, 116 KB raw each, every 10 s
```

The map republish is roughly 12 KB/s sustained to the web on top of everything
else, and the odometry stream is continuous. Neither is a problem on a wired
uplink, and neither crosses the robot-to-ground-station link, but both are
worth checking against whatever connection the site actually has.

## Part 2 - Board Bring-Up (board needed, robot does not walk)

### 2.1 Build the driver

Livox-SDK2, then `livox_ros_driver2` with `./build.sh ROS2`. This alone
unlocks recording bags from a freely walking robot with no network at all.

### 2.2 Build FAST-LIO

With the patch series from 1.B. Budget `-j2`: the MiniPC measured 2825 MB peak
per compiler process, and the board has 13 GB free.

Do **not** bother patching the `MP_PROC_NUM` check. The whole codebase has one
`#pragma omp parallel for`, in `h_share_model`, which the MiniPC measured at
0.1 ms of 11.2 ms. Enabling OpenMP on ARM buys about 1%. The cost is the
single-threaded preprocess loop at 10.0 ms, and the knob for that is
`point_filter_num` in `config/fast_lio2_mid360.yaml`, which is a ROS parameter
and needs no rebuild.

### 2.3 Install the rest from apt

`ros-foxy-navigation2`, `ros-foxy-nav2-amcl`, `ros-foxy-pointcloud-to-laserscan`.
Foxy ships arm64 binaries for Ubuntu 20.04, so nothing here is built.

### 2.4 Measure the board

Replay `02_loop` on the board and record per-process CPU, exactly as was done
on the MiniPC. Reference figures from the MiniPC (i5-1235U), with a goal
active:

```text
planner_server   53.6%    fastlio_mapping          28.8%
controller_serv  53.3%    pointcloud_to_laserscan  18.6%
amcl             45.4%    lio_odom_relay            9.5%
```

**Acceptance:** the robot-side set stays under about 2 of the 4 cores with
Unitree's own stack running, leaving headroom for the leg controller, which
must never be starved.

### 2.5 Time sync

Install and configure chrony on both machines against the same source. Without
it the two clocks drift apart and TF lookups fail with
`Invalid frame ID ... frame does not exist`, which reads like a missing
transform rather than a clock problem.

## Part 3 - Field Day, No Motion

Every step read-only. `enable_cmd_vel:=false`, no `cmd_vel_node` for the
sensor checks, nothing published to `/cmd_vel`.

```text
1. Bring up the dedicated AP. Run the 1.D script. Do not proceed on shared
   Wi-Fi.
2. Confirm chrony has both machines in sync.
3. Robot side up. Check /livox/lidar at 10 Hz, /livox/imu at 200 Hz, /scan
   steady, TF base_link -> livox_frame present.
4. Drive the robot on the handheld remote and record 03_site, a 300 s walk
   over the whole area. This has been outstanding since 2026-08-19.
   Wait for "Subscribed to topic" on both topics before walking away:
   ros2 bag record does not fail on a missing topic, it waits silently and
   writes a 16 KB empty file. Three takes were lost to this.
5. Build the site map with livox_slam.launch.py, or offline from the bag.
6. Ground station up on the site map. Confirm AMCL holds a lock while the
   robot is driven on the remote.
7. Measure loop closure with the sensor mounted: walk a closed loop on the
   remote and read the closing error. This is the number that has been
   deferred through two documents.
8. Put AprilTags up and measure how much they reduce the residual.
9. Send goals from RViz with cmd_vel_topic left at its default. Confirm the
   plan looks sane and that /cmd_vel does not exist:
       ros2 topic info /cmd_vel      expect Unknown topic
       ros2 node list | grep cmd_vel expect nothing
10. Kill the Wi-Fi deliberately and confirm the watchdog stops the robot.
```

## Part 4 - Field Day, Motion

Only after every step of Part 3 passes.

```text
Preconditions   safe open area, operator holding the remote with a thumb on
                the stop, emergency stop reachable, watchdog verified in
                step 3.10, lowest speed configured
Command         one goal, one metre, then stop and assess
Gate            robot_ack = I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
                cmd_vel_topic:=/cmd_vel is what arms the system; it is not a
                convenience setting
```

Only after that behaves: a two-waypoint mission from the web, with the capture
spin and the photo upload, which is the end-to-end goal.

## Open Risks

```text
FAST-LIO will not build on ARM         no fallback now that 1.A is decided;
                                       1.B is the mitigation
Unitree's leg stack competes for CPU   measurable only on the board, 2.4
Site Wi-Fi filters multicast           use the Go2 AP, checked in 1.D
AprilTag benefit is still unmeasured   3.8
Goal tolerance vs localisation         xy_goal_tolerance is 0.25 m while AMCL
                                       leaves 0.41 m mean; re-derive after 3.7
allow_unknown is true in the planner    acceptable while /cmd_vel is off, must
                                       be reconsidered against the site map
```
