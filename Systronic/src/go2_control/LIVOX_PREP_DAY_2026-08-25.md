# Livox Prep Day, 2026-08-25

Everything done at the desk, entirely from the bags recorded on 2026-08-19. No
robot, no board, no site. Nothing published `/cmd_vel`, no `cmd_vel_node` ran,
and `ros2 topic info /cmd_vel` reported the topic did not exist at every check.

Read `LIVOX_DEPLOYMENT_PLAN.md` for what happens next.

## 1. The Day Against The Task List

```text
1. replay bag for prepare-phase testing              done
2. align the simulation topics with the LIO stack    done, but reframed - see 2.2
3. launch with map_server + AMCL + projection,
   AMCL measured against the replay                  done, measured three times
4. TF and the Nav topics, and talking to the
   Go2 movement SDK                                  TF and topics done.
                                                     SDK NOT done - needs the robot
5. tune the costmap, send goals, verify the path
   in RViz with /cmd_vel disconnected                done, two real defects found
6. deploy scripts for the board and onsite config    partly - see 6 below
```

Two of six are incomplete, and both for the same reason: there is no board and
no robot on the desk.

### 2.2 "Align the sim topics" turned into something else

The existing simulation is a TurtleBot in Gazebo publishing `/scan` directly.
Making that resemble the Livox stack would have meant faking a Mid-360 cloud
into Gazebo and running FAST-LIO on it, which tests the LiDAR path that the
replay bags already test better, using real sensor data.

What was actually missing was the opposite end: **a robot that moves**. A
replayed bag proves sensing and localisation, but the robot in a bag has
already finished walking and never arrives anywhere, so everything that fires
on arrival was untestable - reaching a waypoint, the capture spin, the
photograph upload, a mission completing.

`go2_control/fake_base.py` fills that. It integrates the velocity Nav2 sends
into a pose and ray-casts the occupancy map to produce `/scan`, so Nav2 drives,
the robot moves, AMCL follows, and a mission runs end to end - on the real map,
with the real launch files, at the desk.

## 2. What Now Works That Did Not This Morning

```text
AMCL against the replay      0.353-0.409 m mean correction, lock held 180 s
Nav2                         plans, and now drives - a goal returned SUCCEEDED
mission from MQTT            two waypoints, both reached, progress to 100
AprilTag                     works at all, for the first time
sensor watchdog              stops the robot when /scan or /odom goes stale
cmd_vel staleness            was dead code; now stops the robot in 0.47 s
machine split                one launch file per machine
```

## 3. Six Defects Found, All Silent

Every one of these produces a system that looks like it is working, or that
fails somewhere far from the cause.

**`cmd_vel_node` had no timeout at all.** The staleness check was commented
out, and uncommenting it would not have worked - a second `def update` would
have replaced the first and removed the `Move()` call, leaving a node that
could only ever stop the robot. Merged. A dropped link now stops the robot in
0.47 s instead of repeating the last velocity forever.

**A watchdog on sim time can never fire.** rclpy drives timers from `/clock`,
which on a replay comes from the same bag that feeds the sensors. When the data
stops, time stops, the timeout never elapses, and the node's own log freezes at
the moment it was needed. `sensor_watchdog` runs on wall time for that reason.

**`/scan` is BEST_EFFORT.** `pointcloud_to_laserscan` publishes with
SensorDataQoS. A RELIABLE subscriber never connects and never receives, with no
error - the watchdog reported a dead LiDAR while the sensor was healthy, and
would have stopped the robot in the field for no reason.

**Nav2 could not command forward motion.** `max_speed_xy` was absent from the
DWB parameters, and an unset DWB kinematic limit is zero, not "no limit". DWB
emitted 383 consecutive commands with `vx` exactly 0.0 while rotating happily;
the progress checker then failed and Nav2 span through recovery after recovery.
Nothing logged anything about a velocity limit.

**Unknown map space loaded as free.** `map_saver_cli` writes
`free_thresh: 0.25`, and an unknown pixel is 205, an occupancy of 0.196 - below
the threshold, so `map_server` calls it free. This map is about 65%
unobserved, so Nav2 was treating two thirds of it as known drivable floor.
Fixed to 0.19 in `map/livox_slam_02loop.yaml`. **Check this on every map
`map_saver_cli` writes.**

**AprilTag could not run.** `apriltag_msgs` is absent from Foxy and the node
catches the ImportError and continues; there was no detector, the local
`apriltag_detect.py` being an empty file; and nothing published
`base_link -> camera_link`, so every detection would have been discarded. All
three addressed - see `LIVOX_DEPLOYMENT_PLAN.md` section 1.F.

## 4. Two Environment Traps

**Stale DDS shared memory degrades discovery.** After a day of starting and
killing nodes, `/dev/shm` held 192 orphaned `fastrtps_*` segments and new
processes stopped discovering existing ones: a node's subscription appears in
`ros2 topic info -v` with matching QoS and its callback never fires, and even
service calls hang. It also recurs within minutes of heavy use.

```bash
# with no ROS process running
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

Expect this on the board, where nodes will be restarted repeatedly during
bring-up. If a node cannot see something that is plainly running, clean
`/dev/shm` before debugging anything else.

**`pkill -f` kills the shell running the command.** Already recorded in
`LIVOX_FIELD_DAY_2026-08-19.md` section 9, and hit again today: `pkill -f
"broker.py"` matched the shell whose command line contained that string and
took the session with it, losing an unsaved file. Kill by PID.

## 5. Decisions That Changed

**Not everything runs on the board.** The bench day recorded "run everything on
the Unitree board" as locked. Measured with a goal active, the navigation stack
alone costs 2.2 cores on an i5-1235U, which is 4 to 7 on a 4-core ARM board
before the camera, AprilTag, `main.py` and Unitree's own leg control. The board
runs the sensor, odometry and projection; a ground station runs localisation
and planning. Only `/scan`, `/odom`, `/tf` and velocity cross the link, about
0.33 Mbps against the 41.6 Mbps of raw cloud that failed on 2026-08-19.

**AprilTag is required, not optional.** Both earlier documents deferred this
pending a measurement "once the sensor is bolted to the robot". That
measurement had already happened - see section 6.

**FAST-LIO owns odometry.** `go2w_read` also publishes `/odom` and broadcasts
`odom -> base_link` from leg odometry. Two publishers of one transform break
the TF tree silently, with tf2 keeping whichever arrived last. `go2w_read` now
takes `publish_odom` and `publish_tf`, both false on this stack.

**A better map did not improve AMCL.** A `slam_toolbox` map of the same bag has
89.8 m2 of free space against `pcd_to_map`'s 36.2 m2, because it ray-traces
rather than flood-filling a guess. AMCL's mean correction went from 0.359 m to
0.409 m on it. The old map was built from the same drifting trajectory that
AMCL is measured against, so its errors cancelled; the better map exposes the
real drift. Both maps are kept.

**CycloneDDS works.** Everything was measured on FastRTPS while Unitree's stack
uses CycloneDDS. Re-run under CycloneDDS: 0.353 m mean, 0 scans dropped,
indistinguishable from FastRTPS run-to-run variation.

## 6. The Record Was Wrong About How The Data Was Captured

Three documents described `02_loop` as a handheld walk. The operator confirmed
the sensor was mounted on the Go2 for every recording and the robot did the
walking. The data agrees: 0.43 m above the floor is a standing Go2, not chest
height, and the fitted tilt of +10.26 degrees is close to the published 13.

This was load-bearing. Two measurements had been excused by it - FAST-LIO
closing a 35 m loop 1.65 m out, and AMCL leaving 0.4 m of residual - on the
grounds that a carried sensor shakes and that a fixed levelling rotation only
approximates a hand. Neither survives: the rotation is exact for a bolted
mount. Both numbers describe the deployed configuration. The candidate that
does not disappear is the gait, since a trotting quadruped pitches and rolls
with every step and a fixed levelling rotation cannot follow that.

Corrections are marked in place in the three affected documents.

## 7. Still Open

```text
Go2 movement SDK              never exercised. cmd_vel_node was reviewed and
                              its watchdog fixed, but nothing has talked to the
                              SDK. Needs the robot.
deploy scripts                the patch series and READMEs are build
                              instructions, and check_robot_link.sh is an
                              onsite tool, but there is no scripted deploy.
                              Worth writing once the board's paths are known.
photograph upload             the capture spin runs and completes; no image was
                              published because main.py selects its camera
                              topic by sim_mode and the test fed the wrong one.
                              One re-run away.
board build                   nothing has ever been built for aarch64
Wi-Fi baseline                still never measured idle
```

## 8. Artefacts

```text
launch/livox_robot.launch.py       everything that runs on the board
launch/livox_ground.launch.py      everything that runs on the ground station
launch/livox_slam.launch.py        mapping with ray tracing
go2_control/fake_base.py           a robot that exists only in the map
go2_control/sensor_watchdog.py     stops the robot when the sensors stop
go2_control/apriltag_detect.py     cv2.aruco detector
go2_control/lio_odom_relay.py      /Odometry -> /odom with a real twist
go2_control/bag_clock.py           /clock from bag stamps, Foxy has no --clock
go2_control/frames.py              mount arithmetic, shared by both launches
config/nav2_livox_go2.yaml         Foxy Nav2 params with the full DWB set
config/amcl_livox.yaml             AMCL params in the Foxy spelling
config/slam_livox.yaml             slam_toolbox
rviz/livox_ground.rviz             no PointCloud2, for the wireless link
scripts/check_robot_link.sh        isolation, multicast, jitter, throughput
patches/fast_lio/                  six Foxy fixes, verified against a clean clone
patches/apriltag_msgs/             CMake minimum for Focal
test/mini_mqtt_broker.py           MQTT subset, for mission tests
test/test_cmd_vel_watchdog.py      staleness test with a stubbed SDK
map/livox_slam_02loop.*            ray-traced map, 89.8 m2 free
```
