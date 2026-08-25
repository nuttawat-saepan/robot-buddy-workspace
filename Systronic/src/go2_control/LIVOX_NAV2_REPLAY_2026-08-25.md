# Nav2 Against The Replayed Bag, 2026-08-25

Item 2 of the next-steps list: Nav2 with the Go2 footprint, goals sent while
`/cmd_vel` stays disconnected. Done on the same replayed `02_loop` data as the
AMCL work in `LIVOX_AMCL_REPLAY_2026-08-25.md`, which should be read first.

No hardware, no robot, no motion. `/cmd_vel` did not exist as a topic at any
point during the test - see section 2.

## 1. Result

```text
lifecycle bringup       Managed nodes are active
navigate_to_pose        goal accepted
plan produced           218 poses, 5.48 m, frame map
plan endpoint           (3.13, -1.49) against a goal of (3.13, -1.69)
/odom                   published from the FAST-LIO pose, 1 publisher
costmaps                local and global both populated from /scan
```

The endpoint sits 0.20 m from the requested goal because NavfnPlanner is
configured with `tolerance: 0.5` and snaps to the nearest free cell.

On a replayed bag the robot never actually moves, so the goal is never reached
and the progress checker eventually aborts. That is expected. What is being
verified is that a plan is produced, the costmaps populate, and the TF chain
holds end to end - not that navigation completes.

## 2. /cmd_vel Is Disconnected By Construction, Not By Care

Nav2's `controller_server` and its spin/backup/wait recoveries publish velocity
on the relative topic `cmd_vel`. Left alone that resolves to `/cmd_vel`, and
the moment a goal is accepted this stack would be commanding the robot.

`livox_nav2.launch.py` remaps all of them to `cmd_vel_topic`, which defaults to
`/cmd_vel_nav_preview` - a topic nothing subscribes to. Measured during the
run:

```text
/cmd_vel                Unknown topic '/cmd_vel'      <- never even created
/cmd_vel_nav_preview    4 publishers, 0 subscribers
cmd_vel_node            not running
```

Four publishers because `controller_server` and the three recovery plugins each
create one.

The launch file prints which of the two it is on startup, so a terminal read
months later does not have to infer whether that run could move the robot:

```text
velocity output goes to /cmd_vel_nav_preview, which nothing subscribes to -
Nav2 plans but commands nothing
```

Pointing `cmd_vel_topic` at `/cmd_vel` is what turns this into a system that
moves a real robot, and belongs to the final movement stage only.

## 3. Three Foxy Traps, All Of Which Fail Quietly

### `nav2go2_params.yaml` cannot be used on Foxy

It is written for a much newer Nav2: `nav2_mppi_controller::MPPIController`,
`nav2_smac_planner/SmacPlanner2D`, `nav2_costmap_2d::DenoiseLayer`,
`behavior_server`, `smoother_server` - none of which exist in Foxy. It also
still carries a literal `<back_obstacle_layer>` placeholder in two plugin
lists. `config/nav2_livox_go2.yaml` is derived from `nav2_params_go2w.yaml`
instead, which is genuine Foxy: DWB, NavfnPlanner, `recoveries_server`.

This is the same class of problem as the AMCL `robot_model_type` spelling in
`nav2_params.yaml`. Assume any Nav2 parameter file in this package targets a
different distro until checked.

### An explicitly named node section beats the launch file's override

This one cost the most time and is worth knowing generally.

`launch_ros` writes parameter dictionaries into a temporary file under the
`/**` wildcard. rcl gives an **explicitly named node section precedence over
`/**` regardless of the order the parameter files are passed in**. So this,
in a checked-in parameter file:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
```

silently beats `parameters=[params_file, {'use_sim_time': True}]` in the launch
file, and no launch argument can change it.

The symptom was not a clock complaint. Nav2 ran on the wall clock while the TF
tree carried the bag's timestamps, and reported:

```text
Invalid frame ID "odom" passed to canTransform argument target_frame -
frame does not exist
```

which reads like a missing transform. The static bridges were publishing
`odom` correctly the whole time.

Both `config/nav2_livox_go2.yaml` and `config/amcl_livox.yaml` therefore set
`use_sim_time` nowhere at all; the launch files own it, driven by their
`replay` argument. Same reasoning applies to `default_bt_xml_filename`, which
also has to come from the launch file.

### `bt_navigator` needs a full path, and the plugin list must be complete

Foxy's `bt_navigator` opens `default_bt_xml_filename` as given rather than
resolving it against the `nav2_bt_navigator` share directory. A bare filename
fails during configure with `Couldn't open input XML file` and aborts the whole
lifecycle bringup. `livox_nav2.launch.py` resolves it.

Then the trimmed `plugin_lib_names` inherited from `nav2_params_go2w.yaml`
turned out to be missing `nav2_rate_controller_bt_node`, while the stock
`navigate_w_replanning_and_recovery.xml` uses a `RateController` on line 9:

```text
Node not recognized: RateController
```

The full Foxy list is now used. Loading a plugin a tree never uses costs
nothing; missing one is fatal, so the list stays complete rather than tailored.

## 4. /odom Had To Be Synthesised

`main.py`, `bt_navigator` and `controller_server` all want `nav_msgs/Odometry`
on `/odom` in `odom -> base_link`. FAST-LIO publishes `/Odometry` in
`camera_init -> body`, which is neither the right frames nor the right pose.

`go2_control/lio_odom_relay.py` reads `odom -> base_link` out of TF rather than
redoing the mount arithmetic, so there is one definition of the mount pose - in
`livox_amcl.launch.py` - and this node cannot disagree with it.

The velocity is differentiated from consecutive poses, not copied. Upstream
FAST-LIO never assigns `odomAftMapped.twist` anywhere in `laserMapping.cpp`, so
`/Odometry` carries an all-zero twist. Passing that straight through would
leave Nav2's velocity feedback permanently reading "stopped", which both the
progress checker and DWB consume. Measured during the replay, the
differentiated velocity tracked the walk sensibly:

```text
v = (+0.18, +0.04) m/s   w = +0.42 rad/s
```

## 5. Known Weakness Of This Test

The bench map is mostly unknown space:

```text
free       7.4%
occupied   9.2%
unknown    rest
```

`pcd_to_map` can only mark free what its flood fill reached. So
`planner_server` is configured with `allow_unknown: true`, without which almost
no goal on this map is reachable - and that means the plan above routes partly
through cells nothing ever observed.

That is acceptable while `/cmd_vel` is disconnected and a human is reading the
path off RViz. It is **not** acceptable for autonomous movement on a real site,
and `allow_unknown` has to be reconsidered against a proper site map rather
than inherited from this file.

The goal tolerance is a second open point. `xy_goal_tolerance` is 0.25 m while
AMCL leaves 0.648 m of residual on this data. A tolerance tighter than the
localisation residual cannot be met except by luck. Both numbers should be
re-derived once loop closure is measured with the sensor bolted to the robot.

## 6. How To Re-Run It

Four terminals, each sourced with:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_fastlio_livox/install/setup.bash
source install/setup.bash
```

```bash
# 1
ros2 launch go2_control livox_mid360_lio.launch.py \
    enable_driver:=false enable_tf_bridge:=false
# 2  map, AMCL, /scan, TF bridges, and the /clock the replay needs
ros2 launch go2_control livox_amcl.launch.py enable_drift_check:=false
# 3  /odom and Nav2, velocity going nowhere
ros2 launch go2_control livox_nav2.launch.py enable_rviz:=true
# 4
ros2 bag play ~/livox_bags_field/02_loop
```

Confirm exactly one of each node before playing anything - see section 6 of
`LIVOX_AMCL_REPLAY_2026-08-25.md` for why:

```bash
ps -eo pid,comm | awk '$2 ~ /fastlio|amcl|map_serv|pointcloud|bag_clock|controller_s|planner_s/'
```

Then confirm the safety state, every time, before sending a goal:

```bash
ros2 topic info /cmd_vel        # expect: Unknown topic '/cmd_vel'
ros2 node list | grep cmd_vel   # expect: nothing
```

Send a goal from RViz with "2D Goal Pose", or:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, \
    orientation: {w: 1.0}}}}"
```

## 7. Files Added

```text
launch/livox_nav2.launch.py        Nav2 servers, cmd_vel remapped away
config/nav2_livox_go2.yaml         Foxy-compatible Nav2 params, Go2 footprint
go2_control/lio_odom_relay.py      /Odometry -> /odom with a real twist
```

## 8. What This Unblocks

`main.py` needs exactly two things from the navigation stack: the
`navigate_to_pose` action and `/odom`. Both now exist on the Livox stack, so
the next step is connecting `main.py` itself - the MQTT mission interface, the
waypoint queue and the photo upload that are already written - and running a
mission end to end against the replayed bag with `/cmd_vel` still disconnected.

Still open before the robot moves:

```text
1. main.py against the Livox stack, mission driven from MQTT, /cmd_vel off.
2. AprilTag: check april_localizer.py against the camera_init/body/base_link
   tree. It is a requirement of the system, not a fallback.
3. A watchdog that stops the robot when /scan or /Odometry goes stale.
   Nothing in this workspace does that yet.
4. Sensor onto the Unitree board, fast_lio_livox built for aarch64.
5. Re-measure loop closure bolted to the robot; re-derive goal tolerance and
   allow_unknown from a real site map.
6. Only then /cmd_vel, with the robot_ack gate.
```
