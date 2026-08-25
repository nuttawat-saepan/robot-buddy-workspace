# AMCL Against The Replayed Bag, 2026-08-25

Item 1 of the next-steps list in `LIVOX_MID360_MIGRATION.md` section 10: one
launch file carrying map_server, AMCL and the 2D projection, run against
`02_loop` replayed from disk, and a measurement of how far AMCL has to move the
FAST-LIO pose to make it agree with the map.

No hardware. No robot. Nothing published `/cmd_vel`, no `cmd_vel_node` ran, and
nothing in the new launch file can start one.

Read `LIVOX_BENCH_DAY_2026-08-20.md` first: the map, the tilt figures and the
loop-closure error this is measured against all come from there.

## 1. The Headline

AMCL localises against the `livox_02loop` map for the whole 180 s run. It never
lost the pose, never needed a reset, and never diverged. It also never got the
error down to where a 2D Nav2 stack would like it.

```text
run                 02_loop, 179.7 s, replayed at 1x
scans dropped       0
TF lookup failures  0
/Odometry           1792 messages
/amcl_pose          161 samples
path walked         33.74 m
```

Correction AMCL applied to the FAST-LIO pose:

```text
mean                0.359 m
median              0.348 m
rms                 0.423 m
p95                 0.772 m
max                 0.871 m   at t = 50.6 s
final               0.648 m   = 1.92% of path walked
```

Heading correction:

```text
mean absolute       5.51 deg
max absolute       17.49 deg  at t = 47.3 s
final              +0.17 deg
```

## 2. What The Number Means, And What It Does Not

The obvious reading of "how far is `/amcl_pose` from `/Odometry`" is a trap.
AMCL does not produce a pose independently of odometry and then get compared
against it - it estimates the `map->odom` correction, and publishes that
correction composed with the odometry:

```text
amcl_pose = (map->odom) * (odom->base_link)
```

Subtracting the odometry from that comes out as zero by construction, no matter
how badly the odometry has drifted.

What is measured above is the correction itself: the LIO pose expressed in the
frame the map was built in, against the pose AMCL publishes. That is how far
the odometry has walked away from the map, and it is only a meaningful quantity
because `pcd_to_map` rasterised the map straight out of FAST-LIO's own levelled
world frame, so an undrifted run would hold the correction at zero from start
to finish.

`amcl_drift_check` takes `odom->base_link` from TF rather than recomputing it
from `/Odometry`, deliberately: that is the same chain AMCL itself consumed, so
a static bridge wired up wrongly shows up as a large correction instead of
cancelling out on both sides of the subtraction.

## 3. Where The Error Comes From

Split by 30 s window:

```text
  window        mean     max    walked
   0- 30s      0.199   0.370    4.95 m
  30- 60s      0.623   0.871    9.22 m     <- the excursion
  60- 90s      0.211   0.626   15.11 m
  90-120s      0.357   0.659   21.36 m
 120-150s      0.325   0.661   27.22 m
 150-180s      0.492   0.804   33.64 m
```

The error is not a steady accumulation. It is one excursion between 30 s and
60 s that AMCL then pulls back in, and the heading error peaks in the same
window - 17.5 degrees at t = 47.3 s against a 5.5 degree average. A filter that
was simply being dragged by drift would not recover; this one does, twice.

The leading explanation is the one thing about this dataset that will not be
true on the robot. `02_loop` was carried by hand, and the `body -> base_link`
bridge levels the sensor by a *fixed* rotation measured at the start of the
run. That is exact for a sensor bolted to a frame and only approximate for a
sensor held in a hand: when the operator tips the sensor, `base_link` tips with
it, the horizontal band `pointcloud_to_laserscan` cuts tips out of level, and
the scan AMCL correlates stops being a slice of the same room the map is a
slice of. A fast turn while walking is exactly where that would be worst.

So the honest reading is: the pipeline works and holds a lock, and the residual
is not yet attributable between odometry drift and handheld tilt. Bolting the
sensor to the robot removes the second, and that is the measurement that
decides the AprilTag question.

## 4. Against The Bench-Day Question

`LIVOX_BENCH_DAY_2026-08-20.md` section 5 set the test: FAST-LIO closed the
35 m loop 1.65 m out, 4.72%, and the question was whether AMCL alone could
absorb that or whether AprilTag becomes a requirement.

```text
FAST-LIO closing error, alone      1.65 m   4.72% of path
residual after AMCL, final         0.648 m  1.92% of path
worst instantaneous residual       0.871 m  2.58% of path
```

AMCL removes about 60% of the closing error and holds the rest to under a
metre. That is enough to keep a costmap roughly aligned with the world and not
enough to trust a 1 m goal tolerance. It does not settle the AprilTag question
either way, and it should not be read as settling it, because section 3's
handheld-tilt term is in every one of these numbers and will not be present on
the robot.

The measurement that settles it is unchanged and still first in the queue:
re-measure loop closure with the sensor bolted on, then re-run exactly this
replay procedure against a map built from that data.

## 5. The Defect Found On The Way

The first run of this produced no `/amcl_pose` at all. AMCL logged, for every
single scan:

```text
Message Filter dropping message: frame 'base_link' ... for reason 'Unknown'
```

and said nothing else. The cause is a Foxy limitation crossed with an upstream
node's choice of timestamp:

* `ros2 bag play` did not gain `--clock` until Galactic. On Foxy the bag
  replays its recorded stamps while every node reads the wall clock.
* `pointcloud_to_laserscan` stamps `/scan` with `now()` rather than copying the
  stamp of the cloud it projected.

Together those put `/scan` six days in the future relative to FAST-LIO's
`camera_init -> body` transform, which carries the bag's own 2026-08-19 stamps.
Every scan was unlocalisable in time and every scan was dropped.

Two things make this worth writing down beyond the fix. The failure is silent
in the direction that matters - `/scan` publishes at a healthy 10 Hz, the map
loads, the lifecycle manager reports all nodes active, and the only symptom is
a pose topic that stays empty. And the log line names a frame, which sends you
looking at the TF tree, when the problem is the clock.

The fix is `go2_control/bag_clock.py`, which republishes the bag's own header
stamps as `/clock`, plus `use_sim_time` on the nodes that need it. Both are
wired to the `replay` argument, which defaults to true:

```bash
ros2 launch go2_control livox_amcl.launch.py              # replay, clock on
ros2 launch go2_control livox_amcl.launch.py replay:=false  # on the robot
```

After the fix, 0 scans dropped across the whole run.

## 6. A Rule For Replay Runs On This Machine

A first attempt at the full measurement produced a path length of 211 m for a
walk that was 33.7 m, and a correction that jumped 190 m in one step. The cause
was a leftover `fastlio_mapping` from an earlier attempt still publishing
`/Odometry` and `camera_init -> body` alongside the new one. Two LIO instances
on the same topics do not error; they interleave, and `/Odometry` simply
arrives at 20 Hz instead of 10.

The tell is in the data rather than in any log: `/Odometry` message count
should match the bag's `/livox/lidar` count. `02_loop` holds 1797 lidar frames
and the clean run recorded 1792 odometry messages. The contaminated run
recorded twice that.

Count the nodes before playing the bag, every time:

```bash
ps -eo pid,comm | awk '$2 ~ /fastlio|amcl|map_serv|pointcloud|bag_clock/'
```

This is the same class of mistake as the `pkill -f livox_ros_driver2_node`
entry in `LIVOX_FIELD_DAY_2026-08-19.md` section 9: killing ROS processes by
pattern from a shell is unreliable, and what survives is not obvious afterwards.

## 7. How To Re-Run It

Four terminals, each sourced with:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_fastlio_livox/install/setup.bash
source install/setup.bash
```

```bash
# 1. FAST-LIO. Its own tf bridge stays off: livox_amcl.launch.py owns the two
#    static transforms and needs the tilt of the bag, not the robot mount pose.
ros2 launch go2_control livox_mid360_lio.launch.py \
    enable_driver:=false enable_tf_bridge:=false

# 2. Localisation. Defaults are the 02_loop tilt and the 02_loop map.
ros2 launch go2_control livox_amcl.launch.py \
    drift_csv:=/tmp/drift.csv

# 3. Confirm exactly one of each node before playing anything
ps -eo pid,comm | awk '$2 ~ /fastlio|amcl|map_serv|pointcloud|bag_clock/'

# 4. Play
ros2 bag play ~/livox_bags_field/02_loop
```

Ctrl-C terminal 2 when the bag ends; `amcl_drift_check` prints its summary
during shutdown.

On the robot, with a real map and the sensor bolted on:

```bash
ros2 launch go2_control livox_amcl.launch.py \
    replay:=false map:=/abs/path/site_map.yaml \
    lio_pitch_deg:=13.0 lio_roll_deg:=0.0 sensor_height:=0.35
```

`lio_pitch_deg`, `lio_roll_deg` and `sensor_height` must match the values the
map was built with by `pcd_to_map`. They decide which horizontal band of the
cloud becomes `/scan`, and AMCL is correlating that band against the same band
of the map. Mismatched, the scan is a slice of a different room.

## 8. Files Added

```text
launch/livox_amcl.launch.py        map_server + AMCL + projection + tf bridges
config/amcl_livox.yaml             AMCL params, Foxy spelling
pc2scan_livox_lio.yaml             projection of /cloud_registered_body
go2_control/amcl_drift_check.py    the measurement
go2_control/bag_clock.py           /clock from bag stamps, the missing --clock
```

Note on `config/amcl_livox.yaml`: Foxy's nav2_amcl is 0.4.7 and takes
`robot_model_type: "differential"`. The
`"nav2_amcl::DifferentialMotionModel"` spelling in this package's
`nav2_params.yaml` is the Galactic-and-later form and is rejected here. There
is no omnidirectional model on Foxy, which is the wrong model for a handheld
walk; the alphas are raised to 0.4 to compensate and should be dropped back to
0.2 once the motion really is a differential base.

## 9. Still Open

```text
1. Nav2 with the Go2 footprint, goals from RViz, /cmd_vel disconnected.
   Needs no hardware. Next item.
2. Sensor onto the Unitree board, fast_lio_livox built for aarch64.
3. Re-measure loop closure bolted to the robot, then re-run section 7 against
   a map from that data. This is what decides whether AprilTag is optional.
4. Map the actual site, then AMCL on it, then Nav2 on it.
5. Only then /cmd_vel, with the robot_ack gate.
```
