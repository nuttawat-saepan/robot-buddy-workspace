# What Actually Moves AMCL's Error, 2026-09-03

Three things were varied against the same replayed bag and measured the same
way, to find out which of them is worth spending field time on. Two of the
three results contradict what the planning documents assumed.

No hardware. No robot. Nothing published `/cmd_vel`, no `cmd_vel_node` ran, and
`ros2 topic info /cmd_vel` returned `Unknown topic` throughout.

## Method

`02_loop` replayed from disk at 1x, 179.7 s, 33.7 m walked, every run. The
measurement is `amcl_drift_check`: how far AMCL has to move the FAST-LIO pose
to make it agree with the map, sampled at every `/amcl_pose`. Read
`LIVOX_AMCL_REPLAY_2026-08-25.md` section 1 for why that is the number worth
measuring rather than the difference between two poses.

Two or three runs per condition, because a single run is not enough to separate
an effect from run-to-run variation. That variation turned out to matter: the
spread between repeats is quoted alongside every mean below, and in one case it
is larger than the effect being measured.

**Two traps in running this at all**, both of which produced hours of
meaningless numbers before they were spotted:

FAST-LIO diverges when starved of CPU. It does not slow down - it loses the
buffer (`lidar loop back, clear buffer`), and the pose runs away to tens of
thousands of metres while every process stays alive and every topic keeps
publishing. Leftover `static_transform_publisher` processes from earlier runs
had the load average at 34 on a 12-core machine. Every run below waits for the
load to fall under 3.0 before starting.

Deleting `/dev/shm/fastrtps_*` while ROS processes are alive breaks the shared
memory transport for every participant that was using it. The symptom is a
topic that has a publisher and delivers nothing, with no error anywhere. All
runs below use `fastdds_udp_only.xml`, which is what that file exists for.

## 1. AMCL alphas: no better on average, better at the peak

`alpha1` and `alpha4` were lowered from 0.4 on 2026-09-01 on the reasoning that
0.4 was chosen to keep the filter from being over-confident on a replay, and
that FAST-LIO's odometry on the robot is far better than that. Three runs per
condition:

```text
                  alpha 0.4/0.4    alpha 0.10/0.08
mean                  0.407 m          0.398 m
median                0.393 m          0.413 m
rms                   0.483 m          0.442 m
p95                   0.896 m          0.669 m
max                   1.122 m          0.737 m
yaw max abs            19.15 deg       13.20 deg
```

The mean does not move: 0.009 m apart, against a run-to-run spread of 0.111 m.
Anyone reading only the mean would conclude the change did nothing.

The peak does move, and cleanly. Max falls 34%, and the three runs per
condition do not overlap - 1.09/1.11/1.17 against 0.58/0.67/0.96. p95 falls
from 0.896 to 0.669.

That is the result worth having, because a large correction does not arrive
gradually. It arrives as a jump, the robot appears to teleport in the frame
Nav2 is planning in, and the controller lurches. Halving the worst jump is
worth more than moving an average that was never the thing causing trouble.

`update_min_a` went 0.20 -> 0.10 in the same change, visible as 162 -> 226
`/amcl_pose` samples over an identical walk.

**Keep the change.**

## 2. Mount tilt: the physical value beat the fitted one

`LIVOX_BENCH_DAY_2026-08-20.md` reports a mount pose fitted from the `02_loop`
cloud - pitch +10.26 deg, roll +1.72 deg, sensor 0.43 m up - and `livox_02loop`
was rasterised with those numbers. The operator states the bracket was set to
13 degrees for every bag recorded, without variation.

The standing assumption in this repository has been that run-time values must
equal the values the map was built with. Two runs each:

```text
                  fit 10.26/1.72/0.43   nominal 13.0/0.0/0.35
mean                   0.312 m               0.258 m
rms                    0.359 m               0.296 m
p95                    0.559 m               0.463 m
max                    0.575 m               0.494 m
yaw mean abs            4.05 deg              3.13 deg
```

The nominal values win on every measure, by about 17%, and the repeats do not
overlap (0.306/0.319 against 0.248/0.268).

So the assumption is wrong, or at least incomplete. The fitted pose is an
inference from one cloud and absorbs whatever else was happening - a floor that
is not level, the robot's own pitch while walking, acceleration during the
run. The bracket angle is a fact about the rig. On this data the fact beats the
inference.

**Use 13.0 / 0.0 / 0.35.** `scripts/onsite.env` already does.

## 3. Editing the map by hand made AMCL worse, twice

`HESAI_GO2_FIELD_CHECKLIST.md` and the onsite plan both carry the idea that the
map should be retouched so AMCL can lock onto it better. Four maps of the same
room, same bag, same tilt, two runs each:

```text
                 livox_02loop   slam raw    edit 08-31   edit2 09-04
mean                0.258 m      0.271 m      0.365 m      0.594 m
p95                 0.463 m      0.640 m      0.774 m      1.196 m
max                 0.494 m      0.727 m      0.919 m      1.334 m
yaw mean abs         3.13 deg     7.10 deg     9.93 deg    16.35 deg
```

The order is exactly the order of how much hand editing each map received.
`edit2` is 2.2x worse than the unedited SLAM map it was made from, and its yaw
error is 2.3x worse.

It is also unstable. `edit2` gave 0.698 and 0.491 on two identical runs, 40%
apart; the unedited map gave 0.256 and 0.286, 10% apart. A map that AMCL can
sometimes use and sometimes cannot is worse than one that is merely imperfect.

The mechanism is straightforward once the numbers are in front of you. Editing
means drawing walls that are not there - closing a doorway the sensor saw
through, filling a gap at a glass panel. The real scan still goes through those
places. Every time the robot faces one, AMCL is handed a beam saying "clear"
against a map saying "wall". More editing, more contradiction.

**The value of editing a map is to Nav2, not to AMCL.** Sealing a leak stops
the planner routing through a wall into unobserved space. That is a real need
and a different one. If both are wanted, run two files: the raw map for
`amcl_params_file`, the edited one for the Nav2 global costmap, with identical
origin and resolution.

**Do not spend field time editing maps to help localisation.** The onsite plan
item that says otherwise should be struck.

## 4. Where this leaves the error

Best configuration measured: 0.258 m mean, 0.494 m max, over a 33.7 m walk.
From 0.359 m when the Livox AMCL work started on 2026-08-25, so 28% better.

Parameter tuning is close to exhausted as a route to lower numbers. Between
them the three experiments above moved the mean by about 0.10 m, and two of
the three moved it by changing something that was simply set wrongly rather
than by tuning at all.

`alpha2` and `alpha3` are still at 0.4 and have never been tried. That is the
one untested knob left, and on the evidence of `alpha1`/`alpha4` it is likely
to affect the peak rather than the mean.

What 0.258 m is and is not good enough for is a question about clearance, not
about a threshold. The Go2 body is 0.70 x 0.31 m:

```text
open floor, single goal, 0.05 m/s      fine
corridor 1.5 m wide                    fine, 0.60 m clearance each side
doorway 0.9 m wide                     marginal, 0.30 m clearance each side
stopping accurately at a capture point  not good enough - photographs would be
                                        framed 0.26 m off, every time
```

The mission described in the project goal - a waypoint list with capture points
- is the last row. AprilTag is the plan for that, and these numbers are the
argument for treating it as required rather than optional.

## Reproducing any of this

```bash
ros2 launch go2_control livox_mid360_lio.launch.py \
    enable_driver:=false enable_tf_bridge:=false
ros2 launch go2_control livox_amcl.launch.py \
    map:=livox_02loop.yaml \
    lio_pitch_deg:=13.0 lio_roll_deg:=0.0 sensor_height:=0.35 \
    enable_drift_check:=true drift_csv:=/tmp/run.csv
ros2 bag play ~/livox_bags_field/02_loop
```

No `--clock`: Foxy's rosbag2 does not have it. `bag_clock`, started by the
launch file when `replay` is true, publishes `/clock` from the message stamps
instead.

Two runs per condition is the minimum. The alpha result above would have read
as noise from one run, and the map result would have read as twice its true
size.
