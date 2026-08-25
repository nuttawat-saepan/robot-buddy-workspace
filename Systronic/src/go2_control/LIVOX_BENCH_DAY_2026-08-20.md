# Livox Mid-360 Bench Day, 2026-08-20

What was done the day after the field session, working entirely from the bags
recorded on site. No robot, no site access, no sensor for most of it. Read
`LIVOX_FIELD_DAY_2026-08-19.md` first for how the data was captured, and
`LIVOX_MID360_MIGRATION.md` sections 8c to 8e for the technical detail behind
the numbers here.

No robot motion. Nothing published `/cmd_vel` and no `cmd_vel_node` ran.

> **Correction, 2026-08-25.** Passages in this file describing `02_loop` as a
> handheld walk are wrong. The operator confirmed that the sensor was mounted
> on the Go2 for every recording and the robot did the walking; it was never
> carried. The data agrees: the sensor sat 0.43 m above the floor at the start,
> which is a standing Go2 rather than chest height, and the fitted tilt of
> +10.26 degrees is close to Unitree's published 13 degree mount pose. Where
> the text below blames a measurement on hand shake or on a tilt that will not
> be present once the sensor is bolted on, that explanation does not hold and
> the number should be read as a real property of the robot configuration.


## 1. The Point Of The Day

Answer as much as possible about the autonomy stack without the robot, so that
the next site visit spends its time on the questions that genuinely need
hardware.

Three questions were answerable from the bags and all three were answered:

```text
Can a Nav2-ready 2D map be produced from the recorded data?   yes
Is the projected /scan dense enough to feed AMCL?             yes
How heavy is FAST-LIO, really?                                much lighter than assumed
```

A fourth answer arrived unasked and matters more than the other three - see
section 5.

## 2. Map From The Bag

`02_loop`, 180 s of walking, replayed through FAST-LIO and saved:

```text
scans.pcd                20 619 324 points, 630 MB
src/go2_control/map/livox_02loop.pgm + .yaml
grid                     481 x 406 cells @ 0.05 m
free                     36.2 m2
occupied                 44.8 m2
walls                    5.7 cm on one axis, 14.1 cm on the other
room orientation         37.25 degrees, corners square
```

This is a bench artefact, not a site map. It is the room the robot was walked
around, kept only so AMCL and Nav2 can be exercised before the next site visit.

### The mount tilt has to come from the data

The 13 degree figure from Unitree is a nominal mount pose. `02_loop` was
recorded with the sensor on the robot, and the tilt that decides whether a flat
height slice cuts through the floor is the one the sensor actually had - which
depends on how the robot was standing, not only on how the bracket is drawn.

Fitting a plane to the lowest point in each 1 m cell gave the real figure:

```text
pitch  +10.26 deg
roll   +1.72 deg
sensor 0.43 m above the floor at the start
```

After levelling, floor returns cluster 1.25x more tightly and sit at a single
height. That is the check to repeat on any new dataset: if levelling does not
sharpen the floor, the angle is wrong.

## 3. Two Defects In The Map Tooling

Both would have surfaced as "Nav2 cannot find a path" days later, with the
cause several steps upstream.

**Almost the whole map was unknown.** Only 1.7% of cells came out free, because
a cell was marked free only where a beam stopped, never where a beam passed
through. Nav2 does not plan through unknown, so the map had nowhere to drive
despite being an ordinary empty room. Fixed with a flood fill from the sensor's
starting cell.

**The fill then escaped the building.** Real walls have holes - a doorway the
walk never entered, a window - and the flood found one and filled the outdoors,
reporting 90.7% free. Worse than the original defect: Nav2 would have routed
straight through a wall. Fixed by thickening obstacles before filling and
discarding the thickening afterwards. The width had to be measured:

```text
--seal 2   free 88.5%   escaping
--seal 3   free 87.3%   escaping
--seal 5   free  7.4%   sealed     <- new default
```

## 4. FAST-LIO Costs Much Less Than Assumed

Replaying `02_loop` on the MiniPC:

```text
budget per scan at 10 Hz   100.0 ms
used                        11.2 ms   (11%)
  preprocess                10.0 ms   (89% of the work)
  map incremental            0.4 ms
  construct H                0.1 ms
RAM                          134 MB
```

Two consequences. The board is very unlikely to be the bottleneck, and if it
ever is, `point_filter_num` is the knob that matters because the cost is the
per-point loop rather than the filter maths.

Also found while reading the build: `CMakeLists.txt` hard-codes
`MP_PROC_NUM=1` for every non-x86 architecture, so OpenMP is off on ARM no
matter how many cores the board has. The 11.2 ms above used 3 threads. The
board's 4 cores would pass upstream's own `N > 3` test if the check looked at
core count instead of architecture name.

Compile cost, measured on a clean build: 3 source files, 2825 MB peak per
compiler process, 44.5 s wall clock. With 13 GB free on the board, `-j2` is
comfortable.

## 5. The Finding That Changes The Plan

`02_loop` was walked back to its starting point deliberately, which makes the
closing error measurable:

```text
path walked      35.0 m
gap at the end    1.65 m    = 4.72% of distance travelled
height error      0.48 m    on a single flat floor
```

FAST-LIO is normally under 1%. The 14.1 cm wall in section 2 is this drift
showing up as one wall drawn twice.

The sensor was on the robot for this recording, so the explanation this
originally carried - hand shake and fast turns from a carried sensor - does not
apply. 4.72% is what FAST-LIO did in this configuration, against the under 1%
it normally achieves, and it needs a real explanation rather than an excuse.
Candidates worth separating: the `extrinsic_T` in the config, the gait's
vertical motion, and the 0.48 m height error over a single flat floor, which
points at pitch rather than at yaw drift.

It is evidence that one measurement has to move to the front of the queue:

> **Re-measure loop closure on a deliberate closed loop, with the mount pose
> verified.** Under 1% and AMCL alone is enough. Still near 5% and AMCL will
> not hold position on its own.

Since the sensor was already mounted for this recording, the 4.72% is the
number to beat rather than a figure awaiting a fair test. AprilTag is a
requirement of the system regardless - see `LIVOX_DEPLOYMENT_PLAN.md`.

## 6. Scan Quality For AMCL

`pointcloud_to_laserscan` on `/cloud_registered_body`, 1742 scans:

```text
usable beams   43.0% min, 54.5% mean, 60.9% max, std 2.9
```

Against 36-49% and unstable for the raw `/livox/lidar`. The standard deviation
matters more than the mean here: density barely moves frame to frame, and AMCL
subsamples to about 60 beams anyway, so 54.5% of 723 is well past sufficient.

This settles the projection source, and carries a structural consequence:
`/cloud_registered_body` only exists while FAST-LIO is running, so **FAST-LIO
has to run on the robot at all times**, not just during mapping.

## 7. State Of The Decision

```text
locked   FAST-LIO for odometry
locked   project /cloud_registered_body, not the raw sensor cloud
locked   AMCL for localisation - run since, see LIVOX_AMCL_REPLAY_2026-08-25.md
locked   AprilTag is required, not optional
revised  "run everything on the Unitree board" did not survive measurement.
         The navigation stack alone costs 2.2 cores on an i5, which is 4-7 on
         a 4-core ARM board before the camera and Unitree's own leg control.
         The board runs the sensor, odometry and projection; a ground station
         runs localisation and planning. See LIVOX_DEPLOYMENT_PLAN.md.
```

## 8. What Is Ready For The Next Session

```text
map           src/go2_control/map/livox_02loop.pgm + .yaml
tooling       pcd_to_map (offline), livox_grid_map (live), livox_mapping.rviz
data          4 bags, 1.6 GB, no dropped frames
numbers       tilt, drift, scan fill, CPU, RAM, compile cost
```

Next, in order: AMCL against the replayed bag, then Nav2 with `/cmd_vel` left
disconnected. Both need no hardware. Everything after that needs the robot.
