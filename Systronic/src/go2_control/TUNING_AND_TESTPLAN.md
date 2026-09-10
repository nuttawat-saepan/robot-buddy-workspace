# What Can Be Tuned, What It Does, And How To Test It

Written 2026-09-10, from the configuration as it stands after the field day of
2026-09-07. Every value quoted here was read out of the running system or the
committed config, not remembered.

## 1. Why low obstacles do not appear

`pc2scan_livox_lio.yaml` cuts a horizontal slab out of the point cloud and
throws the rest away:

```text
min_height: 0.25     metres above the floor
max_height: 1.05
```

**Anything lower than 25 cm above the floor is not in `/scan` at all**, so the
costmap never learns about it and Nav2 will drive into it. A pallet edge, a
cable tray, a step, a dog: invisible.

The 0.25 is not arbitrary. `pcd_to_map` rasterised the map from the same band,
and AMCL correlates the live scan against that map - so cutting the scan at a
different height compares two different slices of the room and localisation
gets worse. **Lowering min_height without rebuilding the map trades navigation
safety for localisation accuracy.** They are the same parameter pulling in two
directions, which is why it should not be nudged casually.

There is a second floor beneath it. The Mid-360 sees 7 degrees below level, the
mount adds 13, so the lowest ray leaves at about 20 degrees below horizontal
from 0.43 m up: the floor within roughly 1.2 m of the robot is never seen from
where the robot stands, whatever min_height says.

If low obstacles have to be seen, the honest fix is a second projection - a
separate `pointcloud_to_laserscan` with `min_height: 0.05`, published on
`/scan_low`, added to the local costmap as an extra observation source and not
given to AMCL. That keeps localisation on the band the map was built from and
lets the costmap see the floor. It is about twenty lines of launch file.

## 2. Running it without six terminals

Today it takes six: sensors, Nav2, map republisher, MQTT bridge, relay, and
the Unitree bridge - on the board, each needing four `source` lines, plus RViz
on the ground station.

Three ways out, in increasing order of effort:

**A single launch file** that starts everything except the Unitree bridge.
Nav2, the sensors and the map republisher are already launch files and can be
included; `cmd_vel_udp_relay` and `mqtt_mission_bridge` are one `Node` each.
The Unitree bridge stays out on purpose - it is the one process that can move
the robot, and it should keep needing a deliberate command with the gate
phrase in it.

**A systemd unit** for what should survive a reboot. The board has no RTC and
comes up in 1970; a unit that waits for the clock and then starts the stack
would remove most of a morning's setup.

**tmux** for a human who wants to watch all of it. `tmux new-session -d` with
one window per process, then attach. Everything stays visible and one detach
survives an ssh drop - which cost several restarts on 2026-09-07.

The single launch file is the right first step, because it also fixes the
ordering problem: Nav2's `controller_server` fails to activate if the sensors
are not already publishing TF, which happened repeatedly on site. A launch file
can sequence that; six terminals rely on the operator remembering.

## 3. Debugging when it goes wrong

`go2doctor` already walks the chain and reports the first break. What is
missing is that it only runs where you type it, and on site the interesting
machine is the other one.

Worth adding, in order of value:

- **`go2doctor --remote`**, running the same checks over ssh on the board and
  printing them here.
- **A heartbeat on MQTT**: the bridge already publishes pose every second; a
  `/missions/health` alongside it carrying the same PASS/FAIL lines would put
  the diagnosis on the web page where the operator is already looking.
- **`collect_logs.sh` on both machines in one command.**

The single most common failure on 2026-09-07 was not a bug at all: a terminal
that had not sourced the workspace, or had sourced the wrong one. The board's
`~/.bashrc` sources an older `go2_control` from `~/UnitreeRos`, which shadows
ours in every new shell. A launch file removes most of the opportunity.

## 4. What should be tunable from the web, and what each one does

Everything below is a live ROS parameter on `controller_server` and can be set
without restarting anything:

```text
                          now    raise it                 lower it
FollowPath.max_vel_x      0.40   faster, longer stopping  slower, safer
                                 distance than the
                                 costmap sees ahead
FollowPath.max_vel_theta  0.35   turns quicker, overshoots turns slowly; below
                                 and hunts                ~0.2 the wheeled base
                                                          may not turn at all
FollowPath.acc_lim_x      0.35   reaches speed sooner     under ~0.2 DWB changes
                                                          its mind before the
                                                          robot reaches speed
FollowPath.acc_lim_theta  0.50   snappier turns           sluggish, wide corners
goal_checker.
  xy_goal_tolerance       0.35   arrives sooner, stops    circles the goal
                                 further away             forever below the
                                                          localisation error
  yaw_goal_tolerance      0.30   accepts a rougher        spins hunting for a
                                 heading                  heading it cannot hold
```

**The floor on `xy_goal_tolerance` is set by localisation, not preference.**
Measured error is 0.258 m mean and 0.494 m max. A robot cannot decide it has
arrived within a distance smaller than its own uncertainty - at 0.25 it circled
each waypoint for ten seconds and needed a recovery before accepting it. This
comes down when AprilTag brings the error down, and not before.

Two more worth exposing, on `local_costmap`:

```text
inflation_radius     0.35   raise: keeps further from walls, refuses narrow gaps
cost_scaling_factor  2.0    raise: hugs the centre of free space more strongly
```

And on the bridge, which is not a ROS parameter and needs a restart:
`--max-linear` and `--max-angular`. **The bridge's clamp wins over Nav2's.**
On 2026-09-07 Nav2 was set to 0.40 while the bridge was still cutting to 0.25,
and nothing anywhere reports the discrepancy - the robot simply runs slower
than the number the operator set.

## 5. Stop, and a mode switch

Stop already works: `/missions/control` with `{"action": "stop"}` cancels the
Nav2 goal and the robot stops. `pause` and `resume` work the same way. This was
exercised on site.

A mode switch is worth having and the shapes are already there:

```text
idle       nothing sends velocity. /cmd_vel_safe silent.
manual     the operator drives; Nav2 is not asked for goals
mission    what runs today
slow       max_vel_x 0.15, max_vel_theta 0.2, for tight spaces
```

`slow` is the one with immediate value, because tight and open spaces want
different numbers and today changing them means typing parameter commands.
Implementing modes is mostly a table of parameter sets and a `ros2 param set`
loop in the bridge.

## 6. Zoom and drag on the web map

Nothing is needed from the robot side. `/missions/map` already carries what a
canvas needs:

```text
width, height        cells
resolution           metres per cell
origin {x, y, yaw}   the world coordinate of the map's bottom-left corner
image                base64 PNG
```

Zoom and pan are a frontend transform on that image. The robot marker is
placed with:

```text
px = (x - origin.x) / resolution
py = height - (y - origin.y) / resolution
```

The `height -` is not optional. An OccupancyGrid's first row is the bottom of
the map in world terms and every canvas draws the first row at the top; getting
it wrong mirrors the building and is not obvious until the robot appears to
drive through walls.

## 7. Where the map is centred

`origin` in the map's yaml is the world coordinate of the bottom-left cell:

```text
livox_site_20260907_1231
  origin      [-6.95, -12.2, 0]
  resolution  0.05 m/cell
  size        284 x 304 cells = 14.2 x 15.2 m
```

So the map covers x from -6.95 to +7.25 and y from -12.2 to +3.0. The origin is
wherever FAST-LIO happened to start when the map was made - it is not the
centre of anything and carries no meaning beyond that.

The web should centre its view on the robot, or on the map's centre computed
from the metadata, rather than on (0,0).

## 8. Why turning goes wrong when it turns quickly

Observed on site: at `max_vel_theta` 0.6 the robot turned but the pose "went
off". That is not the controller. Rotation is where this sensor is weakest -
the projected scan carries about 54% of its beams and a different set each
frame, so a fast turn changes which walls are visible faster than AMCL can
re-converge, and the correction arrives late and large.

Three things help, in order:

- **Turn more slowly.** 0.35 was settled on for this reason.
- **`alpha1` and `alpha2`**, already lowered from 0.10/0.4 to 0.03/0.10 on
  2026-09-07. They tell AMCL how much rotational error to expect from
  FAST-LIO's odometry, which is IMU-aided and better than the old values
  allowed for. That change is recorded but **has not been measured** - see
  `LIVOX_AMCL_TUNING_2026-09-03.md`.
- **AprilTag.** If the pose still degrades on turns with alpha1 at 0.03, the
  cause is that the scan is too sparse to constrain rotation at all. That is a
  sensor limit and no parameter fixes it.

## 9. The test plan

The seven cases proposed are good and the ordering is right - a single post,
then a post to circle, then an obstruction, then loops. What is missing is
mostly about knowing *why* a case failed rather than only that it did.

**Add before case 1:**

- **A stationary baseline.** Robot still, localised, for five minutes. Record
  the spread of `/amcl_pose`. Without it, an error at a waypoint cannot be told
  apart from an error the robot has while standing still.
- **Repeatability of one goal.** The same goal, ten times, from the same start.
  The spread of where it stops is the number that decides whether
  `xy_goal_tolerance` can come down.

**Add between:**

- **An obstacle lower than 25 cm.** Section 1 says the robot cannot see it.
  Confirm that on purpose, in a controlled way, before discovering it with a
  pallet.
- **An obstacle that appears after the plan is made.** Case 3 has it in place
  from the start; walking something into the path mid-mission is the case that
  matters for a robot working around people.
- **A goal inside an obstacle**, and **a goal outside the map.** Both should be
  refused cleanly rather than driven at.
- **Stop and resume mid-leg**, from the web. Exercised once on site; it belongs
  in the list.
- **A capture point**, since photographs are the deliverable and a mission that
  navigates perfectly and returns no pictures has failed.

**Add after the loops:**

- **Localisation recovery.** Cover the sensor, or carry the robot two metres,
  and see whether AMCL recovers or has to be re-seeded. This will happen in
  service.
- **Wi-Fi loss mid-mission.** With everything on the board it should not matter,
  which is the claim worth testing rather than assuming.
- **Battery to 20%.** Nothing in the stack watches the battery today.

**For every case, record rather than judge by eye:**

```text
ros2 bag record /livox/lidar /livox/imu /Odometry /odom /scan /tf /tf_static \
    /map /amcl_pose /particlecloud /cmd_vel_nav_preview /cmd_vel_safe
```

On 2026-09-07 the difference between "it wandered" and a number came entirely
from having the bag. The loop-stacking cases in particular cannot be answered
by watching - they need `/amcl_pose` against `/Odometry` over several laps.
