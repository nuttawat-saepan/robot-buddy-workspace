# Onsite Plan, Reordered

The plan as originally written puts the sensor and mapping work first and the
command path last. Two field sessions have now ended the same way: the sensor,
the map, AMCL and Nav2 all worked, the blue plan line appeared in RViz, and the
robot did not move - and there was no time left to find out why.

The command path is the only part that cannot be worked on anywhere else. Maps,
Nav2 and localisation can all be replayed from a bag at a desk; whether a
command reaches the legs can only be found out with the robot in front of you.
So it goes first, and it goes first even though it looks like the last step.

Everything here is a revision of the P/O task list, not a replacement for it.

## What changed from the original list, and why

**The command path moves from O4 to the first thing of the day.** Proving it
needs no map, no Nav2, no localisation, and the robot does not walk more than
five centimetres. It takes half an hour and it is the thing that has blocked
two whole trips.

**Map editing is struck from O3.** Measured on 2026-09-03: hand-editing a map
makes AMCL worse in proportion to how much editing is done, and unstable with
it. See `LIVOX_AMCL_TUNING_2026-09-03.md`. Sealing a map is worth doing for
Nav2's planner, which is a different job and not urgent.

**Measuring the board's CPU is promoted to its own item.** The finished system
runs everything on the board's 4 cores while leaving the leg controller alone.
Nobody has measured whether that fits. If it does not, the architecture changes,
and that is worth knowing before more is built on top of it.

**A photograph step is added.** The deliverable is a robot that photographs
capture points and sends the pictures back. No item in the original list
covered the camera or the upload at all.

**P3 gets its number.** The deliverable was "an error in metres and a tuned
amcl.yaml". Both exist now: 0.258 m mean and 0.494 m max against a replayed
bag, with the config committed. What is not done is the same measurement on the
real robot.

## The day, in order

Each step has a gate. If the gate does not pass, do not go on to the next one -
every later step assumes it.

### O1 · Probe the command path · 0.5 h

Robot powered, standing still. No map, no Nav2, no localisation needed.

```bash
go2sdk
ros2 run go2_control unitree_udp_bridge --mode probe --interface $UNITREE_IF
```

Sends no motion command and needs no `robot_ack`.

**Gate:** the probe reports a subscriber on the request topic, or the sport
service answers. If neither, try `--request-topic /api/wheeled_sport/request`
and then `--mode sdk`.

### O2 · One step, without Nav2 · 0.5 h

Area clear, remote in someone's hand.

```bash
# terminal A
go2robot && ros2 run go2_control cmd_vel_udp_relay
# terminal B
go2sdk && ros2 run go2_control unitree_udp_bridge --mode api \
    --interface $UNITREE_IF --robot-ack I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
# terminal C - five seconds, not one: the ROS CLI needs longer than that to
# create a DDS participant, and a 1 s timeout kills it before it publishes
go2robot && timeout 5s ros2 topic pub --rate 10 /cmd_vel_safe \
    geometry_msgs/msg/Twist "{linear: {x: 0.03}}"
```

**Gate: the wheels turn.** This one gate clears six unknowns at once -
interface, service name, DDS config, `robot_ack`, the UDP hop, and whether the
robot is in a mode that accepts commands.

If it fails, spend the rest of the day here. Everything below can be done at a
desk; this cannot.

### O3 · LiDAR on the board · 1 h

```bash
go2robot
ros2 launch go2_control livox_robot.launch.py replay:=false
go2hz
```

**Gate:** `/livox/lidar` at about 10 Hz.

### O4 · FAST-LIO on aarch64, and the CPU budget · 2 h

Build with `-j2`. Each compiler process peaked at 2825 MB on the MiniPC; the
board has about 13 GB free, so `-j4` will hit the ceiling.

```bash
go2cpu 60      # while a goal is running, not while the stack is idle
```

**Gate:** odometry does not drift while the robot stands still, and the robot
side stays under about 2 of the 4 cores.

If `go2cpu` reports OVER, switch to `nav2_livox_go2_lowcpu.yaml` and
`amcl_livox_lowcpu.yaml` rather than editing values by hand. **Record the
number either way** - it is the answer to whether everything can run on the
board, and nobody has it yet.

### O5 · Make the map · 2 h

```bash
go2robot && ros2 launch go2_control livox_slam.launch.py
```

Walk the whole site and close the loop - come back to where you started. Save
as soon as it looks complete.

**Do not retouch it.** Use the raw map.

The floor within about 1.2 m of the robot is never seen from where the robot is
standing: the lowest ray leaves the sensor about 20 degrees below level from
0.43 m up. Walk through those places so they are seen from somewhere else.

**Gate:** walls line up where the loop closes.

### O6 · Localisation · 0.5 h

```bash
go2ground && ros2 launch go2_control livox_ground.launch.py \
    replay:=false map:=$SITE_MAP
```

Set the initial pose, then **walk the robot several metres and turn it a
couple of times** - AMCL does not update at all below `update_min_d` of 0.20 m,
so standing still and waiting achieves nothing.

```bash
go2ready
```

**Gate:** particle spread under 0.35 m and staying there.

### O7 · First goal, one metre · 0.5 h

```bash
go2safe                                  # before every goal
ros2 run go2_control send_mission --goal 1.0 0.0 0.0 --dry-run
ros2 run go2_control send_mission --goal 1.0 0.0 0.0
```

Open floor, straight ahead, nothing to turn around. `send_mission` refuses to
send if the readiness checks fail, prints the remaining distance once a second,
and cancels the goal properly on Ctrl-C.

**Gate:** the robot arrives and stops by itself.

### O8 · A waypoint list · 1 h

```bash
ros2 run go2_control record_waypoint --file missions/site.json
ros2 run go2_control send_mission --file missions/site.json --dry-run
ros2 run go2_control send_mission --file missions/site.json
```

Three or four points, wide clearances, mark one as a capture point.

**Gate:** it walks the list in order and holds at the capture point.

### O9 · The web round trip · 2 h

Press in the browser, the robot walks, progress comes back, stop mid-mission
works.

**Gate:** the mission completes and can be stopped part way through.

### O10 · AprilTag, pilot only · 1.5 h

Two or three tags, not the whole site. Measure their positions into the map and
check that a detection moves the AMCL pose.

**Gate:** a tag detection changes the pose in the direction it should.

The full survey is a separate visit. Four hours of tag placement at the end of
a day is how a trip ends with the tags placed and nothing tested.

### Later · The photograph path · not scheduled

The camera and the upload are not written. `send_mission` holds at capture
points and says so. Until this exists the deliverable is not met, and the
0.258 m localisation error means a photograph would be framed a quarter of a
metre off even once it is.

## Time

```text
O1  probe                    0.5
O2  one step                 0.5
O3  LiDAR on the board       1.0
O4  FAST-LIO + CPU           2.0
O5  map                      2.0
O6  localisation             0.5
O7  first goal               0.5
O8  waypoint list            1.0
                            ----
    through O8               8.0
O9  web round trip           2.0
O10 AprilTag pilot           1.5
                            ----
    everything              11.5
```

The original list totalled 10 hours with three items unestimated. Getting
through O8 is a full day on its own. O9 and O10 should be planned as a second
visit rather than squeezed into the end of the first.

## If O1 or O2 fails

Do not move on to the sensor work to feel productive. The sensor, the map and
Nav2 can all be done at a desk against a recorded bag; the command path cannot.

In order: `--request-topic /api/wheeled_sport/request`, then `--mode sdk`, then
check `UNITREE_IF` against `ip addr` on the machine actually running the
bridge, then confirm `ROS_DOMAIN_ID` is 0 in that terminal - Unitree's own
traffic is on domain 0, not the site domain.

Record a bag throughout, and run `go2logs` before the terminals are closed. The
last two sessions could not be diagnosed afterwards because the evidence was in
scrollback.
