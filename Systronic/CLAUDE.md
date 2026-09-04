# Claude Project Context

ROS 2 Foxy workspace for a Unitree **Go2W** carrying a **Livox Mid-360**, with
the goal of a robot that takes a mission from a web interface, walks it,
photographs designated points, and sends the photographs back.

The Hesai/Pandar XT16 work that this repository started from is superseded. The
files are kept, and marked at the top of each, but the Livox path is the live
one and the Hesai one is not maintained.

## Safety Rules

There may be no real robot connected during local work.

Do not publish to `/cmd_vel` during local checks or no-motion field checks.
Do not run `cmd_vel_node` unless field movement is explicitly approved.
Movement requires the operator's gate, on the command line, for one run:

```text
I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
```

The standing proof that a run cannot move the robot:

```bash
ros2 topic info /cmd_vel        # expect: Unknown topic '/cmd_vel'
ros2 node list | grep cmd_vel   # expect: nothing
```

Movement is the final stage only, after sensor, TF, map and localisation checks
pass, with the area clear and the remote in someone's hand.

## The Robot Is A Go2W, Not A Go2

Its service list carries `wheeled_sport`, not the `sport` service that
unitree_sdk2py's `SportClient` looks for. The SDK's discovery finds nothing and
refuses to send, surfacing as error 3102 with no hint that the name is the
problem. `unitree_udp_bridge --mode api` publishes `unitree_api/msg/Request`
directly and does not depend on that discovery; it is the default for this
reason. Run `--mode probe` at every site before anything else.

## Workspace

```text
Main workspace: /home/sys20/projects/systonic-2307/Systronic
Livox driver:   ~/ws_livox
FAST-LIO:       ~/ws_fastlio_livox
Bags:           ~/livox_bags_field        (outside git, 2.5 GB)
ROS distro:     Foxy
```

Read first:

```text
src/go2_control/RUNBOOK_ONSITE.md              what to do on site, in order
src/go2_control/LIVOX_DEPLOYMENT_PLAN.md       how the deployment is meant to work
src/go2_control/LIVOX_AMCL_TUNING_2026-09-03.md what actually moves the error
src/go2_control/LIVOX_MID360_MIGRATION.md      how the Livox path fits together
```

## Four Environments, And They Are Not Interchangeable

Sourcing the wrong one is not an error. `ros2 topic list` comes back empty, or
a topic has a publisher and delivers nothing, and neither says why. Each script
prints what it set - read it every time.

```bash
source scripts/setup_local_env.sh    # one machine: replay, MiniPC-only, bench
source scripts/setup_robot_env.sh    # the Unitree board
source scripts/setup_ground_env.sh   # the ground station
source scripts/setup_sdk_env.sh      # the terminal running unitree_udp_bridge
```

Or, after adding `scripts/aliases.sh` to `~/.bashrc`: `go2local`, `go2robot`,
`go2ground`, `go2sdk`, and `go2which` for "which side am I on".

Site-specific values live in `scripts/onsite.env`, which is gitignored. Copy
`onsite.env.example` and fill it in.

**Never set `RMW_IMPLEMENTATION` globally in `~/.bashrc`.** The navigation
graph needs Fast DDS and the Unitree bridge needs CycloneDDS; a global default
is silently wrong in half the terminals.

## Build

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
colcon build --packages-select go2_control
source install/setup.bash
```

Editing a launch file or a config without rebuilding leaves the old copy in
`install/` in use, with no warning. It is the most common wasted hour here.

## The Chain, End To End

```text
1  Livox Mid-360                       ->  /livox/lidar, /livox/imu
2  livox_ros_driver2                       10 Hz
3  fastlio_mapping                      ->  /Odometry, /cloud_registered_body
4  lio_odom_relay                       ->  TF odom -> base_link
5  pointcloud_to_laserscan              ->  /scan
6  amcl  (needs map_server)             ->  TF map -> odom
7  Nav2  (needs both TFs + /scan + map) ->  /cmd_vel
8  sensor_watchdog                          cuts on stale sensors
9  cmd_vel_node                             clamps, needs robot_ack
--- ROS ends here ---
10 cmd_vel_udp_relay -> UDP 127.0.0.1:32123 -> unitree_udp_bridge -> the robot
```

Steps 1-9 work. Step 10 has never been shown to move this robot.

`/scan` is projected from `/cloud_registered_body`, not from the raw
`/livox/lidar`, so **FAST-LIO must be running or `/scan` is silent** with no
error anywhere.

Step 10 is two processes because one process picks its RMW once, at startup,
and the two halves need different ones.

## Key Files

```text
launch/livox_robot.launch.py       everything that runs on the board
launch/livox_ground.launch.py      everything on the ground station
launch/livox_slam.launch.py        making a map
launch/livox_amcl.launch.py        localisation only, no path to motion
launch/livox_mid360_lio.launch.py  sensor + FAST-LIO alone

config/nav2_livox_go2.yaml         Nav2      (+ _lowcpu variant)
config/amcl_livox.yaml             AMCL      (+ _lowcpu variant)
config/fast_lio2_mid360.yaml       FAST-LIO
config/fastdds_udp_only.xml        the navigation graph's DDS
config/cyclonedds_unitree_wlan.xml the Unitree bridge's DDS

go2_control/send_mission.py        send a goal or waypoint list, no RViz needed
go2_control/record_waypoint.py     write the waypoint files send_mission reads
go2_control/nav_ready_check.py     is the stack ready for a goal, headless
go2_control/unitree_udp_bridge.py  api / sdk / probe
go2_control/sensor_watchdog.py     stops on stale sensors
go2_control/amcl_drift_check.py    measures AMCL against FAST-LIO

scripts/deploy_to_board.sh         rsync + colcon on the board
scripts/check_robot_link.sh        why is the topic list empty
scripts/measure_board_load.sh      does the board have the CPU for this
scripts/collect_logs.sh            take the evidence home
```

## Where Things Stand

```text
localisation error   0.258 m mean, 0.494 m max over a 33.7 m replayed walk
                     enough for open floor and wide corridors
                     not enough for a 0.9 m doorway or an accurate photo stop
board CPU            never measured. scripts/measure_board_load.sh is for this
step 10              api mode never proven on the robot
camera and upload    not written
AprilTag             never tested on site
```

Parameter tuning is close to exhausted as a route to a lower error. AprilTag is
the plan for the accuracy the mission needs, which makes it required rather
than a fallback.

## Findings That Contradict Older Plans

Measured on 2026-09-03, two to three runs per condition, in
`LIVOX_AMCL_TUNING_2026-09-03.md`:

- **Do not hand-edit maps to help AMCL.** It made things worse both times, in
  proportion to how much editing was done, and made the result unstable.
  Editing draws walls the real scan still goes through. Sealing a map is for
  Nav2's planner; that is a different need and worth doing separately.
- **Use the physical mount angle, 13.0/0.0/0.35**, not the 10.26/1.72/0.43
  fitted from the cloud, even against a map built with the fitted values.
- **Lowering AMCL's alphas does not move the mean, it halves the peak.** That
  is the useful effect: a large correction arrives as a jump, and the jump is
  what makes the controller lurch.

## Traps That Cost Whole Sessions

```text
replay defaults to true on livox_robot.launch.py. A live run without
replay:=false waits forever for a /clock that never comes.

ros2 bag play has no --clock on Foxy. The bag_clock node supplies it.

FAST-LIO diverges when starved of CPU. It does not slow down: the pose runs
away to tens of thousands of metres while every process stays alive and every
topic keeps publishing.

Deleting /dev/shm/fastrtps_* while ROS is running breaks the transport for
every live participant. Symptom: a topic with a publisher and no data.

CycloneDDS 0.7 on Foxy needs the legacy <NetworkInterfaceAddress>. Given the
newer syntax it creates no participant and says nothing.

RViz shows nothing when its Reliability is Reliable and the publisher is best
effort - /scan, /particlecloud, /livox/lidar all are. No error either way.

The board does not answer ICMP. A failed ping proves nothing; test port 22.

The Unitree bridge belongs on ROS_DOMAIN_ID 0, Unitree's own domain, not the
site domain the rest of the stack uses.

Wrong SDK interface: the bridge arms, reports ready, and no command reaches
the legs. Set UNITREE_IF once in onsite.env.
```

## If Editing

Keep changes scoped to `src/go2_control` and `src/go2_interfaces`.

Do not remove safety gates. Do not make `/cmd_vel` enabled by default. Prefer
launch or environment parameters over hardcoded IPs, interfaces and map paths -
every hardcoded one of those has already caused a failure here.

Two runs minimum before believing a measurement. Several results in this
project reversed between the first and second run of the same condition.
