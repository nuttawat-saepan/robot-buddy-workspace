> SUPERSEDES the testing advice in LOCAL_SIM_READINESS.md.

# What Is Different Between The Simulation And The Site

Every difference below is a thing that has to be changed, or is silently wrong,
when moving between the Gazebo simulation and the real robot. They are listed
so they can be set once in an environment script instead of remembered on each
command line.

## The differences

```text
                    Gazebo simulation          Site
robot               TurtleBot3 Waffle          Go2W, 0.70 x 0.31 m
base frame          base_footprint             base_link
sensor              simulated planar lidar     Livox Mid-360
/scan comes from    Gazebo directly            FAST-LIO's registered cloud
                                               (so FAST-LIO must be running or
                                               /scan is silent, with no error)
odometry            exact, from Gazebo         FAST-LIO, drifts
controller velocity /cmd_vel                   /cmd_vel_nav_preview
Nav2 parameters     nav2_params.yaml           nav2_livox_go2.yaml
AMCL parameters     inside nav2_params.yaml    amcl_livox.yaml
map                 house_map.yaml             the site map
max speed           Nav2 defaults              0.05 m/s
path to the robot   Gazebo subscribes directly relay -> UDP -> Unitree bridge
DDS                 Fast DDS, loopback         Fast DDS for Nav2,
                                               CycloneDDS for the bridge
replay argument     not used                   replay:=false, or the stack
                                               waits forever for a /clock
```

The controller topic and the base frame are the two that bite hardest, because
neither produces an error. A readiness check pointed at `/cmd_vel_nav_preview`
in the simulation reports the controller as inactive when it is running
perfectly well; one pointed at `base_link` on a TurtleBot3 reports a missing
transform that is simply named something else.

`/cmd_vel_nav_preview` is not an accident. The Livox launch files remap Nav2's
velocity output away from `/cmd_vel` on purpose, so that a planning run has no
path to the robot at all. `ros2 topic info /cmd_vel` returning
`Unknown topic` is the standing proof that a run cannot move anything.

## Which one to use for what

**The simulation is the only place the mission logic can be tested**, because
the robot in it moves. Replaying a bag cannot show whether a goal succeeds,
whether the second waypoint follows the first, whether a capture point pauses,
or whether Ctrl-C stops anything, since the robot in a bag never moves and Nav2
always aborts in the end.

**A bag replay is the only place localisation accuracy can be measured**,
because the simulation's lidar is complete and clean while the Mid-360's
projected scan carries about 54% of its beams and a different set each frame.
AMCL converges in simulation in a way it does not on the real sensor. See
`LIVOX_AMCL_TUNING_2026-09-03.md` for what the real numbers are.

**Neither says anything about the last hop.** Gazebo subscribes to `/cmd_vel`
itself, so the relay, the UDP link and the Unitree bridge are absent from the
simulation entirely. That is the part that has blocked two field sessions, and
it can only be tested with the robot present.

## Running it

```bash
go2sim                                    # or source scripts/setup_sim_env.sh
ros2 launch go2_control sim.launch.py
```

AMCL then needs an initial pose, and then it needs the robot to move. It does
not update at all below `update_min_d`, so a stationary robot never converges
no matter how long it is left:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}, \
    orientation: {w: 1.0}}}}"

# drive it a little - turning helps more than driving straight, because
# rotation changes which walls are visible and rules out more poses
timeout 10s ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.4}}"
```

The initial pose has to be near where the robot actually is. TurtleBot3 does
not spawn at the map origin in the house world, and seeding AMCL at 0,0 leaves
the particle cloud wide however far the robot is driven - which reads exactly
like AMCL being broken.

Then, with the simulation's topic and frame names:

```bash
ros2 run go2_control nav_ready_check --ros-args \
    -p controller_cmd_topic:=/cmd_vel -p base_frame:=base_footprint
ros2 run go2_control send_mission --goal 1.0 0.0 0.0 --controller-topic /cmd_vel
```

## A fixed bug worth knowing about

`nav2_params.yaml` set `robot_model_type: "nav2_amcl::DifferentialMotionModel"`,
which is the Galactic spelling. Foxy takes `"differential"`, and given the
newer form its AMCL **segfaults at startup** - exit code -11, no message
explaining why.

Everything downstream then reports a missing `map -> odom` transform, which
reads as a localisation problem rather than as a node that died before it ever
ran. `amcl_livox.yaml` had carried a comment warning about exactly this since
it was written; `nav2_params.yaml` had the bad spelling the whole time, which
is why the simulation had not worked.
