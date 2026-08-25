# Livox Mid-360 Migration

Plan changed from Hesai Pandar XT16 to Livox Mid-360. The Hesai path stays in
place and keeps working; Livox is added alongside it as a parallel path.

No robot motion is involved anywhere in this document. Do not publish
`/cmd_vel`, do not run `cmd_vel_node`, and do not use `enable_cmd_vel:=true`.

## 1. Why This Is Not A Rename

| | Pandar XT16 | Livox Mid-360 |
|---|---|---|
| Scan pattern | 16 lines, repetitive | **Non-repetitive** (Risley prism) |
| Vertical FOV | ±15°, symmetric | −7° to +52°, **asymmetric** |
| Driver | `hesai_ros_driver` | `livox_ros_driver2` + Livox-SDK2 |
| Config format | YAML | **JSON** |
| Topic / frame | `/lidar_points` / `hesai_lidar` | `/livox/lidar` / `livox_frame` |
| IMU | unused | **built-in ~200 Hz** on `/livox/imu` |

The scan pattern is the consequential difference. XT16 gives a clean horizontal
ring every frame, so a thin `pointcloud_to_laserscan` slice works. Mid-360
samples different directions every frame, so the same slice produces a scan
full of holes that move around.

## 2. Measured: Why 2D Slicing Alone Is Not Enough

Rehearsal with `livox_fake_scan.launch.py`, which reproduces the non-repetitive
pattern and the asymmetric FOV:

```text
/livox/lidar   20000 points/frame, point_step 26, 10 Hz
/scan          723 beams
scan fill      39.6% - 47.9%, varying frame to frame
```

Under half the beams are populated, and which ones changes every frame. That is
enough to make a Nav2 costmap flicker obstacles in and out. This is the reason
the chosen approach is FAST-LIO2 (using the built-in IMU) with a 2D projection
for Nav2, rather than feeding `pointcloud_to_laserscan` output straight in.

Re-run the measurement any time the projection parameters change:

```bash
ros2 launch go2_control livox_fake_scan.launch.py
```

## 3. Driver Install (done, verified 2026-08-19)

```text
Livox-SDK2 source   ~/Livox-SDK2
SDK2 installed      /usr/local/lib/liblivox_lidar_sdk_shared.so
                    /usr/local/include/livox_lidar_api.h
Driver workspace    ~/ws_livox
Driver package      livox_ros_driver2  (built with ./build.sh ROS2)
Executable          livox_ros_driver2_node
```

`./build.sh ROS2` is correct for Foxy. `./build.sh humble` is for Humble. The
pcap/png/libusb warnings come from PCL and the `unused variable index` warning
is upstream; both are harmless.

Source order for every Livox terminal:

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash
```

## 4. The xfer_format Trap

Upstream `msg_MID360_launch.py` defaults to `xfer_format = 1`, which publishes
`livox_ros_driver2/msg/CustomMsg`. Neither `pointcloud_to_laserscan` nor Nav2
can read that type, and the failure looks like a topic that publishes fine
while nothing downstream ever receives anything.

`livox_mid360_driver.launch.py` in this package forces `xfer_format = 0`
(PointCloud2). Use it instead of the upstream launch file.

Point layout in `xfer_format = 0`, matching `LivoxPointXyzrtlt`:

```text
x         float32  offset 0
y         float32  offset 4
z         float32  offset 8
intensity float32  offset 12
tag       uint8    offset 16
line      uint8    offset 17
timestamp float64  offset 18   (nanoseconds, from offset_time)
point_step 26
```

The timestamp unit matters when configuring FAST-LIO.

## 5. Files Added

```text
launch/livox_mid360_driver.launch.py   real driver, xfer_format forced to 0
launch/livox_fake_scan.launch.py       no-hardware rehearsal + projection
launch/livox_scan.launch.py            real /livox/lidar -> /scan + static TF
launch/livox_mid360_lio.launch.py      FAST-LIO2 odometry, use_fake supported
config/livox_mid360_field.example.json driver network config template
config/fast_lio2_mid360.yaml           FAST-LIO2 params, lidar_type 4
pc2scan_livox.yaml                     projection params for the wide FOV
go2_control/fake_livox_mid360.py       fake Mid-360 (non-repetitive + IMU)
go2_control/livox_grid_map.py          live 2D /map from the LIO cloud
go2_control/pcd_to_map.py              offline .pcd -> Nav2 .pgm/.yaml
rviz/livox_mapping.rviz                map, live scan, odometry, TF
launch/livox_amcl.launch.py            map_server + AMCL + 2D projection
config/amcl_livox.yaml                 AMCL params in the Foxy spelling
pc2scan_livox_lio.yaml                 projection of /cloud_registered_body
go2_control/amcl_drift_check.py        AMCL correction to the LIO pose
go2_control/bag_clock.py               /clock from bag stamps, Foxy has no --clock
```

Nothing under `hesai_*` was modified.

## 6. Why The LIO Lives In Its Own Workspace

Neither existing option could read a Mid-360:

- `src/FAST_LIO_Hesai` is a Hesai fork whose `src/preprocess.h` header comment
  records that the Livox Avia/Horizon, Velodyne, Ouster and **MID360** handlers
  were removed. Its `LID_TYPE` enum is only `JT16 / JT128 / JT32`. This is what
  `install/fast_lio` is built from, per `build/fast_lio/CMakeCache.txt`.
- `src/go2w/spark-fast-lio` is an empty directory, although
  `go2w_fast_lio2/config/fast_lio2_unilidar.yaml` and its `package.xml` are
  written against it. It looks like an uncloned submodule.

Upstream `hku-mars/FAST_LIO` branch `ROS2` does have a MID360 handler, so that
is what was brought in:

```text
Workspace     ~/ws_fastlio_livox
Source        ~/ws_fastlio_livox/src/FAST_LIO
Upstream      hku-mars/FAST_LIO, branch ROS2
Package name  fast_lio_livox   (renamed, see below)
Executable    fastlio_mapping
```

It is a separate workspace with a renamed package because upstream is also
called `fast_lio`, exactly like the Hesai fork this workspace already builds.
Two packages of the same name in overlaid workspaces resolve silently by source
order, which is the kind of failure that costs a field day. Renaming makes both
sourceable at once.

## 7. Local Patches To Upstream FAST_LIO

`~/ws_fastlio_livox` is not under version control, so these are recorded here.
All four were required to get from a fresh clone to a working node on Foxy.

1. **ikd-Tree submodule.** `CMakeLists.txt` compiles
   `include/ikd-Tree/ikd_Tree.cpp`, which is a git submodule that a
   `--depth 1` clone does not fetch. Clone it separately:

   ```bash
   git clone --depth 1 -b fast_lio https://github.com/hku-mars/ikd-Tree.git \
     ~/ws_fastlio_livox/src/FAST_LIO/include/ikd-Tree
   ```

2. **Package rename.** `package.xml` `<name>` and `CMakeLists.txt` `project()`
   changed `fast_lio` -> `fast_lio_livox`. This also requires following the
   generated message namespace in `include/common_lib.h`:

   ```text
   #include <fast_lio/msg/pose6_d.hpp>   ->  fast_lio_livox/msg/pose6_d.hpp
   typedef fast_lio::msg::Pose6D Pose6D; ->  fast_lio_livox::msg::Pose6D
   ```

3. **Foxy service callback.** The ROS2 branch targets Humble. Foxy's
   `create_service` does not accept a `ConstSharedPtr` request, so in
   `src/laserMapping.cpp`:

   ```text
   map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req
   ->
   map_save_callback(std_srvs::srv::Trigger::Request::SharedPtr req
   ```

4. **Field name mismatch.** `mid360_handler` reads
   `pcl::PointCloud<livox_ros::LivoxPointXyzrtl>`, whose registration in
   `src/preprocess.h` tags the reflectivity field as `reflectivity`. The driver
   publishes that field as `intensity` (see `lddc.cpp`), so `pcl::fromROSMsg`
   matched nothing and logged `Failed to find match for field 'reflectivity'`
   on every frame while zeroing intensity. Fixed by mapping the struct member
   to the field name the driver actually sends:

   ```text
   (float, reflectivity, reflectivity)  ->  (float, reflectivity, intensity)
   ```

5. **Ctrl-C never returned from `spin()`.** `main()` installed its own
   `signal(SIGINT, SigHandle)`, replacing the handler `rclcpp::init` had put in
   place, and `SigHandle` then called `rclcpp::shutdown()` from inside the
   signal handler while the main thread sat in `spin()`. On Foxy that
   deadlocks: the node kept running until something escalated to SIGKILL, so
   the map save at the end of `main()` was never reached. Removing the
   `signal()` call leaves rclcpp's handler in place; shutdown now takes about
   two seconds.

6. **The map accumulation was commented out.** In `publish_frame_world`, the
   whole `if (pcd_save_en)` block that appends each registered scan to
   `pcl_wait_save` ships commented out on the ROS2 branch. The save at the end
   of `main()` is guarded by `pcl_wait_save->size() > 0`, so with the block
   commented out `pcd_save_en` could be set and no `.pcd` would ever appear,
   with no error to say why. Uncommented locally.

## 8. Verified Working (2026-08-19, fake sensor, no hardware)

```bash
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_fastlio_livox/install/setup.bash
source install/setup.bash

ros2 launch go2_control livox_mid360_lio.launch.py use_fake:=true
```

```text
/cloud_registered_body   19996 points, frame body, intensity present
/Odometry                camera_init -> body, converged and stable
                         position held within ~1 cm on a static scene
TF camera_init -> body   broadcasting
reflectivity warnings    0 after patch 4
```

This is a smoke test against synthetic data. It shows the pipeline is wired
correctly end to end; it says nothing about accuracy on real hardware.

## 8b. Handheld Mapping On The Bench (verified 2026-08-19, real sensor)

The Mid-360 cabled straight to the MiniPC cannot map from the robot, but it can
map from the hand, and that exercises the entire chain without needing the
mount pose measured: a handheld map lives in FAST-LIO's own frame, so the
`body -> base_link` transform that section 9 is blocked on does not enter into
it.

```bash
ros2 launch go2_control livox_mid360_lio.launch.py \
    enable_driver:=false save_pcd:=true      # driver already running

# walk, then Ctrl-C once, and wait for "current scan saved to /PCD/scans.pcd"

ros2 run go2_control pcd_to_map \
    ~/ws_fastlio_livox/src/FAST_LIO/PCD/scans.pcd \
    --z-min -0.4 --z-max 0.4 \
    -o src/go2_control/map/livox_handheld
```

Result from a 25 s stationary run, sensor on a desk:

```text
scans.pcd    2 840 484 points, 87 MB
extent       x -6.4..7.5   y -13.6..7.7   z -0.7..4.9 m
grid         241 x 457 cells @ 0.05 m
occupied     9511 cells (8.6%)
```

Walls came out clean and straight. Two things to watch:

- `scans.pcd` grows the whole time and is held in RAM until shutdown. A 25 s
  run gave 87 MB; a 5 min run gave 345 MB. Budget accordingly on the board.
- `--z-min/--z-max` are in the **sensor** frame, whose origin is the sensor and
  not the floor. Read the reported z extent first: the floor shows up as the
  lower bound. The defaults assume a handheld sensor at chest height and are
  wrong for a sensor sitting on a desk.

## 8c. Building A Nav2 Map (verified 2026-08-20, replayed field data)

Two ways to the same output. `livox_grid_map` publishes `/map` while the run is
happening, which is what to watch in RViz; `pcd_to_map` converts the saved
`.pcd` afterwards and is the one to use when tuning, because it re-runs in
seconds without replaying anything.

```bash
ros2 run go2_control pcd_to_map <scans.pcd> \
    --pitch-deg 10.26 --roll-deg 1.72 \
    --z-min -0.18 --z-max 0.62 \
    -o src/go2_control/map/<name>
```

### Take the mount tilt from the data, not the datasheet

Section 6 gives 13 degrees for the sensor bolted to the Go2, but any run where
the sensor was carried, or the robot started on a slope, has a different tilt -
and the tilt is what decides whether a flat slice cuts the floor. Measure it
from the cloud instead: take the lowest point in each 1 m cell, fit a plane
through those, and read the tilt off the plane normal.

On `02_loop`, a handheld walk, that gave **pitch +10.26, roll +1.72**, sensor
0.43 m above the floor - not the 13 degrees the datasheet would have supplied.
Confirmation that the correction is right: after levelling, floor returns
cluster **1.25x more tightly** and settle at a single height, `z = -0.38 m`.

### Two defects found while doing this, both fixed

**Almost the whole map came out unknown.** Rasterising hits alone marked only
1.7% of cells free: a cell became free only if a beam stopped in it, while
every cell a beam merely passed through stayed unknown. Nav2 will not plan
through unknown, so that map had nowhere at all to drive despite being an
ordinary empty room. Fixed with a flood fill from the sensor's starting cell -
somewhere the robot demonstrably stood, therefore free.

**The fill then escaped the building.** Real walls have holes: a doorway the
walk never entered, a window, a stretch the beam only grazed. The flood found
one and flooded the outdoors, reporting 90.7% free. That is worse than the
original problem, because Nav2 would confidently route through a wall. Fixed by
thickening obstacles before the fill and discarding the thickening afterwards,
so gaps close without walls moving.

The sealing width had to be measured, not guessed:

```text
--seal 2   free 88.5%   still escaping
--seal 3   free 87.3%   still escaping
--seal 5   free  7.4%   sealed        <- default
```

Raise `--seal` if free space still spills outside the building; lower it if a
narrow but genuine passage gets sealed off.

### Result on 02_loop

```text
grid        481 x 406 cells @ 0.05 m
free        36.2 m2
occupied    44.8 m2
wall width  5.7 cm on one axis (one cell), 14.1 cm on the other
```

The room reads as a room, with straight walls and square corners at 37.25
degrees. The 14 cm wall is the drift in section 8d showing up as one wall drawn
twice, slightly offset.

## 8d. Measured Performance And Drift (2026-08-20)

Both numbers come from replaying `02_loop`, 180 s of real sensor data, on the
MiniPC (i5-1235U, 12 threads).

### FAST-LIO is far lighter than expected

```text
budget per scan at 10 Hz   100.0 ms
actually used                11.2 ms   (11%)
  of which preprocess        10.0 ms   (89% of the work)
  map incremental             0.4 ms
  construct H (EKF)           0.1 ms
RAM                           134 MB
```

The cost is almost entirely the per-point loop, not the filter maths, so
`point_filter_num` is the knob that matters if the board turns out to be tight.

Worth knowing before the aarch64 build: `CMakeLists.txt` hard-codes
`MP_PROC_NUM=1` for every non-x86 architecture, so OpenMP is effectively off on
ARM regardless of core count. The 11.2 ms above was measured with 3 threads.
The board has 4 cores and would pass upstream's own `N > 3` test if the check
looked at core count rather than architecture name.

### Odometry drift is worse than it should be

`02_loop` was walked back to its starting point on purpose, which makes the
closing error measurable:

```text
path walked        35.0 m
gap at the end      1.65 m   = 4.72% of distance travelled
height error        0.48 m   on a single flat floor
```

FAST-LIO would normally be under 1%. The likely cause is that this was a
handheld walk - hand shake and fast turns - rather than a defect, and the
`extrinsic_T` in the config describes the sensor mounted on the Go2, which is
not where it was. **Re-measure loop closure as soon as the sensor is bolted to
the robot, before tuning AMCL.** If it stays near 5%, AMCL alone will not hold
position and AprilTag stops being optional.

## 8e. Measured: Scan Quality For AMCL (2026-08-20)

`pointcloud_to_laserscan` on `/cloud_registered_body`, 1742 scans from
`02_loop`:

```text
usable beams   43.0% min, 54.5% mean, 60.9% max, std 2.9
```

Compare with 36-49% and unstable for the raw `/livox/lidar` in section 2. The
low standard deviation matters more than the mean: density barely varies frame
to frame, and AMCL subsamples to roughly 60 beams anyway, so 54.5% of 723 beams
is far more than it needs.

This is why the localisation plan uses AMCL on a projection of the LIO-deskewed
cloud rather than of the raw sensor cloud. It also means **FAST-LIO has to run
on the robot at all times**, since `/cloud_registered_body` only exists while it
does.

## 8f. AMCL Against The Replayed Bag (2026-08-25)

Full write-up in `LIVOX_AMCL_REPLAY_2026-08-25.md`. `livox_amcl.launch.py`
against `livox_02loop` with `02_loop` replayed at 1x:

```text
scans dropped        0        over the whole 180 s
correction mean      0.359 m
           max       0.871 m  at t = 50.6 s
           final     0.648 m  = 1.92% of 33.74 m walked
heading    mean abs  5.51 deg
           max abs  17.49 deg
```

AMCL held the lock for the whole run without a reset, and removed about 60% of
the 1.65 m closing error section 8d measured. It did not get the residual below
half a metre.

Two cautions before that number is used to decide anything. The error is one
excursion between 30 and 60 s rather than steady accumulation, and it coincides
with the peak heading error. And `body -> base_link` levels the sensor by a
fixed rotation, which is exact for a bolted mount and only approximate for the
hand that carried `02_loop` - when the operator tips the sensor, the band
`pointcloud_to_laserscan` cuts tips out of level and stops being a slice of the
same room the map is a slice of. That term disappears on the robot, so this is
a bench baseline, not the answer to the AprilTag question.

### Replaying a bag on Foxy needs a clock you have to supply

`ros2 bag play` gained `--clock` in Galactic. On Foxy the bag replays its
recorded stamps while every node reads the wall clock, and
`pointcloud_to_laserscan` stamps `/scan` with `now()` instead of copying the
stamp of the cloud it projected. On these bags that put `/scan` six days ahead
of FAST-LIO's `camera_init -> body`, and AMCL dropped every single scan with

```text
Message Filter dropping message: frame 'base_link' ... for reason 'Unknown'
```

while `/scan` published at a healthy 10 Hz, the map loaded, and every lifecycle
node reported active. The log line names a frame, which sends you to the TF
tree; the problem is the clock. `go2_control/bag_clock.py` republishes the
bag's header stamps as `/clock`, and `livox_amcl.launch.py replay:=true`
(the default) starts it and puts the localisation nodes on sim time.

## 9. Open: TF Wiring

FAST-LIO names its world frame `camera_init` and its body frame `body`.
Neither matches the Go2 tree, so `livox_mid360_lio.launch.py` carries an
`enable_tf_bridge` argument that publishes `odom -> camera_init` and
`body -> base_link`.

It defaults to **false**. Both transforms depend on the real mount pose, and
`body` is the IMU frame rather than the LiDAR frame, so the identity values
currently in the launch file are placeholders. Turning the bridge on before
measuring the mount would feed Nav2 a confidently wrong pose.

## 10. Next Steps

Items 1 to 5 of the original list are done - see
`LIVOX_FIELD_DAY_2026-08-19.md` for how, and sections 8c to 8e above for the
numbers. What remains:

Item 1 is now done - see section 8f and
`LIVOX_AMCL_REPLAY_2026-08-25.md`. It did not decide the AprilTag question,
because the handheld tilt of the bag is mixed into the residual; item 4 below
is what decides it.

```text
1. done. AMCL localises against livox_02loop for a full 180 s replay without
   losing the pose, leaving 0.648 m of residual, 1.92% of the path walked.
2. Nav2 with the Go2 footprint, goals sent from RViz, /cmd_vel left
   disconnected.
3. Move the sensor onto the Unitree board and build fast_lio_livox for
   aarch64. Consider patching the MP_PROC_NUM check first (section 8d).
4. On the robot: verify the 13 degree mount pose is real, then re-measure loop
   closure before anything else.
5. Map the actual site, then AMCL on it, then Nav2 on it.
6. Only then /cmd_vel, with the robot_ack gate, one goal, one metre, lowest
   speed, an operator holding the remote.
```

Still outside git and still the single point of failure for the sensor:
`~/ws_livox/src/livox_ros_driver2/config/MID360_minipc.json`.

## 11. Values Still Placeholder

```text
config/livox_mid360_field.example.json  lidar ip, host ip
launch/livox_scan.launch.py             static TF x/y/z/roll/pitch/yaw
pc2scan_livox.yaml                      min_height, max_height, range_min
config/fast_lio2_mid360.yaml            blind, det_range
```

`launch/livox_mid360_lio.launch.py` no longer belongs on this list: both tf
bridge transforms now carry the published Go2 mount pose, and `base_link`
measured +0.11 degrees of pitch in `odom` afterwards. The mount pose itself is
still a datasheet figure rather than something checked on this robot.

The map produced from `02_loop` is a bench artefact, not a site map: it is the
room the sensor was carried around, kept only to exercise AMCL and Nav2 before
going on site.
