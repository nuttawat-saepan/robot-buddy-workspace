# FAST-LIO patches for the Livox Mid-360 on Foxy

`~/ws_fastlio_livox` is not under version control and exists on one machine.
These patches are the only durable record of the six changes that took upstream
`hku-mars/FAST_LIO` from a fresh clone to a working node on Foxy. Two of them
produce a run that appears to succeed and silently writes no file, so
rediscovering them on the board would cost a day.

Upstream this was generated against:

```text
https://github.com/hku-mars/FAST_LIO.git
branch ROS2, commit a4743b0 (Merge pull request #381 from mfassler/ROS2)
```

## Building from scratch, including on the Unitree board

```bash
mkdir -p ~/ws_fastlio_livox/src && cd ~/ws_fastlio_livox/src

# 1. Upstream, ROS2 branch
git clone --depth 1 -b ROS2 https://github.com/hku-mars/FAST_LIO.git

# 2. ikd-Tree. CMakeLists.txt compiles include/ikd-Tree/ikd_Tree.cpp, which is
#    a git submodule that a --depth 1 clone does not fetch. Without this the
#    build fails on a missing source file.
git clone --depth 1 -b fast_lio https://github.com/hku-mars/ikd-Tree.git \
    FAST_LIO/include/ikd-Tree

# 3. The five source changes
cd FAST_LIO
git apply /path/to/go2_control/patches/fast_lio/0001-foxy-mid360-fixes.patch

# 4. Build. 2825 MB peak per compiler process was measured on the MiniPC, so
#    -j2 is right for a board with 13 GB free.
cd ~/ws_fastlio_livox
colcon build --packages-select fast_lio_livox --parallel-workers 2
```

Verify the patch applies before trusting it:

```bash
git apply --check 0001-foxy-mid360-fixes.patch
```

## What the patch changes, and why

**Package renamed `fast_lio` -> `fast_lio_livox`** (`CMakeLists.txt`,
`package.xml`, and the generated message namespace in `include/common_lib.h`).
This workspace already builds a package called `fast_lio` from
`src/FAST_LIO_Hesai`, a Hesai fork whose MID360 handler has been removed. Two
packages of one name in overlaid workspaces resolve silently by source order,
which is the kind of failure that costs a field day. Renaming lets both be
sourced at once.

**`std_srvs::srv::Trigger::Request::ConstSharedPtr` -> `SharedPtr`**
(`src/laserMapping.cpp`). The ROS2 branch targets Humble; Foxy's
`create_service` does not accept a const request. Compile error without it.

**`reflectivity` -> `intensity`** (`src/preprocess.h`). `mid360_handler` reads
`pcl::PointCloud<livox_ros::LivoxPointXyzrtl>`, whose registration tags the
field as `reflectivity`, but `livox_ros_driver2` publishes it as `intensity`
(see `lddc.cpp`). `pcl::fromROSMsg` therefore matched nothing, logged
`Failed to find match for field 'reflectivity'` on every frame, and zeroed
intensity.

**`signal(SIGINT, SigHandle)` removed from `main()`** (`src/laserMapping.cpp`).
It replaced the handler `rclcpp::init` had installed, and `SigHandle` then
called `rclcpp::shutdown()` from inside the signal handler while the main
thread sat in `spin()`. On Foxy that deadlocks: the process never leaves
`spin()`, the PCD save immediately after it never runs, and the node only ends
when something escalates to SIGKILL. **Symptom: Ctrl-C hangs and no map file
appears, with nothing logged.** Leaving rclcpp's own handler in place makes
shutdown take about two seconds.

**Map accumulation uncommented** (`src/laserMapping.cpp`, `publish_frame_world`).
The whole `if (pcd_save_en)` block that appends each registered scan to
`pcl_wait_save` ships commented out on the ROS2 branch, while the save at the
end of `main()` is guarded by `pcl_wait_save->size() > 0`. **Symptom:
`pcd_save_en` is set, the run completes cleanly, and no `.pcd` is ever written,
with no error to say why.**

## Regenerating this patch

If `~/ws_fastlio_livox` is edited further, refresh the patch rather than
letting the two drift apart:

```bash
git clone --depth 1 -b ROS2 https://github.com/hku-mars/FAST_LIO.git /tmp/upstream
cd /tmp/upstream
for f in CMakeLists.txt package.xml include/common_lib.h \
         src/laserMapping.cpp src/preprocess.h; do
  diff -u "/tmp/upstream/$f" "$HOME/ws_fastlio_livox/src/FAST_LIO/$f" \
    | sed -e "1s|.*|--- a/$f|" -e "2s|.*|+++ b/$f|"
done > 0001-foxy-mid360-fixes.patch
```

Confirm afterwards that applying the patch to a fresh clone reproduces the
working tree file for file.
