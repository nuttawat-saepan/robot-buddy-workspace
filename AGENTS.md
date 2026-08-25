# Local ROS2 Foxy Preparation Rules



This workspace is being prepared locally without a real Unitree robot connected.



## Safety



- Do not publish to `/cmd_vel`.

- Do not run `cmd_vel_node`.

- Do not launch with `enable_cmd_vel:=true`.

- Do not run real robot movement commands.

- Use real launch only for syntax/dependency checks with:

  `enable_cmd_vel:=false enable_camera:=false enable_april:=false`

- Real movement is gated. Do not start movement-capable paths unless the user
  explicitly approves field movement and provides:

  `robot_ack:=I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT`

- `cmd_vel_node` must refuse to initialize Unitree SDK without the exact
  `robot_ack` above.

- `main.py` and `april_localizer.py` must keep `/cmd_vel` publishing disabled by
  default. AprilTag scan/rotate mode is movement-capable and must stay disabled
  unless the same movement gate is armed.



## Scope



Focus on:

- workspace structure

- ROS2 Foxy environment

- dependency checks

- `colcon build`

- package executable checks

- launch syntax checks

- simulation/fake testing if available

- identifying hardcoded config before field testing



## Simulation Notes



- `sim.launch.py` is intended for local software checks with no real robot connected.

- `go2.launch.py` is the front-door launcher. Its default mode is `sim` for
  local safety. Use `mode:=sim` for simulation and `mode:=real` only with
  explicit real map/params and the movement gate rules above.

- Keep local simulation launches safe by default:

  `enable_unitree_bridge:=false enable_april:=false enable_stream:=false enable_init_pose:=false`

- When testing the Unitree fake bridge in simulation, use:

  `enable_unitree_bridge:=true enable_april:=false enable_stream:=false enable_init_pose:=false`

- TurtleBot3/Gazebo publishes the source topics as `/joint_states`, `/odom`, and `/imu`.

- The local sim default map is `src/go2_control/map/house_map.yaml` with
  `house_map.pgm`, restored from the old workspace. Keep these installed via
  `setup.py` under `share/go2_control/map`.

- `gazebo_convert` internally expects `/sim_joint_states`, `/sim_odom`, and `/sim_imu`, so `sim.launch.py` must remap:

  - `/sim_joint_states` -> `/joint_states`

  - `/sim_odom` -> `/odom`

  - `/sim_imu` -> `/imu`

- If `/lf/lowstate` exists but `ros2 topic hz /lf/lowstate` reports no data, first check that `/joint_states`, `/odom`, and `/imu` are publishing and that the remaps above are present.

- TurtleBot3/Gazebo publishes lidar as LaserScan on `/scan`, not PointCloud2. The Go2 pipeline expects PointCloud2 on `/utlidar/cloud`, and `go2w_read` republishes that as `/pointcloud`.

- `scan_to_pointcloud` exists for local simulation only. With `enable_unitree_bridge:=true`, `sim.launch.py` starts:

  `/scan` -> `scan_to_pointcloud` -> `/utlidar/cloud` -> `go2w_read` -> `/pointcloud`

- Keep `enable_pc2scan:=false` for the normal TurtleBot3 simulation path. Turning it on converts `/utlidar/cloud` back to `/scan`, which is useful for real/fake PointCloud tests but can create a confusing scan/cloud loop in this sim setup.

- For fake Unitree bridge validation, check these rates after launching sim:

  - `ros2 topic hz /lf/lowstate`

  - `ros2 topic hz /lf/sportmodestate`

  - `ros2 topic hz /utlidar/cloud`

  - `ros2 topic hz /pointcloud`

  - `ros2 topic hz /odom`

  - `ros2 topic hz /imu/data`

- `go2_control local_check` is the preferred read-only smoke test once sim is running. It must be run after `sim.launch.py`/`go2.launch.py mode:=sim` is already running in another terminal; it does not launch simulation by itself. Run it after sourcing the workspace:

  `ros2 run go2_control local_check`

- The workspace also has a helper script:

  `/home/sys20/projects/systonic-2307/Systronic/scripts/local_sim_check.sh`

  It sources ROS Foxy and the workspace, then runs `go2_control local_check`.

- `local_check` only subscribes/inspects graph state. It must not publish `/cmd_vel` or send robot commands. It checks core nodes, required simulated topics, `/map` with transient-local QoS, and warns if non-Gazebo `/cmd_vel` publishers are present.

- If a sandboxed tool run fails with `getifaddrs: Operation not permitted`, that is a ROS DDS discovery permission issue in the tool sandbox. Ask the user to run the checker in their terminal, or rerun with approved network-interface access.

- Do not press RViz `Nav2 Goal` during safety-only checks; Nav2 goals can publish `/cmd_vel` in simulation.

- For sim movement tests, RViz `2D Goal Pose` requires Nav2 lifecycle nodes to
  be active and `/navigate_to_pose` to appear in `ros2 action list`. If nothing
  happens, check `bt_navigator`; on Foxy, newer BT plugin names in
  `nav2_params.yaml` can leave it unconfigured.

- Real robot field checks are documented in:

  `/home/sys20/projects/systonic-2307/Systronic/src/go2_control/REAL_ROBOT_FIELD_CHECKLIST.md`

- Hesai XT16 field handoff/checklist files are:

  `/home/sys20/projects/systonic-2307/Systronic/src/go2_control/FIELD_RUNBOOK_QUICK.md`

  `/home/sys20/projects/systonic-2307/Systronic/src/go2_control/HESAI_GO2_HANDOFF.md`

  `/home/sys20/projects/systonic-2307/Systronic/src/go2_control/HESAI_GO2_FIELD_CHECKLIST.md`

- The local Hesai driver workspace is `~/hesai_ws`, built from the official
  `https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git` repo at tag
  `v2.0.12`. Do not switch to `leggedrobotics/hesai_lidar_ros_driver` during
  first field bringup unless the official driver is confirmed blocked.

- Hesai field config should start from
  `src/go2_control/config/hesai_xt16_field.example.yaml`, then be copied to a
  site-specific `hesai_xt16_field.yaml` with the real LiDAR IP/network values.

- Fake Hesai mapping rehearsal uses
  `ros2 launch go2_control hesai_fake_mapping.launch.py`. It starts
  TurtleBot3/Gazebo, `scan_to_pointcloud`, `pointcloud_to_laserscan`,
  `slam_toolbox`, and RViz. The mapping path is:
  `/scan -> /lidar_points -> /hesai_scan -> /map`, so Gazebo geometry still
  matches the generated map while the pointcloud conversion path is exercised.
  It does not start `cmd_vel_node` or publish `/cmd_vel` by itself. Any movement
  for mapping must be simulation-only.

- Gazebo 3D LiDAR mapping uses
  `ros2 launch go2_control hesai_3d_mapping.launch.py`. It spawns a local
  TurtleBot3 waffle SDF with a 16-layer ray sensor publishing PointCloud2 on
  `/lidar_points`, then converts to `/hesai_scan` for `slam_toolbox`. This is
  closer to Pandar-style data than the `/scan -> PointCloud2` rehearsal, but it
  is still simulation-only and not a real Pandar validation.

- Current 3D mapping projection tuning in `pc2scan_hesai.yaml` is
  `target_frame: base_scan`, `min_height: -0.06`, `max_height: 0.06`, and
  `range_min: 0.7`. This is a simulation self-filter/debug setting after seeing
  dark robot-body rings in `/map`; it may need more tuning before maps look like
  clean rooms.



Keep edits scoped to:

- `src/go2_control`

- `src/go2_interfaces`



Ask before:

- installing dependencies

- deleting files

- changing launch behavior

- changing network or robot movement behavior
