# ชุดคำสั่งหน้างาน

`<B>` = IP บอร์ด เช่น `192.168.80.111`

## 0 · ก่อนเริ่ม (MiniPC)

```bash
nc -vz -w 3 <B> 22                                         # บอร์ดตอบไหม (ping ใช้ไม่ได้)
ssh -t unitree@<B> "sudo date -s \"$(date -u +'%F %T') UTC\""   # ตั้งนาฬิกา ทุกครั้งที่บูต
./scripts/deploy_to_board.sh unitree@<B>                   # ส่งโค้ด
```

## 1 · เปิดใช้งาน (บอร์ด)

```bash
ssh unitree@<B>
tmux new -s go2            # กลับเข้า: tmux attach -t go2
```

**T1 สแตก** — หุ่นนิ่ง 20 วิ

```bash
source ~/go2_ws/scripts/setup_robot_env.sh
ros2 launch go2_control onsite_stack.launch.py map:=$SITE_MAP replay:=false
```

**T2 bridge หุ่น** — `Ctrl-b c` · รีโมทในมือ

```bash
source ~/go2_ws/scripts/setup_sdk_env.sh
ros2 run go2_control unitree_udp_bridge --mode raw --interface eth0 --max-linear 0.45 --max-angular 0.6 --timeout 0.30 --robot-ack I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
```

**T3 ใช้งาน** — `Ctrl-b c`

```bash
source ~/go2_ws/scripts/setup_robot_env.sh
ros2 run go2_control nav_ready_check
```

mqtt bridge มากับ T1 แล้ว **ห้ามรันแยก**

## 2 · ตั้งตำแหน่ง

เว็บ: ตั้งตำแหน่งหุ่น → รอ `localizationSpread < 0.35`

หรือ RViz (MiniPC) แค่ `Map` + `LaserScan` → 2D Pose Estimate → **ปิด RViz ทันที**

```bash
source scripts/setup_ground_env.sh && rviz2
```

## 3 · สั่งเดิน

เว็บ หรือ

```bash
ros2 run go2_control send_mission --goal <x> <y> <yaw>
ros2 run go2_control send_mission --file ~/mission.json
ros2 param set /mqtt_mission_bridge capture_views 4       # จุดถ่ายรูปหมุนถ่าย 4 มุม
```

## 4 · ปรับค่า (T3)

```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.20       # สด
ros2 param set /controller_server FollowPath.max_vel_theta 0.30   # สด
ros2 run go2_control reconfigure --node /amcl --set update_min_d=0.05   # ค่าที่ต้องรีสตาร์ท
```

## 5 · ดู / เช็ค (T3)

```bash
ros2 run tf2_ros tf2_echo map base_link         # ตำแหน่ง
ros2 run go2_control yaw_watch                  # มุมเป็นองศา
ros2 run go2_control pipeline_latency           # ช้าตรงไหน
ros2 topic hz /livox/lidar                      # ~10 Hz
ros2 lifecycle get /map_server                  # ต้อง active
ros2 node list | grep -c "^/mqtt_mission_bridge$"   # ต้อง 1
~/go2_ws/scripts/measure_board_load.sh 60       # CPU ตอนหุ่นเดิน
```

แมพไม่ขึ้น / `Invalid frame ID "map"`:

```bash
for n in /map_server /amcl; do ros2 lifecycle set $n configure; ros2 lifecycle set $n activate; done
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

## 6 · ทำแมพใหม่

```bash
ros2 launch go2_control mapping_stack.launch.py          # เข็นทั่วห้อง ปิดลูป
mkdir -p ~/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/maps/site_$(date +%Y%m%d_%H%M)'}}"
sed -i 's|^image: .*/|image: |' ~/maps/site_*.yaml
sed -i "s|^export SITE_MAP=.*|export SITE_MAP=/home/unitree/maps/<ชื่อ>.yaml|" ~/go2_ws/scripts/onsite.env
```

## 7 · ปิด / เก็บ

```bash
# บอร์ด
tmux kill-server
ps aux | grep -E "fastlio|amcl|controller_server|unitree_udp_bridge" | grep -v grep   # ต้องว่าง
```

```bash
# MiniPC
./scripts/backup_from_board.sh unitree@<B>
```

## กฎ

```text
RViz ข้าม Wi-Fi ห้ามมี PointCloud2 — ทำหุ่นช้า 2 วิ
T2 ใช้ setup_sdk_env (โดเมน 0) ที่เหลือ setup_robot_env (43)
ห้ามฆ่า gst_realsense / mediamtx (กล้อง)
แบ็กอัพก่อน deploy และก่อนล้างบอร์ด
```
