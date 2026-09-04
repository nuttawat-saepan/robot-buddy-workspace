# Runbook หน้างาน — Livox Mid-360 + Go2

ชุดคำสั่งเรียงตามลำดับที่ต้องทำจริง คัดลอกไปวางได้ทันที
แต่ละหัวข้อคือหนึ่งเทอร์มินัล เว้นแต่จะเขียนกำกับไว้เป็นอย่างอื่น

เครื่องที่เกี่ยวข้องมีสองตัว

```text
บอร์ด Unitree     บนตัวหุ่น       รัน driver / FAST-LIO / cmd_vel
ground station    โน้ตบุ๊ก        รัน map / AMCL / Nav2 / rviz
```

## ติดตั้งคำสั่งย่อก่อน ทำครั้งเดียว

ทั้งสองเครื่อง

```bash
echo 'source ~/projects/systonic-2307/Systronic/scripts/aliases.sh' >> ~/.bashrc
exec bash
```

จากนั้นทุกเทอร์มินัลเริ่มด้วยคำเดียว เลือกตามงาน

```text
go2robot    บนบอร์ด
go2ground   บนโน้ตบุ๊ก
go2sdk      เทอร์มินัลที่คุยกับหุ่น  (domain 0 ไม่ใช่ domain ของไซต์)
go2local    เทสที่บ้านด้วย bag
```

แต่ละอันพิมพ์บอกว่าตั้งค่าอะไรไปบ้าง **อ่านทุกครั้ง** — ปัญหาที่กินเวลาที่สุด
ไม่ใช่ลืม source แต่คือ source ผิดฝั่งแล้วไม่รู้ตัว อาการคือ `ros2 topic list`
ว่างเปล่า หรือ topic มี publisher แต่ไม่มีข้อมูล ซึ่งเหมือนกับปัญหาอื่นอีกหลายอย่าง

ลืมว่าอยู่ฝั่งไหน พิมพ์ `go2which`

คำสั่งย่ออื่นที่ใช้บ่อย

```text
go2ready         พร้อมส่ง goal หรือยัง
go2safe          เช็คว่ายังไม่มีอะไรสั่งหุ่นได้  ← ทำก่อนกด goal ทุกครั้ง
go2goal 1 0 0    ส่งเป้า 1 เมตรตรงหน้า
go2hz / go2tf    ดูอัตราข้อมูลและ TF
go2logs          เก็บหลักฐานกลับบ้าน
go2cpu           วัดว่าบอร์ดแบกไหวไหม
```

---

## 0. ก่อนออกจากบ้าน — deploy โค้ดขึ้นบอร์ด

ทำจาก ground station ต่อสาย LAN เข้ากับหุ่น

```bash
cd /home/sys20/projects/systonic-2307/Systronic

# ดูก่อนว่าจะคัดลอกอะไรบ้าง ยังไม่ทำอะไรจริง
./scripts/deploy_to_board.sh unitree@192.168.123.161 --dry-run

# ทำจริง
./scripts/deploy_to_board.sh unitree@192.168.123.161
```

สคริปต์เรียก `ssh` กับ `rsync` หลายรอบ จึงถามรหัสผ่านหลายครั้ง
คัดลอกกุญแจไปก่อนจะรันรวดเดียวไม่ต้องพิมพ์รหัส

```bash
ssh-copy-id unitree@192.168.123.161
```

### บอร์ดไม่ตอบ ping

อย่าใช้ `ping` ตัดสินว่าบอร์ดขึ้นหรือยัง มันไม่ตอบ ICMP ทั้งสองที่อยู่
เช็คพอร์ต SSH แทน

```bash
nc -zv -w4 192.168.123.161 22
```

ที่อยู่ของบอร์ดมีสองอัน เป็นเครื่องเดียวกันคนละการ์ด

```text
unitree@192.168.123.161   ฝั่งสาย บนเน็ตเวิร์กภายในของ Go2
unitree@192.168.68.70     ตัวเดียวกัน ผ่าน Wi-Fi ของไซต์
```

ใช้สายก่อน วันที่ 1 ก.ย. 2026 ฝั่ง Wi-Fi รับ deploy ได้แล้วหลุดหายไปหลังจากนั้นไม่กี่นาที
ส่วนฝั่งสายรับ SSH ตลอด

ผู้ใช้คือ `unitree` — `sys20` เป็นผู้ใช้ของ MiniPC ไม่มีอยู่บนบอร์ด

---

## 1. ตั้งค่าเฉพาะไซต์ ทำครั้งเดียวต่อหนึ่งไซต์

ทำทั้งสองเครื่อง

```bash
cd /home/sys20/projects/systonic-2307/Systronic
cp scripts/onsite.env.example scripts/onsite.env
nano scripts/onsite.env
```

ค่าที่ต้องแก้จริง

```text
ROBOT_NET_IF      การ์ดที่บอร์ดใช้คุยกับ ground station
GROUND_NET_IF     การ์ดที่ ground station ใช้ ปกติ wlp4s0
ROBOT_IP          ที่อยู่บอร์ด ค่าตัวอย่างในไฟล์ยังเป็นของเก่า
                  ต้องแก้เป็น 192.168.123.161 หรือ 192.168.68.70
SITE_MAP          พาธเต็มของแมพไซต์นี้ ค่า default เป็นแมพจากโต๊ะ ไม่ใช่ไซต์
LIO_PITCH_DEG     มุมก้มของตัวยึด ต้องตรงกับตอนทำแมพ
SENSOR_HEIGHT     ความสูงเซนเซอร์จากพื้น
```

`onsite.env` ถูก gitignore ไว้ตั้งใจ เพราะเก็บที่อยู่และมุมยึดของหุ่นตัวเดียวที่ไซต์เดียว

---

## 2. เช็คว่าสองเครื่องคุยกันได้จริง

อาการเวลาลิงก์เสียคือ `ros2 topic list` ว่างเปล่า เหมือนกันหมดไม่ว่าสาเหตุอะไร
สคริปต์นี้แยกสาเหตุให้ในสองนาที ทำก่อนอย่างอื่นเสมอ

ground station ก่อน

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source scripts/setup_ground_env.sh
./scripts/check_robot_link.sh listen
```

แล้วบนบอร์ด

```bash
ssh unitree@192.168.123.161
cd ~/Systronic
source scripts/setup_robot_env.sh
./scripts/check_robot_link.sh talk
```

ถ้าตรงนี้ไม่ผ่าน อย่าไปต่อ ทุกอย่างข้างล่างขึ้นกับลิงก์นี้ทั้งหมด

---

## 3. บนบอร์ด — เปิดเซนเซอร์กับ odometry

ยังไม่มีการเคลื่อนที่ในขั้นนี้

```bash
ssh unitree@192.168.123.161
cd ~/Systronic
source scripts/setup_robot_env.sh

ros2 launch go2_control livox_robot.launch.py \
  replay:=false \
  lio_pitch_deg:=13.0 \
  lio_roll_deg:=0.0 \
  sensor_height:=0.35 \
  unitree_interface:=eth0
```

`replay:=false` สำคัญ ค่า default คือ `true` ซึ่งเป็นโหมดเล่น bag ไม่ใช่หุ่นจริง

อยากดูแบตกับอุณหภูมิมอเตอร์ด้วย เพิ่ม

```bash
  enable_unitree_read:=true
```

---

## 4. เช็คว่าข้อมูลมาครบ

เปิดเทอร์มินัลใหม่บนบอร์ด แล้ว source เหมือนเดิม

```bash
source scripts/setup_robot_env.sh

ros2 topic hz /livox/lidar          # ควรราว 10 Hz
ros2 topic hz /scan                 # ควรราว 10 Hz
ros2 topic hz /Odometry             # FAST-LIO
ros2 run tf2_ros tf2_echo odom base_link
```

ทดสอบว่า odometry นิ่งจริง ให้หุ่นยืนนิ่ง แล้วดูว่าค่าใน `tf2_echo` ขยับหรือไม่
ถ้าค่าไหลทั้งที่ไม่มีใครขยับหุ่น แปลว่า FAST-LIO ยังไม่พร้อม อย่าไปต่อ

---

## 4.5 วัดว่าบอร์ดแบกไหวไหม

ทำเมื่อรันอยู่บนบอร์ด และควรทำระหว่างมี goal ทำงาน ไม่ใช่ตอน stack นิ่ง
ตัวเลขตอนนิ่งต่ำกว่าความจริงมากและจะทำให้ตัดสินใจผิด

```bash
./scripts/measure_board_load.sh 60
```

อ่านอย่างเดียว ไม่สตาร์ตโหนด ไม่ส่งคำสั่งอะไรทั้งสิ้น

บอร์ดมี 4 คอร์ เกณฑ์คือฝั่งหุ่นต้องกินไม่เกินราว 2 คอร์ โดยที่ stack ของ
Unitree ยังรันอยู่ด้วย เพราะ leg controller ห้ามอด สคริปต์บอก PASS หรือ OVER
ให้เอง และถ้า OVER มันจะพิมพ์รายการปุ่มที่หมุนได้ออกมา ทุกปุ่มเป็น ROS
parameter ไม่ต้อง build ใหม่

ถ้าได้ OVER ไม่ต้องไล่แก้ทีละค่า มีชุดพารามิเตอร์เตรียมไว้แล้ว

```bash
ros2 launch go2_control livox_ground.launch.py \
  replay:=false map:=$SITE_MAP \
  nav2_params_file:=nav2_livox_go2_lowcpu.yaml \
  amcl_params_file:=amcl_livox_lowcpu.yaml
```

หรือแก้ `NAV2_PARAMS` กับ `AMCL_PARAMS` ใน `scripts/onsite.env` ให้ถาวร

สองไฟล์นี้ต่างจากของเดิมแค่ความถี่ในการทำงาน **ลิมิตความเร็ว ประตูความปลอดภัย
และค่าความแม่นของ localisation เหมือนเดิมทุกตัว** ที่ลดลงคือ

```text
expected_planner_frequency  10.0 -> 2.0    ประหยัดมากที่สุด
controller_frequency         5.0 -> 3.0    ที่ 0.05 m/s คือคิดใหม่ทุก 17 มม.
bt loop_rate                  20 -> 10
local_costmap update          5.0 -> 3.0
amcl max_particles          3000 -> 1500
amcl max_beams                60 -> 40
```

ยังมีอีกปุ่มที่ไม่ได้อยู่ในไฟล์นี้ ถ้ายังไม่พอ: `point_filter_num` ใน
`config/fast_lio2_mid360.yaml` ปัจจุบัน 3 เพิ่มเป็น 5 หรือ 6 ได้
มันลดจำนวนจุดที่ FAST-LIO ต้องประมวลผล แลกกับ odometry ที่หยาบขึ้น

ใช้ชุด lowcpu **เฉพาะเมื่อวัดแล้วได้ OVER** การรันช้ากว่าที่จำเป็นทำให้
navigation แย่ลงโดยไม่ได้อะไรกลับมา

เก็บผลไว้ด้วย มันคือตัวเลขที่ยังไม่มีใครวัด และเป็นตัวตัดสินว่าสถาปัตยกรรม
"รันบนบอร์ดทั้งหมด" ทำได้จริงหรือไม่

---

## 5. ทำแมพ

ต้องมีขั้น 3 รันอยู่ก่อน แล้วเปิด SLAM ซ้อนขึ้นมาอีกเทอร์มินัล

```bash
source scripts/setup_robot_env.sh

ros2 launch go2_control livox_slam.launch.py \
  clock_topic:=/livox/imu
```

เดินเก็บทั้งไซต์ **ปิดลูปให้ครบ** คือกลับมาที่จุดเดิม ไม่ใช่เดินไปทางเดียวแล้วจบ

ดูแมพระหว่างเดินจาก ground station

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source scripts/setup_ground_env.sh
ros2 run go2_control occupancy_grid_points
rviz2 -d src/go2_control/rviz/livox_map_points.rviz
```

ใช้ `occupancy_grid_points` เพราะ display ชนิด Map ของ RViz มักไม่ติดข้ามเครื่องบน Foxy
ตัวนี้แปลง `/map` เป็นกลุ่มจุดซึ่ง RViz แสดงได้แน่นอน

### พื้นรอบตัวหุ่นจะเป็นรูเสมอ

Mid-360 มองต่ำกว่าแนวระดับได้ 7 องศา บวกมุมก้มจากตัวยึดอีก 13 องศา
ลำแสงต่ำสุดจึงลงล่างราว 20 องศา จากความสูง 0.43 เมตร
แปลว่าพื้นในรัศมีราว 1.2 เมตรรอบตัวหุ่นไม่เคยถูกมองเห็นจากจุดที่หุ่นยืน
ต้องเดินให้ทั่วเพื่อให้พื้นตรงนั้นถูกมองจากมุมอื่นแทน

---

## 6. เซฟแมพ

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source scripts/setup_ground_env.sh

ros2 run nav2_map_server map_saver_cli \
  -f src/go2_control/map/livox_site_$(date +%Y%m%d_%H%M)
```

ได้ไฟล์ `.pgm` กับ `.yaml` คู่กัน จากนั้นเปิด `.pgm` ในโปรแกรมแต่งรูป
ปิดรูรั่วที่ประตูหรือกระจก ลบเงาคนที่เดินผ่าน แล้วเซฟทับ

ถ้าแต่งแล้วเซฟเป็นชื่อใหม่ **ต้องแก้บรรทัด `image:` ใน `.yaml` ให้ชี้ไฟล์ใหม่ด้วย**
ไม่งั้น AMCL จะโหลดแมพตัวที่ยังไม่ได้แต่ง โดยไม่มีข้อความเตือนอะไรเลย

---

## 7. Ground station — localization กับ Nav2

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source scripts/setup_ground_env.sh

ros2 launch go2_control livox_ground.launch.py \
  replay:=false \
  map:=$SITE_MAP \
  enable_rviz:=true
```

เช็คว่า TF ครบก่อนอย่างอื่น ถ้าขาดท่อนใดท่อนหนึ่ง Nav2 หยุดทันที

```bash
ros2 run tf2_ros tf2_echo map odom          # มาจาก AMCL
ros2 run tf2_ros tf2_echo odom base_link    # มาจาก FAST-LIO
```

ตั้งตำแหน่งเริ่มต้นใน RViz ด้วยปุ่ม 2D Pose Estimate แล้วเดินหุ่นด้วยมือช้า ๆ
ดูว่ากลุ่มจุดสีแดงของ AMCL ลู่เข้าหากันหรือกระจายออก
ถ้ากระจายออกเรื่อย ๆ ตำแหน่งยังไม่น่าเชื่อถือ อย่าปล่อยให้ Nav2 สั่งเดิน

### อยากดู localization อย่างเดียวไม่เอา Nav2

```bash
ros2 launch go2_control livox_amcl.launch.py \
  map:=$SITE_MAP \
  enable_rviz:=true
```

ตัวนี้ไม่มีทางไปถึงการเคลื่อนที่เลย ไม่มี controller ไม่มี cmd_vel

---

## 8. สั่งเคลื่อนที่ — ขั้นสุดท้ายเท่านั้น

ทำต่อเมื่อผ่านครบ เซนเซอร์นิ่ง TF ครบ AMCL ลู่เข้า พื้นที่โล่ง มีคนถือรีโมทพร้อมหยุด

ปัจจุบันความเร็วถูกจำกัดไว้ที่ **0.05 เมตรต่อวินาที** ใน `config/nav2_livox_go2.yaml`
ซึ่งเป็นความเร็วระดับคลาน ตั้งใจให้เป็นแบบนั้นสำหรับการทดสอบครั้งแรก
เพิ่มได้ก็ต่อเมื่อพิสูจน์แล้วว่า localization กับทางหยุดเชื่อถือได้

บนบอร์ด ปิดขั้น 3 แล้วรันใหม่พร้อมประตูความปลอดภัย

```bash
source scripts/setup_robot_env.sh

ros2 launch go2_control livox_robot.launch.py \
  replay:=false \
  enable_cmd_vel:=true \
  cmd_vel_topic:=/cmd_vel \
  robot_ack:=I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
```

ไม่ใส่ `robot_ack` ให้ถูกทั้งบรรทัด โหนดจะไม่ยอมสตาร์ท เป็นพฤติกรรมที่ตั้งใจ

### 8.1 ตรวจทางสั่งหุ่นก่อน ห้ามข้าม

ก่อนอย่างอื่น รันโหมด probe มันไม่ส่งคำสั่งเคลื่อนที่เลย จึงไม่ต้องใช้ robot_ack

```bash
export CYCLONEDDS_URI=file:///home/unitree/Systronic/src/go2_control/config/cyclonedds_unitree_wlan.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run go2_control unitree_udp_bridge --mode probe --interface wlp4s0
```

มันรายงานทั้งสองทางแล้วบอกว่าควรใช้ทางไหน ใช้เวลาห้าวินาที

```text
path 1  SDK     ถ้าตอบ API version มาแปลว่า --mode sdk ใช้ได้
                ถ้าตอบ code 3102 แปลว่าหุ่นเป็น Go2W ให้ใช้ --mode api
path 2  api     ถ้าเห็น subscriber มากกว่าศูนย์ แปลว่า --mode api ใช้ได้
                ถ้าเห็นศูนย์ ลอง --request-topic /api/wheeled_sport/request
```

**หุ่นตัวนี้เป็น Go2W** service ภายในชื่อ `wheeled_sport` ไม่ใช่ `sport` ที่ SDK
มองหา ดังนั้นทางที่คาดว่าจะใช้ได้คือ `--mode api` ซึ่งเป็นค่า default อยู่แล้ว
ทาง SDK เก็บไว้เผื่อ เพราะบน Go2 ธรรมดามันใช้ได้

### 8.2 เปิดสองเทอร์มินัล

ตัวเชื่อมยังไม่ได้อยู่ใน launch file ต้องรันมือ และต้องแยกสองโปรเซส
เพราะสองฝั่งใช้ DDS คนละยี่ห้อ อยู่โปรเซสเดียวกันไม่ได้

เทอร์มินัล A ฝั่ง ROS

```bash
source scripts/setup_robot_env.sh
ros2 run go2_control cmd_vel_udp_relay
```

เทอร์มินัล B ฝั่ง Unitree

```bash
export CYCLONEDDS_URI=file:///home/unitree/Systronic/src/go2_control/config/cyclonedds_unitree_wlan.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run go2_control unitree_udp_bridge \
  --mode api \
  --interface wlp4s0 \
  --port 32123 \
  --timeout 0.5 \
  --robot-ack I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
```

ตอนสตาร์ตมันจะพิมพ์ว่ามี subscriber กี่ตัวบน request topic
**ถ้าขึ้น warning ว่าไม่มีใครฟัง อย่ากด goal** คำสั่งจะถูกส่งออกไปแล้วหายไปเฉย ๆ
กลับไปทำข้อ 8.1 ใหม่

ถ้าทาง api เงียบ สลับไปทางอื่นได้โดยไม่ต้องแก้โค้ด

```bash
  --request-topic /api/wheeled_sport/request     # ลอง endpoint อีกชื่อ
  --mode sdk                                     # ลองทาง SportClient
```

`--timeout 0.5` คือถ้าไม่ได้รับคำสั่งเกินครึ่งวินาที ให้หยุดหุ่น ทาง api ส่ง
StopMove ทาง sdk ส่งความเร็วศูนย์

### ทดสอบเป้าแรก

ไม่ต้องใช้ RViz แล้ว ใช้คำสั่งนี้แทน มันจะเช็คความพร้อมให้ก่อนแล้วปฏิเสธถ้ายังไม่พร้อม

```bash
# ดูก่อนว่าจะส่งอะไร ไม่ส่งจริง
ros2 run go2_control send_mission --goal 1.0 0.0 0.0 --dry-run

# ส่งจริง หนึ่งเมตรตรงหน้า
ros2 run go2_control send_mission --goal 1.0 0.0 0.0
```

มันจะพิมพ์ระยะที่เหลือทุกวินาที จะได้รู้ว่าหุ่นขยับจริงไหมโดยไม่ต้องเดา

```text
    1.02 m to go,   3s elapsed, 0 recovery/ies
    0.61 m to go,   8s elapsed, 0 recovery/ies
    reached
```

**Ctrl-C จะส่ง cancel ให้ Nav2 จริง** ไม่ใช่แค่ปิดโปรแกรมทิ้ง หุ่นจะหยุด

ถ้ายังไม่พร้อม มันจะบอกว่าเพราะอะไรแล้วไม่ส่ง goal เลย

```text
  particle spread  FAIL  0.612 m, above 0.35 m - AMCL has not settled
                   NOT READY - do not send a goal
not ready - no goal was sent.
```

อย่าเพิ่งตั้งเป้าที่ต้องเลี้ยว เอาตรงหน้าก่อน

### เดินหลายจุด

บันทึกจุดก่อน เข็นหุ่นไปตำแหน่งที่ต้องการแล้วกด Enter

```bash
ros2 run go2_control record_waypoint --file missions/site_a.json
```

พิมพ์ชื่อก่อนกด Enter เพื่อตั้งชื่อจุด, `c` เพื่อทำเครื่องหมายจุดถ่ายรูป,
`u` เพื่อลบจุดล่าสุด, Ctrl-C เพื่อจบ ไฟล์ถูกเขียนทุกครั้งที่บันทึก

แล้วสั่งเดินตามจุด

```bash
ros2 run go2_control send_mission --file missions/site_a.json --dry-run
ros2 run go2_control send_mission --file missions/site_a.json
```

จุดที่ทำเครื่องหมาย `capture` ไว้ หุ่นจะหยุดรอ แต่**ยังไม่ถ่ายรูป**
กล้องกับการอัปโหลดยังไม่ได้เขียน รูปแบบไฟล์เตรียมรองรับไว้แล้ว

---

## 9. อัดข้อมูลไว้กลับมาดูที่บ้าน

เปิดทิ้งไว้ทั้งวัน คุ้มมาก เพราะปัญหาส่วนใหญ่วิเคราะห์ได้หลังกลับ

```bash
cd ~/livox_bags_field
ros2 bag record -o onsite_$(date +%Y%m%d_%H%M) \
  /livox/lidar /livox/imu /Odometry /scan /tf /tf_static /map /cmd_vel
```

ระวังพื้นที่ดิสก์ ของเดิม `02_loopfix` ใช้ไป 909 MB

เล่นกลับที่บ้าน

```bash
ros2 bag play ~/livox_bags_field/onsite_XXXX
```

อย่าใส่ `--clock` บน Foxy ยังไม่มีตัวเลือกนี้ มันเพิ่มเข้ามาใน Galactic
เวลาของ replay มาจากโหนด `bag_clock` ซึ่ง launch file เปิดให้เองเมื่อ `replay:=true`

---

## หยุดฉุกเฉิน

เรียงจากเร็วสุดไปช้าสุด

```text
1. รีโมทของ Unitree            เร็วที่สุด ให้คนถือไว้ตลอดเวลาที่หุ่นเดิน
2. Ctrl-C ที่เทอร์มินัล B      unitree_udp_bridge ตาย คำสั่งไปไม่ถึงขา
3. Ctrl-C ที่ launch บนบอร์ด   ทั้ง stack ตาย
```

`sensor_watchdog` จะตัดคำสั่งเองถ้าเซนเซอร์หรือ TF หายเกิน `sensor_timeout`
ซึ่งตั้งไว้ที่ 0.5 วินาที แต่อย่าพึ่งพามันแทนคนถือรีโมท

---

## สิ่งที่ยังไม่มี

บันทึกไว้เพื่อไม่ให้คาดหวังผิดตอนอยู่หน้างาน

```text
ตัวเชื่อม UDP ยังไม่อยู่ใน launch file ต้องรันมือสองเทอร์มินัล
ยังไม่เคยยืนยันว่า --mode api ทำให้ Go2W เดินจริง ทดสอบครั้งแรกที่ไซต์
เส้นทางถ่ายรูปและอัปโหลดกลับเว็บยังไม่ได้เขียน
AprilTag ยังไม่เคยทดสอบที่ไซต์จริง
ยังไม่มีตัวเลขว่า AMCL คลาดเคลื่อนกี่เมตรบนหุ่นจริง
```

---

## ภาคผนวก — กรณีใช้ MiniPC ล้วน ไม่ใช้บอร์ด

เป็นการจัดวางแบบก่อนหน้า ยังใช้ได้และง่ายกว่ามาก เหมาะกับการทำแมพและทดสอบ
localization ตอนที่หุ่นยังไม่ต้องเดินไกล

ข้ามขั้น 0 ขั้น 2 และครึ่งหนึ่งของขั้น 3 ไปได้เลย ไม่ต้อง deploy ไม่ต้องเช็คลิงก์
เพราะทุกโหนดอยู่บนเครื่องเดียวกัน

### ต่อสาย

เซนเซอร์เสียบเข้าเน็ตเวิร์กภายในของ Go2 ซึ่ง MiniPC ก็เสียบอยู่เหมือนกัน

```text
192.168.123.20    Mid-360
192.168.123.18    MiniPC ที่การ์ด enp3s0
192.168.123.161   บอร์ด Unitree อยู่ในวงเดียวกัน แต่รอบนี้ไม่ได้ใช้
```

ค่าเหล่านี้อยู่ใน `config/livox_mid360_field.json` แล้ว ไม่ต้องแก้
ตั้ง IP ให้การ์ดก่อนถ้ายังไม่ได้ตั้ง

```bash
sudo ip addr add 192.168.123.18/24 dev enp3s0
sudo ip link set enp3s0 up
ping 192.168.123.20
```

เซนเซอร์ตอบ ping ต่างจากบอร์ดที่ไม่ตอบ

### ตั้งสภาพแวดล้อม

อย่า source `setup_robot_env.sh` หรือ `setup_ground_env.sh` ในโหมดนี้
สองไฟล์นั้นบังคับใช้ CycloneDDS ผูกกับการ์ด Wi-Fi ซึ่งเป็นเรื่องของการแยกสองเครื่อง
โหมดเครื่องเดียวใช้ Fast DDS บน loopback แทน

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/src/go2_control/config/fastdds_udp_only.xml
export ROS_LOCALHOST_ONLY=1
```

`fastdds_udp_only.xml` มีไว้เพื่อโหมดนี้โดยเฉพาะ มันปิด shared memory ของ Fast DDS
ซึ่งบน Foxy ชอบทิ้งพอร์ตล็อกค้างไว้เมื่อโหนดตายแบบไม่สวย แล้วรอบต่อไปสตาร์ทไม่ขึ้น

`ROS_LOCALHOST_ONLY=1` กันไม่ให้ topic รั่วออกไปหาบอร์ดหรือเครื่องอื่นในวง

ทำแบบนี้ **ทุกเทอร์มินัล** ที่จะรัน ROS

### รัน

เทอร์มินัล 1 เซนเซอร์กับ odometry

```bash
ros2 launch go2_control livox_robot.launch.py \
  replay:=false \
  enable_cmd_vel:=false \
  lio_pitch_deg:=13.0 \
  sensor_height:=0.35
```

เทอร์มินัล 2 ทำแมพ

```bash
ros2 launch go2_control livox_slam.launch.py enable_rviz:=true
```

หรือถ้ามีแมพแล้ว ใช้ localization กับ Nav2 แทน

```bash
ros2 launch go2_control livox_ground.launch.py \
  replay:=false \
  map:=$PWD/src/go2_control/map/livox_slam_02loop_edit.yaml \
  enable_rviz:=true
```

จากนั้นเช็คและเซฟแมพเหมือนขั้น 4 ถึง 6 ทุกอย่าง

### ข้อจำกัดที่ต้องรู้

```text
สายเซนเซอร์ลากจากหุ่นมา MiniPC หุ่นจึงเดินได้ไกลเท่าความยาวสายเท่านั้น
เดินเก็บแมพทั้งไซต์ในโหมดนี้ทำได้ยาก ถ้าไซต์ใหญ่กว่าสาย
เหมาะกับทดสอบว่าโค้ดถูก ไม่เหมาะกับพิสูจน์ว่าระบบใช้งานได้จริงที่ไซต์
```

ข้อดีคือไม่มีเรื่อง Wi-Fi เข้ามาเกี่ยวเลย point cloud วิ่งในสายกับใน loopback ทั้งหมด
ปัญหาที่เจอในโหมดนี้จึงเป็นปัญหาของโค้ดจริง ๆ ไม่ใช่ปัญหาเน็ต

### ถ้าจะสั่งหุ่นเดินในโหมดนี้

ยังต้องผ่านประตูเดิมทุกอย่าง และตัวเชื่อมฝั่ง Unitree ยังต้องใช้ CycloneDDS
ผูกกับการ์ดที่คุยกับหุ่นได้ จึงยังต้องแยกสองเทอร์มินัลเหมือนขั้น 8
ต่างกันแค่ `--interface` ที่ต้องเป็นการ์ดของ MiniPC ที่ต่อกับหุ่น ไม่ใช่ `wlp4s0`

```bash
python3 src/go2_control/go2_control/unitree_udp_bridge.py \
  --interface enp3s0 \
  --port 32123 \
  --timeout 0.5 \
  --robot-ack I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
```

---

## ภาคผนวก — รันทดสอบที่บ้าน ไม่มีหุ่น ไม่มีเซนเซอร์

มีสองแบบ เลือกตามว่าจะทดสอบอะไร

```text
เล่น bag ที่เก็บมาจากไซต์    ทดสอบ FAST-LIO, AMCL, Nav2 ด้วยข้อมูลจริง
Gazebo กับ TurtleBot3        ทดสอบว่าโค้ดคอมไพล์และโหนดขึ้นครบ ไม่ใช่ข้อมูลจริง
```

แบบแรกมีค่ามากกว่ามาก ใช้แบบแรกเป็นหลัก

### เตรียมทุกเทอร์มินัล

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_fastlio_livox/install/setup.bash
source install/setup.bash
```

ไม่ต้อง source `setup_robot_env.sh` หรือ `setup_ground_env.sh` งานที่บ้านอยู่เครื่องเดียว

---

### แบบที่ 1 — เล่น bag จากไซต์

ลำดับนี้ทดสอบผ่านมาแล้ว บันทึกไว้ใน `LIVOX_NAV2_REPLAY_2026-08-25.md`
เปิดสี่เทอร์มินัล เรียงตามนี้ อย่าสลับลำดับ

```bash
# 1  FAST-LIO ไม่เปิด driver เพราะข้อมูลมาจาก bag
ros2 launch go2_control livox_mid360_lio.launch.py \
    enable_driver:=false enable_tf_bridge:=false

# 2  แมพ AMCL การแปลงเป็น /scan และนาฬิกาที่ replay ต้องใช้
ros2 launch go2_control livox_amcl.launch.py enable_drift_check:=false

# 3  Nav2 ความเร็วที่ออกมาไม่ไปไหน
ros2 launch go2_control livox_nav2.launch.py enable_rviz:=true

# 4  เล่น bag
ros2 bag play ~/livox_bags_field/02_loopfix
```

**อย่าใส่ `--clock`** บน Foxy ยังไม่มีตัวเลือกนี้ เวลามาจากโหนด `bag_clock` แทน

ก่อนเล่น bag เช็คว่าไม่มีโหนดซ้ำซ้อน เคยเป็นปัญหามาแล้ว

```bash
ps -eo pid,comm | awk '$2 ~ /fastlio|amcl|map_serv|pointcloud|bag_clock|controller_s|planner_s/'
```

ควรเห็นอย่างละหนึ่งตัวเท่านั้น ถ้าเห็นซ้ำ แปลว่ามีเทอร์มินัลเก่าค้างอยู่ ปิดให้หมดก่อน

เช็คความปลอดภัยทุกครั้งก่อนส่งเป้าหมาย

```bash
ros2 topic info /cmd_vel        # ต้องได้ Unknown topic '/cmd_vel'
ros2 node list | grep cmd_vel   # ต้องไม่มีอะไรออกมา
```

ถ้าสองบรรทัดนี้ให้ผลอย่างอื่น แปลว่ามีทางไปถึงขาหุ่นอยู่ หยุดแล้วหาให้เจอก่อน

ส่งเป้าหมายจากปุ่ม 2D Goal Pose ใน RViz หรือจากบรรทัดคำสั่ง

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, \
    orientation: {w: 1.0}}}}"
```

bag ที่มีอยู่

```text
~/livox_bags_field/02_loopfix     909 MB  เดินวนรอบ ใช้ตัวนี้เป็นหลัก
~/livox_bags_field/04_rotateffix  302 MB  หมุนอยู่กับที่
~/livox_bags_field/01_static      151 MB  ยืนนิ่ง ใช้ดูว่า odometry ไหลไหม
```

ตัวที่ลงท้าย `fix` คือตัวที่แก้แล้ว ใช้ตัวนั้น

---

### แบบที่ 2 — Gazebo กับ TurtleBot3

ใช้ตรวจว่าโค้ดคอมไพล์และโหนดขึ้นครบเท่านั้น ข้อมูลไม่ใช่ของ Mid-360 จริง

```bash
ros2 launch go2_control sim.launch.py \
  enable_unitree_bridge:=true \
  enable_april:=false \
  enable_stream:=false \
  enable_init_pose:=false
```

อีกเทอร์มินัล ตรวจความพร้อม

```bash
ros2 run go2_control local_check
```

ควรได้ `Result: OK`

ขับในซิมด้วยคีย์บอร์ด

```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

---

### build ก่อนทดสอบทุกครั้งที่แก้โค้ด

```bash
cd /home/sys20/projects/systonic-2307/Systronic
source /opt/ros/foxy/setup.bash
colcon build --packages-select go2_control
source install/setup.bash
```

ถ้าแก้ launch file หรือ config แล้วไม่ build ระบบจะยังใช้ของเก่าใน `install/`
โดยไม่มีข้อความเตือน เป็นกับดักที่เสียเวลาบ่อยที่สุด
