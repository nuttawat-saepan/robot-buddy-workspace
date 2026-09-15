# ชุดคำสั่งหน้างาน

ตัวอย่างใช้ IP บอร์ด `192.168.80.111` — เปลี่ยนตาม Wi-Fi ไซต์นั้น

---

## ไซต์ใหม่ / Wi-Fi ใหม่ — แก้ env ก่อนทุกอย่าง

ทุกไซต์ได้ IP คนละวง ถ้าไม่แก้ เว็บจะว่างเปล่าโดยไม่มีอะไรฟ้อง
ตอน `source setup_*_env.sh` ถ้า IP ไม่ตรงวง จะขึ้น `!!! onsite.env does not match`

**1. ดู IP ของเครื่องตัวเอง**
```bash
ip -4 addr | grep 192.168
```

**2. MiniPC** `scripts/onsite.env`
```text
GROUND_IP=<IP MiniPC>
ROBOT_IP=<IP บอร์ด>
MQTT_BROKER=<IP broker>
```

**3. บอร์ด** `~/go2_ws/scripts/onsite.env` — สามค่าเดียวกัน
```bash
nano ~/go2_ws/scripts/onsite.env
```

**4. เว็บ** `MQTT_URL=mqtt://<IP broker>:1883` และถ้าเบราว์เซอร์ต่อเอง `ws://<IP broker>:9001`

**5. source ใหม่** แล้วดูว่าไม่มี `!!!`

---

## เปิดหุ่นให้ใช้งาน

**1. เช็คว่าบอร์ดตอบ** (MiniPC)
```bash
nc -vz -w 3 192.168.80.111 22
```

**2. ตั้งนาฬิกาบอร์ด** (MiniPC) — ทุกครั้งที่หุ่นเปิดใหม่
```bash
ssh -t unitree@192.168.80.111 "sudo date -s \"$(date -u +'%F %T') UTC\""
```

**3. ส่งโค้ดขึ้นบอร์ด** (MiniPC) — เฉพาะตอนโค้ดเปลี่ยน
```bash
cd ~/projects/systonic-2307/Systronic
./scripts/deploy_to_board.sh unitree@192.168.80.111
```

**4. เข้าบอร์ด**
```bash
ssh unitree@192.168.80.111
tmux new -s go2
```

**5. เปิดสแตก** — หุ่นยืนนิ่ง 20 วินาที
```bash
source ~/go2_ws/scripts/setup_robot_env.sh
ros2 launch go2_control onsite_stack.launch.py map:=$SITE_MAP replay:=false
```
รอจนเห็น `Managed nodes are active`

**6. เปิดตัวสั่งหุ่น** — กด `Ctrl-b c` ก่อน · รีโมทต้องอยู่ในมือ
```bash
source ~/go2_ws/scripts/setup_sdk_env.sh
ros2 run go2_control unitree_udp_bridge --mode raw --interface eth0 --max-linear 0.45 --max-angular 0.6 --timeout 0.30 --robot-ack I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT
```
รอจนเห็น `armed: raw mode`

**7. เปิดหน้าต่างใช้งาน** — กด `Ctrl-b c`
```bash
source ~/go2_ws/scripts/setup_robot_env.sh
```

**8. ตั้งตำแหน่งหุ่น** — บนเว็บ กดตั้งตำแหน่ง แล้วรอเขียว

**9. เช็คพร้อมเดิน** (หน้าต่างข้อ 7)
```bash
ros2 run go2_control nav_ready_check
```
ต้องขึ้น `READY`

**10. สั่งมิชชันจากเว็บ**

---

## ปิดหุ่นตอนเลิก

**1. ปิดทุกอย่างบนบอร์ด**
```bash
tmux kill-server
```

**2. แบ็กอัพ** (MiniPC)
```bash
cd ~/projects/systonic-2307/Systronic
./scripts/backup_from_board.sh unitree@192.168.80.111
```

---

## ใช้บ่อยระหว่างเทส (หน้าต่างข้อ 7)

**ลดความเร็ว**
```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.20
ros2 param set /controller_server FollowPath.max_vel_theta 0.30
```

**จุดถ่ายรูปให้หมุนถ่าย 4 มุม**
```bash
ros2 param set /mqtt_mission_bridge capture_views 4
```

**ดูตำแหน่งหุ่น**
```bash
ros2 run go2_control yaw_watch
```

**สั่งเดินโดยไม่ใช้เว็บ** — ไปจุด x=0.47 y=0.82 หันทาง -1.60 เรเดียน
```bash
ros2 run go2_control send_mission --goal 0.47 0.82 -1.60
```

**หุ่นช้า/เลยเป้า เช็คว่าช้าตรงไหน**
```bash
ros2 run go2_control pipeline_latency
```

---

## แก้ปัญหา

**แมพไม่ขึ้นเว็บ / `Invalid frame ID "map"` ขึ้นไม่หยุด**
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

**เว็บไม่ได้อะไรเลย** — ต้องได้ 1
```bash
ros2 node list | grep -c "^/mqtt_mission_bridge$"
```

**ssh หลุดแล้วกลับเข้า**
```bash
ssh unitree@192.168.80.111
tmux attach -t go2
```

---

## ทำแมพใหม่ (ห้องใหม่)

**1. เปิดโหมดทำแมพ** แล้วเข็นหุ่นทั่วห้อง กลับมาจุดเดิม
```bash
source ~/go2_ws/scripts/setup_robot_env.sh
ros2 launch go2_control mapping_stack.launch.py
```

**2. บันทึก** (อีกหน้าต่าง)
```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/unitree/maps/site_room2'}}"
```

**3. แก้ไฟล์แมพ**
```bash
sed -i 's|^image: .*/|image: |' ~/maps/site_room2.yaml
```

**4. บอกสแตกให้ใช้แมพใหม่**
```bash
sed -i "s|^export SITE_MAP=.*|export SITE_MAP=/home/unitree/maps/site_room2.yaml|" ~/go2_ws/scripts/onsite.env
```

**5. ปิดโหมดทำแมพ (`Ctrl-c`) แล้วกลับไปข้อ 5 ของ "เปิดหุ่นให้ใช้งาน"**

---

## ห้าม

- เปิด RViz ค้างไว้ตอนหุ่นเดิน — ทำหุ่นช้า 2 วินาที
- รัน `mqtt_mission_bridge` เอง — สแตกเปิดให้แล้ว รันซ้ำจะสั่งแย่งกัน
- ข้อ 6 ใช้ `setup_sdk_env.sh` ที่เหลือใช้ `setup_robot_env.sh` — สลับกันแล้วใช้ไม่ได้
