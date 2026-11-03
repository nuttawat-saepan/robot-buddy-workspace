# แผนทดสอบหน้างาน 4 พ.ย. 2026

เป้าหมาย: หาว่าการไหลเข้าโต๊ะเกิดจากอะไร, `alpha3` ที่ต่ำลงช่วยจริงไหม และตัวกรอง 1 m ใช้งานได้ไหม
ถือรีโมททุกครั้งที่หุ่นเดิน เริ่มที่ความเร็วไม่เกิน 0.2 m/s

## ก่อนออกจากออฟฟิศ
- [ ] Mini PC มีโค้ดล่าสุด และ `colcon build` ผ่าน
- [ ] เตรียมเทปทำเครื่องหมาย ตลับเมตร และรีโมท

## 1. เตรียมที่หน้างาน
1. ต่อทุกเครื่องเข้า Wi-Fi หน้างานเดิม แล้วจด IP ของ Mini PC, broker และบอร์ด
2. แก้ `scripts/onsite.env` บน Mini PC และ `CAMERA_SIGNAL` ใน `~/robot-app/config/backend.env` ให้ตรง IP
3. ส่งโค้ดขึ้นบอร์ด (`--dry-run` ก่อน):
   ```bash
   ./scripts/deploy_to_board.sh unitree@<ROBOT_IP> --dry-run
   ./scripts/deploy_to_board.sh unitree@<ROBOT_IP>
   ```
4. แก้ `~/go2_ws/scripts/onsite.env` บนบอร์ด:
   ```bash
   export GO2_ENFORCE_SPREAD_LIMIT=true
   export UNITREE_AUTO_ARM=0
   ```
   เมื่อ auto arm ปิด คำสั่งเปิด bridge ต้องใส่ `--robot-ack` เอง
5. เช็คเวลาและค่าบนบอร์ด เมื่อสแตกเปิดแล้ว:
   ```bash
   date
   ros2 param get /amcl alpha3
   ```

## 2. ทำจุดอ้างอิงในแต่ละห้อง
ทำเครื่องหมาย 3–5 จุด รวมจุดในทางแคบแถว (2.5, 2.5) วัดพิกัดจริงจากผนัง และถ่ายรูปห้องเทียบกับแมพ

## 3. เก็บค่าฐาน `alpha3 = 0.4` (ตัวกรองปิด)
ค่าหลักตอนนี้คือ `alpha3 = 0.2` การเก็บค่าฐานจึงต้องระบุ profile
```bash
ros2 launch go2_control onsite_stack.launch.py map:=$SITE_MAP replay:=false amcl_params_file:=amcl_livox_alpha3_04.yaml
```
```bash
ros2 bag record -o ~/bags/room_alpha04 /livox/lidar /livox/imu /scan /map /amcl_pose /particlecloud /tf /tf_static /Odometry
```
1. ตั้ง pose จากเว็บ แล้วยืนนิ่ง 10 วินาที ดู `/scan` ซ้อน `/map`
2. เดินผ่านทางแคบไปกลับ 3 รอบ หยุดจดตำแหน่งทุกจุดอ้างอิง
3. เดินทางเดินธรรมดา 1 ชุดเป็นตัวควบคุม

## 4. ทดสอบ `alpha3` ค่าอื่น (ตัวกรองปิด)
เปลี่ยนทีละค่า ทำทางเดิมและอัด bag ใหม่ทุกครั้ง: ค่าหลัก 0.2 (ไม่ต้องระบุ) แล้ว 0.1 และ 0.3 ถ้ามีเวลา
```bash
ros2 launch go2_control onsite_stack.launch.py map:=$SITE_MAP replay:=false
```
```bash
ros2 launch go2_control onsite_stack.launch.py map:=$SITE_MAP replay:=false amcl_params_file:=amcl_livox_alpha3_01.yaml
```
```bash
ros2 launch go2_control onsite_stack.launch.py map:=$SITE_MAP replay:=false amcl_params_file:=amcl_livox_alpha3_03.yaml
```

## 5. ทดสอบตัวกรอง 1 m (เปิดเมื่อเก็บค่าข้อ 3–4 แล้ว)
ตัวกรองส่ง `map->odom` แทน AMCL จึงมีผลกับการนำทางโดยตรง ทดสอบความเร็วต่ำและพร้อมหยุดด้วยรีโมท
```bash
ros2 launch go2_control onsite_stack.launch.py map:=$SITE_MAP replay:=false enable_localization_guard:=true
```
- [ ] เดินปกติแล้วไม่หยุดเอง และตำแหน่งตรงเหมือนตอนปิดตัวกรอง
- [ ] AMCL ต่างเกิน 1 m: ไม่ใช้ค่านั้น และขึ้นเตือน
- [ ] ต่างต่อเนื่อง 3 วินาที: มิชชัน pause และกด Resume แล้วยัง pause อยู่
- [ ] ตั้ง pose ใหม่แล้วกลับมาเดินต่อได้
- [ ] ถ้าผิดปกติ ให้หยุดด้วยรีโมท แล้วกลับไป launch แบบไม่มีตัวกรอง

## 5.1 ทดสอบอื่น ๆ
- [ ] ตั้ง pose ซ้ำ 3 รอบ โดยไม่ restart สแตก แล้วเดินสั้น ๆ
- [ ] เริ่มมิชชันเมื่อ spread เกิน 0.35 m ต้องถูกปฏิเสธและมีเหตุผลบนเว็บ
- [ ] แถบเตือนบนแผนที่ขึ้นเมื่อ AMCL ต่างจาก FAST-LIO เกิน 1 m
- [ ] ปรับ slider ระหว่างเดิน: `max_vel_x` และ `max_vel_theta` เปลี่ยน และหุ่นเดินเลี้ยวช้าลงตาม เริ่มจากช่วงต่ำก่อน เพราะที่ 100% ความเร็วเลี้ยวคือ 0.60 rad/s สูงกว่าค่าไฟล์ 0.35
- [ ] กล้องสดขึ้นเว็บ
- [ ] ดู `clusters` ใน CSV ว่า particle แยกกลุ่มบ่อยแค่ไหน

## 6. เกณฑ์ผ่าน
| หัวข้อ | เกณฑ์ |
|---|---|
| ตำแหน่ง | คลาดไม่เกิน 0.35 m ที่จุดอ้างอิง |
| ทิศ | คลาดไม่เกินราว 10° |
| เดินซ้ำ | 3 รอบ ไม่ไหลเข้าโต๊ะ |
| ตัวตรวจ | ไม่เตือนตอนตำแหน่งถูก และเตือนตอนไหลจริง |

## 7. ถ้าเจอปัญหา
| อาการ | ทำอะไร |
|---|---|
| Spread limit บล็อกจนทดสอบไม่ได้ | ตั้ง `GO2_ENFORCE_SPREAD_LIMIT=false` โดยจดเหตุผลและถือรีโมท |
| ตำแหน่งหลุด | หยุดด้วยรีโมท ตั้ง pose ใหม่ แล้วอัด bag รอบใหม่ |
| ตำแหน่งกระโดดหลายกิโลเมตร | FAST-LIO พัง: หยุดหุ่น รีสตาร์ทสแตกขณะหุ่นนิ่ง |
| `/scan` ไม่ทับแมพ | ทำแมพใหม่ก่อนจูนค่า |

## 8. ก่อนกลับ
เก็บกลับมาทุกอย่าง:
- `~/bags/room_alpha*`
- `~/go2_logs/localization_watch_*.csv`
- `~/go2_logs/` และ log ของสแตก
- ตารางจุดอ้างอิงที่จด

```bash
./scripts/backup_from_board.sh
```
