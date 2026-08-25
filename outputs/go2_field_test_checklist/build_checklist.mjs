import fs from "node:fs/promises";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const outputDir = "C:/Users/nutta/Desktop/projects/Systronic-20260722T074952Z-1-001/outputs/go2_field_test_checklist";
const workbook = Workbook.create();

const theme = {
  navy: "#0F172A",
  blue: "#2563EB",
  teal: "#0F766E",
  green: "#16A34A",
  amber: "#F59E0B",
  red: "#DC2626",
  slate: "#475569",
  lightBlue: "#DBEAFE",
  lightGreen: "#DCFCE7",
  lightAmber: "#FEF3C7",
  lightRed: "#FEE2E2",
  lightGray: "#F8FAFC",
  border: "#CBD5E1",
};

function styleTitle(sheet, title, subtitle) {
  sheet.showGridLines = false;
  sheet.getRange("A1:H1").merge();
  sheet.getRange("A1").values = [[title]];
  sheet.getRange("A1").format = {
    fill: theme.navy,
    font: { bold: true, color: "#FFFFFF", size: 16 },
  };
  sheet.getRange("A2:H2").merge();
  sheet.getRange("A2").values = [[subtitle]];
  sheet.getRange("A2").format = {
    fill: theme.lightBlue,
    font: { color: theme.navy, italic: true },
  };
  sheet.getRange("A1:H2").format.borders = { preset: "outside", style: "medium", color: theme.navy };
}

function styleHeader(range, fill = theme.teal) {
  range.format = {
    fill,
    font: { bold: true, color: "#FFFFFF" },
  };
  range.format.borders = { preset: "all", style: "thin", color: "#FFFFFF" };
}

function styleBody(range) {
  range.format = {
    fill: "#FFFFFF",
    font: { color: theme.navy },
  };
  range.format.borders = { preset: "all", style: "thin", color: theme.border };
  range.format.wrapText = true;
}

function addStatusValidation(sheet, rangeAddress) {
  sheet.getRange(rangeAddress).dataValidation = {
    rule: { type: "list", values: ["Not Started", "Checking", "Pass", "Fail", "Blocked", "N/A"] },
  };
}

const overview = workbook.worksheets.add("Overview");
styleTitle(
  overview,
  "Unitree Go2/Go2W Field Test Checklist",
  "Mini PC ROS2 Foxy → sensor bridge → lidar/map → manual movement → Nav2 waypoint → AprilTag pose adjustment"
);
overview.getRange("A4:B11").values = [
  ["เป้าหมายวันเทส", "ยืนยันว่า mini PC รับ sensor จากหุ่น, แปลงเป็น ROS2 topics, ดู lidar/map, สั่ง movement, แล้วค่อยเปิด Nav2/MQTT/AprilTag"],
  ["Architecture", "Unitree = sensor source + actuator | Mini PC = ROS2 brain / mapping / navigation / command calculation"],
  ["ROS distro", "Foxy"],
  ["Workspace path ที่คาดไว้", "~/UnitreeRos/unitree_ros2"],
  ["คำสั่ง real mode หลัก", "ros2 launch go2_control real.launch.py enable_cmd_vel:=true enable_camera:=false enable_april:=false"],
  ["คำสั่งหลังทุก terminal", "source /opt/ros/foxy/setup.bash && source install/setup.bash"],
  ["Red flag #1", "cmd_vel_node.py hardcode interface เป็น eth0 ต้องเทียบกับ interface จริงของ mini PC"],
  ["Red flag #2", "main.py hardcode MQTT broker เป็น 192.168.68.62:1883 ต้องเช็กกับ dashboard/broker จริง"],
];
styleHeader(overview.getRange("A4:A11"), theme.blue);
styleBody(overview.getRange("B4:B11"));
overview.getRange("A13:D18").values = [
  ["Metric", "Formula / Meaning", "Current", "Target"],
  ["Total checks", "จำนวน check ทั้งหมดใน Phase Checklist", null, "Reference"],
  ["Passed checks", "จำนวน Status = Pass", null, "เพิ่มขึ้นเรื่อย ๆ"],
  ["Failed / Blocked", "จำนวน Status = Fail หรือ Blocked", null, "ต้องเป็น 0 ก่อน autonomy"],
  ["Progress", "Passed / Total", null, "100% สำหรับ phase ที่จะใช้งาน"],
  ["Next gate", "ถ้า phase ปัจจุบัน fail อย่าไปต่อ", "ดู Decision Gates", "ทำตาม gate"],
];
overview.getRange("C14").formulas = [["=COUNTA('Phase Checklist'!A2:A200)"]];
overview.getRange("C15").formulas = [["=COUNTIF('Phase Checklist'!H2:H200,\"Pass\")"]];
overview.getRange("C16").formulas = [["=COUNTIF('Phase Checklist'!H2:H200,\"Fail\")+COUNTIF('Phase Checklist'!H2:H200,\"Blocked\")"]];
overview.getRange("C17").formulas = [["=IF(C14=0,0,C15/C14)"]];
styleHeader(overview.getRange("A13:D13"), theme.teal);
styleBody(overview.getRange("A14:D18"));
overview.getRange("C17").format.numberFormat = "0%";
overview.getRange("A:A").format.columnWidth = 22;
overview.getRange("B:B").format.columnWidth = 78;
overview.getRange("C:D").format.columnWidth = 18;

const phases = [
  [1, "Network", "mini PC คุยกับหุ่นได้", "ip addr", "เห็น interface + IP ของ mini PC", "ถ้าไม่มี IP ให้แก้ DHCP/static IP/สาย LAN", "High", "Not Started", ""],
  [1, "Network", "ping หุ่น", "ping <IP_หุ่น>", "ping ตอบกลับนิ่ง", "ถ้าไม่ตอบ เช็ก subnet, route, Wi‑Fi/LAN, firewall", "High", "Not Started", ""],
  [2, "Raw Unitree Topics", "เห็น topic ดิบจากหุ่น", "source /opt/ros/foxy/setup.bash; ros2 topic list", "เห็น /lf/sportmodestate, /lf/lowstate, /utlidar/cloud", "ถ้าไม่เห็น ปัญหาอยู่ที่ network/DDS/Unitree bridge ยังไม่ใช่ go2_control", "Critical", "Not Started", ""],
  [2, "Raw Unitree Topics", "ข้อมูล sensor ไหลจริง", "ros2 topic hz /lf/sportmodestate; ros2 topic hz /utlidar/cloud", "hz ขึ้น ไม่ค้าง", "ถ้า topic มีแต่ไม่มี hz ให้ดู publisher ฝั่งหุ่น/service lidar", "Critical", "Not Started", ""],
  [3, "Workspace Build", "build ROS2 workspace", "cd ~/UnitreeRos/unitree_ros2; colcon build", "build ผ่าน", "ถ้า error unitree_go/nav2_msgs/apriltag_msgs ให้จด dependency ที่ขาด", "High", "Not Started", ""],
  [3, "Workspace Build", "source แล้ว ROS เห็น package", "source install/setup.bash; ros2 pkg list | grep go2", "เห็น go2_control และ go2_interfaces", "ถ้าไม่เห็น แปลว่ายังไม่ได้ build/source workspace", "High", "Not Started", ""],
  [4, "Sensor Bridge", "รัน go2w_read", "ros2 run go2_control go2w_read", "node ไม่ crash", "ถ้า import error ให้แก้ dependency ก่อน", "Critical", "Not Started", ""],
  [4, "Sensor Bridge", "ได้ ROS standard topics", "ros2 topic list", "เห็น /odom, /imu/data, /pointcloud, /battery, /joint_states", "ถ้าไม่มี /odom กลับไปเช็ก /lf/sportmodestate; ถ้าไม่มี /pointcloud เช็ก /utlidar/cloud", "Critical", "Not Started", ""],
  [4, "Sensor Bridge", "เช็ก odom/imu/pointcloud", "ros2 topic hz /odom; ros2 topic hz /pointcloud", "data ไหลสม่ำเสมอ", "ถ้า hz แปลก/หลุด เช็ก network load และ frame_id", "High", "Not Started", ""],
  [5, "Lidar / Scan", "แปลง pointcloud เป็น scan", "ros2 launch go2_control mapping.launch.py หรือ real.launch.py", "เห็น /scan", "ถ้ามี /pointcloud แต่ไม่มี /scan ให้เช็ก pointcloud_to_laserscan และ pc2scan.yaml", "High", "Not Started", ""],
  [5, "Lidar / Scan", "เปิด RViz ดู lidar", "rviz2", "เห็น /pointcloud หรือ /scan ไม่ error TF หนัก", "ถ้า No transform ให้เช็ก /tf, fixed frame, base_link/radar/odom", "High", "Not Started", ""],
  [6, "Manual Movement", "เช็ก interface Unitree SDK", "ip addr; compare with ChannelFactoryInitialize(0, \"eth0\")", "ชื่อ interface ในโค้ดตรงกับ port ที่ต่อหุ่น", "ถ้าไม่ตรง แก้ eth0 เป็น enp*/eno*/wlan* ที่ถูกต้อง", "Critical", "Not Started", ""],
  [6, "Manual Movement", "รัน cmd_vel_node", "ros2 run go2_control cmd_vel_node", "node ไม่ crash และพร้อมรับ /cmd_vel", "ถ้า SDK error กลับไป network/interface", "Critical", "Not Started", ""],
  [6, "Manual Movement", "สั่งเดินช้ามาก", "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {z: 0.0}}\" -r 5", "หุ่นขยับเบา ๆ ตามคำสั่ง", "ถ้าไม่ขยับแต่ /cmd_vel มี ให้เช็ก SDK/interface; ถ้าแรงไปลดเหลือ 0.03", "Critical", "Not Started", ""],
  [6, "Manual Movement", "หยุดหุ่น", "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}\" -1", "หุ่นหยุด", "ถ้าไม่หยุด kill publisher / ส่ง zero ซ้ำ / E-stop", "Critical", "Not Started", ""],
  [7, "Mapping", "เปิด SLAM/map", "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false", "เห็น /map", "ถ้าไม่มี /map แปลว่า SLAM ไม่ได้รันหรือรับ /scan ไม่ได้", "Medium", "Not Started", ""],
  [7, "Mapping", "เดินช้า ๆ เพื่อสร้าง map", "manual cmd_vel หรือ joystick speed ต่ำ", "map ขยายและไม่เละ", "ถ้า map เละ เช็ก odom drift, scan frame, ความเร็ว, TF", "High", "Not Started", ""],
  [8, "Nav2 RViz Goal", "รัน Nav2 real แบบยังปิด camera/april", "ros2 launch go2_control real.launch.py enable_cmd_vel:=true enable_camera:=false enable_april:=false", "Nav2 active และมี /navigate_to_pose", "ถ้า lifecycle ไม่ active ดู log map/params", "High", "Not Started", ""],
  [8, "Nav2 RViz Goal", "set initial pose + ส่ง goal ใกล้ ๆ", "RViz: 2D Pose Estimate → Nav2 Goal", "path ขึ้นและหุ่นเดินถึง goal", "ถ้ามี path แต่ไม่เดิน เช็ก /cmd_vel/cmd_vel_node; ถ้าไม่มี path เช็ก map/pose/costmap", "Critical", "Not Started", ""],
  [9, "MQTT Mission", "เช็ก broker", "ping 192.168.68.62", "ping broker ได้ หรือแก้ IP ให้ตรง", "ถ้า main.py ค้าง connect ให้แก้ broker IP", "Medium", "Not Started", ""],
  [9, "MQTT Mission", "ทดสอบ mission waypoint", "Dashboard publish /missions/start", "mission accepted + status/progress กลับ dashboard", "ถ้าไม่ไป เช็ก payload/topic/statusTopic/progressTopic", "High", "Not Started", ""],
  [10, "AprilTag", "เปิด camera/april", "ros2 launch go2_control real.launch.py enable_cmd_vel:=true enable_camera:=true enable_april:=true", "node กล้องและ april_localizer ไม่ crash", "ถ้า camera/TF error อย่า localize ก่อน", "Medium", "Not Started", ""],
  [10, "AprilTag", "เช็ก detection", "ros2 topic hz /detections", "เห็น AprilTag detection", "ถ้าไม่เห็น เช็ก apriltag_ros/camera_info/light/tag size", "High", "Not Started", ""],
  [10, "AprilTag", "ทดสอบ pose adjustment", "mission waypoint localize.enabled=true", "/localization/status = SUCCESS และ /initialpose ถูก publish", "ถ้า fail เช็ก tagPoses, expectedTagId, max_pose_jump, camera_link→base_link TF", "High", "Not Started", ""],
];

const checklist = workbook.worksheets.add("Phase Checklist");
styleTitle(checklist, "Phase Checklist", "ไล่จากบนลงล่าง: ถ้า phase ปัจจุบันไม่ผ่าน อย่าเพิ่งไปต่อ phase ถัดไป");
checklist.getRange("A4:I4").values = [["Phase", "Area", "Check", "Command / Action", "Pass Criteria", "If Fail / Fallback", "Risk", "Status", "Notes"]];
checklist.getRange(`A5:I${4 + phases.length}`).values = phases;
styleHeader(checklist.getRange("A4:I4"), theme.teal);
styleBody(checklist.getRange(`A5:I${4 + phases.length}`));
checklist.tables.add(`A4:I${4 + phases.length}`, true, "PhaseChecklistTable");
addStatusValidation(checklist, `H5:H${4 + phases.length}`);
checklist.getRange(`G5:G${4 + phases.length}`).dataValidation = {
  rule: { type: "list", values: ["Critical", "High", "Medium", "Low"] },
};
checklist.getRange("A:A").format.columnWidth = 8;
checklist.getRange("B:B").format.columnWidth = 20;
checklist.getRange("C:C").format.columnWidth = 26;
checklist.getRange("D:D").format.columnWidth = 62;
checklist.getRange("E:F").format.columnWidth = 42;
checklist.getRange("G:H").format.columnWidth = 14;
checklist.getRange("I:I").format.columnWidth = 32;
checklist.freezePanes.freezeRows(4);

const risks = [
  ["No raw Unitree topics", "ros2 topic list ไม่เห็น /lf/sportmodestate หรือ /utlidar/cloud", "Critical", "Network/DDS/Unitree bridge/interface", "หยุดที่ Phase 2; เทียบ network กับเครื่องเดิม; เช็ก IP/subnet/firewall/ROS_DOMAIN_ID", "อย่าเสียเวลา debug Nav2"],
  ["go2w_read crash", "import error หรือ node ตายทันที", "High", "dependency เช่น unitree_go/go2_interfaces ขาด", "colcon build ใหม่; เช็ก package list; จด error แรก", "ห้ามแก้หลายอย่างพร้อมกัน"],
  ["No /scan", "มี /pointcloud แต่ไม่มี /scan", "High", "pointcloud_to_laserscan ไม่ได้รันหรือ config filter ผิด", "รัน mapping/real launch; เช็ก pc2scan.yaml; ดู frame_id ของ pointcloud", "Nav2 จะหลบไม่ได้"],
  ["TF error in RViz", "No transform จาก frame หนึ่งไปอีก frame", "High", "robot_state_publisher/odom TF/lidar frame ขาด", "ดู /tf; fixed frame ใช้ odom/map ให้ถูก; เช็ก base_link/radar", "อย่า test autonomy จน TF นิ่ง"],
  ["cmd_vel no movement", "publish /cmd_vel แล้วหุ่นนิ่ง", "Critical", "Unitree SDK interface ผิด เช่น eth0 ไม่ใช่พอร์ตจริง", "เช็ก ip addr; แก้ ChannelFactoryInitialize; เช็ก network", "จุดนี้มักกินเวลาสุด"],
  ["Robot does not stop", "ส่ง zero แล้วไม่หยุด", "Critical", "publisher ยังส่ง cmd ต่อ / SDK delay / node ค้าง", "kill publisher; ส่ง zero ซ้ำ; E-stop; ลด test speed", "ต้องมีคนพร้อม stop"],
  ["Map distorted", "แผนที่บิด/เละ/กระโดด", "High", "odom drift, scan frame ผิด, เดินเร็วเกิน", "เดินช้าลง; เช็ก TF; เช็ก /scan alignment", "อย่าใช้ map นี้กับ Nav2 จริง"],
  ["Nav2 path but no movement", "RViz ขึ้น path แต่หุ่นไม่เดิน", "High", "/cmd_vel ไม่ถึง cmd_vel_node หรือ cmd_vel_node ไม่ส่ง SDK", "echo /cmd_vel; เช็ก cmd_vel_node; เช็ก enable_cmd_vel:=true", "แยกปัญหา planner vs actuator"],
  ["MQTT stuck", "main.py ค้าง connect หรือไม่ได้ mission", "Medium", "broker IP/topic/payload ไม่ตรง", "เช็ก 192.168.68.62; mosquitto_sub; เทียบ payload", "ทดสอบ RViz goal ผ่านก่อน MQTT"],
  ["AprilTag localization fails", "ไม่มี /detections หรือ status FAILED", "High", "camera/camera_info/tag pose/TF ผิด", "เช็ก /detections; tagPoses; expectedTagId; camera_link→base_link", "เปิดเป็น phase สุดท้ายเท่านั้น"],
];
const riskSheet = workbook.worksheets.add("Risk Playbook");
styleTitle(riskSheet, "Risk Playbook", "อาการเสียหลัก ๆ พร้อมวิธีรับมือเร็วหน้างาน");
riskSheet.getRange("A4:F4").values = [["Risk", "Symptom", "Severity", "Likely Cause", "Immediate Response", "Note"]];
riskSheet.getRange(`A5:F${4 + risks.length}`).values = risks;
styleHeader(riskSheet.getRange("A4:F4"), theme.red);
styleBody(riskSheet.getRange(`A5:F${4 + risks.length}`));
riskSheet.tables.add(`A4:F${4 + risks.length}`, true, "RiskPlaybookTable");
riskSheet.getRange("A:A").format.columnWidth = 28;
riskSheet.getRange("B:B").format.columnWidth = 36;
riskSheet.getRange("C:C").format.columnWidth = 12;
riskSheet.getRange("D:E").format.columnWidth = 44;
riskSheet.getRange("F:F").format.columnWidth = 32;
riskSheet.freezePanes.freezeRows(4);

const commands = [
  ["Network", "ดู interface", "ip addr", "หา eth0/enp*/wlan* และ IP"],
  ["Network", "ping หุ่น", "ping <IP_หุ่น>", "ต้องตอบกลับ"],
  ["ROS env", "source ROS", "source /opt/ros/foxy/setup.bash", "ทำทุก terminal"],
  ["Build", "build workspace", "cd ~/UnitreeRos/unitree_ros2; colcon build; source install/setup.bash", "หลัง build ต้อง source"],
  ["Package", "เช็ก package", "ros2 pkg list | grep go2", "ต้องเห็น go2_control/go2_interfaces"],
  ["Raw topics", "list topics", "ros2 topic list", "หา /lf/sportmodestate /lf/lowstate /utlidar/cloud"],
  ["Raw topics", "topic rate", "ros2 topic hz /utlidar/cloud", "ดูว่าข้อมูลไหล"],
  ["Sensor bridge", "run go2w_read", "ros2 run go2_control go2w_read", "แปลง raw → /odom /imu /pointcloud"],
  ["Sensor bridge", "check odom", "ros2 topic hz /odom", "ต้องมี rate"],
  ["Lidar", "check scan", "ros2 topic hz /scan", "ต้องมีหลัง pointcloud_to_laserscan"],
  ["RViz", "open RViz", "rviz2", "ดู /scan /pointcloud /odom /tf"],
  ["Movement", "run movement bridge", "ros2 run go2_control cmd_vel_node", "รับ /cmd_vel แล้วสั่ง SDK"],
  ["Movement", "slow forward", "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {z: 0.0}}\" -r 5", "ใช้พื้นที่โล่ง"],
  ["Movement", "stop", "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}\" -1", "หยุดทันที"],
  ["Mapping", "SLAM fallback", "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false", "ถ้า mapping.launch.py ไม่เปิด SLAM"],
  ["Nav2", "real launch basic", "ros2 launch go2_control real.launch.py enable_cmd_vel:=true enable_camera:=false enable_april:=false", "ลอง RViz goal ก่อน dashboard"],
  ["Nav2", "check action", "ros2 action list | grep navigate", "ควรมี /navigate_to_pose"],
  ["MQTT", "ping broker", "ping 192.168.68.62", "แก้ IP ใน main.py ถ้าไม่ตรง"],
  ["AprilTag", "check detections", "ros2 topic hz /detections", "ต้องเห็น tag ก่อน localize"],
];
const cmdSheet = workbook.worksheets.add("Commands");
styleTitle(cmdSheet, "Command Cheat Sheet", "คำสั่งที่ใช้บ่อย แยกตามจุดเทส");
cmdSheet.getRange("A4:D4").values = [["Area", "Purpose", "Command", "Expected / Notes"]];
cmdSheet.getRange(`A5:D${4 + commands.length}`).values = commands;
styleHeader(cmdSheet.getRange("A4:D4"), theme.blue);
styleBody(cmdSheet.getRange(`A5:D${4 + commands.length}`));
cmdSheet.tables.add(`A4:D${4 + commands.length}`, true, "CommandsTable");
cmdSheet.getRange("A:B").format.columnWidth = 20;
cmdSheet.getRange("C:C").format.columnWidth = 78;
cmdSheet.getRange("D:D").format.columnWidth = 44;
cmdSheet.freezePanes.freezeRows(4);

const gates = [
  ["Gate 1", "เห็น /lf/sportmodestate /lf/lowstate /utlidar/cloud ไหม", "เห็นครบ + hz มีข้อมูล", "ไป Sensor Bridge", "หยุด: แก้ network/DDS/Unitree bridge"],
  ["Gate 2", "รัน go2w_read แล้วเห็น /odom /imu/data /pointcloud ไหม", "เห็นครบ + data ไหล", "ไป Lidar/Scan", "หยุด: แก้ dependency/raw topic/frame"],
  ["Gate 3", "มี /scan และ RViz เห็น lidar ไหม", "scan ถูกทิศ + TF พอใช้", "ไป Manual Movement", "หยุด: แก้ pointcloud_to_laserscan/TF"],
  ["Gate 4", "ส่ง /cmd_vel แล้วหุ่นขยับและหยุดได้ไหม", "ขยับช้า ๆ + stop ได้", "ไป Mapping/Nav2", "หยุด: แก้ SDK interface/E-stop safety"],
  ["Gate 5", "RViz Nav2 Goal เดินได้ไหม", "path ขึ้น + เดินถึง goal", "ไป MQTT Mission", "หยุด: แก้ map/initialpose/costmap/cmd_vel"],
  ["Gate 6", "MQTT mission เดิน waypoint ได้ไหม", "accepted + progress/status กลับ", "ไป AprilTag", "หยุด: แก้ broker/topic/payload"],
  ["Gate 7", "AprilTag detection + localization success ไหม", "/detections มี + /localization/status SUCCESS", "เปิด pose adjustment ใน mission", "หยุด: แก้ camera_info/tagPoses/TF/guard"],
];
const gateSheet = workbook.worksheets.add("Decision Gates");
styleTitle(gateSheet, "Decision Gates", "ใช้ตัดสินใจว่าไปต่อได้ไหม ไม่ให้ debug ข้ามชั้น");
gateSheet.getRange("A4:E4").values = [["Gate", "Question", "Pass Criteria", "If Pass", "If Fail"]];
gateSheet.getRange(`A5:E${4 + gates.length}`).values = gates;
styleHeader(gateSheet.getRange("A4:E4"), theme.amber);
styleBody(gateSheet.getRange(`A5:E${4 + gates.length}`));
gateSheet.tables.add(`A4:E${4 + gates.length}`, true, "DecisionGatesTable");
gateSheet.getRange("A:A").format.columnWidth = 12;
gateSheet.getRange("B:C").format.columnWidth = 44;
gateSheet.getRange("D:E").format.columnWidth = 36;
gateSheet.freezePanes.freezeRows(4);

const sheets = [overview, checklist, riskSheet, cmdSheet, gateSheet];
for (const sheet of sheets) {
  const used = sheet.getUsedRange();
  used.format.wrapText = true;
  used.format.autofitRows();
}

const statusRange = checklist.getRange(`H5:H${4 + phases.length}`);
statusRange.conditionalFormats.add("containsText", { text: "Pass", format: { fill: theme.lightGreen, font: { color: "#166534", bold: true } } });
statusRange.conditionalFormats.add("containsText", { text: "Fail", format: { fill: theme.lightRed, font: { color: "#991B1B", bold: true } } });
statusRange.conditionalFormats.add("containsText", { text: "Blocked", format: { fill: theme.lightRed, font: { color: "#991B1B", bold: true } } });
statusRange.conditionalFormats.add("containsText", { text: "Checking", format: { fill: theme.lightAmber, font: { color: "#92400E", bold: true } } });

const riskRange = checklist.getRange(`G5:G${4 + phases.length}`);
riskRange.conditionalFormats.add("containsText", { text: "Critical", format: { fill: theme.lightRed, font: { color: "#991B1B", bold: true } } });
riskRange.conditionalFormats.add("containsText", { text: "High", format: { fill: "#FFEDD5", font: { color: "#9A3412", bold: true } } });

await fs.mkdir(outputDir, { recursive: true });

const inspect = await workbook.inspect({
  kind: "table",
  range: "Phase Checklist!A4:I12",
  include: "values,formulas",
  tableMaxRows: 12,
  tableMaxCols: 9,
});
console.log(inspect.ndjson);

const errors = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
  options: { useRegex: true, maxResults: 100 },
  summary: "final formula error scan",
});
console.log(errors.ndjson);

for (const sheetName of ["Overview", "Phase Checklist", "Risk Playbook", "Commands", "Decision Gates"]) {
  const preview = await workbook.render({ sheetName, autoCrop: "all", scale: 1, format: "png" });
  await fs.writeFile(`${outputDir}/${sheetName.replaceAll(" ", "_")}.png`, new Uint8Array(await preview.arrayBuffer()));
}

const xlsx = await SpreadsheetFile.exportXlsx(workbook);
await xlsx.save(`${outputDir}/go2_field_test_checklist.xlsx`);
console.log(`${outputDir}/go2_field_test_checklist.xlsx`);
