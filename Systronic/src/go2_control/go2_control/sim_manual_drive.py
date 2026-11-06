"""Drive the simulated box by hand from a small web page. Simulation only.

    browser -> this node -> /cmd_vel_nav_preview -> sensor_watchdog
            -> collision_guard -> sim_actuator -> the box

Started only by sim_obstacle.launch.py. Commands go in where Nav2's do, so the
watchdog and the guard both apply. It publishes neither /cmd_vel nor
/cmd_vel_guarded, and the relay and the Unitree bridge are not part of the
simulation, so this cannot reach a robot.

The page repeats the command while a key or button is held. When the repeats
stop, for whatever reason, the node sends one stop and goes quiet. While a
navigation goal is active it refuses to drive, so it never fights Nav2.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ACTIVE_GOAL = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
               GoalStatus.STATUS_CANCELING)

PAGE = """<!doctype html>
<html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sim Manual Drive</title>
<style>
:root { --bg:#0b1a22; --panel:#13303c; --text:#e8f3f6; --dim:#9db4bd;
        --accent:#00d0e0; --stop:#f25555; }
body { margin:0; padding:16px; background:var(--bg); color:var(--text);
       font-family:system-ui,sans-serif; display:flex; justify-content:center; }
main { width:100%; max-width:360px; }
h1 { font-size:20px; margin:0 0 4px; }
p { color:var(--dim); margin:0 0 16px; font-size:14px; }
#state { background:var(--panel); border-radius:12px; padding:12px;
         margin-bottom:16px; font-size:15px; min-height:20px; }
#state.bad { background:var(--stop); color:#fff; }
.pad { display:grid; grid-template-columns:repeat(3,1fr); gap:8px;
       margin-bottom:16px; }
button { height:80px; font-size:28px; border:0; border-radius:14px;
         background:var(--panel); color:var(--text); touch-action:none;
         user-select:none; cursor:pointer; }
button.on { background:var(--accent); color:#000; }
button.halt { background:var(--stop); color:#fff; font-size:16px; }
label { display:block; font-size:14px; margin-bottom:12px; color:var(--dim); }
input { width:100%; }
</style></head><body><main>
<h1>ขับหุ่นจำลอง</h1>
<p>Simulation เท่านั้น กดค้างเพื่อขับ ปล่อยแล้วหยุด ใช้ลูกศรบนคีย์บอร์ดได้</p>
<div id="state">กำลังเชื่อมต่อ...</div>
<div class="pad">
  <span></span><button data-key="ArrowUp">&uarr;</button><span></span>
  <button data-key="ArrowLeft">&#8634;</button>
  <button class="halt" id="halt">หยุด</button>
  <button data-key="ArrowRight">&#8635;</button>
  <span></span><button data-key="ArrowDown">&darr;</button><span></span>
</div>
<label>ความเร็วเดินหน้า/ถอย <b id="lv"></b> m/s
  <input id="linear" type="range" min="0.05" max="0.45" step="0.05" value="0.25"></label>
<label>ความเร็วหมุน <b id="av"></b> rad/s
  <input id="angular" type="range" min="0.1" max="0.6" step="0.1" value="0.4"></label>
</main><script>
const held = new Set();
const el = id => document.getElementById(id);
const label = () => {
  el('lv').textContent = el('linear').value;
  el('av').textContent = el('angular').value;
};
const send = (linear, angular) => fetch('/drive', {
  method: 'POST', body: JSON.stringify({linear, angular})}).catch(() => {});
const tick = () => {
  if (!held.size) return;
  const axis = (a, b) => (held.has(a) ? 1 : 0) - (held.has(b) ? 1 : 0);
  send(axis('ArrowUp', 'ArrowDown') * el('linear').value,
       axis('ArrowLeft', 'ArrowRight') * el('angular').value);
};
const paint = () => document.querySelectorAll('[data-key]').forEach(
  b => b.classList.toggle('on', held.has(b.dataset.key)));
const press = key => { held.add(key); paint(); tick(); };
const release = key => {
  held.delete(key); paint();
  if (held.size) tick(); else send(0, 0);
};
const releaseAll = () => { held.clear(); paint(); send(0, 0); };
document.querySelectorAll('[data-key]').forEach(b => {
  b.addEventListener('pointerdown', e => { e.preventDefault(); press(b.dataset.key); });
  b.addEventListener('pointerup', () => release(b.dataset.key));
  b.addEventListener('pointerleave', () => {
    if (held.has(b.dataset.key)) release(b.dataset.key);
  });
  b.addEventListener('pointercancel', () => release(b.dataset.key));
});
el('halt').addEventListener('click', releaseAll);
const arrows = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
addEventListener('keydown', e => {
  if (!arrows.includes(e.key)) return;
  e.preventDefault();
  if (!e.repeat) press(e.key);
});
addEventListener('keyup', e => { if (arrows.includes(e.key)) release(e.key); });
addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
el('linear').addEventListener('input', label);
el('angular').addEventListener('input', label);
label();
setInterval(tick, 100);
const poll = async () => {
  const state = el('state');
  try {
    const s = await (await fetch('/status')).json();
    state.textContent = s.message;
    state.classList.toggle('bad', s.blocked);
  } catch (e) {
    state.textContent = 'เชื่อมต่อ sim ไม่ได้';
    state.classList.add('bad');
  }
};
poll();
setInterval(poll, 500);
</script></body></html>
"""


def limit(value, largest):
    """A finite number within +-largest. Anything else is a stop."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    if value != value:
        return 0.0
    return max(-largest, min(largest, value))


class Deadman:
    """The last command, for as long as commands keep arriving."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.linear = 0.0
        self.angular = 0.0
        self.at = None

    def set(self, linear, angular, now):
        self.linear, self.angular, self.at = linear, angular, now

    def current(self, now):
        """(linear, angular), or None once the commands have stopped."""
        if self.at is None or now - self.at > self.timeout:
            return None
        return self.linear, self.angular

    def clear(self):
        self.at = None


def status_text(goal_active, guard_reason):
    """(message for the operator, whether driving is being refused)."""
    if goal_active:
        return 'มิชชันกำลังวิ่ง ขับเองไม่ได้จนกว่าจะหยุดมิชชัน', True
    if guard_reason:
        return 'ตัวกันชนไม่ให้ไปทางนี้: %s' % guard_reason, True
    return 'พร้อมขับ', False


class SimManualDrive(Node):

    def __init__(self):
        super().__init__('sim_manual_drive')
        declare = self.declare_parameter
        self.max_linear = float(declare('max_linear', 0.45).value)
        self.max_angular = float(declare('max_angular', 0.60).value)
        host = declare('host', '127.0.0.1').value
        port = int(declare('port', 8077).value)

        self.lock = threading.Lock()
        self.deadman = Deadman(float(declare('timeout', 0.40).value))
        self.moving = False
        self.goal_active = False
        self.guard_reason = None

        self.pub = self.create_publisher(Twist, '/cmd_vel_nav_preview', 10)
        # The QoS rclcpp_action gives this topic. Latched, so a goal that was
        # already running when this node started is still seen.
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.on_goal_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            String, '/collision_guard/status', self.on_guard_status, 10)
        self.create_timer(0.1, self.tick)

        node = self

        class Handler(BaseHTTPRequestHandler):

            def reply(self, code, kind, body):
                self.send_response(code)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/':
                    self.reply(200, 'text/html; charset=utf-8', PAGE.encode())
                elif self.path == '/status':
                    self.reply(200, 'application/json',
                               json.dumps(node.status()).encode())
                else:
                    self.reply(404, 'text/plain', b'not found')

            def do_POST(self):
                if self.path != '/drive':
                    self.reply(404, 'text/plain', b'not found')
                    return
                size = int(self.headers.get('Content-Length') or 0)
                try:
                    data = json.loads(self.rfile.read(size).decode())
                    node.drive(data.get('linear'), data.get('angular'))
                except (ValueError, AttributeError):
                    self.reply(400, 'text/plain', b'bad command')
                    return
                self.reply(200, 'application/json',
                           json.dumps(node.status()).encode())

            def log_message(self, *_args):
                pass

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(
            'simulation only - drive page at http://%s:%d' % (host, port))

    def drive(self, linear, angular):
        linear = limit(linear, self.max_linear)
        angular = limit(angular, self.max_angular)
        with self.lock:
            self.deadman.set(linear, angular, time.monotonic())

    def status(self):
        with self.lock:
            message, blocked = status_text(self.goal_active, self.guard_reason)
        return {'message': message, 'blocked': blocked}

    def on_goal_status(self, msg):
        active = any(s.status in ACTIVE_GOAL for s in msg.status_list)
        with self.lock:
            self.goal_active = active
            if active:
                self.deadman.clear()

    def on_guard_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.guard_reason = status.get('stopped')

    def tick(self):
        with self.lock:
            command = (None if self.goal_active
                       else self.deadman.current(time.monotonic()))
            goal_active = self.goal_active
        if command is not None:
            twist = Twist()
            twist.linear.x, twist.angular.z = command
            self.pub.publish(twist)
            self.moving = any(command)
        elif self.moving:
            self.moving = False
            # A running goal owns the topic; everything else gets one stop.
            if not goal_active:
                self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SimManualDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
