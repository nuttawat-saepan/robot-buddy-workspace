"""Make a map from a web page: start SLAM, drive, save, stop.

    ros2 run go2_control map_maker --ros-args -p slam_command:="..." -p map_dir:=~/maps

Serves a page at /slam on its own port; the dashboard server forwards /slam to
it, so it shows up as http://<mini pc>:5173/slam beside the main page.

Saving does the two things slam_toolbox's save leaves undone, both of which
stopped the whole stack from coming up on 2026-09-14: the yaml's image line is
made a bare file name next to the .pgm, and the result is checked to load. It
does not point SITE_MAP at the new map; the page says so.

It refuses to start while AMCL is running. AMCL and slam_toolbox both publish
map -> odom, and the navigation stack has to be stopped before mapping.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import re
import signal
import struct
import subprocess
import threading
import time
import zlib

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

NAME = re.compile(r'^[A-Za-z0-9_-]{1,60}$')


def default_name(now=None):
    return time.strftime('site_%Y%m%d_%H%M', time.localtime(now))


def check_name(name, map_dir):
    """None if name can be saved in map_dir, else why not, in Thai."""
    if not NAME.match(name or ''):
        return 'ชื่อใช้ได้เฉพาะ a-z A-Z 0-9 _ - ไม่เกิน 60 ตัว'
    if os.path.exists(os.path.join(map_dir, name + '.yaml')):
        return 'มีแผนที่ชื่อนี้แล้ว ตั้งชื่อใหม่'
    return None


def fix_image_line(yaml_text):
    """map_server wants the .pgm by bare name, next to the yaml."""
    return re.sub(r'^image:\s*.*/', 'image: ', yaml_text, flags=re.M)


def grid_rgb(data, width, height):
    """OccupancyGrid cells as RGB rows, top row first: grey unknown, white free."""
    rows = []
    for row in range(height - 1, -1, -1):
        line = bytearray()
        for value in data[row * width:(row + 1) * width]:
            if value < 0:
                line += b'\xa0\xa0\xa0'
            elif value >= 65:
                line += b'\x10\x10\x10'
            else:
                line += b'\xff\xff\xff'
        rows.append(line)
    return rows


def png(rows, scale=1):
    """A PNG from RGB rows, each pixel repeated scale times both ways."""
    height = len(rows)
    width = len(rows[0]) // 3 if rows else 0
    raw = bytearray()
    for line in rows:
        wide = bytearray()
        for x in range(width):
            wide += line[3 * x:3 * x + 3] * scale
        for _ in range(scale):
            raw += b'\0' + wide

    def chunk(kind, body):
        return (struct.pack('>I', len(body)) + kind + body
                + struct.pack('>I', zlib.crc32(kind + body) & 0xffffffff))

    return (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', struct.pack('>IIBBBBB', width * scale,
                                         height * scale, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(bytes(raw)))
            + chunk(b'IEND', b''))


def known_area(data, resolution):
    """Square metres of the map that are known, free or occupied."""
    return sum(1 for value in data if value >= 0) * resolution * resolution


PAGE = """<!doctype html>
<html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Make a Map</title>
<style>
:root { --bg:#0b1a22; --panel:#13303c; --text:#e8f3f6; --dim:#9db4bd;
        --go:#00d0e0; --save:#2fbf71; --stop:#f25555; }
* { box-sizing:border-box; }
body { margin:0; padding:16px; background:var(--bg); color:var(--text);
       font-family:system-ui,sans-serif; }
main { max-width:1100px; margin:0 auto; display:grid; gap:16px;
       grid-template-columns:minmax(0,1fr) 340px; }
@media (max-width:820px) { main { grid-template-columns:1fr; } }
h1 { font-size:22px; margin:0 0 4px; }
a { color:var(--go); }
.card { background:var(--panel); border-radius:14px; padding:16px; }
#map { width:100%; min-height:300px; background:#a0a0a0; border-radius:10px;
       display:flex; align-items:center; justify-content:center; overflow:hidden; }
#map img { max-width:100%; max-height:70vh; image-rendering:pixelated; }
#state { font-size:18px; font-weight:600; margin-bottom:4px; }
#detail { color:var(--dim); font-size:14px; min-height:20px; }
button { width:100%; height:56px; border:0; border-radius:12px; font-size:18px;
         font-weight:600; margin-top:10px; cursor:pointer; color:#00181c; }
button:disabled { opacity:.35; cursor:not-allowed; }
#start { background:var(--go); }
#save { background:var(--save); }
#stop { background:var(--stop); color:#fff; }
input { width:100%; height:44px; border-radius:10px; border:1px solid #2b5161;
        background:#0b1a22; color:var(--text); padding:0 12px; font-size:16px;
        margin-top:10px; }
ol { padding-left:20px; color:var(--dim); font-size:14px; line-height:1.6; }
#saved { color:var(--save); font-size:14px; word-break:break-all; }
#error { color:var(--stop); font-size:14px; }
</style></head><body>
<h1>ทำแผนที่</h1>
<p style="color:var(--dim);margin:0 0 16px">
  ขับหุ่นด้วยแท็บ Joystick ที่ <a href="/" target="_blank">หน้าหลัก</a>
  หรือรีโมต ระหว่างทำแผนที่</p>
<main>
  <div class="card"><div id="map">ยังไม่มีแผนที่</div></div>
  <div>
    <div class="card">
      <div id="state">กำลังเชื่อมต่อ...</div>
      <div id="detail"></div>
      <button id="start">1. เริ่มทำแผนที่</button>
      <input id="name" placeholder="ชื่อแผนที่">
      <button id="save">2. บันทึกแผนที่</button>
      <button id="stop">3. หยุดทำแผนที่</button>
      <p id="saved"></p><p id="error"></p>
    </div>
    <div class="card" style="margin-top:16px">
      <b>วิธีทำ</b>
      <ol>
        <li>ให้หุ่นยืนนิ่งก่อนกดเริ่ม แล้วนิ่งต่ออีก 15 วินาที</li>
        <li>ขับช้า ๆ ให้ทั่วพื้นที่ เลี้ยวช้า ๆ</li>
        <li>วนกลับมาจุดเริ่ม แผนที่จะต่อกันพอดี</li>
        <li>แผนที่ครบแล้วกดบันทึก แล้วค่อยกดหยุด</li>
      </ol>
    </div>
  </div>
</main>
<script>
const el = id => document.getElementById(id);
let state = null;
const call = async (path, body) => {
  el('error').textContent = '';
  try {
    const r = await fetch('/slam/api/' + path, {method: 'POST',
      headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body || {})});
    const j = await r.json();
    if (!r.ok) el('error').textContent = j.message || ('ไม่สำเร็จ ' + r.status);
    if (j.saved) el('saved').textContent = 'บันทึกแล้ว: ' + j.saved;
  } catch (e) { el('error').textContent = 'ติดต่อตัวทำแผนที่ไม่ได้'; }
  poll();
};
el('start').onclick = () => call('start');
el('save').onclick = () => call('save', {name: el('name').value.trim()});
el('stop').onclick = () => {
  if (!state || state.unsaved === false || confirm('ยังไม่ได้บันทึกแผนที่ หยุดเลยไหม'))
    call('stop');
};
const poll = async () => {
  try {
    state = await (await fetch('/slam/api/status')).json();
  } catch (e) {
    el('state').textContent = 'ติดต่อตัวทำแผนที่ไม่ได้';
    return;
  }
  el('state').textContent = state.title;
  el('detail').textContent = state.detail || '';
  el('start').disabled = state.state !== 'idle';
  el('save').disabled = state.state !== 'mapping' || !state.hasMap;
  el('stop').disabled = state.state !== 'mapping';
  if (!el('name').value) el('name').value = state.suggestedName;
  if (state.hasMap) {
    const img = new Image();
    img.onload = () => { el('map').replaceChildren(img); };
    img.src = '/slam/api/map.png?t=' + Date.now();
  }
};
poll();
setInterval(poll, 2000);
</script></body></html>
"""


class MapMaker(Node):

    def __init__(self):
        super().__init__('map_maker')
        declare = self.declare_parameter
        self.slam_command = declare(
            'slam_command',
            'ros2 launch go2_control mapping_stack.launch.py replay:=false').value
        self.map_dir = os.path.expanduser(declare('map_dir', '~/maps').value)
        host = declare('host', '127.0.0.1').value
        port = int(declare('port', 8093).value)
        os.makedirs(self.map_dir, exist_ok=True)

        self.lock = threading.Lock()
        self.state = 'idle'
        self.message = ''
        self.process = None
        self.started_at = None
        self.saved_since_start = False
        self.last_saved = None
        self.grid = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self._watch_process)

        node = self

        class Handler(BaseHTTPRequestHandler):

            def reply(self, code, kind, body):
                self.send_response(code)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def json(self, code, data):
                self.reply(code, 'application/json', json.dumps(data).encode())

            def route(self):
                path = self.path.split('?')[0]
                return path[len('/slam'):] if path.startswith('/slam') else path

            def do_GET(self):
                path = self.route()
                if path in ('', '/'):
                    self.reply(200, 'text/html; charset=utf-8', PAGE.encode())
                elif path == '/api/status':
                    self.json(200, node.status())
                elif path == '/api/map.png':
                    image = node.map_png()
                    if image is None:
                        self.json(404, {'message': 'no map yet'})
                    else:
                        self.reply(200, 'image/png', image)
                else:
                    self.json(404, {'message': 'not found'})

            def do_POST(self):
                path = self.route()
                size = int(self.headers.get('Content-Length') or 0)
                try:
                    body = json.loads(self.rfile.read(size) or b'{}')
                except ValueError:
                    body = {}
                actions = {'/api/start': node.start, '/api/stop': node.stop,
                           '/api/save': lambda: node.save(body.get('name'))}
                if path not in actions:
                    self.json(404, {'message': 'not found'})
                    return
                code, data = actions[path]()
                self.json(code, data)

            def log_message(self, *_args):
                pass

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info('map page at http://%s:%d/slam, maps go to %s'
                               % (host, port, self.map_dir))

    # ------------------------------------------------------------ state

    def _on_map(self, msg):
        with self.lock:
            self.grid = msg

    def _watch_process(self):
        with self.lock:
            process = self.process
        if process is not None and process.poll() is not None \
                and self.state == 'mapping':
            with self.lock:
                self.state = 'idle'
                self.process = None
                self.message = ('SLAM หยุดเอง (exit %d) ดู log ของ map_maker'
                                % process.returncode)
            self.get_logger().error(self.message)

    def _robot(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link',
                                                 rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return None
        q = tf.transform.rotation
        return (tf.transform.translation.x, tf.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y),
                           1 - 2 * (q.y * q.y + q.z * q.z)))

    def status(self):
        with self.lock:
            state, grid, message = self.state, self.grid, self.message
            started, saved = self.started_at, self.saved_since_start
            last = self.last_saved
        has_map = grid is not None and state != 'idle'
        title = {'idle': 'พร้อมเริ่ม', 'starting': 'กำลังเริ่ม SLAM…',
                 'mapping': 'กำลังทำแผนที่', 'saving': 'กำลังบันทึก…',
                 'stopping': 'กำลังหยุด…'}[state]
        detail = message
        if state == 'mapping':
            elapsed = int(time.time() - started)
            area = known_area(grid.data, grid.info.resolution) if grid else 0
            detail = 'ผ่านไป %d:%02d · เห็นแล้ว %.0f ตร.ม.' % (
                elapsed // 60, elapsed % 60, area)
            if not grid:
                detail += ' · รอแผนที่แรก ขยับหุ่นเล็กน้อย'
        return {'state': state, 'title': title, 'detail': detail,
                'hasMap': has_map, 'unsaved': state == 'mapping' and not saved,
                'lastSaved': last, 'suggestedName': default_name(),
                'mapDir': self.map_dir}

    # ---------------------------------------------------------- actions

    def start(self):
        with self.lock:
            if self.state != 'idle':
                return 409, {'message': 'กำลังทำงานอยู่แล้ว'}
        if 'amcl' in self.get_node_names():
            return 409, {'message': 'ระบบนำทาง (AMCL) ยังรันอยู่ ต้องปิดก่อนทำแผนที่'}
        with self.lock:
            self.state = 'starting'
            self.grid = None
            self.message = ''
        # Own process group, so stop takes the whole launch down with it.
        process = subprocess.Popen(
            ['bash', '-c', 'exec ' + self.slam_command],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            stdin=subprocess.DEVNULL, start_new_session=True)
        with self.lock:
            self.process = process
            self.state = 'mapping'
            self.started_at = time.time()
            self.saved_since_start = False
        self.get_logger().info('SLAM started: %s' % self.slam_command)
        return 200, {'message': 'เริ่มแล้ว'}

    def stop(self):
        with self.lock:
            process = self.process
            if self.state != 'mapping' or process is None:
                return 409, {'message': 'ไม่ได้ทำแผนที่อยู่'}
            self.state = 'stopping'
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass
        with self.lock:
            self.process = None
            self.state = 'idle'
            self.grid = None
            self.message = 'หยุดแล้ว'
        return 200, {'message': 'หยุดแล้ว'}

    def save(self, name):
        name = (name or '').strip() or default_name()
        why = check_name(name, self.map_dir)
        if why:
            return 400, {'message': why}
        with self.lock:
            if self.state != 'mapping' or self.grid is None:
                return 409, {'message': 'ยังไม่มีแผนที่ให้บันทึก'}
            self.state = 'saving'
        base = os.path.join(self.map_dir, name)
        try:
            done = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', base,
                 '--ros-args', '-p', 'save_map_timeout:=10000'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                universal_newlines=True, timeout=40)
            ok = done.returncode == 0 and os.path.exists(base + '.yaml') \
                and os.path.exists(base + '.pgm')
            if ok:
                with open(base + '.yaml') as source:
                    text = fix_image_line(source.read())
                with open(base + '.yaml', 'w') as out:
                    out.write(text)
            else:
                self.get_logger().error('map_saver_cli: %s' % done.stdout[-500:])
        except subprocess.TimeoutExpired:
            ok = False
        with self.lock:
            self.state = 'mapping'
            if ok:
                self.saved_since_start = True
                self.last_saved = base + '.yaml'
        if not ok:
            return 500, {'message': 'บันทึกไม่สำเร็จ ลองอีกครั้ง'}
        self.get_logger().info('saved %s' % base)
        return 200, {'message': 'บันทึกแล้ว', 'saved': base + '.yaml'}

    # ------------------------------------------------------------ image

    def map_png(self):
        with self.lock:
            grid = self.grid
        if grid is None:
            return None
        info = grid.info
        rows = grid_rgb(grid.data, info.width, info.height)
        robot = self._robot()
        if robot is not None:
            col = int((robot[0] - info.origin.position.x) / info.resolution)
            row = info.height - 1 - int(
                (robot[1] - info.origin.position.y) / info.resolution)
            for dy in range(-3, 4):
                for dx in range(-3, 4):
                    r, c = row + dy, col + dx
                    if dx * dx + dy * dy <= 9 and 0 <= r < info.height \
                            and 0 <= c < info.width:
                        rows[r][3 * c:3 * c + 3] = b'\x00\xb8\xc8'
        scale = max(1, min(6, 700 // max(info.width, info.height, 1)))
        return png(rows, scale)

    def shutdown(self):
        if self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.process.wait(timeout=10)
            except Exception:                      # noqa: BLE001
                pass
        self.server.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MapMaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
