"""Verify the staleness watchdog in cmd_vel_node without any robot.

unitree_sdk2py is replaced by a stub before the module is imported, so
ChannelFactoryInitialize does nothing and SportClient.Move only records the
call. Nothing is connected, nothing is published, nothing can move.
"""
import os, sys, types, time
import rclpy
from geometry_msgs.msg import Twist

moves = []

sdk = types.ModuleType('unitree_sdk2py')
core = types.ModuleType('unitree_sdk2py.core')
channel = types.ModuleType('unitree_sdk2py.core.channel')
go2 = types.ModuleType('unitree_sdk2py.go2')
sport = types.ModuleType('unitree_sdk2py.go2.sport')
sport_client = types.ModuleType('unitree_sdk2py.go2.sport.sport_client')

channel.ChannelFactoryInitialize = lambda *a, **k: None
class FakeSportClient:
    def SetTimeout(self, *a, **k): pass
    def Init(self): pass
    def Move(self, vx, vy, wz): moves.append((round(time.time(), 2), vx, vy, wz))
    def StopMove(self): moves.append(('stop',))
sport_client.SportClient = FakeSportClient

for name, mod in [('unitree_sdk2py', sdk), ('unitree_sdk2py.core', core),
                  ('unitree_sdk2py.core.channel', channel),
                  ('unitree_sdk2py.go2', go2), ('unitree_sdk2py.go2.sport', sport),
                  ('unitree_sdk2py.go2.sport.sport_client', sport_client)]:
    sys.modules[name] = mod

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from go2_control.cmd_vel_node import CmdVelBridge, REQUIRED_ROBOT_ACK

rclpy.init(args=['--ros-args',
                 '-p', f'robot_ack:={REQUIRED_ROBOT_ACK}',
                 '-p', 'cmd_timeout:=0.5'])
node = CmdVelBridge()

def spin(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

def feed(vx, wz):
    t = Twist(); t.linear.x = vx; t.angular.z = wz
    node.callback(t)

print('--- ป้อน cmd_vel vx=0.3 ต่อเนื่อง 1 วินาที ---')
end = time.time() + 1.0
while time.time() < end:
    feed(0.3, 0.0); spin(0.05)
n_move = sum(1 for m in moves if len(m) == 4 and m[1] == 0.3)
print(f'   Move(0.3) ถูกเรียก {n_move} ครั้ง')

print('--- หยุดป้อน cmd_vel (จำลอง Wi-Fi หลุด) รอ 2 วินาที ---')
t_last_feed = time.time()
before = len(moves)
spin(2.0)
after = [m for m in moves[before:] if len(m) == 4]

zeros = [m for m in after if m[1] == 0.0 and m[3] == 0.0]
nonzero = [m for m in after if m[1] != 0.0 or m[3] != 0.0]

t_stop = zeros[0][0] - t_last_feed if zeros else None
t_last_nonzero = (nonzero[-1][0] - t_last_feed) if nonzero else 0.0

print(f'   ทำคำสั่งเดิมต่ออีก {len(nonzero)} ครั้ง จนถึงวินาทีที่ {t_last_nonzero:.2f}')
print(f'   สั่งหยุดครั้งแรกที่วินาทีที่ {t_stop:.2f}' if t_stop else '   ไม่เคยสั่งหยุด')
print(f'   หลังจากนั้นสั่งซ้ำอีก {len(zeros) - 1} ครั้ง (ควรเป็น 0 = latch แล้ว)')

TIMEOUT = 0.5
ok = (
    t_stop is not None
    and t_stop <= TIMEOUT + 0.25          # หยุดภายใน timeout + หนึ่ง tick
    and t_last_nonzero <= TIMEOUT + 0.15  # ไม่มีคำสั่งเดินหลุดหลัง timeout
    and len(zeros) == 1                   # latch ไม่สั่งซ้ำ
)
print()
print('ผ่าน: หยุดภายใน timeout, ไม่มีคำสั่งเดินหลุดหลังจากนั้น, latch อยู่'
      if ok else 'ไม่ผ่าน')

node.destroy_node(); rclpy.shutdown()
