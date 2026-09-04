"""Unitree-side receiver for the localhost velocity relay, with two send paths.

This is the last hop before the robot's legs. It reads JSON velocity packets
from `cmd_vel_udp_relay` over UDP on loopback and turns them into Unitree
commands. It runs as its own process because it needs the Unitree CycloneDDS
environment, which cannot coexist with the Fast DDS environment the Livox and
Nav2 graph runs in.

## Why there are two modes

The Go2W does not answer on the plain `sport` service that unitree_sdk2py's
SportClient looks for - its service list carries `wheeled_sport` instead. The
SDK's own discovery therefore finds nothing and refuses to send, which surfaces
as error 3102 with no indication that the service name is the problem.

    --mode api   publish unitree_api/msg/Request onto a request topic.
                 No service discovery, so the Go2W's service naming cannot
                 break it. This is the path proven to move this robot, and
                 the default.

    --mode sdk   call SportClient, which is simpler but depends on the SDK
                 finding a service named `sport`. Kept because it works on a
                 standard Go2 and is worth trying if the api path is quiet.

    --mode probe Report what the robot answers on both paths and exit without
                 sending a single motion command. Needs no robot_ack.

Run probe first at every site. It is five seconds and it tells you which of
the two modes to use, instead of finding out by watching a robot not move.
"""

import argparse
import json
import socket
import time

REQUIRED_ROBOT_ACK = 'I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT'

# Unitree high-level API ids, the same ones go2w_cmd_vel_control uses.
API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_MOVE = 1008
API_SWITCH_GAIT = 1011
API_SWITCH_JOYSTICK = 1027


def parse_args():
    parser = argparse.ArgumentParser(
        description='Send velocity to a Unitree Go2/Go2W over one of two paths.')
    parser.add_argument('--mode', choices=('api', 'sdk', 'probe'), default='api',
                        help='api publishes unitree_api Request messages; sdk '
                             'calls SportClient; probe reports and sends nothing.')
    parser.add_argument('--robot-ack', default='',
                        help='Required for api and sdk. Not required for probe, '
                             'which cannot move the robot.')
    parser.add_argument('--interface', default='wlp4s0',
                        help='Interface CycloneDDS binds to for sdk and probe modes.')
    parser.add_argument('--request-topic', default='/api/sport/request',
                        help='api mode only. Try /api/wheeled_sport/request if '
                             'the default is not picked up by this robot.')
    parser.add_argument('--port', type=int, default=32123)
    parser.add_argument('--timeout', type=float, default=0.5,
                        help='Seconds without a packet before the robot is stopped.')
    parser.add_argument('--max-linear', type=float, default=0.05)
    parser.add_argument('--max-angular', type=float, default=0.20)
    parser.add_argument('--no-prereqs', action='store_true',
                        help='api mode only. Skip the periodic joystick-off and '
                             'gait commands the wheeled base needs before it '
                             'will accept Move.')
    parser.add_argument('--gait', type=int, default=-1,
                        help='api mode only. Gait to select on startup, or -1 '
                             'to leave the gait alone.')
    return parser.parse_args()


def clamp(value, limit):
    return max(-limit, min(limit, float(value)))


def open_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', port))
    sock.settimeout(0.05)
    return sock


def read_command(sock, args, stats):
    """Return (x, y, z) from one packet, or None if there was nothing usable.

    A malformed packet is dropped, not raised. Anything on the machine can
    send to a UDP port, and one stray datagram must not be able to kill the
    process that is currently driving a robot. Dropped packets are counted so
    a relay sending nonsense is still visible rather than silently ignored.
    """
    try:
        packet, _ = sock.recvfrom(256)
    except socket.timeout:
        return None
    try:
        command = json.loads(packet.decode('ascii'))
        return (clamp(command['x'], args.max_linear),
                clamp(command['y'], args.max_linear),
                clamp(command['z'], args.max_angular))
    except (ValueError, KeyError, TypeError, UnicodeDecodeError) as exc:
        stats['dropped'] += 1
        if stats['dropped'] in (1, 10, 100) or stats['dropped'] % 1000 == 0:
            print(f'dropped {stats["dropped"]} malformed packet(s), latest: {exc}',
                  flush=True)
        return None


# --------------------------------------------------------------------- probe

def run_probe(args):
    print(f'probe: no motion command is sent in this mode\n')

    print('--- path 1: SDK service discovery ---')
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        ChannelFactoryInitialize(0, args.interface)
        client = SportClient()
        client.SetTimeout(1.0)
        client.Init()
        code, version = client.GetServerApiVersion()
        if code == 0:
            print(f'  sport service answered, API version {version}')
            print('  --mode sdk should work')
        else:
            print(f'  sport service did not answer, code {code}')
            print('  3102 means the request never went out, which on a Go2W '
                  'means the service is named wheeled_sport')
            print('  use --mode api')
    except Exception as exc:                      # noqa: BLE001 - report anything
        print(f'  SDK path unavailable: {exc}')
        print('  use --mode api')

    print()
    print('--- path 2: api request topic ---')
    try:
        import rclpy
        from unitree_api.msg import Request
        rclpy.init(args=None)
        node = rclpy.create_node('unitree_bridge_probe')
        pub = node.create_publisher(Request, args.request_topic, 10)
        deadline = time.monotonic() + 3.0
        peers = 0
        while time.monotonic() < deadline:
            peers = pub.get_subscription_count()
            if peers:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f'  topic {args.request_topic}')
        print(f'  subscribers seen: {peers}')
        if peers:
            print('  the robot is listening, --mode api should work')
        else:
            print('  nobody is listening on that topic. Either the robot is '
                  'not reachable on this interface, or it takes requests on '
                  '/api/wheeled_sport/request - try --request-topic')
        node.destroy_node()
        rclpy.shutdown()
    except Exception as exc:                      # noqa: BLE001 - report anything
        print(f'  api path unavailable: {exc}')

    print()
    print('--- topics the robot publishes ---')
    print('  run this separately, it needs the same CycloneDDS environment:')
    print('    ros2 topic list | grep -E "api|sport|lf/"')


# ----------------------------------------------------------------- sdk mode

def run_sdk(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    ChannelFactoryInitialize(0, args.interface)
    client = SportClient()
    client.SetTimeout(1.0)
    client.Init()
    code, version = client.GetServerApiVersion()
    if code != 0:
        raise SystemExit(
            f'Unitree sport API unavailable: {code}. On a Go2W this is expected - '
            f'the service is named wheeled_sport, not sport. Use --mode api.')
    print(f'armed: sdk mode, sport API {version}, udp port {args.port}', flush=True)

    sock = open_socket(args.port)
    stats = {'dropped': 0}
    last_rx = 0.0
    stopped = True
    try:
        while True:
            command = read_command(sock, args, stats)
            if command is not None:
                client.Move(*command)
                last_rx = time.monotonic()
                stopped = False
            elif not stopped and time.monotonic() - last_rx > args.timeout:
                client.Move(0.0, 0.0, 0.0)
                stopped = True
                print('timeout: stopped', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        client.Move(0.0, 0.0, 0.0)


# ----------------------------------------------------------------- api mode

def run_api(args):
    import rclpy
    from unitree_api.msg import Request

    rclpy.init(args=None)
    node = rclpy.create_node('unitree_udp_bridge')
    pub = node.create_publisher(Request, args.request_topic, 10)

    def send(api_id, parameter=''):
        request = Request()
        request.header.identity.api_id = api_id
        request.parameter = parameter
        pub.publish(request)

    # Give discovery a moment, then say plainly whether anyone is listening.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    peers = pub.get_subscription_count()
    print(f'armed: api mode, topic {args.request_topic}, udp port {args.port}',
          flush=True)
    if peers == 0:
        print('warning: nothing is subscribed to that topic. Commands will be '
              'published and go nowhere. Run --mode probe, and try '
              '--request-topic /api/wheeled_sport/request.', flush=True)
    else:
        print(f'  {peers} subscriber(s) on the request topic', flush=True)

    sock = open_socket(args.port)
    stats = {'dropped': 0}
    last_rx = 0.0
    stopped = True
    sent = 0
    try:
        while True:
            command = read_command(sock, args, stats)
            if command is not None:
                # The wheeled base drops back into joystick control unless it is
                # told otherwise periodically, which is why go2w_cmd_vel_control
                # repeats these rather than sending them once at startup.
                if not args.no_prereqs and sent % 20 == 0:
                    send(API_SWITCH_JOYSTICK, '{"data":false}')
                    if args.gait >= 0:
                        send(API_SWITCH_GAIT, '{"data":%d}' % args.gait)
                x, y, z = command
                send(API_MOVE, '{"x":%.6f,"y":%.6f,"z":%.6f}' % (x, y, z))
                sent += 1
                last_rx = time.monotonic()
                stopped = False
            elif not stopped and time.monotonic() - last_rx > args.timeout:
                send(API_STOP_MOVE)
                stopped = True
                print('timeout: sent StopMove', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        send(API_STOP_MOVE)
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


def main():
    args = parse_args()
    if args.mode == 'probe':
        run_probe(args)
        return
    if args.robot_ack != REQUIRED_ROBOT_ACK:
        raise SystemExit(
            'Refusing to arm Unitree bridge: robot acknowledgement is invalid.\n'
            'Use --mode probe to check the link without arming anything.')
    if args.mode == 'api':
        run_api(args)
    else:
        run_sdk(args)


if __name__ == '__main__':
    main()
