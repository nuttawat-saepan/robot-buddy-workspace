"""A minimal MQTT 3.1.1 broker, for testing the mission path without a server.

`main.py` is driven entirely over MQTT, so exercising a mission at the desk
needs a broker. Mosquitto is not installed here and needs root; amqtt installs
from pip but would not deliver a published message back to a subscriber in the
default configuration, and debugging someone else's broker is not the point of
the exercise.

This implements only what `main.py` uses: CONNECT, SUBSCRIBE, PUBLISH at QoS 0,
PINGREQ and DISCONNECT, with `+` and `#` wildcards. It keeps no session state,
persists nothing, and is not a broker to run in the field - it exists so a
mission can be pushed through the stack against a replayed bag.

    python3 test/mini_mqtt_broker.py            # 127.0.0.1:1883
    python3 test/mini_mqtt_broker.py 0.0.0.0 1883
"""
import socket
import struct
import sys
import threading


CONNECT, CONNACK, PUBLISH, SUBSCRIBE, SUBACK = 1, 2, 3, 8, 9
UNSUBSCRIBE, UNSUBACK, PINGREQ, PINGRESP, DISCONNECT = 10, 11, 12, 13, 14


def read_varint(sock):
    """MQTT remaining-length: 7 bits per byte, high bit means continue."""
    value = 0
    multiplier = 1
    for _ in range(4):
        byte = sock.recv(1)
        if not byte:
            return None
        value += (byte[0] & 0x7F) * multiplier
        if not byte[0] & 0x80:
            return value
        multiplier *= 128
    raise ValueError('malformed remaining length')


def encode_varint(value):
    out = bytearray()
    while True:
        byte = value % 128
        value //= 128
        if value:
            byte |= 0x80
        out.append(byte)
        if not value:
            return bytes(out)


def recv_exact(sock, count):
    buf = b''
    while len(buf) < count:
        chunk = sock.recv(count - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def read_string(buf, offset):
    (length,) = struct.unpack_from('!H', buf, offset)
    start = offset + 2
    return buf[start:start + length].decode('utf-8', 'replace'), start + length


def topic_matches(filt, topic):
    """MQTT wildcard matching: + is one level, # is the rest."""
    f = filt.split('/')
    t = topic.split('/')
    for i, part in enumerate(f):
        if part == '#':
            return True
        if i >= len(t):
            return False
        if part != '+' and part != t[i]:
            return False
    return len(f) == len(t)


class Broker:

    def __init__(self, host='127.0.0.1', port=1883, verbose=True):
        self.host, self.port, self.verbose = host, port, verbose
        self.subs = {}
        self.lock = threading.Lock()

    def log(self, message):
        if self.verbose:
            print(message, flush=True)

    def serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(16)
        self.log('mini broker on %s:%d' % (self.host, self.port))
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=self.client, args=(conn,),
                             daemon=True).start()

    def client(self, conn):
        with self.lock:
            self.subs[conn] = set()
        try:
            while True:
                head = conn.recv(1)
                if not head:
                    break
                kind = head[0] >> 4
                length = read_varint(conn)
                if length is None:
                    break
                body = recv_exact(conn, length) if length else b''
                if body is None:
                    break

                if kind == CONNECT:
                    conn.sendall(bytes([CONNACK << 4, 2, 0, 0]))
                elif kind == SUBSCRIBE:
                    packet_id = struct.unpack_from('!H', body, 0)[0]
                    offset, codes = 2, bytearray()
                    while offset < len(body):
                        filt, offset = read_string(body, offset)
                        offset += 1
                        with self.lock:
                            self.subs[conn].add(filt)
                        codes.append(0)
                        self.log('  subscribe %s' % filt)
                    payload = struct.pack('!H', packet_id) + bytes(codes)
                    conn.sendall(bytes([SUBACK << 4]) +
                                 encode_varint(len(payload)) + payload)
                elif kind == PUBLISH:
                    qos = (head[0] >> 1) & 0x03
                    topic, offset = read_string(body, 0)
                    if qos:
                        offset += 2
                    self.dispatch(topic, body[offset:])
                elif kind == UNSUBSCRIBE:
                    packet_id = struct.unpack_from('!H', body, 0)[0]
                    conn.sendall(bytes([UNSUBACK << 4, 2]) +
                                 struct.pack('!H', packet_id))
                elif kind == PINGREQ:
                    conn.sendall(bytes([PINGRESP << 4, 0]))
                elif kind == DISCONNECT:
                    break
        except (OSError, ValueError, struct.error):
            pass
        finally:
            with self.lock:
                self.subs.pop(conn, None)
            try:
                conn.close()
            except OSError:
                pass

    def dispatch(self, topic, payload):
        encoded = topic.encode('utf-8')
        frame = struct.pack('!H', len(encoded)) + encoded + payload
        packet = bytes([PUBLISH << 4]) + encode_varint(len(frame)) + frame
        with self.lock:
            targets = [c for c, filters in self.subs.items()
                       if any(topic_matches(f, topic) for f in filters)]
        for conn in targets:
            try:
                conn.sendall(packet)
            except OSError:
                pass
        self.log('  publish %s -> %d subscriber(s), %d bytes'
                 % (topic, len(targets), len(payload)))


if __name__ == '__main__':
    host = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 1883
    Broker(host, port).serve()
