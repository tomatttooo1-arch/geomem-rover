"""Loopback-only ROS CDR relay over an explicitly established SSH TCP tunnel.

Server: simulator ROS domain 73; client: AGX ROS domain 42. Each TCP generation
starts empty. No command or permission is periodically replayed. A server nonce
with a local monotonic deadline conservatively bounds command transport age,
without assuming the hosts' clocks agree. ROS serialization API reference:
https://docs.ros.org/en/humble/p/rclpy/rclpy.serialization.html

The client->server direction also carries /fpga/localization_status so the server side can
see the localization gate's lease. It is relayed only: it is not in MOTION, so it neither
grants motion nor stops it, and the same generation/sequence/TTL contract still applies.
"""

import argparse
import gc
from collections import OrderedDict, deque
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import json
import math
import os
import select
import socket
import struct
import threading
import time
import uuid
import zlib


MAX_BYTES = 8 * 1024 * 1024
MAX_HEADER = 16 * 1024
TTL = .5
SENSORS = {"/fpga/rgb": "sensor_msgs/msg/Image", "/fpga/depth": "sensor_msgs/msg/Image",
           "/fpga/camera_info": "sensor_msgs/msg/CameraInfo", "/fpga/points": "sensor_msgs/msg/PointCloud2",
           "/odom": "nav_msgs/msg/Odometry", "/tf": "tf2_msgs/msg/TFMessage",
           "/clock": "rosgraph_msgs/msg/Clock"}
COMMANDS = {"/cmd_vel": "geometry_msgs/msg/Twist", "/fpga/motion_permit": "std_msgs/msg/Bool",
            "/fpga/localization_status": "std_msgs/msg/String"}
# The client->server topics that carry motion authority. Everything else in COMMANDS is
# relayed only: it can neither grant motion nor, by expiring, stop it.
MOTION = frozenset(("/cmd_vel", "/fpga/motion_permit"))
RGBD = frozenset(("/fpga/rgb", "/fpga/depth", "/fpga/camera_info"))
PROFILES = ("reference", "estimated_inspect")


def relay_topics(profile):
    """Return the exact wire/ROS contract for one side of a run.

    estimated_inspect carries camera data and simulation time only.  In
    particular, simulator odometry/TF and every command channel are absent at
    both the ROS endpoints and the serialized protocol boundary.
    """
    if profile not in PROFILES:
        raise ValueError("unknown relay profile")
    sensors = dict(SENSORS)
    commands = dict(COMMANDS)
    if profile == "estimated_inspect":
        sensors = {topic: kind for topic, kind in sensors.items()
                   if topic not in ("/odom", "/tf")}
        commands = {}
    return sensors, commands


def trace_delay(stage, started, **fields):
    elapsed = time.monotonic() - started
    if elapsed >= .1:
        print(json.dumps({'relay_delay': stage, 'elapsed_s': elapsed,
                          'wall_ns': time.time_ns(), **fields}), flush=True)


class CaptureEpoch:
    """Bind samples before ATS; a new connection cannot consume pre-reconnect captures."""

    def __init__(self):
        self.generation = None
        self.watermark = self.floor = -math.inf
        self.last_clock = -math.inf

    def check_clock(self, stamp):
        if not math.isfinite(stamp) or stamp < self.last_clock:
            raise ValueError("simulation clock rewound: restart relay AND runtime with fresh memory")
        self.last_clock = stamp

    def transition(self, generation):
        if generation == self.generation:
            return False
        self.generation = generation
        self.floor = self.watermark
        return True

    def observe(self, stamp):
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError("invalid acquisition timestamp")
        self.watermark = max(self.watermark, stamp)
        return self.generation if self.generation and stamp > self.floor else None

    def allows(self, generation, stamps):
        return bool(generation and generation == self.generation
                    and all(math.isfinite(stamp) and stamp > self.floor for stamp in stamps))


class StampedEnvelope:
    """message_filters sees the original header; generation is immutable per arrival."""

    def __init__(self, message, generation):
        self.header = message.header
        self.message = message
        self.generation = generation


def sensor_order(records):
    """Prefer clock and transform delivery before images; cross-topic DDS may still reorder."""
    order = {topic: i for i, topic in enumerate(("/clock", "/odom", "/tf", "/fpga/camera_info",
                                               "/fpga/depth", "/fpga/rgb", "/fpga/points"))}
    return sorted(records, key=lambda record: order[record["topic"]])


class TransformHistory:
    """A bounded history survives latest-message coalescing; reset on simulation rewind."""

    def __init__(self, seconds=2., capacity=128):
        self.seconds = seconds
        self.entries = deque(maxlen=capacity)

    def append(self, stamp, transform):
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError("invalid transform timestamp")
        if self.entries and stamp < self.entries[-1][0]:
            self.entries.clear()
        if self.entries and stamp == self.entries[-1][0]:
            self.entries.pop()
        self.entries.append((stamp, transform))
        while self.entries and stamp-self.entries[0][0] > self.seconds:
            self.entries.popleft()

    def values(self):
        return [entry[1] for entry in self.entries]


def shuffle24(data, inverse=False):
    """Lossless byte permutation; includes every byte and preserves a short tail."""
    count = len(data)//24
    end = count*24
    if not inverse:
        return b"".join(data[i:end:24] for i in range(24))+data[end:]
    restored = bytearray(data)
    for i in range(24):
        restored[i:end:24] = data[i*count:(i+1)*count]
    return bytes(restored)


def encode_packet(metadata, records=(), compress=False):
    """Bounded JSON envelope plus concatenated CDR bytes; never pickle."""
    descriptors, payload = [], bytearray()
    for record in records:
        item = {key: value for key, value in record.items() if key != "data"}
        body = record["data"]
        if not isinstance(body, bytes) or not body or len(body) > MAX_BYTES:
            raise ValueError("invalid CDR payload")
        item["size"] = len(body)
        descriptors.append(item)
        payload.extend(body)
        if len(payload) > MAX_BYTES:
            raise ValueError("relay packet exceeds size limit")
    header = json.dumps({**metadata, "records": descriptors}, allow_nan=False,
                        separators=(",", ":")).encode()
    if len(header) > MAX_HEADER:
        raise ValueError("relay header exceeds size limit")
    body = struct.pack("!I", len(header)) + header + payload
    if len(body) > MAX_BYTES:
        raise ValueError("relay packet exceeds size limit")
    if compress and len(body) >= 65536:
        packed = b'S24D' + zlib.compress(shuffle24(body), level=1)
        if len(packed) < len(body):
            body = packed
    return struct.pack("!I", len(body)) + body


def decode_packet(body, direction, profile="reference"):
    """Check topic/type identity and require RGB/depth/info as an indivisible batch."""
    if len(body) < 4 or len(body) > MAX_BYTES:
        raise ValueError("invalid relay packet size")
    if body[:4] in (b'ZFPGA', b'S24D'):
        shuffled = body[:4] == b'S24D'
        decoder = zlib.decompressobj()
        try:
            unpacked = decoder.decompress(body[4:], MAX_BYTES + 1)
        except zlib.error as exc:
            raise ValueError('invalid compressed relay payload') from exc
        if (len(unpacked) > MAX_BYTES or not decoder.eof or decoder.unused_data
                or decoder.unconsumed_tail):
            raise ValueError('compressed relay size or stream boundary invalid')
        body = shuffle24(unpacked, inverse=True) if shuffled else unpacked
        if len(body) < 4:
            raise ValueError('truncated decompressed relay payload')
    header_size = struct.unpack("!I", body[:4])[0]
    if not 0 < header_size <= MAX_HEADER or 4+header_size > len(body):
        raise ValueError("invalid relay header size")
    metadata = json.loads(body[4:4+header_size])
    if not isinstance(metadata, dict):
        raise ValueError("relay header must be an object")
    records = metadata.pop("records", None)
    if not isinstance(records, list) or len(records) > 7:
        raise ValueError("invalid record count")
    sensors, commands = relay_topics(profile)
    allowed = sensors if direction == "sensors" else commands if direction == "commands" else {}
    seen, result = set(), []
    offset = 4 + header_size
    for item in records:
        if not isinstance(item, dict):
            raise ValueError("invalid descriptor")
        topic, size = item.get("topic"), item.get("size")
        if (topic not in allowed or topic in seen or item.get("type") != allowed[topic]
                or isinstance(size, bool) or not isinstance(size, int) or size <= 0
                or offset+size > len(body)):
            raise ValueError("topic/type/size outside relay contract")
        seen.add(topic)
        result.append({**item, "data": body[offset:offset+size]})
        offset += size
    if offset != len(body):
        raise ValueError("unclaimed relay payload")
    if seen & RGBD and not RGBD <= seen:
        raise ValueError("RGB, depth and camera info must travel together")
    if seen & RGBD:
        ids = {record.get("batch") for record in result if record["topic"] in RGBD}
        if len(ids) != 1 or None in ids:
            raise ValueError("mixed acquisition batches")
    return metadata, result


def receive_packet(stream, direction, timeout=.4, profile="reference"):
    started = time.monotonic()
    deadline = started + timeout

    def exact(size):
        out = bytearray()
        while len(out) < size:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError("relay packet deadline exceeded")
            stream.settimeout(remaining)
            block = stream.recv(size-len(out))
            if not block:
                raise ConnectionError("relay disconnected mid-packet")
            out.extend(block)
        return bytes(out)

    size = struct.unpack("!I", exact(4))[0]
    if not 0 < size <= MAX_BYTES:
        raise ValueError("relay message exceeds size limit")
    body = exact(size)
    received = time.monotonic()
    decode_cpu = time.thread_time()
    result = decode_packet(body, direction, profile)
    trace_delay('packet_receive_decode', started, direction=direction,
                wire_bytes=size + 4, read_s=received-started,
                decode_s=time.monotonic()-received, decode_cpu_s=time.thread_time()-decode_cpu,
                sensor_sequence=result[0].get('sensor_sequence'))
    return result


class LatestBatches:
    """At most one pending batch per fixed key, with an aggregate byte bound."""

    def __init__(self, keys, max_bytes=MAX_BYTES-MAX_HEADER):
        self.keys, self.max_bytes = set(keys), max_bytes
        self.pending = OrderedDict()
        self.lock = threading.Lock()
        self.dropped = 0
        self.last_clock_sent = -math.inf

    def clear(self):
        with self.lock:
            self.pending.clear()
            self.last_clock_sent = -math.inf

    def put(self, key, records):
        if key not in self.keys:
            raise ValueError("unknown queue key")
        size = sum(len(r["data"]) for r in records)
        if not records or size > self.max_bytes:
            raise ValueError("batch exceeds queue size limit")
        with self.lock:
            if key in self.pending:
                self.dropped += 1
                del self.pending[key]
            self.pending[key] = list(records)
            while sum(len(r["data"]) for batch in self.pending.values() for r in batch) > self.max_bytes:
                self.pending.popitem(last=False)
                self.dropped += 1

    def take(self, order_by_clock=False):
        with self.lock:
            records = [record for batch in self.pending.values() for record in batch]
            if order_by_clock:
                stamps = [r['acquisition_s'] for r in records if 'acquisition_s' in r]
                if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in stamps):
                    raise ValueError('invalid sensor acquisition time')
                clocks = [r['acquisition_s'] for r in records
                          if r['topic'] == '/clock' and 'acquisition_s' in r]
                clock = max([self.last_clock_sent]+clocks)
                ready = [key for key, batch in self.pending.items()
                         if all(r.get('acquisition_s', -math.inf) <= clock for r in batch)]
                records = [r for key in ready for r in self.pending.pop(key)]
                self.last_clock_sent = clock
            else:
                self.pending.clear()
        return records


class CommandFreshness:
    """One connection's nonce and sequence window; ages use server monotonic time."""

    def __init__(self, generation):
        self.generation = generation
        self.challenges = OrderedDict()
        self.last_sequence = -1
        self.lock = threading.Lock()

    def challenge(self, now):
        nonce = uuid.uuid4().hex
        with self.lock:
            self.challenges[nonce] = now
            while len(self.challenges) > 16:
                self.challenges.popitem(last=False)
        return nonce

    def deadline(self, generation, challenge, sequence, local_age, now):
        """Subtract sender queue age AND full challenge roundtrip as conservative age."""
        with self.lock:
            sent = self.challenges.get(challenge)
            if (generation != self.generation or sent is None or isinstance(sequence, bool)
                    or not isinstance(sequence, int) or sequence <= self.last_sequence
                    or not isinstance(local_age, (float, int)) or not math.isfinite(local_age)
                    or local_age < 0 or now < sent):
                raise ValueError("stale, foreign or invalid command generation/sequence")
            self.last_sequence = sequence
            expiry = now + TTL - local_age - (now-sent)
            if expiry <= now:
                raise ValueError("original command/permission lease expired in transit")
            return expiry


def encode_sensor_job(metadata, records):
    """Owned local worker only; the network still carries the same bounded packet."""
    started, cpu = time.monotonic(), time.thread_time()
    packet = encode_packet(metadata, records, compress=True)
    return packet, time.monotonic()-started, time.thread_time()-cpu


def new_sensor_encoder():
    # spawn avoids forking a process that already owns ROS/DDS threads.
    return ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))


class Tunnel:
    """Single-client connection manager; socket thread never calls ROS or replays data."""

    def __init__(self, mode, host, port, profile="reference"):
        if mode not in ("server", "client") or host != "127.0.0.1":
            raise ValueError("relay endpoints must be explicit loopback server/client")
        self.sensors, self.commands = relay_topics(profile)
        self.mode, self.host, self.port, self.profile = mode, host, port, profile
        sensor_keys = tuple("rgbd" if topic in RGBD else topic for topic in self.sensors)
        sensor_keys = tuple(dict.fromkeys(sensor_keys))
        keys = sensor_keys if mode == "server" else tuple(self.commands) + (("stop",) if self.commands else ())
        self.outgoing = LatestBatches(keys)
        self.incoming = LatestBatches(sensor_keys if mode == "client" else self.commands)
        self.running = threading.Event()
        self.running.set()
        self.generation = None
        self.freshness = None
        self.connected = False
        self.safety_lock = threading.Lock()
        self.stop_requested = True
        self.error = None
        # client: a challenge of this connection has been read (commands can carry a nonce from now on). Read-only
        # state for tests, like connected/generation; set by the reader, cleared by disconnect(). No behaviour hangs on it.
        self.challenge_seen = False
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def request_stop(self):
        with self.safety_lock:
            self.stop_requested = True

    def consume_stop(self):
        with self.safety_lock:
            requested = self.stop_requested
            self.stop_requested = False
        return requested

    def disconnect(self):
        self.connected = False
        self.challenge_seen = False
        self.generation = None
        self.request_stop()
        self.outgoing.clear()
        self.incoming.clear()

    def run(self):
        listener = None
        try:
            if self.mode == "server":
                listener = socket.socket()
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((self.host, self.port))
                self.port = listener.getsockname()[1]
                listener.listen(1)
                listener.settimeout(.2)
            while self.running.is_set():
                try:
                    if listener:
                        stream, _ = listener.accept()
                    else:
                        stream = socket.create_connection((self.host, self.port), timeout=.4)
                    with stream:
                        self.session(stream)
                except (OSError, ValueError, ConnectionError, TimeoutError, RuntimeError) as exc:
                    self.error = str(exc)
                finally:
                    self.disconnect()
                if not listener:
                    time.sleep(.1)
        except OSError as exc:
            self.error = str(exc)
            self.disconnect()
        finally:
            if listener:
                listener.close()

    def session(self, stream):
        self.disconnect()
        stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        stream.settimeout(.4)
        if self.mode == "server":
            generation = uuid.uuid4().hex
            stream.sendall(encode_packet({"kind": "hello", "generation": generation,
                                          "relay_profile": self.profile}))
            self.freshness = CommandFreshness(generation)
        else:
            metadata, records = receive_packet(stream, "sensors", profile=self.profile)
            if metadata.get("kind") != "hello" or records:
                raise ValueError("missing relay handshake")
            if metadata.get("relay_profile", "reference") != self.profile:
                raise ValueError("relay profile mismatch")
            generation = metadata.get("generation")
            if not isinstance(generation, str) or len(generation) != 32:
                raise ValueError("invalid relay generation")
        self.generation, self.connected = generation, True
        state = {'last_received': time.monotonic(), 'challenge': None,
                 'sent_sensor': 0, 'acked_sensor': 0, 'received_sensor': 0, 'error': None}
        stopping = threading.Event()
        # Separate socket wrappers retain independent Python timeout budgets.
        # Both use nonblocking kernel sockets; only this thread reads, one writes.
        reader = stream.dup()
        encoder = new_sensor_encoder() if self.mode == 'server' else None

        def send_loop():
            last_ping, last_ack = -math.inf, 0
            encoding = None
            encoding_started = None
            encoding_sequence = None
            try:
                while self.running.is_set() and not stopping.is_set():
                    now = time.monotonic()
                    if now-state['last_received'] > TTL:
                        raise TimeoutError('relay heartbeat expired')
                    if now-last_ping >= .1:
                        if self.mode == 'server':
                            nonce = self.freshness.challenge(now)
                            metadata = {'kind': 'challenge', 'generation': generation, 'challenge': nonce,
                                        'relay_profile': self.profile}
                        else:
                            metadata = {'kind': 'alive', 'generation': generation,
                                        'relay_profile': self.profile}
                        stream.sendall(encode_packet(metadata))
                        last_ping = now
                    if self.mode == 'client' and state['received_sensor'] != last_ack:
                        ack = state['received_sensor']
                        stream.sendall(encode_packet({'kind': 'sensor_ack', 'generation': generation,
                                                      'relay_profile': self.profile,
                                                      'sensor_sequence': ack}))
                        last_ack = ack
                    # A slow codec must not prevent challenge/ACK/command handling.
                    # Never submit another job while this one or its ACK is pending.
                    if encoding is not None:
                        if not encoding.done():
                            stopping.wait(.005)
                            continue
                        payload, encode_s, encode_cpu_s = encoding.result()
                        encoding = None
                        started = time.monotonic()
                        state['sent_sensor'] = encoding_sequence
                        stream.sendall(payload)
                        trace_delay('sensor_encoding_dispatch', encoding_started,
                                    mode=self.mode, bytes=len(payload), encode_s=encode_s,
                                    encode_cpu_s=encode_cpu_s, send_s=time.monotonic()-started,
                                    sensor_sequence=state['sent_sensor'])
                        continue
                    # At most one sensor batch on the wire. While waiting for its
                    # receipt, coalesce new captures in the existing bounded queue.
                    if self.mode == 'server' and state['sent_sensor'] != state['acked_sensor']:
                        stopping.wait(.005)
                        continue
                    queued = self.outgoing.take(order_by_clock=True) if self.mode == 'server' else self.outgoing.take()
                    records = [r for r in queued if r.get('generation') == generation]
                    if self.mode == 'client':
                        drained_at = time.monotonic()
                        nonce = state['challenge']
                        records = [dict(r, age_s=drained_at-r['queued_at']) for r in records
                                   if nonce and drained_at-r['queued_at'] < TTL]
                    else:
                        nonce = None
                    if records:
                        metadata = {'kind': 'sensors' if self.mode == 'server' else 'commands',
                                    'generation': generation, 'challenge': nonce,
                                    'relay_profile': self.profile}
                        if self.mode == 'server':
                            encoding_sequence = state['sent_sensor'] + 1
                            metadata['sensor_sequence'] = encoding_sequence
                            encoding_started = time.monotonic()
                            encoding = encoder.submit(encode_sensor_job, metadata, sensor_order(records))
                            continue
                        batches = ([[r] for r in sorted(records, key=lambda r: r['sequence'])]
                                   if self.mode == 'client' else [sensor_order(records)])
                        for batch in batches:
                            started = time.monotonic()
                            encode_cpu = time.thread_time()
                            payload = encode_packet(metadata, batch, compress=self.mode == 'server')
                            encoded = time.monotonic()
                            encode_cpu_s = time.thread_time()-encode_cpu
                            stream.sendall(payload)
                            trace_delay('socket_send', started, mode=self.mode, bytes=len(payload),
                                        encode_s=encoded-started, encode_cpu_s=encode_cpu_s,
                                        send_s=time.monotonic()-encoded,
                                        sensor_sequence=metadata.get('sensor_sequence'))
                    stopping.wait(.005)
            except Exception as exc:
                # Includes worker startup/death; fail closed instead of losing the sender silently.
                state['error'] = exc
                stopping.set()
                try:
                    stream.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

        sender = threading.Thread(target=send_loop, daemon=True)
        sender.start()
        try:
            while self.running.is_set() and not stopping.is_set():
                if not select.select([reader], [], [], .01)[0]:
                    continue
                started = time.monotonic()
                metadata, records = receive_packet(reader, 'commands' if self.mode == 'server' else 'sensors',
                                                   profile=self.profile)
                trace_delay('socket_receive', started, mode=self.mode,
                            bytes=sum(len(r['data']) for r in records))
                if metadata.get('generation') != generation:
                    raise ValueError('foreign relay generation')
                if metadata.get('relay_profile', 'reference') != self.profile:
                    raise ValueError('relay profile mismatch')
                for record in records:
                    record['generation'] = generation
                    record['_received_at'] = time.monotonic()
                kind = metadata.get('kind')
                if self.mode == 'client':
                    if kind == 'challenge' and not records:
                        state['challenge'] = metadata.get('challenge')
                        self.challenge_seen = True
                    elif kind == 'sensors':
                        sequence = metadata.get('sensor_sequence')
                        if type(sequence) is not int or sequence != state['received_sensor'] + 1:
                            raise ValueError('invalid or replayed sensor sequence')
                        rgbd = [r for r in records if r['topic'] in RGBD]
                        if rgbd:
                            self.incoming.put('rgbd', rgbd)
                        for record in records:
                            if record['topic'] not in RGBD:
                                self.incoming.put(record['topic'], [record])
                        state['received_sensor'] = sequence
                    else:
                        raise ValueError('invalid server message kind')
                else:
                    if kind == 'alive' and not records:
                        pass
                    elif kind == 'sensor_ack' and not records:
                        sequence = metadata.get('sensor_sequence')
                        if (type(sequence) is not int or sequence != state['sent_sensor']
                                or sequence <= state['acked_sensor']):
                            raise ValueError('invalid or replayed sensor acknowledgement')
                        state['acked_sensor'] = sequence
                    elif kind == 'commands':
                        for record in sorted(records, key=lambda r: r.get('sequence', -1)):
                            record['expires'] = self.freshness.deadline(
                                generation, metadata.get('challenge'), record.get('sequence'),
                                record.get('age_s'), time.monotonic())
                            if record['topic'] == '/fpga/motion_permit':
                                if not isinstance(record.get('permit'), bool):
                                    raise ValueError('permission value missing')
                                if not record['permit']:
                                    self.request_stop()
                                    self.incoming.clear()
                            self.incoming.put(record['topic'], [record])
                    else:
                        raise ValueError('invalid client message kind')
                state['last_received'] = time.monotonic()
            if state['error'] is not None:
                raise state['error']
        except Exception as exc:
            self.error = str(state['error'] if state['error'] is not None else exc)
            if state['error'] is not None:
                raise state['error']
            raise
        finally:
            stopping.set()
            self.disconnect()  # revoke immediately, before awaiting an outstanding codec job
            try:
                stream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            reader.close()
            sender.join(timeout=1)
            if encoder is not None:
                # Only one bounded pure codec job exists; discard its result on disconnect.
                encoder.shutdown(wait=True, cancel_futures=True)

    def close(self):
        self.running.clear()
        self.thread.join(timeout=1)
        self.disconnect()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("server", "client"))
    parser.add_argument("--host", default="127.0.0.1", choices=("127.0.0.1",))
    parser.add_argument("--port", type=int, default=18730)
    parser.add_argument("--domain", type=int)
    parser.add_argument("--profile", choices=PROFILES, default="reference")
    parser.add_argument("--fortress-cloud-frame", action="store_true",
                        help="label Fortress forward-left-up XYZ as camera_link (simulation server only)")
    args, ros_args = parser.parse_known_args(argv)
    if args.fortress_cloud_frame and args.mode != "server":
        parser.error("Fortress cloud adaptation is server-only")
    domain = args.domain if args.domain is not None else (73 if args.mode == "server" else 42)
    if not 0 <= domain <= 232:
        parser.error("ROS domain must be between 0 and 232")
    if "ROS_DOMAIN_ID" in os.environ and int(os.environ["ROS_DOMAIN_ID"]) != domain:
        parser.error("environment ROS_DOMAIN_ID conflicts with the explicit relay domain")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    # The AGX launcher enforces UDP loopback in its private Fast DDS profile.
    # Humble's localhost flag breaks cross-PID-namespace data delivery there.
    # Keep the ordinary server/default path restricted by the ROS flag.
    os.environ["ROS_LOCALHOST_ONLY"] = "0" if os.environ.get("FPGA_DDS_LOOPBACK_ONLY") == "1" else "1"

    gc_started = {}

    def gc_diagnostic(phase, info):
        generation = info['generation']
        if phase == 'start':
            gc_started[generation] = (time.monotonic(), time.thread_time())
        elif generation in gc_started:
            wall, cpu = gc_started.pop(generation)
            trace_delay('python_gc', wall, mode=args.mode, generation=generation,
                        cpu_s=time.thread_time()-cpu, collected=info.get('collected'))

    gc.callbacks.append(gc_diagnostic)

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.serialization import serialize_message, deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from message_filters import Subscriber, SimpleFilter, ApproximateTimeSynchronizer
    from std_msgs.msg import Bool
    from geometry_msgs.msg import Twist

    class RelayNode(Node):
        def __init__(self):
            super().__init__("fpga_tunnel_" + args.mode)
            self.sensors, self.commands = relay_topics(args.profile)
            self.link = Tunnel(args.mode, args.host, args.port, args.profile)
            self.types = {topic: get_message(kind) for topic, kind in
                          {**self.sensors, **self.commands}.items()}
            incoming = self.sensors if args.mode == "client" else self.commands
            # Reliable readers (TF/odom/camera_info) and best-effort sensor readers can both subscribe.
            self.outputs = {topic: self.create_publisher(self.types[topic], topic, 3) for topic in incoming}
            self.safety = {topic: self.outputs.get(topic) or self.create_publisher(self.types[topic], topic, 1)
                           for topic in MOTION if topic in self.commands}
            self.sequence = 0
            self.tf_history = TransformHistory()
            self.tf_last_published = -math.inf
            self.capture_epoch = CaptureEpoch()
            self.capture_fault = False
            self.permit_expiry = self.command_expiry = -math.inf
            self.stopped = True
            self.link_state = None
            self.last_poll_started = time.monotonic()
            self.capture_trace = {}
            self.subscriptions_keep = []
            if args.mode == "server":
                self.raw_filters = [Subscriber(self, self.types[topic], topic, qos_profile=qos_profile_sensor_data)
                                    for topic in ("/fpga/rgb", "/fpga/depth", "/fpga/camera_info")]
                self.filters = [SimpleFilter() for _ in self.raw_filters]
                self.sync = ApproximateTimeSynchronizer(self.filters, 10, .02)
                self.sync.registerCallback(self.images)
                for raw, gate, topic in zip(self.raw_filters, self.filters, ("/fpga/rgb", "/fpga/depth", "/fpga/camera_info")):
                    raw.registerCallback(lambda message, target=gate, topic=topic: self.camera_sample(message, target, topic))
                for topic in self.sensors:
                    if topic not in RGBD:
                        qos = QoSProfile(depth=3, reliability=(ReliabilityPolicy.RELIABLE
                            if topic == "/fpga/points" else ReliabilityPolicy.BEST_EFFORT))
                        self.subscriptions_keep.append(self.create_subscription(
                            self.types[topic], topic, lambda msg, t=topic: self.sensor(t, msg), qos))
            else:
                for topic in self.commands:
                    self.subscriptions_keep.append(self.create_subscription(
                        self.types[topic], topic, lambda msg, t=topic: self.command(t, msg), 1))
            self.timer = self.create_timer(.02, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.safe_stop()
            self.link.start()
            self.get_logger().info("{} relay domain {} via 127.0.0.1:{}; no cross-host DDS".format(
                args.mode, domain, args.port))

        def safe_stop(self):
            if "/fpga/motion_permit" in self.safety:
                self.safety["/fpga/motion_permit"].publish(Bool(data=False))
            if "/cmd_vel" in self.safety:
                self.safety["/cmd_vel"].publish(Twist())
            self.permit_expiry = self.command_expiry = -math.inf
            self.stopped = True

        def record(self, topic, message, **extra):
            generation = self.link.generation
            if args.mode == 'server' and topic in self.sensors:
                if topic == '/tf':
                    stamps = [t.header.stamp for t in message.transforms]
                else:
                    stamps = [message.clock if topic == '/clock' else message.header.stamp]
                extra['acquisition_s'] = max(s.sec+s.nanosec/1e9 for s in stamps)
            started = time.monotonic()
            data = serialize_message(message)
            trace_delay('ros_serialize', started, mode=args.mode, topic=topic)
            contract = self.sensors if topic in self.sensors else self.commands
            return {"topic": topic, "type": contract[topic],
                    "data": data, "generation": generation, **extra}

        def sync_generation(self):
            generation = self.link.generation if self.link.connected else None
            if self.capture_epoch.transition(generation):
                # Called before ATS.add or from the node timer, never inside its callback lock.
                with self.sync.lock:
                    for messages in self.sync.queues:
                        messages.clear()
                self.tf_history.entries.clear()
            return generation

        def capture_diagnostic(self, key, stamp):
            now = time.monotonic()
            previous = self.capture_trace.get(key)
            if previous and (now - previous[0] > .2 or stamp - previous[1] > .2):
                print(json.dumps({'capture_gap': key, 'wall_ns': time.time_ns(),
                                  'receipt_gap_s': now-previous[0], 'stamp_gap_s': stamp-previous[1],
                                  'stamp_s': stamp, 'previous_stamp_s': previous[1],
                                  'raw_latest': self.capture_trace}), flush=True)
            self.capture_trace[key] = (now, stamp)

        def camera_sample(self, message, gate, topic):
            self.capture_diagnostic(topic, message.header.stamp.sec+message.header.stamp.nanosec/1e9)
            self.sync_generation()
            stamp = message.header.stamp
            token = self.capture_epoch.observe(stamp.sec+stamp.nanosec/1e9)
            if token:
                gate.signalMessage(StampedEnvelope(message, token))

        def images(self, rgb, depth, info):
            if not self.link.connected:
                return
            envelopes = (rgb, depth, info)
            generations = {item.generation for item in envelopes}
            stamps = [item.header.stamp.sec+item.header.stamp.nanosec/1e9 for item in envelopes]
            if (len(generations) != 1 or rgb.generation != self.link.generation
                    or not self.capture_epoch.allows(rgb.generation, stamps)):
                return
            generation = rgb.generation
            rgb, depth, info = (item.message for item in envelopes)
            stamp = rgb.header.stamp
            self.capture_diagnostic("synchronized_rgbd", stamp.sec+stamp.nanosec/1e9)
            batch = "{}:{}".format(stamp.sec, stamp.nanosec)
            try:
                self.link.outgoing.put("rgbd", [self.record(topic, msg, batch=batch, generation=generation)
                    for topic, msg in zip(("/fpga/rgb", "/fpga/depth", "/fpga/camera_info"), (rgb, depth, info))])
            except ValueError as exc:
                self.get_logger().error(str(exc))

        def sensor(self, topic, message):
            if topic == "/fpga/points":
                self.capture_diagnostic(topic, message.header.stamp.sec+message.header.stamp.nanosec/1e9)
                if args.fortress_cloud_frame:
                    # Installed Fortress emits X-forward/Y-left/Z-up, despite the
                    # shared RGB-D optical header. Preserve XYZ and fix only its frame.
                    if message.header.frame_id != "camera_optical_frame":
                        raise ValueError("unexpected Fortress cloud source frame")
                    message.header.frame_id = "camera_link"
            if self.capture_fault:
                return
            if topic == "/clock":
                try:
                    self.capture_epoch.check_clock(message.clock.sec+message.clock.nanosec/1e9)
                except ValueError as exc:
                    self.capture_fault = True
                    self.safe_stop()
                    self.get_logger().error(str(exc))
                    self.link.close()
                    return
            self.sync_generation()
            if topic == "/tf":
                # Only the declared robot reference transform; never object/model poses.
                transforms = [t for t in message.transforms
                              if t.header.frame_id == "odom" and t.child_frame_id == "base_link"]
                if not transforms:
                    return
                stamps = [t.header.stamp.sec+t.header.stamp.nanosec/1e9 for t in transforms]
                generation = self.capture_epoch.observe(max(stamps))
                if not generation:
                    return
                for transform in transforms:
                    stamp = transform.header.stamp
                    timestamp = stamp.sec+stamp.nanosec/1e9
                    if timestamp > self.capture_epoch.floor:
                        self.tf_history.append(timestamp, transform)
                message.transforms = self.tf_history.values()
            else:
                stamp = message.clock if topic == "/clock" else message.header.stamp
                generation = self.capture_epoch.observe(stamp.sec+stamp.nanosec/1e9)
                if not generation:
                    return
            if topic == "/odom" and (message.header.frame_id != "odom" or message.child_frame_id != "base_link"):
                self.get_logger().error("reference odometry frame differs from declared robot frames")
                return
            try:
                self.link.outgoing.put(topic, [self.record(topic, message, generation=generation)])
            except ValueError as exc:
                self.get_logger().error(str(exc))

        def command(self, topic, message):
            if not self.link.connected:
                return
            generation = self.link.generation
            self.sequence += 1
            extra = {"sequence": self.sequence, "queued_at": time.monotonic(), "generation": generation}
            if topic == "/fpga/motion_permit":
                extra["permit"] = bool(message.data)
                if not message.data:
                    self.link.outgoing.clear()
            key = "stop" if topic == "/fpga/motion_permit" and not message.data else topic
            self.link.outgoing.put(key, [self.record(topic, message, **extra)])

        def poll(self):
            poll_started = time.monotonic()
            trace_delay('ros_poll_interval', self.last_poll_started, mode=args.mode)
            self.last_poll_started = poll_started
            if args.mode == "server":
                self.sync_generation()
            state = (self.link.connected, self.link.generation, self.link.error)
            if state != self.link_state:
                self.get_logger().info("relay connection: " + repr(state))
                self.link_state = state
            if self.link.consume_stop() or not self.link.connected:
                self.safe_stop()
            now = time.monotonic()
            records = self.link.incoming.take()
            if not self.link.connected:
                return
            if args.mode == "server":
                records.sort(key=lambda r: r["sequence"])
            else:
                records = sensor_order(records)
            for record in records:
                try:
                    if record.get("generation") != self.link.generation:
                        continue
                    topic = record["topic"]
                    trace_delay('incoming_queue', record.get('_received_at', time.monotonic()),
                                mode=args.mode, topic=topic)
                    decode_started = time.monotonic()
                    message = deserialize_message(record["data"], self.types[topic])
                    trace_delay('ros_deserialize', decode_started, mode=args.mode, topic=topic)
                    if args.mode == "client" and topic == "/tf":
                        stamped = [(t.header.stamp.sec+t.header.stamp.nanosec/1e9, t)
                                   for t in message.transforms]
                        if stamped and max(stamp for stamp, _ in stamped) < self.tf_last_published:
                            self.tf_last_published = -math.inf  # explicit simulation rewind
                        message.transforms = [t for stamp, t in stamped if stamp > self.tf_last_published]
                        if not message.transforms:
                            continue
                        self.tf_last_published = max(stamp for stamp, _ in stamped)
                    if args.mode == "server" and topic in MOTION:
                        if record["expires"] <= now:
                            self.safe_stop()
                            continue
                        if topic == "/fpga/motion_permit":
                            if bool(message.data) != record["permit"]:
                                raise ValueError("permission metadata and CDR disagree")
                            if message.data:
                                self.permit_expiry = record["expires"]
                                self.stopped = False
                            else:
                                self.safe_stop()
                        else:
                            self.command_expiry = record["expires"]
                            self.stopped = False
                    publish_started = time.monotonic()
                    self.outputs[topic].publish(message)
                    trace_delay('ros_publish', publish_started, mode=args.mode, topic=topic)
                except Exception as exc:
                    self.safe_stop()
                    self.get_logger().error("relay rejected input: " + str(exc))
            if args.mode == "server":
                if self.permit_expiry > 0 and now >= self.permit_expiry:
                    self.safe_stop()
                elif self.command_expiry > 0 and now >= self.command_expiry:
                    # A new permission may arrive before the next velocity sample.
                    # Expired velocity stops motion without deleting that live permission.
                    self.safety["/cmd_vel"].publish(Twist())
                    self.command_expiry = -math.inf

            trace_delay('ros_poll', poll_started, mode=args.mode, records=len(records))

        def close(self):
            self.safe_stop()
            self.link.close()

    rclpy.init(args=ros_args)
    node = None
    try:
        node = RelayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        gc.callbacks.remove(gc_diagnostic)


if __name__ == "__main__":
    main()
