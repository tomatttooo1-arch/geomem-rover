"""Protocol and localhost socket tests; no fabricated ROS/CDR decoding results."""

import inspect
import socket
import struct
import threading
import time
import unittest

import host.ros_tunnel as tunnel
from host.ros_tunnel import (COMMANDS, MAX_BYTES, MOTION, RGBD, SENSORS, CaptureEpoch,
                            CommandFreshness, LatestBatches, TransformHistory, Tunnel,
                            decode_packet, encode_packet, receive_packet, relay_topics,
                            sensor_order, shuffle24)

STATUS = "/fpga/localization_status"


def record(topic, body=b"fixture-cdr", **fields):
    return {"topic": topic, "type": {**SENSORS, **COMMANDS}[topic], "data": body, **fields}


# 10-17 (docs/design/LOAD_TIMING_TESTS_20261017.md): a server session encodes sensors in a spawned worker process
# that is started on its first batch, so the first batch of a session waits for a cold interpreter start. That took
# 0.05-0.09 s on an idle host, 0.26-0.38 s in full runs and > 2 s twice (oorinv1016a / rcvbuf1016a, python3.13), which
# the 2 s windows below counted against the relay. Only the waits that include that start get this bound; every later
# wait keeps its 2 s, and what is asserted about the transfer is unchanged.
SPAWN_S = 30


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("timed out waiting for socket state")


class TunnelProtocolTests(unittest.TestCase):
    def test_estimated_inspect_contract_has_no_reference_pose_or_commands(self):
        sensors, commands = relay_topics("estimated_inspect")
        self.assertFalse({"/odom", "/tf"} & sensors.keys())
        self.assertEqual(commands, {})
        self.assertTrue(RGBD <= sensors.keys())
        self.assertIn("/clock", sensors)
        for topic, direction in (("/odom", "sensors"), ("/tf", "sensors"),
                                 ("/cmd_vel", "commands"),
                                 ("/fpga/motion_permit", "commands")):
            with self.subTest(topic=topic), self.assertRaises(ValueError):
                decode_packet(encode_packet({"kind": direction}, [record(topic)])[4:],
                              direction, "estimated_inspect")

    def test_estimated_inspect_tunnel_queues_exclude_pose_and_commands(self):
        server = Tunnel("server", "127.0.0.1", 0, "estimated_inspect")
        client = Tunnel("client", "127.0.0.1", 1, "estimated_inspect")
        self.assertFalse({"/odom", "/tf"} & server.outgoing.keys)
        self.assertEqual(server.incoming.keys, set())
        self.assertFalse({"/odom", "/tf"} & client.incoming.keys)
        self.assertEqual(client.outgoing.keys, set())

    def test_fortress_cloud_adapter_is_server_only_and_relabels_declared_source(self):
        source = inspect.getsource(tunnel.main)
        self.assertIn('parser.add_argument("--fortress-cloud-frame"', source)
        self.assertIn('args.fortress_cloud_frame and args.mode != "server"', source)
        self.assertIn('message.header.frame_id != "camera_optical_frame"', source)
        self.assertIn('message.header.frame_id = "camera_link"', source)

    def test_command_enqueued_during_sender_iteration_has_nonnegative_age(self):
        server = Tunnel("server", "127.0.0.1", 0)
        server.start()
        client = None
        inject = threading.Event()
        try:
            wait_for(lambda: server.port != 0)
            client = Tunnel("client", "127.0.0.1", server.port)
            original_take = client.outgoing.take

            def enqueue_while_draining():
                if inject.is_set():
                    inject.clear()
                    # Model a ROS callback arriving after the sender's loop began.
                    time.sleep(.005)
                    client.outgoing.put("/fpga/motion_permit", [record(
                        "/fpga/motion_permit", permit=True, sequence=1,
                        queued_at=time.monotonic(), generation=client.generation)])
                return original_take()

            client.outgoing.take = enqueue_while_draining
            client.start()
            wait_for(lambda: client.connected and server.connected)
            # receive the connection's challenge first: the client drops what it drains before one (by design: no
            # nonce, no command). Was a fixed 0.2 s; 10-17 the client's own flag (LOAD_TIMING_TESTS_20261017)
            wait_for(lambda: client.challenge_seen)
            generation = server.generation
            inject.set()
            wait_for(lambda: bool(server.incoming.pending))
            row = server.incoming.take()[0]
            self.assertGreaterEqual(row["age_s"], 0)
            self.assertEqual(server.generation, generation)
            self.assertLessEqual(row["expires"] - time.monotonic(), .5)
        finally:
            if client:
                client.close()
            server.close()

    def test_sensor_batch_waits_for_real_clock_without_losing_newest_data(self):
        queue = LatestBatches(("/clock", "/odom"))
        queue.put("/clock", [record("/clock", acquisition_s=10.)])
        queue.put("/odom", [record("/odom", b"old", acquisition_s=10.01)])
        self.assertEqual([r["topic"] for r in queue.take(order_by_clock=True)], ["/clock"])
        queue.put("/odom", [record("/odom", b"new", acquisition_s=10.02)])
        queue.put("/clock", [record("/clock", acquisition_s=10.02)])
        rows = queue.take(order_by_clock=True)
        self.assertEqual(next(r["data"] for r in rows if r["topic"] == "/odom"), b"new")
        queue.put("/odom", [record("/odom", acquisition_s=10.01)])
        self.assertEqual(len(queue.take(order_by_clock=True)), 1)
        queue.clear()
        queue.put("/odom", [record("/odom", acquisition_s=10.01)])
        self.assertEqual(queue.take(order_by_clock=True), [])
        queue.put("/clock", [record("/clock", acquisition_s=float("nan"))])
        with self.assertRaises(ValueError):
            queue.take(order_by_clock=True)

    def test_shuffle_preserves_all_bytes_and_every_tail_length(self):
        for size in list(range(49)) + [65536 + i for i in range(24)]:
            raw = bytes(i % 251 for i in range(size))
            self.assertEqual(shuffle24(shuffle24(raw), inverse=True), raw)

    def test_shuffled_stream_still_rejects_bombs_and_bad_boundaries(self):
        import zlib
        for raw in (b"x" * (MAX_BYTES + 1), b""):
            with self.assertRaises(ValueError):
                decode_packet(b"S24D" + zlib.compress(raw), "sensors")
        packet = encode_packet(
            {"kind": "sensors"}, [record("/fpga/points", b"x" * 100000)], compress=True)[4:]
        self.assertEqual(packet[:4], b"S24D")
        for bad in (packet[:-1], packet + b"extra", b"S24Dnot-zlib"):
            with self.assertRaises(ValueError):
                decode_packet(bad, "sensors")

    def test_lossless_compressed_sensor_packet_matches_original_bytes(self):
        rows = [record(topic, bytes(range(256)) * 1024, batch="same-frame")
                for topic in sorted(RGBD)]
        raw = encode_packet({"kind": "sensors"}, rows)
        packed = encode_packet({"kind": "sensors"}, rows, compress=True)
        self.assertLess(len(packed), len(raw))
        self.assertEqual(decode_packet(packed[4:], "sensors"),
                         decode_packet(raw[4:], "sensors"))
        with self.assertRaises(ValueError):
            decode_packet(packed[4:], "commands")

    def test_compressed_bomb_truncation_trailing_and_malformed_are_rejected(self):
        import zlib
        packets = [b"ZFPGA" + zlib.compress(b"x" * (MAX_BYTES + 1)),
                   b"ZFPGA" + zlib.compress(b"x" * 100)[:-1],
                   b"ZFPGA" + zlib.compress(b"x" * 100) + b"trailing",
                   b"ZFPGAnot-zlib", b"ZFPGA" + zlib.compress(b"")]
        for body in packets:
            with self.assertRaises(ValueError):
                decode_packet(body, "sensors")

    def test_sensor_send_delay_must_not_queue_fresh_command_packets_until_expiry(self):
        from concurrent.futures import ThreadPoolExecutor
        from unittest.mock import patch
        from host import ros_tunnel
        original = ros_tunnel.encode_packet

        def slow_sensor_send(metadata, records=(), **kwargs):
            if metadata.get("kind") == "sensors":
                time.sleep(.06)
            return original(metadata, records, **kwargs)

        server = Tunnel("server", "127.0.0.1", 0)
        client = None
        with patch.object(ros_tunnel, "encode_packet", side_effect=slow_sensor_send), \
                patch.object(ros_tunnel, "new_sensor_encoder",
                             side_effect=lambda: ThreadPoolExecutor(max_workers=1)):
            try:
                server.start()
                wait_for(lambda: server.port != 0)
                client = Tunnel("client", "127.0.0.1", server.port)
                client.start()
                wait_for(lambda: server.connected and client.connected)
                time.sleep(.15)
                generation = server.generation
                sequence = 0
                deadline = time.monotonic() + 1.5
                next_command = 0.
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    server.outgoing.put("/clock", [record("/clock", generation=generation)])
                    if now >= next_command:
                        for topic in ("/cmd_vel", "/fpga/motion_permit"):
                            sequence += 1
                            extra = {"permit": True} if topic == "/fpga/motion_permit" else {}
                            client.outgoing.put(topic, [record(
                                topic, generation=generation, sequence=sequence,
                                queued_at=time.monotonic(), **extra)])
                        next_command = now + .05
                    self.assertEqual(server.generation, generation, server.error)
                    time.sleep(.01)
                self.assertTrue(server.incoming.take())
            finally:
                if client:
                    client.close()
                server.close()

    def test_blocked_encoder_keeps_challenges_commands_and_latest_queue_live(self):
        from concurrent.futures import ThreadPoolExecutor
        from unittest.mock import patch
        from host import ros_tunnel
        entered, release = threading.Event(), threading.Event()
        original = ros_tunnel.encode_sensor_job
        jobs = []

        def slow_job(metadata, records):
            jobs.append(records[0]["data"])
            entered.set()
            if not release.wait(2):
                raise TimeoutError("test encoder was not released")
            return original(metadata, records)

        server = Tunnel("server", "127.0.0.1", 0)
        client = None
        with patch.object(ros_tunnel, "new_sensor_encoder",
                          side_effect=lambda: ThreadPoolExecutor(max_workers=1)), \
                patch.object(ros_tunnel, "encode_sensor_job", side_effect=slow_job):
            try:
                server.start()
                wait_for(lambda: server.port != 0)
                client = Tunnel("client", "127.0.0.1", server.port)
                client.start()
                wait_for(lambda: server.connected and client.connected)
                generation = server.generation
                server.outgoing.put("/clock", [record(
                    "/clock", b"first", generation=generation)])
                self.assertTrue(entered.wait(1))
                before = len(server.freshness.challenges)
                server.outgoing.put("/clock", [record(
                    "/clock", b"old", generation=generation)])
                server.outgoing.put("/clock", [record(
                    "/clock", b"latest", generation=generation)])
                time.sleep(.25)
                self.assertGreaterEqual(len(server.freshness.challenges) - before, 2)
                client.outgoing.put("/cmd_vel", [record(
                    "/cmd_vel", generation=generation, sequence=1,
                    queued_at=time.monotonic())])
                wait_for(lambda: bool(server.incoming.pending))
                command = server.incoming.take()[0]
                self.assertGreater(command["expires"], time.monotonic())
                self.assertEqual(jobs, [b"first"])
                self.assertEqual(server.generation, generation)
                release.set()
                wait_for(lambda: len(jobs) == 2)
                self.assertEqual(jobs, [b"first", b"latest"])
            finally:
                release.set()
                if client:
                    client.close()
                server.close()

    def test_disconnect_revokes_before_blocked_encoding_finishes(self):
        from concurrent.futures import ThreadPoolExecutor
        from unittest.mock import patch
        from host import ros_tunnel
        entered, release = threading.Event(), threading.Event()
        original = ros_tunnel.encode_sensor_job

        def blocked(metadata, records):
            entered.set()
            if not release.wait(2):
                raise TimeoutError("test encoder was not released")
            return original(metadata, records)

        server = Tunnel("server", "127.0.0.1", 0)
        client = None
        with patch.object(ros_tunnel, "new_sensor_encoder",
                          side_effect=lambda: ThreadPoolExecutor(max_workers=1)), \
                patch.object(ros_tunnel, "encode_sensor_job", side_effect=blocked):
            try:
                server.start()
                wait_for(lambda: server.port != 0)
                client = Tunnel("client", "127.0.0.1", server.port)
                client.start()
                wait_for(lambda: server.connected and client.connected)
                server.consume_stop()
                server.outgoing.put("/clock", [record(
                    "/clock", generation=server.generation)])
                self.assertTrue(entered.wait(1))
                client.close()
                wait_for(lambda: not server.connected)
                self.assertTrue(server.consume_stop())
                self.assertIsNone(server.generation)
                self.assertFalse(server.incoming.pending)
                self.assertFalse(release.is_set())
            finally:
                release.set()
                if client:
                    client.close()
                server.close()

    def test_encoder_failure_disconnects_and_requests_stop(self):
        from concurrent.futures import ThreadPoolExecutor
        from unittest.mock import patch
        from host import ros_tunnel
        server = Tunnel("server", "127.0.0.1", 0)
        client = None
        with patch.object(ros_tunnel, "new_sensor_encoder",
                          side_effect=lambda: ThreadPoolExecutor(max_workers=1)), \
                patch.object(ros_tunnel, "encode_sensor_job",
                             side_effect=RuntimeError("codec failed")):
            try:
                server.start()
                wait_for(lambda: server.port != 0)
                client = Tunnel("client", "127.0.0.1", server.port)
                client.start()
                wait_for(lambda: server.connected and client.connected)
                generation = server.generation
                server.consume_stop()
                server.outgoing.put("/clock", [record(
                    "/clock", generation=generation)])
                wait_for(lambda: server.error == "codec failed")
                self.assertTrue(server.consume_stop())
                self.assertNotEqual(server.generation, generation)
            finally:
                if client:
                    client.close()
                server.close()

    def test_worker_packet_is_byte_identical_and_runs_in_owned_process(self):
        import os
        from host.ros_tunnel import encode_sensor_job, new_sensor_encoder
        rows = [record("/fpga/points", bytes(range(256)) * 1024)]
        metadata = {"kind": "sensors", "generation": "local-test", "sensor_sequence": 1}
        with new_sensor_encoder() as pool:
            packet, elapsed, cpu = pool.submit(
                encode_sensor_job, metadata, rows).result(timeout=SPAWN_S)   # (the worker's start)
            self.assertEqual(packet, encode_packet(metadata, rows, compress=True))
            self.assertNotEqual(pool.submit(os.getpid).result(timeout=5), os.getpid())
            self.assertGreaterEqual(elapsed, 0)
            self.assertGreaterEqual(cpu, 0)

    def test_sensor_ack_bounds_inflight_and_rejects_invalid_ack(self):
        server = Tunnel("server", "127.0.0.1", 0)
        server.start()
        peer = None
        try:
            wait_for(lambda: server.port != 0)
            peer = socket.create_connection(("127.0.0.1", server.port), timeout=.4)
            hello, _ = receive_packet(peer, "sensors")
            generation = hello["generation"]
            wait_for(lambda: server.connected)
            server.outgoing.put("/clock", [record(
                "/clock", b"first", generation=generation)])

            def next_sensor(within_s=2.0):
                # (was 20 packets = 20 challenges at 0.1 s = the same 2 s, as a deadline now)
                end = time.monotonic() + within_s
                while time.monotonic() < end:
                    metadata, rows = receive_packet(peer, "sensors")
                    if metadata["kind"] == "sensors":
                        return metadata, rows
                    peer.sendall(encode_packet({"kind": "alive", "generation": generation}))
                self.fail("sensor packet not received")

            first, rows = next_sensor(SPAWN_S)   # the session's first batch: the encoder worker starts
            self.assertEqual(rows[0]["data"], b"first")
            server.outgoing.put("/clock", [record(
                "/clock", b"old", generation=generation)])
            server.outgoing.put("/clock", [record(
                "/clock", b"latest", generation=generation)])
            metadata, rows = receive_packet(peer, "sensors")
            self.assertEqual(metadata["kind"], "challenge")
            self.assertFalse(rows)
            peer.sendall(encode_packet({
                "kind": "sensor_ack", "generation": generation,
                "sensor_sequence": first["sensor_sequence"]}))
            second, rows = next_sensor()
            self.assertEqual(rows[0]["data"], b"latest")
            self.assertEqual(second["sensor_sequence"], first["sensor_sequence"] + 1)
            peer.sendall(encode_packet({
                "kind": "sensor_ack", "generation": generation, "sensor_sequence": 999}))
            wait_for(lambda: not server.connected)
            self.assertIn("acknowledgement", server.error)
        finally:
            if peer:
                peer.close()
            server.close()

    def test_reconnect_cannot_join_old_rgb_with_late_depth_info(self):
        epoch = CaptureEpoch()
        epoch.transition("connection-a")
        rgb_token = epoch.observe(10.)
        self.assertEqual(rgb_token, "connection-a")
        epoch.transition(None)
        epoch.observe(10.6)
        epoch.transition("connection-b")
        self.assertIsNone(epoch.observe(10.))
        self.assertFalse(epoch.allows(rgb_token, [10., 10., 10.]))
        self.assertEqual(epoch.observe(10.7), "connection-b")
        self.assertTrue(epoch.allows("connection-b", [10.7, 10.7, 10.7]))

    def test_capture_floor_covers_camera_message_newer_than_last_clock(self):
        epoch = CaptureEpoch()
        epoch.transition("old")
        epoch.observe(9.9)
        epoch.observe(10.)
        epoch.transition("new")
        self.assertEqual(epoch.floor, 10.)
        self.assertIsNone(epoch.observe(10.))
        self.assertFalse(epoch.allows("new", [10., 10.1, 10.1]))

    def test_simulation_clock_rewind_requires_explicit_restart_not_watermark_lowering(self):
        epoch = CaptureEpoch()
        epoch.transition("session")
        epoch.observe(10.)
        epoch.check_clock(10.)
        epoch.check_clock(10.)  # a paused clock is allowed
        with self.assertRaisesRegex(ValueError, "restart relay AND runtime"):
            epoch.check_clock(0.)
        self.assertEqual(epoch.watermark, 10.)

    def test_clock_odom_tf_precede_camera_info_depth_and_rgb(self):
        output = sensor_order([record(topic) for topic in reversed(list(SENSORS))])
        topics = [row["topic"] for row in output]
        self.assertEqual(topics[:3], ["/clock", "/odom", "/tf"])
        self.assertLess(topics.index("/fpga/camera_info"), topics.index("/fpga/rgb"))
        self.assertLess(topics.index("/fpga/depth"), topics.index("/fpga/rgb"))

    def test_tf_history_survives_coalescing_and_has_time_and_capacity_bounds(self):
        history = TransformHistory(seconds=2, capacity=3)
        for stamp in (0, 1, 2, 2.5):
            history.append(stamp, "tf-{}".format(stamp))
        self.assertEqual(history.values(), ["tf-1", "tf-2", "tf-2.5"])
        history.append(4, "tf-4")
        self.assertEqual(history.values(), ["tf-2", "tf-2.5", "tf-4"])
        history.append(4, "replacement")
        self.assertEqual(history.values()[-1], "replacement")
        history.append(.1, "rewound")
        self.assertEqual(history.values(), ["rewound"])

    def test_binary_roundtrip_and_rgbd_identity(self):
        records = [record(topic, batch="frame-1") for topic in sorted(RGBD)]
        metadata, decoded = decode_packet(encode_packet({"kind": "sensors"}, records)[4:], "sensors")
        self.assertEqual(metadata["kind"], "sensors")
        self.assertEqual({r["topic"] for r in decoded}, RGBD)
        self.assertTrue(all(r["data"] == b"fixture-cdr" for r in decoded))
        for bad in (records[:2], [dict(r, batch=str(i)) for i, r in enumerate(records)]):
            with self.assertRaises(ValueError):
                decode_packet(encode_packet({}, bad)[4:], "sensors")

    def test_wrong_direction_type_unlisted_topic_and_extra_bytes_rejected(self):
        sensor = record("/clock")
        with self.assertRaises(ValueError):
            decode_packet(encode_packet({}, [sensor])[4:], "commands")
        for bad in (dict(sensor, type="wrong/msg/Type"), dict(sensor, topic="/world/objects")):
            with self.assertRaises(ValueError):
                decode_packet(encode_packet({}, [bad])[4:], "sensors")
        with self.assertRaises(ValueError):
            decode_packet(encode_packet({}, [sensor])[4:]+b"extra", "sensors")

    def test_declared_size_checked_before_payload_allocation_and_disconnect(self):
        left, right = socket.socketpair()
        try:
            left.sendall(struct.pack("!I", MAX_BYTES+1))
            with self.assertRaisesRegex(ValueError, "size limit"):
                receive_packet(right, "sensors")
        finally:
            left.close()
            right.close()
        left, right = socket.socketpair()
        try:
            left.sendall(struct.pack("!I", 100)+b"partial")
            left.close()
            with self.assertRaises(ConnectionError):
                receive_packet(right, "sensors")
        finally:
            left.close()
            right.close()

    def test_receive_deadline_catches_stalled_partial_sender(self):
        left, right = socket.socketpair()
        try:
            left.sendall(struct.pack("!I", 100))
            started = time.monotonic()
            with self.assertRaises((TimeoutError, socket.timeout)):
                receive_packet(right, "sensors", timeout=.04)
            self.assertLess(time.monotonic()-started, .3)
        finally:
            left.close()
            right.close()

    def test_latest_buffers_keep_rgbd_atomic_and_total_bytes_bounded(self):
        buffer = LatestBatches(("rgbd", "points"), max_bytes=20)
        first = [record(topic, b"old", batch="old") for topic in RGBD]
        latest = [record(topic, b"new", batch="new") for topic in RGBD]
        buffer.put("rgbd", first)
        buffer.put("rgbd", latest)
        output = buffer.take()
        self.assertEqual(len(output), 3)
        self.assertTrue(all(row["batch"] == "new" for row in output))
        self.assertEqual(buffer.take(), [])
        buffer.put("rgbd", latest)
        buffer.put("points", [record("/fpga/points", b"x"*15)])
        self.assertEqual(len(buffer.take()), 1)
        with self.assertRaises(ValueError):
            buffer.put("points", [record("/fpga/points", b"x"*21)])

    def test_frame_sized_payload_can_transfer_and_sustained_slow_link_hits_deadline(self):
        # Size-shaped byte fixtures, not a claim of real CDR/SSH compatibility.
        records = [record("/fpga/rgb", b"r"*(320*240*3), batch="large"),
                   record("/fpga/depth", b"d"*(320*240*4), batch="large"),
                   record("/fpga/camera_info", b"i"*512, batch="large")]
        packet = encode_packet({}, records)
        self.assertGreater(len(packet), 530000)
        for slow in (False, True):
            left, right = socket.socketpair()

            def send():
                try:
                    for offset in range(0, len(packet), 16384):
                        left.sendall(packet[offset:offset+16384])
                        if slow:
                            time.sleep(.02)
                except OSError:
                    pass

            worker = threading.Thread(target=send)
            worker.start()
            try:
                if slow:
                    with self.assertRaises((TimeoutError, socket.timeout)):
                        receive_packet(right, "sensors")
                else:
                    _, output = receive_packet(right, "sensors")
                    self.assertEqual(sum(len(r["data"]) for r in output), 538112)
            finally:
                right.close()
                left.close()
                worker.join(timeout=1)

    def test_permission_lease_deducts_queue_age_and_roundtrip_without_clock_sync(self):
        window = CommandFreshness("session")
        nonce = window.challenge(10)
        # Issued 10, received 10.2, sender had queued it 0.1 s: only 0.2 s remains.
        self.assertAlmostEqual(window.deadline("session", nonce, 1, .1, 10.2), 10.4)
        with self.assertRaises(ValueError):
            window.deadline("session", nonce, 2, .1, 10.5)

    def test_cross_generation_replay_duplicate_sequence_and_invalid_ages_rejected(self):
        window = CommandFreshness("new")
        nonce = window.challenge(100)
        for generation, token, sequence, age in (("old", nonce, 1, 0), ("new", "unknown", 1, 0),
                                                  ("new", nonce, 1, -1), ("new", nonce, 1, float("nan"))):
            with self.assertRaises(ValueError):
                window.deadline(generation, token, sequence, age, 100.1)
        window.deadline("new", nonce, 1, 0, 100.1)
        with self.assertRaises(ValueError):
            window.deadline("new", nonce, 1, 0, 100.2)

    def test_false_permission_has_separate_slot_from_later_true(self):
        pending = LatestBatches(("stop", "/fpga/motion_permit", "/cmd_vel"))
        pending.put("stop", [record("/fpga/motion_permit", permit=False, sequence=1)])
        pending.put("/fpga/motion_permit", [record("/fpga/motion_permit", permit=True, sequence=2)])
        records = pending.take()
        self.assertEqual([row["permit"] for row in records], [False, True])

    def test_localhost_connection_transfers_batches_once_and_stops_on_disconnect(self):
        server = Tunnel("server", "127.0.0.1", 0)
        client = None
        server.start()
        try:
            wait_for(lambda: server.port != 0)
            client = Tunnel("client", "127.0.0.1", server.port)
            client.start()
            wait_for(lambda: client.connected and server.connected)
            generation = server.generation
            self.assertEqual(generation, client.generation)
            server.outgoing.put("rgbd", [record(topic, batch="sample", generation=generation) for topic in RGBD])
            wait_for(lambda: bool(client.incoming.pending), SPAWN_S)   # the first batch: the encoder worker starts
            self.assertEqual({r["topic"] for r in client.incoming.take()}, RGBD)
            time.sleep(.15)
            self.assertEqual(client.incoming.take(), [])
            client.outgoing.put("/fpga/motion_permit", [record(
                "/fpga/motion_permit", permit=True, sequence=1, queued_at=time.monotonic(), generation=generation)])
            wait_for(lambda: bool(server.incoming.pending))
            command = server.incoming.take()[0]
            self.assertTrue(command["permit"])
            self.assertLessEqual(command["expires"]-time.monotonic(), .5)
            time.sleep(.15)
            self.assertEqual(server.incoming.take(), [])  # heartbeat cannot replay permission
            client.close()
            wait_for(lambda: not server.connected)
            self.assertTrue(server.stop_requested)
        finally:
            if client:
                client.close()
            server.close()

    def test_endpoint_is_loopback_only(self):
        with self.assertRaises(ValueError):
            Tunnel("server", "0.0.0.0", 1234)


class LocalizationStatusRelayTests(unittest.TestCase):
    """The gate's status travels the same direction as the permission it feeds."""

    def test_status_is_whitelisted_client_to_server_with_its_declared_type(self):
        self.assertEqual(COMMANDS[STATUS], "std_msgs/msg/String")
        self.assertNotIn(STATUS, SENSORS)
        metadata, decoded = decode_packet(
            encode_packet({"kind": "commands"}, [record(STATUS, sequence=1)])[4:], "commands")
        self.assertEqual(metadata["kind"], "commands")
        self.assertEqual([r["topic"] for r in decoded], [STATUS])
        self.assertEqual(decoded[0]["type"], "std_msgs/msg/String")
        self.assertEqual(decoded[0]["data"], b"fixture-cdr")

    def test_status_in_the_sensor_direction_and_a_wrong_type_are_rejected(self):
        status = record(STATUS, sequence=1)
        with self.assertRaises(ValueError):
            decode_packet(encode_packet({}, [status])[4:], "sensors")
        for bad in (dict(status, type="std_msgs/msg/Bool"), dict(status, type="tf2_msgs/msg/TFMessage")):
            with self.assertRaises(ValueError):
                decode_packet(encode_packet({}, [bad])[4:], "commands")

    def test_the_gate_s_other_topics_are_still_outside_the_relay_contract(self):
        for topic in ("/fpga/localization_tf", "/fpga/localization_pose", "/fpga/odom_adapter_status",
                      "/fpga/localization_raw_tf"):
            self.assertNotIn(topic, {**SENSORS, **COMMANDS})
            bad = {"topic": topic, "type": "std_msgs/msg/String", "data": b"fixture-cdr"}
            for direction in ("commands", "sensors"):
                with self.assertRaises(ValueError):
                    decode_packet(encode_packet({}, [bad])[4:], direction)

    def test_status_carries_no_motion_authority(self):
        # MOTION is what the server's poll treats as a command: expiry stops the robot,
        # arrival clears the stop. A localization heartbeat must do neither.
        self.assertNotIn(STATUS, MOTION)
        self.assertEqual(set(MOTION), {"/cmd_vel", "/fpga/motion_permit"})
        self.assertLessEqual(MOTION, set(COMMANDS))
        source = inspect.getsource(tunnel.main)
        self.assertIn('if args.mode == "server" and topic in MOTION:', source)
        self.assertIn("self.command_expiry = record[\"expires\"]", source)
        # safe_stop only ever republishes the two motion topics.
        self.assertIn("for topic in MOTION if topic in self.commands}", source)
        self.assertNotIn("for topic in COMMANDS}", source)

    def test_status_has_its_own_outgoing_slot_and_never_evicts_a_permission(self):
        pending = LatestBatches(tuple(COMMANDS) + ("stop",))
        pending.put("/fpga/motion_permit", [record("/fpga/motion_permit", permit=True, sequence=1)])
        pending.put(STATUS, [record(STATUS, b"status-a", sequence=2)])
        pending.put(STATUS, [record(STATUS, b"status-b", sequence=3)])
        records = {row["topic"]: row for row in pending.take()}
        self.assertEqual(set(records), {"/fpga/motion_permit", STATUS})
        self.assertTrue(records["/fpga/motion_permit"]["permit"])
        self.assertEqual(records[STATUS]["data"], b"status-b")  # latest status only

    def test_status_reaches_the_server_over_a_localhost_connection(self):
        server = Tunnel("server", "127.0.0.1", 0)
        client = None
        server.start()
        try:
            wait_for(lambda: server.port != 0)
            client = Tunnel("client", "127.0.0.1", server.port)
            client.start()
            wait_for(lambda: client.connected and server.connected)
            # receive the connection's challenge first: the client drops what it drains before one (by design: no
            # nonce, no command). Was a fixed 0.2 s; 10-17 the client's own flag (LOAD_TIMING_TESTS_20261017)
            wait_for(lambda: client.challenge_seen)
            generation = server.generation
            server.consume_stop()  # clear the connection's initial fail-closed stop
            client.outgoing.put(STATUS, [record(
                STATUS, b"status-cdr", sequence=1, queued_at=time.monotonic(), generation=generation)])
            wait_for(lambda: bool(server.incoming.pending))
            arrived = server.incoming.take()
            self.assertEqual([row["topic"] for row in arrived], [STATUS])
            self.assertEqual(arrived[0]["type"], "std_msgs/msg/String")
            self.assertEqual(arrived[0]["data"], b"status-cdr")
            self.assertNotIn("permit", arrived[0])  # no permission metadata
            self.assertFalse(server.consume_stop())  # and no stop latch
        finally:
            if client:
                client.close()
            server.close()


if __name__ == "__main__":
    unittest.main()
