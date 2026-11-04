import os
from dataclasses import asdict
import tempfile
import unittest

import numpy as np

from host.m7_pipeline import CaptureIdentity
from tools.m7_memory_ros2_probe import (
    BoundedProbeAdapter,
    GraphEvidence,
    MAX_REQUEST_BYTES,
    PROTOCOL,
    WireError,
    append_event,
    close_event_stream,
    validate_ack,
)
from tools.m7_memory_transport_probe import encode_request


def metadata(kind, request_id="request-1", operation_id="operation-1", attempt=1, **fields):
    return {"schema": PROTOCOL, "request_id": request_id,
            "operation_id": operation_id, "attempt": attempt, "kind": kind,
            "client_sent_steady_ns": 123456789, **fields}


class FakeService:
    def __init__(self):
        self.frames = {}
        self.outputs = {}
        self.calls = []

    def handle(self, request, arrays):
        self.calls.append((request, arrays))
        if request["kind"] == "geometry":
            identity = CaptureIdentity(**request["identity"])
            self.frames[identity] = object()
        if request["kind"] == "semantic":
            self.outputs[request["save_as"]] = object()
        return {"handled": request["kind"]}


def geometry_body(frame_id, request_id):
    identity = CaptureIdentity("test-stream", frame_id, frame_id * 1000,
                               "test-clock", "test-calibration", "test-local", "test-pose")
    return encode_request(metadata(
        "geometry", request_id=request_id, operation_id=request_id,
        identity=asdict(identity), capture={}, anchor={}, pose_stamp_ns=identity.acquired_ns,
        pose_valid=True, now_ns=identity.acquired_ns + 1),
        {"rgb": np.zeros((1, 1, 3), np.uint8),
         "depth": np.ones((1, 1), np.float32), "K": np.eye(3), "T": np.eye(4)})


def semantic_body(request_id):
    identity = CaptureIdentity("test-stream", 1, 1000, "test-clock",
                               "test-calibration", "test-local", "test-pose")
    return encode_request(metadata(
        "semantic", request_id=request_id, operation_id=request_id,
        identity=asdict(identity), frame_token="token", object_id="object-1",
        detection={}, now_ns=1001), {"mask": np.ones((1, 1), np.bool_)})


class BoundedAdapterTests(unittest.TestCase):
    def test_bad_identifier_and_extra_field_never_reach_service_or_path(self):
        with tempfile.TemporaryDirectory() as directory:
            service = FakeService()
            adapter = BoundedProbeAdapter(service)
            before = set(os.listdir(directory))
            bad_id = encode_request(metadata("query", request_id="../escape"))
            extra = encode_request({**metadata("query", request_id="strict-extra"),
                                    "unexpected": "not-in-schema"})
            for body in (bad_id, extra):
                with self.assertRaises(WireError):
                    adapter.handle_body(body)
            self.assertEqual(service.calls, [])
            self.assertEqual(set(os.listdir(directory)), before)

    def test_external_identifier_is_replaced_by_safe_internal_artifact_token(self):
        service = FakeService()
        adapter = BoundedProbeAdapter(service)
        response = adapter.handle_body(encode_request(metadata(
            "query", request_id="external.request-1", operation_id="stable-op")))
        internal = service.calls[0][0]
        self.assertEqual(response["request_id"], "external.request-1")
        self.assertRegex(internal["request_id"], r"^ros-[0-9a-f]{32}$")
        self.assertNotIn("external.request-1", internal["request_id"])

    def test_request_frame_and_output_limits_reject_before_service(self):
        service = FakeService()
        adapter = BoundedProbeAdapter(service, max_requests=1)
        query = encode_request(metadata("query"))
        adapter.handle_body(query)
        with self.assertRaisesRegex(WireError, "request count"):
            adapter.handle_body(query)
        self.assertEqual(len(service.calls), 1)

        service = FakeService()
        adapter = BoundedProbeAdapter(service, max_frames=1)
        adapter.handle_body(geometry_body(1, "geometry-1"))
        with self.assertRaisesRegex(WireError, "frame reference"):
            adapter.handle_body(geometry_body(2, "geometry-2"))
        self.assertEqual(len(service.calls), 1)
        self.assertEqual(len(service.frames), 1)

        service = FakeService()
        adapter = BoundedProbeAdapter(service, max_outputs=1)
        adapter.handle_body(semantic_body("semantic-1"))
        with self.assertRaisesRegex(WireError, "output reference"):
            adapter.handle_body(semantic_body("semantic-2"))
        self.assertEqual(len(service.calls), 1)
        self.assertEqual(len(service.outputs), 1)

    def test_post_deserialization_size_limit_is_explicit(self):
        adapter = BoundedProbeAdapter(FakeService())
        with self.assertRaisesRegex(WireError, "post-deserialization"):
            adapter.handle_body(b"")
        with self.assertRaisesRegex(WireError, "post-deserialization"):
            adapter.handle_body(b"x" * (MAX_REQUEST_BYTES + 1))

    def test_ack_rejects_wrong_payload_hash_and_old_attempt(self):
        correct = {"schema": PROTOCOL, "request_id": "request-1",
                   "operation_id": "operation-1", "attempt": 2, "kind": "semantic",
                   "request_payload_sha256": "a" * 64}
        self.assertIs(validate_ack(correct, request_id="request-1",
                                  operation_id="operation-1", attempt=2, kind="semantic",
                                  payload_sha256="a" * 64), correct)
        wrong_hash = {**correct, "request_payload_sha256": "b" * 64}
        old_attempt = {**correct, "attempt": 1}
        for response in (wrong_hash, old_attempt):
            with self.assertRaisesRegex(ValueError, "correlation mismatch"):
                validate_ack(response, request_id="request-1",
                             operation_id="operation-1", attempt=2, kind="semantic",
                             payload_sha256="a" * 64)


class FakeQos:
    reliability = "RELIABLE"
    durability = "VOLATILE"
    history = "KEEP_LAST"
    depth = 4


class FakeEndpointInfo:
    node_name = "peer"
    node_namespace = "/"
    topic_type = "std_msgs/msg/UInt8MultiArray"
    endpoint_type = "PUBLISHER"
    qos_profile = FakeQos()


class FakeEndpoint:
    def __init__(self, matched):
        self.matched = matched

    def get_publisher_count(self):
        return self.matched


class FakeSubscriptionWithoutMatchCount:
    pass


class FakeEndpointLosesContext:
    def __init__(self):
        self.calls = 0

    def get_publisher_count(self):
        self.calls += 1
        if self.calls > 1:
            raise RuntimeError("publisher context is invalid")
        return 1


class FakeNode:
    def get_publishers_info_by_topic(self, topic):
        return [FakeEndpointInfo()]


class GraphEvidenceTests(unittest.TestCase):
    def test_discovery_and_compatible_match_are_recorded_separately(self):
        endpoint = FakeEndpoint(0)
        evidence = GraphEvidence(FakeNode(), endpoint,
                                 peer_method="get_publishers_info_by_topic", topic="/request")
        evidence.observe()
        endpoint.matched = 1
        evidence.observe()
        result = evidence.result()
        self.assertEqual(result["discovered_max"], 1)
        self.assertEqual(result["matched_max"], 1)
        self.assertEqual([row["matched"] for row in result["observations"]], [0, 1])

    def test_subscription_missing_match_api_is_none_not_zero(self):
        evidence = GraphEvidence(FakeNode(), FakeSubscriptionWithoutMatchCount(),
                                 peer_method="get_publishers_info_by_topic", topic="/request")
        evidence.observe()
        result = evidence.result()
        self.assertFalse(result["match_count_supported"])
        self.assertIsNone(result["matched_max"])
        self.assertIsNone(result["observations"][0]["matched"])

    def test_context_invalid_keeps_prior_match_evidence_and_records_error(self):
        evidence = GraphEvidence(FakeNode(), FakeEndpointLosesContext(),
                                 peer_method="get_publishers_info_by_topic", topic="/request")
        evidence.observe()
        evidence.observe()
        result = evidence.result()
        self.assertTrue(result["match_count_supported"])
        self.assertEqual(result["matched_max"], 1)
        self.assertEqual(result["inspection_errors"][0]["error"], "RuntimeError")
        self.assertIn("context is invalid", result["inspection_errors"][0]["reason"])

    def test_event_log_helper_fsyncs_and_explicitly_closes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "events.jsonl")
            stream = open(path, "x", encoding="utf8")
            append_event(stream, {"event": "known"})
            close_event_stream(stream)
            self.assertTrue(stream.closed)
            with open(path, encoding="utf8") as saved:
                self.assertEqual(saved.read(), '{"event": "known"}\n')


if __name__ == "__main__":
    unittest.main()
