#!/usr/bin/env python3
"""Bounded ROS 2 two-process M7->M4 probe; never a deployment service.

The formal run is confined to one ``--network none`` ROS Humble container.  Request and
ACK topics carry bounded UInt8MultiArray payloads.  The server delegates valid requests
to the already accepted Unix probe's ProbeService so this file does not replace the M7/M4
production logic with a mock callback.
"""

import argparse
from array import array
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import numpy as np

from tools.m7_memory_transport_probe import (
    ProbeService,
    canonical,
    decode_request,
    encode_request,
    file_sha,
    frame_fixture,
    identity_from,
    sha,
    write_json,
    write_new,
)


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = "fpga.m7m4.ros_probe.v1"
REQUEST_TOPIC = "/fpga/probe/memory_request"
ACK_TOPIC = "/fpga/probe/memory_ack"
MAX_REQUEST_BYTES = 32 * 1024 * 1024
MAX_ACK_BYTES = 1024 * 1024
MAX_REQUESTS = 32
MAX_FRAMES = 4
MAX_OUTPUTS = 8
ACK_SECONDS = 2.0
DISCOVERY_SECONDS = 5.0
QOS_NEGATIVE_SECONDS = 2.0
ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
EXPECTED_IMAGE = "fpga-dev1-rtabmap:humble-0.23.7-20260910"
EXPECTED_IMAGE_ID = "sha256:c96783dbd884583a013eabfc393344f42473bfb12e51fc46979f172fed83ace7"
FROZEN_LIBRARY = ROOT / "runs/team-continuous-transport-20260921-v1/native-m7.so"
FROZEN_MANIFEST = ROOT / "runs/team-continuous-transport-20260921-v1/manifest.json"
FROZEN_LIBRARY_SHA256 = "93f42743186842ea2f5d3b16917f9fc8b25188d8c74b85ba6f9dc2a6d96d7e21"
FROZEN_CPP_SHA256 = {
    "hw/geometry/geometry.cpp": "1952661919c926403867958c55802fd09af23c090e9486e875ea39b528606034",
    "hw/m7/ray_flags.cpp": "f2dd87037d22b24e2a0328a180581d58673109df3bb21257a4439d363ac75574",
}

BASE_FIELDS = {
    "schema", "request_id", "operation_id", "attempt", "kind",
    "client_sent_steady_ns",
}
KIND_FIELDS = {
    "geometry": {
        "identity", "capture", "anchor", "pose_stamp_ns", "pose_valid", "now_ns",
    },
    "semantic": {
        "identity", "frame_token", "object_id", "detection", "now_ns",
    },
    "query": set(),
    "shutdown": set(),
}


class WireError(ValueError):
    """A request was rejected by the ROS probe wrapper before ProbeService."""


def require_identifier(value, name):
    if not isinstance(value, str) or ID_PATTERN.fullmatch(value) is None:
        raise WireError(f"{name} is not a bounded protocol identifier")
    return value


def validate_wire_metadata(metadata):
    if not isinstance(metadata, dict):
        raise WireError("request metadata must be an object")
    kind = metadata.get("kind")
    if kind not in KIND_FIELDS:
        raise WireError("unsupported ROS probe request kind")
    expected = BASE_FIELDS | KIND_FIELDS[kind]
    if set(metadata) != expected:
        raise WireError("request fields do not exactly match the ROS probe schema")
    if metadata.get("schema") != PROTOCOL:
        raise WireError("wrong ROS probe schema")
    require_identifier(metadata.get("request_id"), "request_id")
    require_identifier(metadata.get("operation_id"), "operation_id")
    attempt = metadata.get("attempt")
    if type(attempt) is not int or not 1 <= attempt <= 8:
        raise WireError("attempt must be an integer in [1,8]")
    sent = metadata.get("client_sent_steady_ns")
    if type(sent) is not int or sent <= 0:
        raise WireError("client_sent_steady_ns must be a positive integer")
    return metadata


def artifact_token(metadata):
    material = canonical({"request_id": metadata["request_id"],
                          "operation_id": metadata["operation_id"],
                          "attempt": metadata["attempt"]})
    return "ros-" + sha(material)[:32]


class BoundedProbeAdapter:
    """Strict and count-bounded adapter around the old one-shot ProbeService."""

    def __init__(self, service, *, max_requests=MAX_REQUESTS,
                 max_frames=MAX_FRAMES, max_outputs=MAX_OUTPUTS):
        if any(type(n) is not int or n <= 0 for n in
               (max_requests, max_frames, max_outputs)):
            raise ValueError("positive adapter limits required")
        self.service = service
        self.max_requests = max_requests
        self.max_frames = max_frames
        self.max_outputs = max_outputs
        self.received_count = 0
        self.service_calls = 0

    def handle_body(self, body):
        self.received_count += 1
        if self.received_count > self.max_requests:
            raise WireError("request count limit exceeded")
        if not isinstance(body, bytes) or not 0 < len(body) <= MAX_REQUEST_BYTES:
            raise WireError("request exceeds post-deserialization application limit")
        metadata, arrays = decode_request(body)
        validate_wire_metadata(metadata)
        kind = metadata["kind"]
        if kind == "geometry":
            identity = identity_from(metadata["identity"])
            if identity not in self.service.frames and len(self.service.frames) >= self.max_frames:
                raise WireError("wrapper frame reference limit exceeded")
        if kind == "semantic" and len(self.service.outputs) >= self.max_outputs:
            raise WireError("wrapper output reference limit exceeded")

        internal = {key: deepcopy(value) for key, value in metadata.items()
                    if key not in BASE_FIELDS - {"kind"}}
        token = artifact_token(metadata)
        internal["request_id"] = token
        if kind == "semantic":
            internal["save_as"] = token
        before = {"frames": len(self.service.frames), "outputs": len(self.service.outputs)}
        self.service_calls += 1
        result = self.service.handle(internal, arrays)
        after = {"frames": len(self.service.frames), "outputs": len(self.service.outputs)}
        if after["frames"] > self.max_frames or after["outputs"] > self.max_outputs:
            raise RuntimeError("legacy service exceeded wrapper reference bounds")
        return {
            "request_id": metadata["request_id"],
            "operation_id": metadata["operation_id"],
            "attempt": metadata["attempt"],
            "kind": kind,
            "artifact_token": token,
            "result": result,
            "reference_counts_before": before,
            "reference_counts_after": after,
        }


def reliability_name(value):
    name = getattr(value, "name", None)
    if name:
        return str(name)
    text = str(value)
    return text.rsplit(".", 1)[-1]


def endpoint_rows(node, method_name, topic):
    rows = []
    try:
        infos = getattr(node, method_name)(topic)
    except Exception as exc:
        return [{"inspection_error": type(exc).__name__ + ": " + str(exc)}]
    for info in infos:
        qos = getattr(info, "qos_profile", None)
        rows.append({
            "node_name": getattr(info, "node_name", None),
            "node_namespace": getattr(info, "node_namespace", None),
            "topic_type": getattr(info, "topic_type", None),
            "endpoint_type": str(getattr(info, "endpoint_type", None)),
            "reliability": reliability_name(getattr(qos, "reliability", None)),
            "durability": reliability_name(getattr(qos, "durability", None)),
            "history": reliability_name(getattr(qos, "history", None)),
            "depth": getattr(qos, "depth", None),
        })
    return rows


class GraphEvidence:
    """Keep bounded evidence of ROS graph discovery and compatible matches."""

    def __init__(self, node, endpoint, *, peer_method, topic):
        self.node = node
        self.endpoint = endpoint
        self.peer_method = peer_method
        self.topic = topic
        self.observations = []
        self.discovered_max = 0
        self.matched_max = None
        self.match_count_supported = None
        self.inspection_errors = []
        self._last = None

    def observe(self):
        rows = endpoint_rows(self.node, self.peer_method, self.topic)
        discovered = len([row for row in rows if "inspection_error" not in row])
        match_getter = (getattr(self.endpoint, "get_subscription_count", None) or
                        getattr(self.endpoint, "get_publisher_count", None))
        matched = None
        if match_getter is None:
            self.match_count_supported = False
        else:
            self.match_count_supported = True
            try:
                matched = int(match_getter())
            except Exception as exc:
                if len(self.inspection_errors) < 8:
                    self.inspection_errors.append({
                        "steady_ns": time.monotonic_ns(), "stage": "match_count",
                        "error": type(exc).__name__, "reason": str(exc)})
        for row in rows:
            if "inspection_error" in row and len(self.inspection_errors) < 8:
                self.inspection_errors.append({"steady_ns": time.monotonic_ns(),
                                               "stage": "endpoint_graph",
                                               "reason": row["inspection_error"]})
        self.discovered_max = max(self.discovered_max, discovered)
        if matched is not None:
            self.matched_max = (matched if self.matched_max is None else
                                max(self.matched_max, matched))
        signature = canonical({"rows": rows, "matched": matched})
        if signature != self._last and len(self.observations) < 16:
            self.observations.append({"steady_ns": time.monotonic_ns(),
                                      "discovered": discovered,
                                      "matched": matched, "endpoints": rows})
            self._last = signature

    def result(self):
        return {"topic": self.topic, "discovered_max": self.discovered_max,
                "match_count_supported": self.match_count_supported,
                "matched_max": self.matched_max, "observations": self.observations,
                "inspection_errors": self.inspection_errors}


def qos_incompatibility_row(event):
    from rclpy.qos_event import qos_policy_name_from_kind
    kind = getattr(event, "last_policy_kind", None)
    try:
        policy = qos_policy_name_from_kind(kind)
    except Exception:
        policy = str(kind)
    return {"steady_ns": time.monotonic_ns(), "policy": policy,
            "total_count": int(getattr(event, "total_count", -1)),
            "total_count_change": int(getattr(event, "total_count_change", -1))}


def append_event(stream, row):
    stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())


def close_event_stream(stream):
    if not stream.closed:
        stream.flush()
        os.fsync(stream.fileno())
        stream.close()


def qos_profile(reliability):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    policy = {"reliable": ReliabilityPolicy.RELIABLE,
              "best_effort": ReliabilityPolicy.BEST_EFFORT}[reliability]
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=4,
                      reliability=policy, durability=DurabilityPolicy.VOLATILE)


def message_from(body):
    from std_msgs.msg import UInt8MultiArray
    return UInt8MultiArray(data=array("B", body))


def role_environment():
    import rclpy
    from rclpy.utilities import get_rmw_implementation_identifier
    return {
        "pid": os.getpid(),
        "python": sys.version,
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
        "rmw_implementation_env": os.environ.get("RMW_IMPLEMENTATION"),
        "rmw_implementation_actual": get_rmw_implementation_identifier(),
    }


def validate_ack(response, *, request_id, operation_id, attempt, kind, payload_sha256):
    expected = {"schema": PROTOCOL, "request_id": request_id,
                "operation_id": operation_id, "attempt": attempt, "kind": kind,
                "request_payload_sha256": payload_sha256}
    if not isinstance(response, dict):
        raise ValueError("application ACK must be a JSON object")
    mismatches = {key: {"expected": value, "actual": response.get(key)}
                  for key, value in expected.items() if response.get(key) != value}
    if mismatches:
        raise ValueError("application ACK correlation mismatch: " +
                         json.dumps(mismatches, sort_keys=True))
    return response


def run_server(args):
    import rclpy
    from rclpy.qos_event import SubscriptionEventCallbacks
    from rclpy.serialization import serialize_message

    result = args.result.resolve()
    result.mkdir(parents=True, exist_ok=False)
    (result / "requests").mkdir()
    (result / "acks").mkdir()
    event_path = result / "events.jsonl"
    store = args.store.resolve()
    service = ProbeService(store, args.library.resolve())
    adapter = BoundedProbeAdapter(service)
    state = {
        "ok": False, "callbacks": 0, "acks_published": 0, "ack_dropped": False,
        "explicit_close": False, "drop_operation_id": args.drop_ack_on,
        "request_incompatible_callback_registered": True,
        "request_incompatible_events": [],
    }
    deadline = time.monotonic() + args.max_seconds
    rclpy.init(args=[])
    node = rclpy.create_node("fpga_m7m4_probe_server_" + args.name)
    ack_pub = node.create_publisher(message_from(b" ").__class__, ACK_TOPIC,
                                    qos_profile("reliable"))
    event_stream = event_path.open("x", encoding="utf8")
    def callback(message):
        received_ns = time.monotonic_ns()
        sequence = state["callbacks"] + 1
        state["callbacks"] = sequence
        body = bytes(message.data)
        cdr = serialize_message(message)
        request_path = result / "requests" / f"{sequence:03d}.body.bin"
        cdr_path = result / "requests" / f"{sequence:03d}.cdr.bin"
        write_new(request_path, body)
        write_new(cdr_path, cdr)
        before = file_sha(store)
        metadata = None
        service_calls_before = adapter.service_calls
        handled_start = time.monotonic_ns()
        try:
            try:
                metadata, _ = decode_request(body)
            except Exception:
                metadata = None
            handled = adapter.handle_body(body)
            response = {"ok": True, **handled}
        except Exception as exc:
            response = {
                "ok": False,
                "request_id": metadata.get("request_id") if isinstance(metadata, dict) else None,
                "operation_id": metadata.get("operation_id") if isinstance(metadata, dict) else None,
                "attempt": metadata.get("attempt") if isinstance(metadata, dict) else None,
                "kind": metadata.get("kind") if isinstance(metadata, dict) else None,
                "error": type(exc).__name__, "reason": str(exc),
            }
        handled_end = time.monotonic_ns()
        response.update({
            "schema": PROTOCOL,
            "server_pid": os.getpid(),
            "request_payload_sha256": sha(body),
            "server_callback_steady_ns": received_ns,
            "server_handle_start_steady_ns": handled_start,
            "server_handle_end_steady_ns": handled_end,
            "store_sha256": file_sha(store),
        })
        drop = bool(args.drop_ack_on and response.get("ok") and
                    response.get("operation_id") == args.drop_ack_on)
        row = {
            "event": "request", "sequence": sequence,
            "request_id": response.get("request_id"),
            "operation_id": response.get("operation_id"),
            "attempt": response.get("attempt"), "kind": response.get("kind"),
            "request_payload_sha256": sha(body), "request_cdr_sha256": sha(cdr),
            "callback_steady_ns": received_ns,
            "handle_start_steady_ns": handled_start,
            "handle_end_steady_ns": handled_end,
            "service_called": adapter.service_calls > service_calls_before,
            "store_before": before, "store_after": file_sha(store),
            "ok": response["ok"], "error": response.get("error"),
            "ack_dropped": drop,
        }
        if drop:
            append_event(event_stream, row)
            state["ack_dropped"] = True
            state["drop_event"] = row
            state["stop"] = True
            return
        response["server_ack_publish_steady_ns"] = time.monotonic_ns()
        ack_body = canonical(response)
        if len(ack_body) > MAX_ACK_BYTES:
            raise RuntimeError("ACK exceeds post-serialization application limit")
        ack_message = message_from(ack_body)
        ack_cdr = serialize_message(ack_message)
        write_new(result / "acks" / f"{sequence:03d}.body.json", ack_body)
        write_new(result / "acks" / f"{sequence:03d}.cdr.bin", ack_cdr)
        row.update({"ack_payload_sha256": sha(ack_body), "ack_cdr_sha256": sha(ack_cdr),
                    "ack_publish_steady_ns": response["server_ack_publish_steady_ns"]})
        append_event(event_stream, row)
        ack_pub.publish(ack_message)
        state["acks_published"] += 1
        if response.get("kind") == "shutdown" and response.get("ok"):
            state["stop_after"] = time.monotonic() + 0.25

    def incompatible_qos(event):
        if len(state["request_incompatible_events"]) < 8:
            state["request_incompatible_events"].append(qos_incompatibility_row(event))

    subscription = node.create_subscription(message_from(b" ").__class__, REQUEST_TOPIC,
                                            callback, qos_profile(args.reliability),
                                            event_callbacks=SubscriptionEventCallbacks(
                                                incompatible_qos=incompatible_qos,
                                                use_default_callbacks=False))
    request_graph = GraphEvidence(node, subscription, peer_method="get_publishers_info_by_topic",
                                  topic=REQUEST_TOPIC)
    ack_graph = GraphEvidence(node, ack_pub, peer_method="get_subscriptions_info_by_topic",
                              topic=ACK_TOPIC)
    write_json(result / "ready.json", {**role_environment(), "ready": True,
                                       "request_reliability": args.reliability})
    try:
        while not state.get("stop") and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            request_graph.observe()
            ack_graph.observe()
            if state.get("stop_after", float("inf")) <= time.monotonic():
                state["stop"] = True
        state["timed_out"] = time.monotonic() >= deadline and not state.get("stop")
        state["ok"] = True
    except BaseException as exc:
        state["error"] = {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        state["final_graph_audit_errors"] = []
        for label, graph in (("request", request_graph), ("ack", ack_graph)):
            try:
                graph.observe()
            except Exception as exc:
                state["final_graph_audit_errors"].append({
                    "graph": label, "error": type(exc).__name__, "reason": str(exc)})
        state.update({"environment": role_environment(),
                      "request_reliability": args.reliability,
                      "request_graph": request_graph.result(),
                      "ack_graph": ack_graph.result(),
                      "adapter_received_count": adapter.received_count,
                      "adapter_service_calls": adapter.service_calls,
                      "frame_references": len(service.frames),
                      "output_references": len(service.outputs),
                      "store_sha256": file_sha(store)})
        close_event_stream(event_stream)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        state["explicit_close"] = True
        write_json(result / "status.json", state)


def load_fixture(root, label):
    metadata = json.loads((root / "fixtures.json").read_text(encoding="utf8"))[label]
    return {**metadata,
            **{name: np.load(root / f"{label}.{name}.npy", allow_pickle=False)
               for name in ("rgb", "depth", "K", "T", "mask")}}


class RosProbeClient:
    def __init__(self, node, result, reliability):
        from rclpy.qos_event import PublisherEventCallbacks
        from rclpy.serialization import serialize_message
        self.node = node
        self.result = result
        self.serialize_message = serialize_message
        self.request_incompatible_events = []

        def incompatible_qos(event):
            if len(self.request_incompatible_events) < 8:
                self.request_incompatible_events.append(qos_incompatibility_row(event))

        self.publisher = node.create_publisher(message_from(b" ").__class__, REQUEST_TOPIC,
                                               qos_profile(reliability),
                                               event_callbacks=PublisherEventCallbacks(
                                                   incompatible_qos=incompatible_qos,
                                                   use_default_callbacks=False))
        self.subscription = node.create_subscription(message_from(b" ").__class__, ACK_TOPIC,
                                                     self._ack, qos_profile("reliable"))
        self.request_graph = GraphEvidence(
            node, self.publisher, peer_method="get_subscriptions_info_by_topic",
            topic=REQUEST_TOPIC)
        self.ack_graph = GraphEvidence(
            node, self.subscription, peer_method="get_publishers_info_by_topic",
            topic=ACK_TOPIC)
        self.requests = []
        self.acks = []
        self._sequence = 0
        self._pending = {}
        (result / "requests").mkdir()
        (result / "acks").mkdir()
        self.events = (result / "events.jsonl").open("x", encoding="utf8")

    def _ack(self, message):
        received_ns = time.monotonic_ns()
        body = bytes(message.data)
        if not 0 < len(body) <= MAX_ACK_BYTES:
            raise RuntimeError("ACK exceeds post-deserialization application limit")
        response = json.loads(body)
        self._sequence += 1
        sequence = self._sequence
        cdr = self.serialize_message(message)
        write_new(self.result / "acks" / f"{sequence:03d}.body.json", body)
        write_new(self.result / "acks" / f"{sequence:03d}.cdr.bin", cdr)
        row = {"sequence": sequence, "client_ack_callback_steady_ns": received_ns,
               "payload_sha256": sha(body), "cdr_sha256": sha(cdr),
               "response": response}
        self.acks.append(row)
        key = (response.get("request_id"), response.get("attempt"))
        self._pending[key] = row
        append_event(self.events, {"event": "ack", **row})

    def observe(self):
        self.request_graph.observe()
        self.ack_graph.observe()

    def wait_graph(self, *, require_match, expected_peer_node):
        deadline = time.monotonic() + DISCOVERY_SECONDS
        while time.monotonic() < deadline:
            import rclpy
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.observe()
            request_found = self.request_graph.discovered_max > 0
            request_matched = (self.request_graph.match_count_supported is True and
                               self.request_graph.matched_max is not None and
                               self.request_graph.matched_max > 0)
            ack_found = self.ack_graph.discovered_max > 0
            request_names = graph_node_names(self.request_graph.result())
            ack_names = graph_node_names(self.ack_graph.result())
            if (request_found and (request_matched or not require_match) and
                    ack_found and expected_peer_node in request_names and
                    expected_peer_node in ack_names and
                    "RELIABLE" in graph_reliabilities(self.ack_graph.result())):
                return
        raise RuntimeError("ROS request/ACK endpoint discovery or match timeout")

    def send(self, request_id, operation_id, attempt, kind, *, arrays=None,
             expect_ack=True, timeout=ACK_SECONDS, extra_fields=None, **fields):
        sent_ns = time.monotonic_ns()
        metadata = {"schema": PROTOCOL, "request_id": request_id,
                    "operation_id": operation_id, "attempt": attempt,
                    "kind": kind, "client_sent_steady_ns": sent_ns, **fields}
        if extra_fields:
            metadata.update(extra_fields)
        body = encode_request(metadata, arrays)
        message = message_from(body)
        cdr = self.serialize_message(message)
        sequence = len(self.requests) + 1
        write_new(self.result / "requests" / f"{sequence:03d}.body.bin", body)
        write_new(self.result / "requests" / f"{sequence:03d}.cdr.bin", cdr)
        row = {"sequence": sequence, "request_id": request_id,
               "operation_id": operation_id, "attempt": attempt, "kind": kind,
               "client_publish_steady_ns": sent_ns, "payload_sha256": sha(body),
               "cdr_sha256": sha(cdr), "expect_ack": expect_ack}
        self.requests.append(row)
        append_event(self.events, {"event": "publish", **row})
        self.publisher.publish(message)
        key = (request_id, attempt)
        deadline = time.monotonic() + timeout
        import rclpy
        while time.monotonic() < deadline and key not in self._pending:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.observe()
        ack = self._pending.get(key)
        row["ack_received"] = ack is not None
        if expect_ack and ack is None:
            raise TimeoutError("application ACK timeout")
        if not expect_ack and ack is not None:
            raise AssertionError("unexpected application ACK")
        if ack:
            validate_ack(ack["response"], request_id=request_id,
                         operation_id=operation_id, attempt=attempt, kind=kind,
                         payload_sha256=sha(body))
        return ack["response"] if ack else None

    def settle_without_ack(self, seconds):
        deadline = time.monotonic() + seconds
        initial = len(self.acks)
        import rclpy
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=min(0.05, deadline-time.monotonic()))
            self.observe()
        if len(self.acks) != initial:
            raise AssertionError("unexpected ACK during bounded QoS-negative window")

    def close(self):
        try:
            self.observe()
        finally:
            close_event_stream(self.events)


def geometry_fields(fixture, now_ns):
    return {"arrays": {name: fixture[name] for name in ("rgb", "depth", "K", "T")},
            "identity": fixture["identity"], "capture": fixture["capture"],
            "anchor": fixture["anchor"],
            "pose_stamp_ns": fixture["identity"]["acquired_ns"],
            "pose_valid": True, "now_ns": now_ns}


def semantic_fields(fixture, frame_token, now_ns):
    return {"arrays": {"mask": fixture["mask"]},
            "identity": fixture["identity"], "frame_token": frame_token,
            "object_id": fixture["object_id"], "detection": fixture["detection"],
            "now_ns": now_ns}


def run_client(args):
    import rclpy
    result = args.result.resolve()
    result.mkdir(parents=True, exist_ok=False)
    state = {"ok": False, "scenario": args.scenario, "requests": [], "acks": []}
    rclpy.init(args=[])
    node = rclpy.create_node("fpga_m7m4_probe_client_" + args.name)
    client = RosProbeClient(node, result, args.reliability)
    try:
        require_match = args.scenario != "qos-negative"
        client.wait_graph(require_match=require_match,
                          expected_peer_node="fpga_m7m4_probe_server_" + args.name)
        if args.scenario == "qos-negative":
            for index in range(1, 4):
                client.send(f"qos-negative-{index}", "qos-negative", 1, "query",
                            expect_ack=False, timeout=0)
            negative_started_ns = time.monotonic_ns()
            client.settle_without_ack(QOS_NEGATIVE_SECONDS)
            state["negative_window"] = {
                "started_steady_ns": negative_started_ns,
                "ended_steady_ns": time.monotonic_ns(),
                "required_ns": int(QOS_NEGATIVE_SECONDS * 1_000_000_000),
            }
        elif args.scenario == "qos-positive":
            bad_id = client.send("../escape", "invalid-id", 1, "query")
            bad_schema = client.send("strict-extra", "strict-extra", 1, "query",
                                     extra_fields={"unexpected": "rejected"})
            positive = client.send("qos-positive", "qos-positive", 1, "query")
            shutdown = client.send("qos-shutdown", "qos-shutdown", 1, "shutdown")
            if (bad_id.get("ok") or bad_schema.get("ok") or not positive.get("ok") or
                    not shutdown.get("ok")):
                raise AssertionError("strict controls or reliable positive had wrong ACK")
            state["strict_control_responses"] = [bad_id, bad_schema]
        else:
            fixtures = args.fixtures.resolve()
            first, second = load_fixture(fixtures, "frame-1"), load_fixture(fixtures, "frame-2")
            g1 = client.send("geometry-1", "geometry-1", args.attempt, "geometry",
                             **geometry_fields(first, 1_001_000_000))
            g2 = client.send("geometry-2", "geometry-2", args.attempt, "geometry",
                             **geometry_fields(second, 1_021_000_000))
            if not g1.get("ok") or not g2.get("ok"):
                raise AssertionError("geometry request failed")
            token1 = g1["result"]["frame_token"]
            token2 = g2["result"]["frame_token"]
            state["frame_tokens"] = [token1, token2]
            if args.scenario == "phase1":
                s1 = client.send("semantic-1", "persist-frame-1", 1, "semantic",
                                 **semantic_fields(first, token1, 1_022_000_000))
                if not s1.get("ok") or not s1["result"]["imported"]:
                    raise AssertionError("late first semantic did not persist")
                client.send("semantic-2", "persist-frame-2", 1, "semantic",
                            expect_ack=False,
                            **semantic_fields(second, token2, 1_023_000_000))
                state["lost_ack_observed"] = True
            elif args.scenario == "phase2":
                retry = client.send("semantic-2", "persist-frame-2", 2, "semantic",
                                    **semantic_fields(second, token2, 1_023_000_000))
                batches = retry.get("result", {}).get("import_result", {}).get("batches", [])
                if not retry.get("ok") or not batches or any(
                        item.get("status") != "duplicate" for item in batches):
                    raise AssertionError("restart retry was not an M4 duplicate")
                state["retry_response"] = retry
                state["query_response"] = client.send(
                    "final-query", "final-query", 1, "query")
                client.send("main-shutdown", "main-shutdown", 1, "shutdown")
            else:
                raise ValueError("unknown client scenario")
        state["ok"] = True
    except BaseException as exc:
        state["error"] = {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        state.update({"environment": role_environment(),
                      "request_reliability": args.reliability,
                      "request_incompatible_callback_registered": True,
                      "request_incompatible_events": client.request_incompatible_events,
                      "request_graph": client.request_graph.result(),
                      "ack_graph": client.ack_graph.result(),
                      "requests": client.requests, "acks": client.acks})
        client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        state["explicit_close"] = True
        write_json(result / "status.json", state)


def wait_ready(process, ready, seconds=10):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if ready.is_file():
            return json.loads(ready.read_text(encoding="utf8"))
        if process.poll() is not None:
            raise RuntimeError("server exited before readiness")
        time.sleep(0.05)
    raise TimeoutError("server readiness timeout")


def launch_role(out, label, argv):
    stdout = (out / f"{label}.stdout.log").open("xb")
    stderr = (out / f"{label}.stderr.log").open("xb")
    process = subprocess.Popen([sys.executable, "-B", str(Path(__file__).resolve()), *argv],
                               stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr,
                               env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    return {"label": label, "process": process, "stdout": stdout, "stderr": stderr,
            "closed": False,
            "argv": [sys.executable, "-B", str(Path(__file__).resolve()), *argv]}


def finish_role(role, timeout=15):
    process = role["process"]
    try:
        code = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.send_signal(signal.SIGINT)
        try:
            code = process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            code = process.wait(timeout=3)
        raise TimeoutError(role["label"] + " exceeded process deadline")
    finally:
        if not role["closed"]:
            role["stdout"].close()
            role["stderr"].close()
            role["closed"] = True
    if code != 0:
        raise RuntimeError(f"{role['label']} exited {code}")
    return {"label": role["label"], "pid": process.pid, "exit_code": code,
            "argv": role["argv"]}


def cleanup_role(role):
    """Stop only the exact child PID launched by this runner and reap it."""
    if role is None:
        return None
    process = role["process"]
    action = "already_exited"
    if process.poll() is None:
        action = "sigint"
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            action = "sigkill"
            process.kill()
            process.wait(timeout=2)
    if not role["closed"]:
        role["stdout"].close()
        role["stderr"].close()
        role["closed"] = True
    return {"label": role["label"], "pid": process.pid,
            "exit_code": process.returncode, "cleanup": action}


def run_pair(out, label, *, server_reliability, client_reliability, scenario,
             store, library, fixtures=None, attempt=1, drop_ack_on=None,
             server_seconds=30.0):
    server_dir = out / label / "server"
    client_dir = out / label / "client"
    server = client = None
    completed = False
    try:
        server = launch_role(out, label + "-server", [
            "server", "--name", label.replace("-", "_"), "--result", str(server_dir),
            "--store", str(store), "--library", str(library),
            "--reliability", server_reliability, "--max-seconds", str(server_seconds),
            *(["--drop-ack-on", drop_ack_on] if drop_ack_on else []),
        ])
        wait_ready(server["process"], server_dir / "ready.json")
        client_argv = [
            "client", "--name", label.replace("-", "_"), "--result", str(client_dir),
            "--scenario", scenario, "--reliability", client_reliability,
            "--attempt", str(attempt),
        ]
        if fixtures is not None:
            client_argv += ["--fixtures", str(fixtures)]
        client = launch_role(out, label + "-client", client_argv)
        client_receipt = finish_role(client, timeout=20)
        server_receipt = finish_role(server, timeout=max(8, int(server_seconds) + 4))
        server_status = json.loads((server_dir / "status.json").read_text())
        client_status = json.loads((client_dir / "status.json").read_text())
        if not all(status.get("ok") and status.get("explicit_close")
                   for status in (server_status, client_status)):
            raise RuntimeError(label + " did not close both ROS roles cleanly")
        completed = True
        return {"server": server_receipt, "client": client_receipt,
                "server_status": server_status, "client_status": client_status}
    finally:
        if not completed:
            cleanup = {"client": cleanup_role(client), "server": cleanup_role(server)}
            cleanup_path = out / f"{label}.cleanup.json"
            if not cleanup_path.exists():
                write_json(cleanup_path, cleanup)


def object_summary(query):
    return sorted((row["id"], row["label"], row["physical_observation_count"],
                   row["state"], row["position_m"]) for row in query["objects"])


def recursive_manifest(out):
    rows = []
    for path in sorted(out.rglob("*")):
        if path.is_file() and path != out / "manifest.json":
            rows.append({"path": str(path.relative_to(out)), "bytes": path.stat().st_size,
                         "sha256": file_sha(path)})
    write_json(out / "manifest.json", {"files": rows})


def graph_reliabilities(graph):
    return {row.get("reliability") for observation in graph["observations"]
            for row in observation["endpoints"] if "inspection_error" not in row}


def graph_node_names(graph):
    return {row.get("node_name") for observation in graph["observations"]
            for row in observation["endpoints"] if "inspection_error" not in row}


def check_incompatible_event(status, role):
    if status.get("request_incompatible_callback_registered") is not True:
        raise AssertionError(role + " did not register explicit incompatible-QoS callback")
    events = status.get("request_incompatible_events", [])
    if not any(row.get("policy") == "RELIABILITY" and row.get("total_count", 0) >= 1 and
               row.get("total_count_change", 0) >= 1 for row in events):
        raise AssertionError(role + " did not record the expected RELIABILITY incompatibility")


def check_graph(status, *, compatible, role, local_role, expected_peer_reliability,
                expected_peer_node):
    if status.get("request_incompatible_callback_registered") is not True:
        raise AssertionError(role + " did not register explicit incompatible-QoS callback")
    if compatible and status.get("request_incompatible_events"):
        raise AssertionError(role + " unexpectedly reported incompatible QoS")
    graph = status["request_graph"]
    if graph["discovered_max"] < 1:
        raise AssertionError(role + " did not discover request peer endpoint")
    if local_role == "client":
        if graph["match_count_supported"] is not True:
            raise AssertionError(role + " publisher lacks required real match count")
        if compatible and (graph["matched_max"] is None or graph["matched_max"] < 1):
            raise AssertionError(role + " did not match compatible request peer")
        if not compatible and graph["matched_max"] != 0:
            raise AssertionError(role + " unexpectedly matched incompatible request peer")
    elif local_role == "server":
        if graph["match_count_supported"] is not False or graph["matched_max"] is not None:
            raise AssertionError(role + " did not preserve unsupported subscription match count")
    else:
        raise ValueError("local_role must be server or client")
    if expected_peer_reliability not in graph_reliabilities(graph):
        raise AssertionError(role + " did not record the expected peer QoS reliability")
    if expected_peer_node not in graph_node_names(graph):
        raise AssertionError(role + " only observed a stale or unrelated request endpoint")
    ack_graph = status["ack_graph"]
    if ack_graph["discovered_max"] < 1:
        raise AssertionError(role + " did not discover the reverse ACK endpoint")
    if local_role == "server":
        if (ack_graph["match_count_supported"] is not True or
                ack_graph["matched_max"] is None or ack_graph["matched_max"] < 1):
            raise AssertionError(role + " ACK publisher did not match the client subscriber")
    elif (ack_graph["match_count_supported"] is not False or
          ack_graph["matched_max"] is not None):
        raise AssertionError(role + " did not preserve unsupported ACK subscription match count")
    if "RELIABLE" not in graph_reliabilities(ack_graph):
        raise AssertionError(role + " did not record reliable reverse ACK QoS")
    if expected_peer_node not in graph_node_names(ack_graph):
        raise AssertionError(role + " only observed a stale or unrelated ACK endpoint")


def run_probe(out, source_commit, source_tags):
    from perception.offline_memory import OfflineMemory

    out = Path(out).resolve()
    if out.exists() and any(out.iterdir()):
        raise FileExistsError("formal output directory must be empty")
    out.mkdir(parents=True, exist_ok=True)
    if not str(out).startswith("/rosresults"):
        raise ValueError("formal output must use the unique /rosresults mount")
    if not FROZEN_LIBRARY.is_file() or not FROZEN_MANIFEST.is_file():
        raise FileNotFoundError("accepted Unix probe native library is missing")
    if re.fullmatch(r"[0-9a-f]{40}", source_commit) is None:
        raise ValueError("host-frozen source commit must be a full lowercase Git object ID")
    if not source_tags or not all(ID_PATTERN.fullmatch(tag) for tag in source_tags):
        raise ValueError("bounded source tags must be supplied by the host preflight")
    if not any("ros2-transport" in tag and "prerun" in tag for tag in source_tags):
        raise RuntimeError("ROS2 transport pre-run tag required at HEAD")
    manifest = json.loads(FROZEN_MANIFEST.read_text(encoding="utf8"))
    library_rows = [row for row in manifest.get("files", [])
                    if row.get("path") == "native-m7.so"]
    if (len(library_rows) != 1 or library_rows[0].get("sha256") != FROZEN_LIBRARY_SHA256 or
            file_sha(FROZEN_LIBRARY) != FROZEN_LIBRARY_SHA256):
        raise RuntimeError("accepted Unix probe library no longer matches its frozen manifest")
    for relative, expected in FROZEN_CPP_SHA256.items():
        if file_sha(ROOT / relative) != expected:
            raise RuntimeError("native M7 source differs from the source that produced the frozen library")

    library = out / "native-m7.so"
    write_new(library, FROZEN_LIBRARY.read_bytes())
    fixtures = out / "fixtures"
    fixtures.mkdir()
    first = frame_fixture(fixtures, "frame-1", instance="ros-probe-stream",
                          frame_id=1, acquired_ns=1_000_000_000, object_id="cup-1")
    second = frame_fixture(fixtures, "frame-2", instance="ros-probe-stream",
                           frame_id=2, acquired_ns=1_020_000_000,
                           depth_value=3.0, object_id="bottle-1")
    fixture_json = {}
    for label, fixture in (("frame-1", first), ("frame-2", second)):
        fixture_json[label] = {key: value for key, value in fixture.items()
                               if key not in ("rgb", "depth", "K", "T", "mask")}
    write_json(fixtures / "fixtures.json", fixture_json)

    trace = {"ok": False, "source_commit": source_commit, "source_tags": source_tags,
             "runner_pid": os.getpid(), "environment": role_environment(),
             "runner_command": sys.argv,
             "expected_image": EXPECTED_IMAGE, "expected_image_id": EXPECTED_IMAGE_ID,
             "source_sha256": {name: file_sha(ROOT / name) for name in (
                 "docs/design/CONTINUOUS_MEMORY_ROS2_TRANSPORT_20260921.md",
                 "tests/test_m7_memory_ros2_probe.py",
                 "tools/m7_memory_ros2_probe.py",
                 "tools/m7_memory_transport_probe.py",
                 "host/m7_memory.py", "host/m7_pipeline.py", "host/m7_native.py",
                 "perception/offline_memory.py")},
             "frozen_library_source": str(FROZEN_LIBRARY),
             "frozen_manifest_sha256": file_sha(FROZEN_MANIFEST),
             "native_source_sha256": {name: file_sha(ROOT / name)
                                      for name in FROZEN_CPP_SHA256},
             "frozen_library_sha256": file_sha(FROZEN_LIBRARY),
             "copied_library_sha256": file_sha(library), "pairs": {}}
    try:
        negative = run_pair(out, "qos-negative", server_reliability="reliable",
                            client_reliability="best_effort", scenario="qos-negative",
                            store=out / "qos-negative-memory.json", library=library,
                            server_seconds=10.0)
        trace["pairs"]["qos_negative"] = negative
        check_graph(negative["server_status"], compatible=False, role="negative server",
                    local_role="server",
                    expected_peer_reliability="BEST_EFFORT",
                    expected_peer_node="fpga_m7m4_probe_client_qos_negative")
        check_graph(negative["client_status"], compatible=False, role="negative client",
                    local_role="client",
                    expected_peer_reliability="RELIABLE",
                    expected_peer_node="fpga_m7m4_probe_server_qos_negative")
        negative_window = negative["client_status"].get("negative_window", {})
        check_incompatible_event(negative["server_status"], "negative server")
        check_incompatible_event(negative["client_status"], "negative client")
        if (negative["server_status"]["callbacks"] != 0 or
                len(negative["client_status"]["requests"]) != 3 or
                negative["client_status"]["acks"] or
                (out / "qos-negative-memory.json").exists() or
                negative_window.get("ended_steady_ns", 0) -
                negative_window.get("started_steady_ns", 0) <
                negative_window.get("required_ns", 1)):
            raise AssertionError("QoS negative did not remain a discovered zero-callback control")

        positive = run_pair(out, "qos-positive", server_reliability="reliable",
                            client_reliability="reliable", scenario="qos-positive",
                            store=out / "qos-positive-memory.json", library=library)
        trace["pairs"]["qos_positive"] = positive
        check_graph(positive["server_status"], compatible=True, role="positive server",
                    local_role="server",
                    expected_peer_reliability="RELIABLE",
                    expected_peer_node="fpga_m7m4_probe_client_qos_positive")
        check_graph(positive["client_status"], compatible=True, role="positive client",
                    local_role="client",
                    expected_peer_reliability="RELIABLE",
                    expected_peer_node="fpga_m7m4_probe_server_qos_positive")
        if positive["server_status"]["callbacks"] != 4:
            raise AssertionError("reliable positive/control callback count differs")
        positive_events = [json.loads(line) for line in
                           (out / "qos-positive/server/events.jsonl").read_text().splitlines()]
        rejected = [row for row in positive_events if row["operation_id"] in
                    ("invalid-id", "strict-extra")]
        if (len(rejected) != 2 or any(row["service_called"] for row in rejected) or
                any(row["store_before"] != row["store_after"] for row in rejected) or
                (out / "escape.m4-input.json").exists()):
            raise AssertionError("strict wrapper controls reached ProbeService or changed files")

        store = out / "memory.json"
        phase1 = run_pair(out, "main-phase1", server_reliability="reliable",
                          client_reliability="reliable", scenario="phase1", store=store,
                          library=library, fixtures=fixtures, attempt=1,
                          drop_ack_on="persist-frame-2")
        trace["pairs"]["main_phase1"] = phase1
        check_graph(phase1["server_status"], compatible=True, role="phase1 server",
                    local_role="server",
                    expected_peer_reliability="RELIABLE",
                    expected_peer_node="fpga_m7m4_probe_client_main_phase1")
        check_graph(phase1["client_status"], compatible=True, role="phase1 client",
                    local_role="client",
                    expected_peer_reliability="RELIABLE",
                    expected_peer_node="fpga_m7m4_probe_server_main_phase1")
        drop_event = phase1["server_status"].get("drop_event", {})
        if (not phase1["server_status"]["ack_dropped"] or
                not phase1["server_status"]["explicit_close"] or
                not phase1["client_status"].get("lost_ack_observed") or
                not drop_event.get("service_called") or
                drop_event.get("store_after") is None or
                drop_event.get("store_before") == drop_event.get("store_after")):
            raise AssertionError("save-then-drop-ACK phase was not explicitly observed")
        after_drop_sha = file_sha(store)
        if after_drop_sha is None:
            raise AssertionError("ACK was dropped before persistent save")
        write_new(out / "memory-after-lost-ack.json", store.read_bytes())
        after_drop_query = OfflineMemory.load(store).query()
        after_drop_objects = object_summary(after_drop_query)
        if len(after_drop_objects) != 2:
            raise AssertionError("two source acquisitions did not persist before restart")
        known = {row[1]: row for row in after_drop_objects}
        for label, position in (("cup", [0., 0., 2.]), ("bottle", [0., 0., 3.])):
            row = known.get(label)
            if row is None or row[2] != 1 or not np.allclose(row[4], position,
                                                             atol=1e-9, rtol=0):
                raise AssertionError("persistent object differs from independent known geometry")

        phase2 = run_pair(out, "main-phase2", server_reliability="reliable",
                          client_reliability="reliable", scenario="phase2", store=store,
                          library=library, fixtures=fixtures, attempt=2)
        trace["pairs"]["main_phase2"] = phase2
        check_graph(phase2["server_status"], compatible=True, role="phase2 server",
                    local_role="server",
                    expected_peer_reliability="RELIABLE",
                    expected_peer_node="fpga_m7m4_probe_client_main_phase2")
        check_graph(phase2["client_status"], compatible=True, role="phase2 client",
                    local_role="client",
                    expected_peer_reliability="RELIABLE",
                    expected_peer_node="fpga_m7m4_probe_server_main_phase2")
        final_query = OfflineMemory.load(store).query()
        final_objects = object_summary(final_query)
        if file_sha(store) != after_drop_sha or final_objects != after_drop_objects:
            raise AssertionError("restart retry changed persistent evidence")
        old_token = phase1["client_status"]["frame_tokens"][1]
        new_token = phase2["client_status"]["frame_tokens"][1]
        if old_token == new_token:
            raise AssertionError("new M7 process reused old frame token")
        if phase1["server"]["pid"] == phase2["server"]["pid"]:
            raise AssertionError("server restart did not use a new PID")

        trace.update({"store_sha256_after_lost_ack": after_drop_sha,
                      "final_store_sha256": file_sha(store),
                      "objects_after_lost_ack": after_drop_objects,
                      "final_objects": final_objects,
                      "server_pids": [phase1["server"]["pid"], phase2["server"]["pid"]],
                      "frame_tokens_changed_after_restart": old_token != new_token,
                      "ok": True})
        write_json(out / "run-info.json", {key: trace[key] for key in
            ("source_commit", "source_tags", "runner_pid", "environment",
             "runner_command", "expected_image", "expected_image_id", "source_sha256",
             "frozen_library_source", "frozen_manifest_sha256", "native_source_sha256",
             "frozen_library_sha256", "copied_library_sha256")})
        write_json(out / "summary.json", trace)
    except BaseException as exc:
        trace["error"] = {"type": type(exc).__name__, "message": str(exc)}
        write_json(out / "failure.json", trace)
        raise
    finally:
        recursive_manifest(out)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    server = sub.add_parser("server")
    server.add_argument("--name", required=True)
    server.add_argument("--result", type=Path, required=True)
    server.add_argument("--store", type=Path, required=True)
    server.add_argument("--library", type=Path, required=True)
    server.add_argument("--reliability", choices=("reliable", "best_effort"), required=True)
    server.add_argument("--max-seconds", type=float, default=30.0)
    server.add_argument("--drop-ack-on")
    client = sub.add_parser("client")
    client.add_argument("--name", required=True)
    client.add_argument("--result", type=Path, required=True)
    client.add_argument("--scenario", choices=("qos-negative", "qos-positive",
                                                "phase1", "phase2"), required=True)
    client.add_argument("--reliability", choices=("reliable", "best_effort"), required=True)
    client.add_argument("--fixtures", type=Path)
    client.add_argument("--attempt", type=int, default=1)
    run = sub.add_parser("run")
    run.add_argument("--out", type=Path, required=True)
    run.add_argument("--source-commit", required=True)
    run.add_argument("--source-tag", action="append", required=True)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.mode == "server":
        if args.max_seconds <= 0 or (args.drop_ack_on is not None and
                                     ID_PATTERN.fullmatch(args.drop_ack_on) is None):
            raise ValueError("invalid bounded server arguments")
        run_server(args)
    elif args.mode == "client":
        if args.scenario in ("phase1", "phase2") and args.fixtures is None:
            raise ValueError("main phases require frozen fixtures")
        run_client(args)
    else:
        run_probe(args.out, args.source_commit, args.source_tag)


if __name__ == "__main__":
    main()
