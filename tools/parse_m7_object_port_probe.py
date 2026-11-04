"""Strict parser for the two-arm M7 object xyz-port HLS diagnostic."""
import argparse
import json
from pathlib import Path
import re


TRANSACTIONS = (
    "present_25_masked", "removed_roi", "occluded_roi", "missing_mask",
    "bad_mask", "oversized_width", "too_small_roi", "loose_margin",
    "dense_24x24_576_points", "sparse_24x24_72_points")
LOOP_TOKEN = "VITIS_LOOP_78_4_VITIS_LOOP_79_5"
# The C++ testbench prints its own transaction index and point count for the
# two calibrated cases. TRANSACTIONS attaches a semantic name to each of the
# ten cycle rows purely by position, so an inserted or reordered top-level
# call would silently re-label all ten numbers. Cross-check the printed index
# against the position instead of only checking that the string exists.
MARKED_TRANSACTIONS = (("dense_24x24_576_points", "dense", 576),
                       ("sparse_24x24_72_points", "sparse", 72))


def _marked_transaction_indexes(text, source):
    """Reject any printed index or point count that contradicts TRANSACTIONS.

    The RTL log repeats these lines, so every occurrence must agree; a single
    disagreement means the positional name-to-cycle mapping is unsafe and the
    parse is refused rather than quietly relabelled.
    """
    indexes = {}
    for name, kind, points in MARKED_TRANSACTIONS:
        expected = TRANSACTIONS.index(name)
        matches = re.findall(
            r"M7 object port %s transaction (\d+) points=(\d+) PASS" % kind, text)
        if not matches:
            raise ValueError("%s: %s transaction PASS marker missing" % (source, kind))
        for index, counted in matches:
            if int(index) != expected:
                raise ValueError(
                    "%s: %s marker reports transaction %s but %s is transaction %d; "
                    "the top-level call order changed, so every transaction label "
                    "is unsafe" % (source, kind, index, name, expected))
            if int(counted) != points:
                raise ValueError(
                    "%s: %s marker reports points=%s, the fixed case has %d"
                    % (source, kind, counted, points))
        indexes[name] = expected
    return indexes


def _read(path):
    path = Path(path)
    if not path.is_file():
        raise ValueError("missing probe artifact: %s" % path)
    return path.read_text(errors="replace")


def _metadata(solution, expected_arm):
    path = Path(solution).parents[1] / "PROBE_METADATA.txt"
    values = {}
    source_hashes = {}
    for line in _read(path).splitlines():
        source_match = re.fullmatch(r"source_([0-9a-f]{64})\s+(.+)", line)
        if source_match:
            source_path = source_match.group(2)
            if source_path in source_hashes:
                raise ValueError("duplicate source hash path: %s" % source_path)
            source_hashes[source_path] = source_match.group(1)
        elif "=" in line:
            key, value = line.split("=", 1)
            if key in values:
                raise ValueError("duplicate metadata key: %s" % key)
            values[key] = value
    required = {"probe", "arm", "source_commit", "part", "clock_ns", "directive"}
    if set(values) != required or values["probe"] != "object_xyz_port_counterfactual":
        raise ValueError("invalid or incomplete probe metadata")
    if values["arm"] != expected_arm or not re.fullmatch(r"[0-9a-f]{40}", values["source_commit"]):
        raise ValueError("probe arm or source commit mismatch")
    expected_directive = ("none_original_flat_xyz_local" if expected_arm == "A" else
                          "array_partition_cyclic_factor3_dim1_xyz_local")
    if values["directive"] != expected_directive:
        raise ValueError("probe directive does not match arm")
    if len(source_hashes) != 4:
        raise ValueError("expected four source hash mappings")
    values["source_hashes"] = dict(sorted(source_hashes.items()))
    return values


def _transactions(solution):
    path = Path(solution) / "sim/report/verilog/result.transaction.rpt"
    rows = {}
    pattern = re.compile(r"^transaction\s+(\d+):\s+(\d+)\s+(\d+|x)\s*$")
    for line in _read(path).splitlines():
        match = pattern.match(line.strip())
        if not match:
            continue
        index = int(match.group(1))
        if index in rows:
            raise ValueError("duplicate transaction %d" % index)
        rows[index] = {"latency_cycles": int(match.group(2)),
                       "interval_cycles": None if match.group(3) == "x" else int(match.group(3))}
    if set(rows) != set(range(len(TRANSACTIONS))):
        raise ValueError("expected exactly ten transactions numbered 0..9")
    return {name: rows[index] for index, name in enumerate(TRANSACTIONS)}


def _main_loop(solution):
    reports = list((Path(solution) / "syn/report").glob(
        "fpga_m7_object_evidence_Pipeline_*%s*_csynth.rpt" % LOOP_TOKEN))
    if len(reports) != 1:
        raise ValueError("expected one main pixel-loop synthesis report")
    for line in _read(reports[0]).splitlines():
        if LOOP_TOKEN not in line or "|" not in line:
            continue
        fields = [field.strip() for field in line.split("|")]
        if len(fields) >= 9 and fields[5].isdigit() and fields[6].isdigit():
            return {"achieved_ii": int(fields[5]), "target_ii": int(fields[6]),
                    "report": str(reports[0])}
    raise ValueError("main pixel-loop target/achieved II missing")


def _top_report(solution):
    path = Path(solution) / "syn/report/fpga_m7_object_evidence_csynth.rpt"
    text = _read(path)
    resource_match = re.search(
        r"\|Total\s*\|\s*(\d+)\|\s*(\d+)\|\s*(\d+)\|\s*(\d+)\|\s*(\d+)\|", text)
    if not resource_match:
        raise ValueError("top resource total missing")
    resources = dict(zip(("bram_18k", "dsp", "ff", "lut", "uram"),
                         map(int, resource_match.groups())))
    interface = text.split("== Interface", 1)
    if len(interface) != 2:
        raise ValueError("RTL interface section missing")
    xyz_lines = []
    bank_objects = set()
    write_data_ports = []
    for line in interface[1].splitlines():
        if "xyz_local" not in line or "|" not in line:
            continue
        fields = [field.strip() for field in line.split("|")]
        if len(fields) < 7 or not fields[1].startswith("xyz_local"):
            continue
        xyz_lines.append(line.strip())
        if fields[5].startswith("xyz_local"):
            bank_objects.add(fields[5])
        if re.search(r"_d\d+$", fields[1]):
            write_data_ports.append(fields[1])
    if not xyz_lines or not bank_objects or not write_data_ports:
        raise ValueError("xyz_local physical interface missing")
    return {"resources": resources, "xyz_bank_objects": sorted(bank_objects),
            "xyz_bank_count": len(bank_objects),
            "xyz_write_data_ports": sorted(write_data_ports),
            "xyz_interface_lines": xyz_lines, "report": str(path)}


def _cosim_summary(text):
    for line in text.splitlines():
        fields = [field.strip() for field in line.split("|")]
        if len(fields) >= 11 and fields[1] == "Verilog" and fields[2] == "Pass":
            numbers = fields[3:10]
            if not all(value.isdigit() for value in numbers):
                break
            keys = ("latency_min", "latency_avg", "latency_max", "interval_min",
                    "interval_avg", "interval_max", "total_execution_cycles")
            return dict(zip(keys, map(int, numbers)))
    raise ValueError("complete Verilog co-simulation cycle summary missing")


def parse_arm(solution, expected_arm):
    solution = Path(solution).resolve()
    metadata = _metadata(solution, expected_arm)
    csim = _read(solution / "csim/report/fpga_m7_object_evidence_csim.log")
    rtl = _read(solution / "sim/report/verilog/fpga_m7_object_evidence.log")
    cosim = _read(solution / "sim/report/fpga_m7_object_evidence_cosim.rpt")
    marker = ("M7 object port probe C++ testbench PASS transactions=%d"
              % len(TRANSACTIONS))
    if "CSim done with 0 errors" not in csim or marker not in csim:
        raise ValueError("C simulation PASS markers missing")
    if ("C/RTL co-simulation finished: PASS" not in rtl or marker not in rtl or
            not re.search(r"\|\s*Verilog\|\s*Pass\|", cosim)):
        raise ValueError("RTL co-simulation PASS markers missing")
    _marked_transaction_indexes(csim, "C simulation log")
    _marked_transaction_indexes(rtl, "RTL co-simulation log")
    top = _top_report(solution)
    return {"metadata": metadata, "main_pixel_loop": _main_loop(solution),
            "transactions": _transactions(solution),
            "cosim_cycles": _cosim_summary(cosim), **top}


def compare(arm_a, arm_b):
    a = parse_arm(arm_a, "A")
    b = parse_arm(arm_b, "B")
    if a["metadata"]["source_commit"] != b["metadata"]["source_commit"]:
        raise ValueError("arms use different source commits")
    if a["metadata"]["source_hashes"] != b["metadata"]["source_hashes"]:
        raise ValueError("arms use different source files")
    if (a["metadata"]["part"] != b["metadata"]["part"] or
            a["metadata"]["clock_ns"] != b["metadata"]["clock_ns"]):
        raise ValueError("arms use different device or clock target")
    latency_delta = {}
    for name in TRANSACTIONS:
        latency_delta[name] = (b["transactions"][name]["latency_cycles"] -
                               a["transactions"][name]["latency_cycles"])
    increased = b["xyz_bank_count"] > a["xyz_bank_count"]
    supports = (a["main_pixel_loop"]["achieved_ii"] == 2 and
                b["main_pixel_loop"]["achieved_ii"] == 1 and increased)
    return {"source_commit": a["metadata"]["source_commit"], "arm_a": a, "arm_b": b,
            "comparison": {"partition_increased_physical_banks": increased,
                           "supports_two_port_bottleneck": supports,
                           "latency_delta_b_minus_a_cycles": latency_delta}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-a", required=True, type=Path)
    parser.add_argument("--arm-b", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = compare(args.arm_a, args.arm_b)
    if args.output.exists() or not args.output.parent.is_dir():
        raise ValueError("output must be a new file in an existing directory")
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"result": "PASS", "output": str(args.output),
                      "supports_two_port_bottleneck":
                          result["comparison"]["supports_two_port_bottleneck"]}))


if __name__ == "__main__":
    main()
