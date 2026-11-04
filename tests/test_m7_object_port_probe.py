"""Calibration of the strict two-arm object-port report parser."""
from pathlib import Path
import tempfile
import unittest

from tools.parse_m7_object_port_probe import TRANSACTIONS, compare, parse_arm


MARKER = "M7 object port probe C++ testbench PASS transactions=10"
DENSE = "M7 object port dense transaction 8 points=576 PASS"
SPARSE = "M7 object port sparse transaction 9 points=72 PASS"
COMMIT = "1" * 40


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def make_arm(root, arm, ii, cycles, banks, *, pass_rtl=True, transaction_count=10,
             dense=DENSE, sparse=SPARSE):
    run = Path(root) / ("arm-" + arm.lower())
    solution = run / "hls/solution"
    metadata = ["probe=object_xyz_port_counterfactual", "arm=" + arm,
                "source_commit=" + COMMIT, "part=xck26-sfvc784-2LV-c",
                "clock_ns=10", "directive=" +
                ("none_original_flat_xyz_local" if arm == "A" else
                 "array_partition_cyclic_factor3_dim1_xyz_local")]
    metadata += ["source_%064d  /source/%d" % (i, i) for i in range(4)]
    write(run / "PROBE_METADATA.txt", "\n".join(metadata) + "\n")
    write(solution / "csim/report/fpga_m7_object_evidence_csim.log",
          "CSim done with 0 errors\n%s\n%s\n%s\n" % (dense, sparse, MARKER))
    rtl_status = "C/RTL co-simulation finished: PASS" if pass_rtl else "cosim failed"
    # The real Vitis RTL log prints the two markers twice; repeat them here so
    # the fixture matches the artifact the parser is calibrated against.
    write(solution / "sim/report/verilog/fpga_m7_object_evidence.log",
          "%s\n%s\n%s\n%s\n%s\n%s\n" % (dense, sparse, MARKER,
                                            dense, sparse, rtl_status))
    write(solution / "sim/report/fpga_m7_object_evidence_cosim.rpt",
          "| Verilog| %s| 1| 10| 20| 2| 11| 21| 123|\n" %
          ("Pass" if pass_rtl else "Fail"))
    transaction_lines = ["latency interval"]
    for index in range(transaction_count):
        interval = "x" if index == transaction_count - 1 else str(cycles[index] + 1)
        transaction_lines.append("transaction %7d: %15d %15s" %
                                 (index, cycles[index], interval))
    write(solution / "sim/report/verilog/result.transaction.rpt",
          "\n".join(transaction_lines) + "\n")
    loop = ("|- VITIS_LOOP_78_4_VITIS_LOOP_79_5  | ? | ? | 53 | %d | 1 | ? | yes |\n" % ii)
    write(solution / ("syn/report/fpga_m7_object_evidence_Pipeline_" +
          "VITIS_LOOP_78_4_VITIS_LOOP_79_5_csynth.rpt"), loop)
    interface = ["== Utilization Estimates", "|Total | 0| 132| 13649| 13967| 0|",
                 "== Interface"]
    for bank in range(banks):
        suffix = "" if banks == 1 else "_" + str(bank)
        obj = "xyz_local" + suffix
        interface += ["|%s_address0 | out | 20 | ap_memory | %s | array|" % (obj, obj),
                      "|%s_d0 | out | 64 | ap_memory | %s | array|" % (obj, obj),
                      "|%s_address1 | out | 20 | ap_memory | %s | array|" % (obj, obj),
                      "|%s_d1 | out | 64 | ap_memory | %s | array|" % (obj, obj)]
    write(solution / "syn/report/fpga_m7_object_evidence_csynth.rpt",
          "\n".join(interface) + "\n")
    return solution


class ObjectPortParserTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="m7-object-port-parser-")
        self.addCleanup(self.tmp.cleanup)
        self.a_cycles = [100 + i for i in range(len(TRANSACTIONS))]
        self.b_cycles = [90 + i for i in range(len(TRANSACTIONS))]

    def arms(self, **kwargs):
        a = make_arm(self.tmp.name, "A", 2, self.a_cycles, 1)
        b = make_arm(self.tmp.name, "B", 1, self.b_cycles, 3, **kwargs)
        return a, b

    def test_known_two_port_and_three_bank_answers(self):
        result = compare(*self.arms())
        self.assertEqual(result["arm_a"]["main_pixel_loop"]["achieved_ii"], 2)
        self.assertEqual(result["arm_b"]["main_pixel_loop"]["achieved_ii"], 1)
        self.assertEqual(result["arm_a"]["xyz_bank_count"], 1)
        self.assertEqual(result["arm_b"]["xyz_bank_count"], 3)
        self.assertEqual(result["arm_a"]["cosim_cycles"]["total_execution_cycles"], 123)
        self.assertTrue(result["comparison"]["supports_two_port_bottleneck"])
        self.assertEqual(result["arm_a"]["transactions"]["dense_24x24_576_points"]
                         ["latency_cycles"], self.a_cycles[8])

    def test_missing_cosim_pass_is_rejected(self):
        a, b = self.arms(pass_rtl=False)
        with self.assertRaisesRegex(ValueError, "co-simulation PASS"):
            compare(a, b)

    def test_missing_transaction_is_rejected(self):
        a = make_arm(self.tmp.name, "A", 2, self.a_cycles, 1, transaction_count=9)
        with self.assertRaisesRegex(ValueError, "exactly ten"):
            parse_arm(a, "A")

    def test_cycle_change_changes_parsed_comparison(self):
        a, b = self.arms()
        first = compare(a, b)
        before = first["comparison"]["latency_delta_b_minus_a_cycles"][
            "dense_24x24_576_points"]
        transaction = b / "sim/report/verilog/result.transaction.rpt"
        text = transaction.read_text()
        text = text.replace("transaction       8:              98              99",
                            "transaction       8:             321             322")
        transaction.write_text(text)
        second = compare(a, b)
        after = second["comparison"]["latency_delta_b_minus_a_cycles"][
            "dense_24x24_576_points"]
        self.assertNotEqual(before, after)
        self.assertEqual(after, 321 - self.a_cycles[8])

    def test_transaction_index_in_marker_must_match_its_position(self):
        # The ten cycle rows are named purely by position. If a top-level call
        # were inserted or reordered, the testbench would print a different
        # index while every row silently kept its old name.
        moved = "M7 object port dense transaction 7 points=576 PASS"
        a = make_arm(self.tmp.name, "A", 2, self.a_cycles, 1, dense=moved)
        with self.assertRaisesRegex(ValueError, "reports transaction 7"):
            parse_arm(a, "A")

    def test_marker_point_count_and_absent_marker_are_rejected(self):
        b = make_arm(self.tmp.name, "B", 1, self.b_cycles, 3,
                     sparse="M7 object port sparse transaction 9 points=71 PASS")
        with self.assertRaisesRegex(ValueError, "points=71"):
            parse_arm(b, "B")
        a = make_arm(self.tmp.name, "A", 2, self.a_cycles, 1, dense="")
        with self.assertRaisesRegex(ValueError, "dense transaction PASS marker missing"):
            parse_arm(a, "A")

    def test_repeated_rtl_markers_must_all_agree(self):
        # Only the co-simulation log repeats the markers, so a disagreement
        # that appears in just one copy must still be refused.
        a = make_arm(self.tmp.name, "A", 2, self.a_cycles, 1)
        log = a / "sim/report/verilog/fpga_m7_object_evidence.log"
        text = log.read_text()
        self.assertEqual(text.count(DENSE), 2)
        log.write_text(text.replace(
            DENSE, "M7 object port dense transaction 3 points=576 PASS", 1))
        with self.assertRaisesRegex(ValueError,
                                    "RTL co-simulation log.*reports transaction 3"):
            parse_arm(a, "A")


if __name__ == "__main__":
    unittest.main()
