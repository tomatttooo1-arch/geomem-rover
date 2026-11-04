"""Calibration controls for the dense HLS transaction parser."""

from pathlib import Path
import importlib.util
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "parse_m7_dense_hls", ROOT / "tools/parse_m7_dense_hls.py")
PARSER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PARSER)

def case_line(index, name, valid):
    unique = 0 if valid == 0 else 1
    return (f"DENSE_CASE index={index} name={name} pixels=4096 valid={valid} "
            f"unique={unique} dda_steps={valid*10} accepted={valid} "
            f"invalid={4096-valid} outside=0\n")


CASE_LINES = [case_line(i, name, valid)
              for i, (name, valid) in enumerate(PARSER.EXPECTED_CASES)]
CASES = "".join(CASE_LINES) + "M7 dense frame testbench PASS\n"
PASS_LOG = CASES + "INFO: [COSIM] *** C/RTL co-simulation finished: PASS ***\n"
TX_LINES = [f"transaction {i:7d}: {100+i*128:15d} " +
            ("              x\n" if i == len(PARSER.EXPECTED_CASES)-1 else
             f"{101+i*128:15d}\n") for i in range(len(PARSER.EXPECTED_CASES))]
TX = "                             latency        interval\n" + "".join(TX_LINES)


class DenseParserCalibration(unittest.TestCase):
    def test_known_positive_and_cycle_mutation_are_observed(self):
        records = PARSER.parse_dense(PASS_LOG, TX)
        self.assertEqual(records[0]["latency_cycles"], 100)
        self.assertEqual(records[1]["incremental_cycles"], 128)
        self.assertEqual(records[1]["incremental_cycles_per_valid"], 2.)
        changed = PARSER.parse_dense(PASS_LOG, TX.replace("228", "292"))
        self.assertEqual(changed[1]["incremental_cycles"], 192)
        self.assertEqual(changed[1]["incremental_cycles_per_valid"], 3.)

    def test_missing_pass_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "lacks C/RTL"):
            PARSER.parse_dense(CASES, TX)

    def test_missing_or_duplicate_transactions_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "transaction count"):
            PARSER.parse_dense(PASS_LOG, TX.replace(TX_LINES[-1], ""))
        with self.assertRaisesRegex(ValueError, "duplicate RTL"):
            PARSER.parse_dense(PASS_LOG, TX + TX_LINES[-1])

    def test_missing_or_corrupt_case_metadata_is_rejected(self):
        one_case = PASS_LOG.replace(CASE_LINES[-1], "")
        with self.assertRaisesRegex(ValueError, "truncated group"):
            PARSER.parse_dense(one_case, TX)
        bad_count = PASS_LOG.replace("invalid=4032", "invalid=4031", 1)
        with self.assertRaisesRegex(ValueError, "conservation"):
            PARSER.parse_dense(bad_count, TX)

    def test_simultaneous_case_transaction_truncation_and_partial_late_group_rejected(self):
        with self.assertRaises(ValueError):
            PARSER.parse_dense(PASS_LOG.replace(CASE_LINES[-1], ""),
                               TX.replace(TX_LINES[-1], ""))
        partial_late = PASS_LOG.replace("M7 dense frame testbench PASS\n",
                                        CASE_LINES[0] + "M7 dense frame testbench PASS\n")
        with self.assertRaisesRegex(ValueError, "truncated group"):
            PARSER.parse_dense(partial_late, TX)

    def test_json_and_csv_are_derived_from_same_records(self):
        records = PARSER.parse_dense(PASS_LOG, TX)
        with tempfile.TemporaryDirectory(prefix="m7-dense-parser-") as directory:
            json_path = Path(directory) / "out.json"
            csv_path = Path(directory) / "out.csv"
            PARSER.write_outputs(records, json_path, csv_path)
            self.assertIn('"latency_cycles": 228', json_path.read_text())
            self.assertIn("same_short_64", csv_path.read_text())
            with self.assertRaises(FileExistsError):
                PARSER.write_outputs(records, json_path, csv_path)


if __name__ == "__main__":
    unittest.main()
