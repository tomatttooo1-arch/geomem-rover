import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

from tools.rtabmap_derive_availability import derive, main


FROZEN_ROOT = Path(__file__).resolve().parents[2] / "rtabmap-reload-coordinate-20260912"

# local_index, original_index, approved, translation_error_m, numeric_pose, healthy_prior
RECORDS = [
    (0, 100, False, 0.30, True, False),
    (1, 101, True, 0.05, True, True),
    (2, 102, True, 0.20, True, True),
    (3, 103, False, 0.10, True, True),
    (4, 104, False, None, True, False),
    (5, 105, True, 0.02, True, True),
    (6, 106, False, 0.15, True, True),
    (7, 107, True, 0.16, True, True),
]
REASONS = ["invalid_covariance", "approved", "approved", "reconfirmation", "no_prior_match",
           "approved", "reconfirmation", "approved"]


class DeriveAvailabilityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.write_inputs(RECORDS, REASONS)

    def write_inputs(self, records, reasons, skip_ledger_indices=()):
        rows = []
        for (local, original, approved, error, numeric, healthy) in records:
            rows.append({
                "local_index": local,
                "original_index": original,
                "approved": approved,
                "translation_error_m": error,
                "numeric_pose": numeric,
                "healthy_prior_localization": healthy,
            })
        score = {
            "denominators": {"healthy_prior_localization": sum(row[5] for row in records)},
            "completion": {"complete_events": [{"event": "complete", "published_frames": len(records)}]},
            "records": rows,
        }
        primary_reasons = {}
        for reason in reasons:
            primary_reasons[reason] = primary_reasons.get(reason, 0) + 1
        ledger = {
            "summary": {
                "primary_reasons": primary_reasons,
                "raw_flags": {"info:no_prior_map_match": 2, "odom:invalid_covariance": 1},
                "healthy_unapproved_reasons": {"reconfirmation": 2},
            },
            "rows": [{"index": record[0], "primary_reason": reason}
                     for record, reason in zip(records, reasons)
                     if record[0] not in skip_ledger_indices],
        }
        self.score_path = self.root / "normal-score.json"
        self.ledger_path = self.root / "normal-availability-ledger.json"
        self.score_path.write_text(json.dumps(score, indent=2))
        self.ledger_path.write_text(json.dumps(ledger, indent=2))

    def test_counts_and_passthrough_fields(self):
        result = derive(self.root)
        self.assertEqual(result["schema"], 2)
        self.assertEqual(result["published"], 8)
        self.assertEqual(result["approved"], 4)
        self.assertEqual(result["unapproved"], 4)
        self.assertEqual(result["healthy_prior_localization"], 6)
        self.assertEqual(result["raw_overlapping_flags"],
                         {"info:no_prior_map_match": 2, "odom:invalid_covariance": 1})
        self.assertEqual(result["healthy_unapproved_reasons"], {"reconfirmation": 2})
        self.assertEqual(result["completion"]["complete_events"][0]["published_frames"], 8)

    def test_unapproved_primary_reasons_drop_approved_and_sum_to_unapproved(self):
        result = derive(self.root)
        self.assertEqual(result["unapproved_primary_reasons"],
                         {"invalid_covariance": 1, "reconfirmation": 2, "no_prior_match": 1})
        self.assertNotIn("approved", result["unapproved_primary_reasons"])
        self.assertEqual(sum(result["unapproved_primary_reasons"].values()), result["unapproved"])

    def test_approved_and_unapproved_segments(self):
        result = derive(self.root)
        self.assertEqual(result["approved_segments"], [
            {"local_range": [1, 3], "original_range": [101, 103], "frames": 2, "primary_reasons": {}},
            {"local_range": [5, 6], "original_range": [105, 106], "frames": 1, "primary_reasons": {}},
            {"local_range": [7, 8], "original_range": [107, 108], "frames": 1, "primary_reasons": {}},
        ])
        self.assertEqual(result["unapproved_segments"], [
            {"local_range": [0, 1], "original_range": [100, 101], "frames": 1,
             "primary_reasons": {"invalid_covariance": 1}},
            {"local_range": [3, 5], "original_range": [103, 105], "frames": 2,
             "primary_reasons": {"reconfirmation": 1, "no_prior_match": 1}},
            {"local_range": [6, 7], "original_range": [106, 107], "frames": 1,
             "primary_reasons": {"reconfirmation": 1}},
        ])
        self.assertEqual(result["longest_approved_segment"], result["approved_segments"][0])

    def test_threshold_reference_counts_skip_missing_errors(self):
        # 0.15 sits exactly on the boundary and must not be counted as over; 0.16 must.
        reference = derive(self.root)["absolute_position_threshold_reference"]
        self.assertEqual(reference["raw_localization"],
                         {"samples": 7, "over_0_15m": 3, "max_m": 0.30})
        self.assertEqual(reference["healthy_prior"],
                         {"samples": 6, "over_0_15m": 2, "max_m": 0.20})
        self.assertEqual(reference["approved"],
                         {"samples": 4, "over_0_15m": 2, "max_m": 0.20})

    def test_source_sha256_matches_input_files(self):
        result = derive(self.root)
        self.assertEqual(result["source_sha256"], {
            "normal-score.json": hashlib.sha256(self.score_path.read_bytes()).hexdigest(),
            "normal-availability-ledger.json": hashlib.sha256(self.ledger_path.read_bytes()).hexdigest(),
        })

    def test_ledger_missing_record_index_raises(self):
        self.write_inputs(RECORDS, REASONS, skip_ledger_indices=(0,))
        with self.assertRaises(KeyError):
            derive(self.root)

    def test_main_refuses_to_overwrite_existing_output(self):
        output = self.root / "availability-summary-v2.json"
        output.write_text("sentinel")
        self.addCleanup(setattr, sys, "argv", sys.argv)
        sys.argv = ["rtabmap_derive_availability.py", str(self.root), str(output)]
        with self.assertRaises(FileExistsError):
            main()
        self.assertEqual(output.read_text(), "sentinel")


@unittest.skipUnless((FROZEN_ROOT / "availability-summary-v2.json").is_file(),
                     "frozen reload-coordinate run root is not available")
class FrozenAvailabilityRegressionTests(unittest.TestCase):
    def test_derivation_reproduces_frozen_summary_bytes(self):
        frozen = (FROZEN_ROOT / "availability-summary-v2.json").read_text()
        result = derive(FROZEN_ROOT)
        self.assertEqual(json.dumps(result, indent=2, allow_nan=False) + "\n", frozen)


if __name__ == "__main__":
    unittest.main()
