import copy
import unittest

from tools.rtabmap_match_gap_audit import analyze, REQUIRED_STATS


def info(stamp, ref, highest, rejected, matches, inliers, ratio=None, loop=0):
    values = {"Loop/Highest_hypothesis_id/": highest,
              "Loop/RejectedHypothesis/": rejected,
              "Loop/Visual_matches/": matches,
              "Loop/Visual_inliers/": inliers,
              "Loop/Optimization_max_error_ratio/": ratio}
    return {"event": "info", "stamp_ns": stamp, "ref_id": ref,
            "loop_closure_id": loop, "proximity_detection_id": 0,
            "stats_keys": list(values), "stats_values": list(values.values())}


def fixture():
    observations, gate, rows = [], [], []
    specs = [
        (10, 1, 1, 0, 30, 25, .1, 1, "approved"),
        (11, 2, 1, 1, 42, 0, None, 0, "no_prior_match"),
        (12, 3, 2, 1, 70, 40, 4.5, 0, "no_prior_match"),
        (13, 4, 1, 1, 60, 30, .2, 0, "no_prior_match"),
        (14, 5, 2, 0, 0, 0, None, 0, "no_prior_match"),
    ]
    for index, (stamp, ref, highest, rejected, matches, inliers, ratio, loop, primary) in enumerate(specs):
        observations += [{"event": "publish_begin", "stamp_ns": stamp, "index": index},
                         info(stamp, ref, highest, rejected, matches, inliers, ratio, loop)]
        gate.append({"event": "received_info", "stamp_ns": stamp,
                     "approved": [{"stamp_ns": stamp}] if index == 0 else []})
        rows.append({"index": index, "stamp_ns": stamp, "primary_reason": primary,
                     "raw_health_flags": [] if loop else ["info:no_prior_map_match"]})
    log = ("Rejected loop closure 1 -> 2: Not enough inliers 0/20 (matches=42)\n"
           "Rejecting localization (3 <-> 2) because maximum graph error ratio of 4.500000\n")
    expected = {"published": 5, "prior_link_observed": 1, "approved": 1,
                "primary_no_prior_match": 4, "raw_no_prior_link": 4}
    return observations, gate, {"rows": rows}, {1, 2}, log, expected


class MatchGapAuditTest(unittest.TestCase):
    def test_positive_all_categories_and_conservation(self):
        summary, ledger = analyze(*fixture())
        self.assertEqual(summary["all_frames_by_category"], {
            "prior_link_observed": 1, "geometry_inliers_rejected": 1,
            "graph_consistency_rejected": 1, "rejected_hypothesis_stage_unknown": 1,
            "candidate_screen_or_no_saved_geometry_evidence": 1})
        self.assertEqual(len(ledger), 5)

    def test_wrong_current_ref_is_rejected(self):
        args = list(fixture())
        args[4] = args[4].replace("1 -> 2", "1 -> 99")
        with self.assertRaisesRegex(ValueError, "no Info"):
            analyze(*args)

    def test_prior_outside_map_is_rejected(self):
        args = list(fixture())
        args[4] = args[4].replace("1 -> 2", "9 -> 2")
        with self.assertRaisesRegex(ValueError, "outside frozen map"):
            analyze(*args)

    def test_missing_info_is_rejected(self):
        args = list(fixture())
        args[0] = [row for row in args[0] if not (row["event"] == "info" and row["ref_id"] == 5)]
        with self.assertRaisesRegex(ValueError, "stamp sets differ"):
            analyze(*args)

    def test_missing_stats_is_rejected(self):
        args = list(fixture())
        target = next(row for row in args[0] if row["event"] == "info" and row["ref_id"] == 5)
        target["stats_keys"].remove(REQUIRED_STATS[0]); target["stats_values"].pop(0)
        with self.assertRaisesRegex(ValueError, "missing stats"):
            analyze(*args)

    def test_optimization_ratio_may_be_absent_without_graph_log(self):
        args = list(fixture())
        target = next(row for row in args[0] if row["event"] == "info" and row["ref_id"] == 5)
        position = target["stats_keys"].index("Loop/Optimization_max_error_ratio/")
        target["stats_keys"].pop(position); target["stats_values"].pop(position)
        summary, _ = analyze(*args)
        self.assertEqual(summary["published"], 5)

    def test_optimization_ratio_required_for_graph_log(self):
        args = list(fixture())
        target = next(row for row in args[0] if row["event"] == "info" and row["ref_id"] == 3)
        position = target["stats_keys"].index("Loop/Optimization_max_error_ratio/")
        target["stats_keys"].pop(position); target["stats_values"].pop(position)
        with self.assertRaisesRegex(ValueError, "missing optimization ratio"):
            analyze(*args)

    def test_ledger_approval_mismatch_is_rejected(self):
        args = list(fixture())
        args[2]["rows"][0]["primary_reason"] = "reconfirmation"
        with self.assertRaisesRegex(ValueError, "approval mismatch"):
            analyze(*args)

    def test_ledger_prior_membership_mismatch_is_rejected(self):
        args = list(fixture())
        args[2]["rows"][1]["raw_health_flags"] = []
        with self.assertRaisesRegex(ValueError, "prior membership mismatch"):
            analyze(*args)

    def test_conflicting_log_types_are_rejected(self):
        args = list(fixture())
        args[4] += "Rejecting localization (2 <-> 1) because maximum graph error ratio of 4.0\n"
        with self.assertRaisesRegex(ValueError, "duplicate or conflicting"):
            analyze(*args)


if __name__ == "__main__":
    unittest.main()
