import copy
import json
from pathlib import Path
import tempfile
import unittest
from tools.compare_replays import compare


class CompareTests(unittest.TestCase):
    def test_equal_changed_and_foreign_inputs(self):
        row={"frame_id":"1","timestamp":1,"observation_count":1,
             "detections":[{"label":"cup","score":.9,"bbox":[0,0,10,10]}],
             "strategies":{"geometry":{"tracks":[{"id":"a","label":"cup","color":"red",
                 "state":"historical_memory","reason":"out_of_view","position_m":[0,0,2]}]}}}
        with tempfile.TemporaryDirectory() as td:
            left,right=Path(td)/"left",Path(td)/"right"
            for p in (left,right):
                p.mkdir();(p/"input.jsonl").write_text('same input\n')
                (p/"events.jsonl").write_text(json.dumps(row))
            result=compare(left,right)
            self.assertEqual(result["max_box_coordinate_error_px_for_matching_order"],0)
            changed=copy.deepcopy(row)
            changed["detections"][0]["bbox"][0]=5
            changed["strategies"]["geometry"]["tracks"][0]["state"]="position_invalid"
            (right/"events.jsonl").write_text(json.dumps(changed))
            result=compare(left,right)
            self.assertEqual(result["max_box_coordinate_error_px_for_matching_order"],5)
            self.assertEqual(len(result["track_state_or_reason_changes"]),1)
            (right/"input.jsonl").write_text('different input\n')
            with self.assertRaisesRegex(ValueError,"manifests"):compare(left,right)


if __name__=="__main__":unittest.main()
