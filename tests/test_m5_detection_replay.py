"""Wire representation comparison; no detector or device is loaded."""
import copy
import unittest
from tools.m5_detection_replay import matches_online

class ComparisonTests(unittest.TestCase):
    def fixture(self):
        online = {'frame_id': 'rgb-1', 'acquisition_timestamp': 1., 'detections': [
            {'label': 'bottle', 'score': .7, 'bbox': [1., 2., 3., 4.]}]}
        reply = {'frame_id': 'rgb-1', 'timestamp': 1., 'detections': [
            {'label': 'bottle', 'score': .7, 'bbox': (1., 2., 3., 4.), 'frame_id': 'rgb-1', 'timestamp': 1.}]}
        return reply, online

    def test_wire_tuple_list_equivalence_and_no_numeric_relaxation(self):
        reply, online = self.fixture()
        self.assertTrue(matches_online(reply, online))
        for field, value in [('score', .7000000001), ('label', 'cup'), ('bbox', (1., 2., 3., 4.00000001)),
                             ('frame_id', 'rgb-old'), ('timestamp', .9)]:
            candidate = copy.deepcopy(reply); candidate['detections'][0][field] = value
            with self.subTest(field=field): self.assertFalse(matches_online(candidate, online))
        reply['detections'] = []
        self.assertFalse(matches_online(reply, online))

    def test_old_top_level_reply_and_empty_matching_frames(self):
        reply, online = self.fixture()
        reply['frame_id'] = 'old'
        self.assertFalse(matches_online(reply, online))
        reply['frame_id'] = 'rgb-1'; reply['timestamp'] = .9
        self.assertFalse(matches_online(reply, online))
        reply['timestamp'] = 1.; reply['detections'] = []; online['detections'] = []
        self.assertTrue(matches_online(reply, online))
