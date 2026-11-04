import unittest
import xml.etree.ElementTree as ET
from host.nav_recovery import bounded_recovery


SOURCE = """<root><RecoveryNode name="NavigateRecovery" number_of_retries="6">
<PipelineSequence><RecoveryNode name="ComputePathToPose" number_of_retries="1">
<ComputePathToPose/><ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
</RecoveryNode><RecoveryNode name="FollowPath" number_of_retries="1">
<FollowPath/><ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
</RecoveryNode></PipelineSequence><RoundRobin><Sequence>
<ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
<ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
</Sequence><Spin/><Wait wait_duration="5"/></RoundRobin></RecoveryNode></root>"""


class RecoveryTests(unittest.TestCase):
    def test_bounded_retry_preserves_observations_and_rest_of_tree(self):
        original = ET.fromstring(SOURCE)
        result = ET.fromstring(bounded_recovery(SOURCE))
        target = next(n for n in result.iter('RecoveryNode') if n.get('name') == 'NavigateRecovery')
        self.assertEqual(target.get('number_of_retries'), '1')
        self.assertFalse(list(result.iter('ClearEntireCostmap')))
        target.set('number_of_retries', '6')
        pipeline = next(result.iter('PipelineSequence'))
        monitor = next(result.iter('ReactiveSequence'))
        patience, latched, follow = list(monitor)
        initialize = next(n for n in result.iter('SequenceStar') if n.get('name') == 'InitializeObservation')
        self.assertEqual(initialize[0].attrib, {'output_key': 'fpga_observation', 'value': 'idle'})
        result.remove(initialize)
        result.append(target)
        self.assertEqual(patience[0].attrib, {'output_key': 'fpga_observation', 'value': 'pending'})
        self.assertEqual(latched.attrib, {'value_A': '{fpga_observation}', 'value_B': 'pending', 'return_on_mismatch': 'SUCCESS'})
        observation = latched[0]
        self.assertEqual(observation[-1].attrib, {'output_key': 'fpga_observation', 'value': 'idle'})
        self.assertEqual(patience.tag, 'PathLongerOnApproach')
        self.assertEqual(patience.attrib, {'path': '{path}', 'prox_len': '3.0', 'length_factor': '1.5'})
        self.assertEqual([n.tag for n in observation],
                         ['CancelControl', 'Wait', 'Spin', 'Spin', 'Spin', 'SetBlackboard'])
        self.assertEqual([n.get('spin_dist') for n in observation.iter('Spin')], ['0.45', '-0.9', '0.45'])
        self.assertEqual(next(observation.iter('Wait')).get('wait_duration'), '5')
        index = list(pipeline).index(monitor)
        pipeline.remove(monitor)
        pipeline.insert(index, follow)
        replacements = [n for n in result.iter('Wait') if n.get('name', '').startswith('Clear')]
        self.assertEqual(len(replacements), 4)
        for node in replacements:
            self.assertEqual(node.get('wait_duration'), '1')
            old = next(n for n in original.iter('ClearEntireCostmap') if n.get('name') == node.get('name'))
            node.tag = old.tag
            node.attrib.clear()
            node.attrib.update(old.attrib)
        self.assertEqual(ET.tostring(result), ET.tostring(original))

    def test_unreviewed_tree_is_rejected(self):
        for source in ('<root/>', SOURCE.replace('number_of_retries="6"', 'number_of_retries="2"'),
                       SOURCE.replace('ClearGlobalCostmap-Context', 'UnknownClear'),
                       SOURCE.replace('service_name="global_costmap/clear_entirely_global_costmap"', 'service_name="other"'),
                       SOURCE.replace('<ComputePathToPose/>', '<ClearEntireCostmap/>'),
                       SOURCE.replace('name="FollowPath"', 'name="UnknownFollower"')):
            with self.subTest(source=source), self.assertRaises(ValueError):
                bounded_recovery(source)
