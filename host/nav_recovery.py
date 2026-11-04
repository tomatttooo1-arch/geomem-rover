"""Bound recovery without erasing sensor-confirmed obstacles on planner failure."""
import xml.etree.ElementTree as ET


def bounded_recovery(source):
    tree = ET.fromstring(source)
    nodes = [n for n in tree.iter('RecoveryNode') if n.get('name') == 'NavigateRecovery']
    if len(nodes) != 1 or nodes[0].get('number_of_retries') != '6':
        raise ValueError('installed navigation recovery tree differs from reviewed Humble default')
    expected = {
        'ClearGlobalCostmap-Context': 'global_costmap/clear_entirely_global_costmap',
        'ClearLocalCostmap-Context': 'local_costmap/clear_entirely_local_costmap',
        'ClearLocalCostmap-Subtree': 'local_costmap/clear_entirely_local_costmap',
        'ClearGlobalCostmap-Subtree': 'global_costmap/clear_entirely_global_costmap',
    }
    clears = list(tree.iter('ClearEntireCostmap'))
    if (len(clears) != len(expected) or
            {n.get('name'): n.get('service_name') for n in clears} != expected or
            any(set(n.attrib) != {'name', 'service_name'} or len(n) for n in clears)):
        raise ValueError('installed costmap recovery differs from reviewed Humble default')
    # A failed plan is not evidence that a measured obstacle disappeared.
    # Wait for fresh depth/raytracing instead of briefly creating an empty map.
    # The original finite retry budgets and normal replanning remain in force.
    for node in clears:
        name = node.get('name')
        node.tag = 'Wait'
        node.attrib.clear()
        node.attrib.update(name=name, wait_duration='1')
    nodes[0].set('number_of_retries', '1')
    pipelines = list(tree.iter('PipelineSequence'))
    follows = [n for n in tree.iter('RecoveryNode') if n.get('name') == 'FollowPath']
    if (len(pipelines) != 1 or len(follows) != 1 or follows[0] not in list(pipelines[0])
            or len(follows[0]) != 2 or follows[0][0].tag != 'FollowPath'):
        raise ValueError('installed follow-path pipeline differs from reviewed Humble default')
    # Use Nav2's existing goal-patience nodes: cancel control before looking
    # at a newly observed large detour, while the upstream planner keeps running.
    # No scenario coordinates or obstacle-removal signal enter this policy.
    pipeline, follow = pipelines[0], follows[0]
    index = list(pipeline).index(follow)
    pipeline.remove(follow)
    monitor = ET.Element('ReactiveSequence', name='ObserveBeforeLargeDetour')
    patience = ET.SubElement(monitor, 'PathLongerOnApproach',
                             path='{path}', prox_len='3.0', length_factor='1.5')
    ET.SubElement(patience, 'SetBlackboard', output_key='fpga_observation', value='pending')
    latched = ET.SubElement(monitor, 'BlackboardCheckString', value_A='{fpga_observation}',
                            value_B='pending', return_on_mismatch='SUCCESS')
    sequence = ET.SubElement(latched, 'SequenceStar', name='CancelControlAndObserve')
    ET.SubElement(sequence, 'CancelControl', name='ControlCancel')
    ET.SubElement(sequence, 'Wait', wait_duration='5')
    # A close obstacle can span beyond the fixed camera's horizontal view.
    # A bounded stationary sweep obtains real rays over both edges; Spin's
    # collision check remains enabled and can fail the observation safely.
    for angle in ('0.45', '-0.9', '0.45'):
        ET.SubElement(sequence, 'Spin', spin_dist=angle)

    ET.SubElement(sequence, 'SetBlackboard', output_key='fpga_observation', value='idle')
    monitor.append(follow)
    pipeline.insert(index, monitor)
    # Initialize for each new navigation action; updates cannot bypass a started
    # observation sequence. A tree halt still cancels the current Spin/Wait.
    outer = nodes[0]
    parent = next(p for p in tree.iter() if outer in list(p))
    position = list(parent).index(outer)
    parent.remove(outer)
    initialize = ET.Element('SequenceStar', name='InitializeObservation')
    ET.SubElement(initialize, 'SetBlackboard', output_key='fpga_observation', value='idle')
    initialize.append(outer)
    parent.insert(position, initialize)
    return ET.tostring(tree, encoding='unicode')
