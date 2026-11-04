import importlib.util
import ast
import math
from pathlib import Path
import unittest
from types import SimpleNamespace
spec=importlib.util.spec_from_file_location('nav_suite',Path(__file__).resolve().parents[1]/'robot_sim/tests/nav_point_to_point.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class NavigationSafetyTests(unittest.TestCase):
    def test_main_timeout_cleanup_cannot_turn_late_terminal_into_pass(self):
        # Execute the actual main cleanup branch and verdict expression. The
        # asynchronous terminal is injected only during the cleanup wait.
        tree = ast.parse(Path(m.__file__).read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        cleanup = next(n for n in ast.walk(main) if isinstance(n, ast.If)
                       and any(isinstance(x, ast.Assign)
                               and any(isinstance(t, ast.Name) and t.id == 'reason' for t in x.targets)
                               for x in n.body))
        verdict = next(n for n in ast.walk(main) if isinstance(n, ast.Assign)
                       and any(isinstance(t, ast.Name) and t.id == 'passed' for t in n.targets))
        code = compile(ast.fix_missing_locations(ast.Module(body=[cleanup, verdict], type_ignores=[])),
                       m.__file__, 'exec')
        for name, status, wall_case in [('straight', 4, False), ('cancel', 5, False),
                                       ('unreachable', 6, False), ('c_navigation', 6, True)]:
            for late in (False, True):
                with self.subTest(name=name, late=late):
                    lifecycle = m.GoalLifecycle()
                    context = lifecycle.start(('reference',))
                    if not late:
                        lifecycle.result(context, status)
                    def spin_until(predicate, seconds):
                        self.assertEqual(seconds, 5)
                        lifecycle.result(context, status)
                        return predicate()
                    env = dict(m.__dict__, context=context, lifecycle=lifecycle,
                               state={'fault': None, 'pose': (0., 0., 0.)},
                               spin_until=spin_until, log=lambda *a, **kw: None,
                               name=name, status=status, stopped=True, cancelled=name == 'cancel',
                               wall_case=wall_case, expected_wall_abort=wall_case,
                               target=(0., 0., 0.))
                    exec(code, env)
                    self.assertEqual(env['passed'], not late)
                    self.assertEqual(env['state']['fault'], 'goal_wall_timeout' if late else None)

    def test_cancel_transport_failure_preserves_slot_and_false_permission(self):
        lifecycle = m.GoalLifecycle()
        context = lifecycle.start(('reference',))
        calls = []
        def broken_cancel():
            calls.append(1)
            raise RuntimeError('delivery unknown')
        lifecycle.accepted(context, SimpleNamespace(accepted=True, cancel_goal_async=broken_cancel))
        self.assertIsNone(lifecycle.revoke(context))
        self.assertIs(lifecycle.active, context)
        self.assertFalse(m.motion_allowed(True, True, lifecycle))
        self.assertIn('goal_cancel_transport_failed', context.transport_error)
        self.assertIsNone(lifecycle.revoke(context))
        self.assertEqual(len(calls), 1)
        with self.assertRaises(RuntimeError):
            lifecycle.start(('new',))

    def test_reference_remains_the_cli_default(self):
        args, localization = m.parse_arguments(['--out', 'unused.jsonl'])
        self.assertEqual(args.pose_source, 'reference')
        self.assertEqual(localization.identity(0.0, 0), ('reference',))

    def test_estimated_cli_is_blocked_without_correction_generation_contract(self):
        # D-035 item 3 (2026-09-27): unlocked now that the correction rule exists; the
        # database declarations are still required and must agree.
        args, localization = m.parse_arguments(['--out', 'unused.jsonl', '--pose-source', 'estimated',
                                                '--localization-map-id', 'a'*64,
                                                '--navigation-map-id', 'a'*64])
        self.assertEqual((args.pose_source, localization.mode), ('estimated', 'estimated'))
        for ids in (('a'*64, 'b'*64), ('a'*64, None), ('A'*64, 'A'*64)):
            with self.assertRaises(SystemExit):
                m.parse_arguments(['--out', 'unused.jsonl', '--pose-source', 'estimated',
                                   '--localization-map-id', ids[0]]
                                  + (['--navigation-map-id', ids[1]] if ids[1] else []))

    def test_stale_future_missing_nonfinite_input_rejected(self):
        self.assertTrue(m.fresh(10,{'odom':9.9},{'odom':20},20.1))
        for stamps, receipts in [({},{}),({'odom':8.9},{'odom':20}),({'odom':10.1},{'odom':20}),
                                  ({'odom':float('nan')},{'odom':20}),({'odom':9.9},{}),
                                  ({'odom':9.9},{'odom':19})]:
            self.assertFalse(m.fresh(10,stamps,receipts,20.1))
    def test_position_and_heading_required(self):
        self.assertTrue(m.within_goal((0,0,3.14159),(0,0,-3.14159)))
        self.assertFalse(m.within_goal((.031,0,0),(0,0,0)))
        self.assertFalse(m.within_goal((0,0,.031),(0,0,0)))
        self.assertFalse(m.within_goal((float('nan'),0,0),(0,0,0)))

    def test_future_input_waits_for_clock_without_refreshing_old_receipt(self):
        stamps={'odom':9.9};receipts={'odom':20.};pending={'odom':[(10.1,20.1)]}
        m.promote(10,pending,stamps,receipts)
        self.assertEqual(stamps['odom'],9.9)
        self.assertEqual(receipts['odom'],20.)
        self.assertFalse(m.fresh(10,stamps,receipts,20.6))
        m.promote(10.1,pending,stamps,receipts)
        self.assertEqual(stamps['odom'],10.1)
        self.assertEqual(receipts['odom'],20.1)
        self.assertFalse(m.fresh(10.1,stamps,receipts,20.7))

    def test_arrival_pose_comes_from_map_transform_with_rotation(self):
        transform = SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=3.0, y=-2.0),
            rotation=SimpleNamespace(x=0.0, y=0.0,
                                     z=math.sin(math.pi/4), w=math.cos(math.pi/4))))
        pose = m.planar_pose(transform)
        self.assertTrue(m.within_goal(pose, (3.0, -2.0, math.pi/2)))
        # The old odom origin cannot accidentally pass this map-frame goal.
        self.assertFalse(m.within_goal((0.0, 0.0, 0.0), (3.0, -2.0, math.pi/2)))

    def test_real_send_path_latches_revoke_until_terminal_result(self):
        class Future:
            def __init__(self, value): self.value, self.callbacks = value, []
            def add_done_callback(self, callback): self.callbacks.append(callback)
            def result(self): return self.value
            def finish(self):
                for callback in self.callbacks: callback(self)
        class Handle:
            accepted = True
            def __init__(self): self.cancel_calls, self.result = 0, Future(None)
            def cancel_goal_async(self):
                self.cancel_calls += 1
                return object()
            def get_result_async(self): return self.result
        class Action:
            def __init__(self): self.goals = []
            def send_goal_async(self, goal):
                self.goals.append(goal)
                self.response = Future(handle)
                return self.response

        action, lifecycle, handle = Action(), m.GoalLifecycle(), Handle()
        context, future = m.send_goal(action, lifecycle, ('lease', 1), 'goal-message')
        self.assertEqual(action.goals, ['goal-message'])
        self.assertIsNone(lifecycle.revoke(context))  # response has not arrived
        future.finish()  # the real registered response callback owns the late handle
        self.assertIs(context.result_future, handle.result)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertTrue(lifecycle.cancel_response(context))
        with self.assertRaisesRegex(RuntimeError, 'has not terminated'):
            lifecycle.start(('lease', 2))
        self.assertFalse(lifecycle.result(context, 5))
        replacement = lifecycle.start(('lease', 2))
        self.assertIs(lifecycle.active, replacement)

    def test_late_acceptance_of_retired_request_is_cancelled(self):
        class Handle:
            accepted = True
            def __init__(self): self.cancel_calls = 0
            def cancel_goal_async(self):
                self.cancel_calls += 1
                return object()

        lifecycle = m.GoalLifecycle()
        old = lifecycle.start(('old',))
        # Rejection is terminal and permits a new request.
        lifecycle.accepted(old, SimpleNamespace(accepted=False))
        current = lifecycle.start(('new',))
        late = Handle()
        lifecycle.accepted(old, late)
        self.assertEqual(late.cancel_calls, 1)
        self.assertIs(lifecycle.active, current)

    def test_cancel_response_does_not_enable_or_release_goal(self):
        handle = SimpleNamespace(accepted=True, cancel_goal_async=lambda: object())
        lifecycle = m.GoalLifecycle()
        context = lifecycle.start(('lease',))
        lifecycle.accepted(context, handle)
        self.assertTrue(m.motion_allowed(True, True, lifecycle))
        lifecycle.revoke(context)
        self.assertTrue(lifecycle.cancel_response(context))
        self.assertFalse(m.motion_allowed(True, True, lifecycle))
        self.assertIs(lifecycle.active, context)
        lifecycle.result(context, 5)
        self.assertFalse(m.motion_allowed(True, True, lifecycle))

    def test_retire_waits_for_result_and_sends_cancel_only_once(self):
        class Result:
            def __init__(self): self.complete = False
            def done(self): return self.complete
            def result(self): return SimpleNamespace(status=5)
        class Handle:
            accepted = True
            def __init__(self): self.calls = 0
            def cancel_goal_async(self): self.calls += 1; return object()
        lifecycle, handle, result = m.GoalLifecycle(), Handle(), Result()
        context = lifecycle.start(('lease',))
        lifecycle.accepted(context, handle)
        def spin(predicate, _seconds):
            self.assertFalse(predicate())
            result.complete = True
            lifecycle.result(context, 5)
            return predicate()
        self.assertTrue(m.retire_and_wait(lifecycle, context, spin))
        self.assertEqual(handle.calls, 1)
        self.assertFalse(lifecycle.cancel_response(context))
        self.assertIsNone(lifecycle.active)

    def test_pending_input_loss_revokes_and_late_accept_is_cancelled(self):
        class Handle:
            accepted = True
            def __init__(self): self.calls = 0
            def cancel_goal_async(self): self.calls += 1; return object()
            def get_result_async(self):
                return SimpleNamespace(add_done_callback=lambda callback: None)
        lifecycle = m.GoalLifecycle()
        context = lifecycle.start(('reference',))
        state = {'fault': None, 'enabled': False}
        self.assertFalse(m.evaluate_navigation_tick(
            state, lifecycle, m.NavigationLocalization('reference'), False, 1.0, 1))
        self.assertTrue(context.revoked)
        handle = Handle()
        m.accept_goal_response(lifecycle, context,
                               SimpleNamespace(result=lambda: handle))
        self.assertEqual(handle.calls, 1)

    def test_terminal_result_callback_revokes_permit_before_next_tick(self):
        lifecycle = m.GoalLifecycle()
        context = lifecycle.start(('reference',))
        lifecycle.accepted(context, SimpleNamespace(accepted=True))
        m.accept_goal_result(lifecycle, context,
                             SimpleNamespace(result=lambda: SimpleNamespace(status=4)))
        self.assertIsNone(lifecycle.active)
        state = {'fault': None, 'enabled': True}
        self.assertFalse(m.evaluate_navigation_tick(
            state, lifecycle, m.NavigationLocalization('reference'), True, 1.0, 1))

    def test_transport_exceptions_remain_revoked_and_occupied(self):
        class BrokenAction:
            def send_goal_async(self, _goal): raise RuntimeError('wire')
        lifecycle = m.GoalLifecycle()
        with self.assertRaisesRegex(RuntimeError, 'wire'):
            m.send_goal(BrokenAction(), lifecycle, ('reference',), object())
        self.assertTrue(lifecycle.active.revoked)
        self.assertNotEqual(lifecycle.active.phase, 'terminal')

        context = lifecycle.active
        m.accept_goal_response(lifecycle, context,
            SimpleNamespace(result=lambda: (_ for _ in ()).throw(RuntimeError('response'))))
        self.assertTrue(context.revoked)
        self.assertIn('response', context.transport_error)
        self.assertIs(lifecycle.active, context)

    def test_health_uses_admitted_odom_stamp_and_exact_map_transform(self):
        query_ns = 9_900_000_000
        transform = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=9, nanosec=900_000_000)),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=2.0, y=-1.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(.2), w=math.cos(.2))))
        class Buffer:
            def __init__(self, value=transform): self.value, self.queries = value, []
            def lookup_transform(self, parent, child, stamp):
                self.queries.append((parent, child, stamp.nanoseconds))
                if isinstance(self.value, Exception): raise self.value
                return self.value
        class Stamp:
            def __init__(self, nanoseconds): self.nanoseconds = nanoseconds
        state = {'fault': None, 'clock': 10.0, 'clock_wall': 20.0, 'pose': None}
        stamps, receipts, topics = {'/odom': 9.9}, {'/odom': 20.0}, {'/odom': object()}
        buffer = Buffer()
        self.assertTrue(m.evaluate_health(state, stamps, receipts, topics, 20.1,
                                          buffer, Stamp, m.NavigationLocalization('reference')))
        self.assertEqual(buffer.queries, [('map', 'base_link', query_ns)])
        self.assertTrue(m.within_goal(state['pose'], (2.0, -1.0, .4)))
        self.assertFalse(m.evaluate_health(state, stamps, receipts, topics, 20.1,
                                           Buffer(RuntimeError('missing')), Stamp,
                                           m.NavigationLocalization('reference')))
        future = dict(stamps); future['/odom'] = 10.1
        self.assertFalse(m.evaluate_health(state, future, receipts, topics, 20.1,
                                           Buffer(), Stamp,
                                           m.NavigationLocalization('reference')))
        wrong_stamp = SimpleNamespace(**transform.__dict__)
        wrong_stamp.header = SimpleNamespace(stamp=SimpleNamespace(sec=9, nanosec=800_000_000))
        self.assertFalse(m.evaluate_health(state, stamps, receipts, topics, 20.1,
                                           Buffer(wrong_stamp), Stamp,
                                           m.NavigationLocalization('reference')))
