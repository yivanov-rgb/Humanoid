"""Offline regressions. Importing tag_grab does not import ROS or create a Runner."""
import importlib.util
import io
import hashlib
import json
import sqlite3
import types
from contextlib import closing
from contextlib import redirect_stderr
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'tag_grab.py'
spec = importlib.util.spec_from_file_location('tag_grab_under_test', SCRIPT)
grab = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = grab
spec.loader.exec_module(grab)

ROOT = SCRIPT.parent
ACTIONS = ROOT / 'robot/ActionGroups'
SOURCE = (ROOT / 'simulation/artifacts/d6a_from_robot/home/ubuntu/software'
          '/ainex_controller/ActionGroups')
MOTION_MANAGER = (ROOT / 'simulation/vendor/ainex/src/ainex_driver'
                  '/ainex_kinematics/src/ainex_kinematics/motion_manager.py')
# Fingerprints include Index, Time and every Servo1..22 value, in source order.
CROUCH = {
    'left': (1, 'fd2475fade225aabcf3ae16ea4991867ade5b08c1e4eaf04e2ede0184ce7bbf5',
             '20c7dd29376eb08a1db62df43c8cd05981302b3271a29f70273dfebc65e70e96'),
    'right': (2, '48bdce21104192cbc792c6c8e685caa3c276fab9c7ff4a0ea2541f9758fa875c',
              'efd60fb6ecdcd48bcf471fb13ca63c57dc3fb2d83e4209571a9930f4e7e4882e'),
}


def read_action(path):
    with closing(sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)) as db:
        columns = [column[1] for column in db.execute('PRAGMA table_info(ActionGroup)')]
        rows = db.execute('SELECT * FROM ActionGroup').fetchall()
        integrity = db.execute('PRAGMA integrity_check').fetchall()
    return columns, rows, integrity


class CalibrationTests(unittest.TestCase):
    def test_baseline_and_demo_keep_independent_hand_bounds(self):
        base = ['--execute', '--ids', '0', '1', '--left-x-min', '280',
                '--left-x-max', '310', '--right-x-min', '240', '--right-x-max', '270']
        for mode in ([], ['--demo-transfer']):
            with self.subTest(mode=mode):
                args = grab.arguments(base + mode)
                self.assertEqual(grab.bounds_for(0, args), (280, 310, 400, 420))
                self.assertEqual(grab.bounds_for(1, args), (240, 270, 400, 420))

    def test_right_overrides_reach_coordinate_decision(self):
        # Synthetic values only; no script defaults or calibration are changed.
        args = grab.arguments(['--right-x-min', '245', '--right-x-max', '265'])
        right = grab.bounds_for(1, args)
        self.assertEqual(grab.decide(243, 410, right), 'move_left')
        self.assertEqual(grab.decide(267, 410, right), 'move_right')
        self.assertEqual(grab.decide(255, 410, right), 'READY')
        self.assertEqual(grab.bounds_for(0, args), (280, 310, 400, 420))

    def test_x_is_rechecked_during_forward_approach(self):
        bounds = grab.bounds_for(1, grab.arguments([]))
        self.assertEqual(grab.decide(271, 390, bounds), 'move_right')
        self.assertEqual(grab.decide(255, 390, bounds), 'forward_one_step')
        self.assertEqual(grab.decide(255, 421, bounds), 'back_step')

    def test_invalid_sequences_are_rejected(self):
        invalid = [
            ['--ids', '1', '0'], ['--ids', '0', '0'], ['--ids', '2'],
            ['--demo-transfer', '--ids', '0', '1'],
            ['--execute', '--demo-transfer', '--ids', '0'],
            ['--execute', '--demo-transfer', '--ids', '0', '1', '--prepare-only'],
            ['--demo-turns', '-1'],
        ]
        for argv in invalid:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    grab.arguments(argv)
                self.assertEqual(error.exception.code, 2)


class SequenceTests(unittest.TestCase):
    def runner(self, argv, fail_target=None, fail_action=None, fail_measurement=False):
        # Bypass __init__: no ROS imports, subscriptions, services or hardware.
        runner = grab.Runner.__new__(grab.Runner)
        runner.args = grab.arguments(['--execute'] + argv)
        runner.ros = SimpleNamespace(loginfo=lambda *args: None,
                                     logwarn=lambda *args: None)
        runner.motion = SimpleNamespace(servo_position={str(i): 500 for i in range(1, 25)})
        runner.left_arm_ids = (13, 15, 17, 19, 21)
        runner.right_arm_ids = (14, 16, 18, 20, 22)
        runner.held_servos = {}
        runner.head_locked = False
        runner.target = runner.args.ids[0]
        events = []
        runner.prepare = lambda: events.append(('prepare', None))
        runner.set_head = lambda: events.append(('head', None))
        runner.check_running = lambda: None

        def approach(target):
            events.append(('approach', target))
            if target == fail_target:
                raise RuntimeError('Injected approach failure')

        def action(name, settle=None):
            events.append((name, settle))
            if name == fail_action:
                raise RuntimeError('Injected action failure')

        def measure_after_crouch():
            events.append(('measure_after_crouch', None))
            if fail_measurement:
                raise RuntimeError('Injected missing post-crouch measurement')
            return [(100, 255, 410)]

        runner.approach = approach
        runner.action = action
        runner.measure_after_crouch = measure_after_crouch
        return runner, events

    def test_final_steps_need_no_visibility_before_crouching(self):
        runner, events = self.runner(['--ids', '0'])
        with self.assertRaisesRegex(RuntimeError, 'CALIBRATION_REQUIRED.*100, 255, 410'):
            runner.run()
        self.assertEqual([name for name, _ in events], [
            'prepare', 'approach', 'forward_one_step', 'forward_one_step',
            'clamp_left_crouch', 'measure_after_crouch'])
        self.assertFalse(runner.head_locked)

    def test_zero_final_steps_still_measures_only_after_crouching(self):
        runner, events = self.runner(['--ids', '1', '--final-steps', '0'])
        with self.assertRaisesRegex(RuntimeError, 'CALIBRATION_REQUIRED'):
            runner.run()
        self.assertEqual([name for name, _ in events], [
            'prepare', 'approach', 'clamp_right_crouch', 'measure_after_crouch'])

    def test_measurement_or_timeout_preserves_hold_and_prevents_grasp_and_transfer(self):
        for failure in (False, True):
            with self.subTest(timeout=failure):
                runner, events = self.runner(
                    ['--ids', '0', '1', '--demo-transfer'], fail_measurement=failure)
                runner.held_servos = {14: 177, 22: 577}
                with self.assertRaises(RuntimeError):
                    runner.run()
                self.assertEqual(runner.held_servos, {14: 177, 22: 577})
                self.assertEqual(events.count(('prepare', None)), 1)
                self.assertNotIn(('approach', 1), events)
                self.assertTrue({'stand', 'clamp_left', 'clamp_right',
                                 'clamp_right_crouch', 'turn_right', 'forward',
                                 'place_block'}.isdisjoint(name for name, _ in events))

    def test_demo_helper_retains_both_arms_and_original_order(self):
        runner, events = self.runner(['--ids', '0', '1', '--demo-transfer'])
        runner.demo_transfer()
        names = [event[0] for event in events]
        self.assertEqual(runner.held_servos,
                         {i: 500 for i in runner.left_arm_ids + runner.right_arm_ids})
        self.assertEqual(events.count(('turn_right', 0.1)), 8)
        self.assertEqual(events.count(('forward', 0.0)), 12)
        self.assertEqual(names, ['turn_right'] * 8 + ['forward'] * 12)

    def test_hold_left_arm_uses_commanded_positions(self):
        runner, _ = self.runner(['--ids', '0', '1'])
        runner.motion.servo_position['21'] = 400
        runner.hold_left_arm()
        self.assertEqual(runner.held_servos, {13: 500, 15: 500, 17: 500, 19: 500, 21: 400})

    def test_failure_prevents_post_crouch_measurement(self):
        for failure in ({'fail_target': 0}, {'fail_action': 'clamp_left_crouch'}):
            with self.subTest(failure=failure):
                runner, events = self.runner(['--ids', '0', '1', '--demo-transfer'], **failure)
                with self.assertRaises(RuntimeError):
                    runner.run()
                self.assertNotIn(('measure_after_crouch', None), events)

    def test_prepare_and_approach_only_remain_available(self):
        for mode, expected in (('--prepare-only', ['prepare']),
                               ('--approach-only', ['prepare', 'approach'])):
            with self.subTest(mode=mode):
                runner, events = self.runner([mode])
                runner.run()
                self.assertEqual([name for name, _ in events], expected)


class DetectionTests(unittest.TestCase):
    @staticmethod
    def detection(tag_id, x=255, y=410):
        corner = SimpleNamespace(x=x, y=y)
        return SimpleNamespace(id=[tag_id], corners=[corner] * 4)

    def test_requested_id_selected_instead_of_first_detection(self):
        detections = [self.detection(0, 295), self.detection(1, 255)]
        self.assertEqual(grab.target_center(detections, 1), (255, 410))
        self.assertIsNone(grab.target_center(detections, 2))

    def test_duplicate_target_is_ambiguous(self):
        self.assertIsNone(grab.target_center([self.detection(1), self.detection(1)], 1))

    def test_frame_buffer_rejects_old_and_duplicate_frames(self):
        buffer = grab.FrameBuffer()
        buffer.reset(1, 100)
        def frame(stamp):
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                to_nsec=lambda: stamp)), detections=[self.detection(1)])
        buffer.push(frame(100), 1.0)
        self.assertIsNone(buffer.observation)
        buffer.push(frame(101), 2.0)
        first = buffer.observation
        buffer.push(frame(101), 3.0)
        self.assertIs(buffer.observation, first)
        buffer.reset(0, 200)
        self.assertIsNone(buffer.observation)


class PostCrouchMeasurementTests(unittest.TestCase):
    def runner(self, observations):
        runner = grab.Runner.__new__(grab.Runner)
        runner.args = grab.arguments([])
        runner.target = 1
        runner.ros = SimpleNamespace(
            is_shutdown=lambda: False, loginfo=lambda *args: None,
            Time=SimpleNamespace(now=lambda: SimpleNamespace(to_nsec=lambda: 100_000_000_000)))
        events = []
        stream = iter(observations)

        def wait_new(serial):
            obs = next(stream)
            events.append(obs)
            return obs

        runner.frames = SimpleNamespace(
            reset=lambda *args: events.append(('reset', *args)), wait_new=wait_new)
        return runner, events

    @staticmethod
    def observation(serial, xy=(255, 410), received=10, stamp=None):
        return grab.Observation(serial, received, 100_000_000_000 + serial
                                if stamp is None else stamp, xy)

    def test_invalid_or_old_observations_reset_measurement_series(self):
        for invalid in (self.observation(2, xy=None),
                        self.observation(2, received=8),
                        self.observation(2, stamp=98_000_000_000),
                        self.observation(2, stamp=101_000_000_000),
                        self.observation(2, xy=(640, 410))):
            with self.subTest(invalid=invalid):
                observations = [self.observation(1), invalid] + [
                    self.observation(i) for i in (3, 4, 5)]
                runner, events = self.runner(observations)
                with patch.object(grab.time, 'monotonic', return_value=10):
                    result = runner.measure_after_crouch()
                self.assertEqual(events, observations)
                self.assertEqual(result, [(100_000_000_000 + i, 255, 410)
                                          for i in (3, 4, 5)])

    def test_missing_measurement_expires_without_motion(self):
        runner, events = self.runner([])
        runner.args.lost_timeout = 1
        with patch.object(grab.time, 'monotonic', side_effect=[10, 12]):
            with self.assertRaisesRegex(RuntimeError, 'нет серии свежих измерений после подсаживания'):
                runner.measure_after_crouch()
        self.assertEqual(events, [])

    def test_measurements_use_no_standing_bounds_or_position_stability_threshold(self):
        observations = [self.observation(i, xy=xy) for i, xy in
                        enumerate(((1, 1), (639, 479), (320, 240)), 1)]
        runner, _ = self.runner(observations)
        with patch.object(grab.time, 'monotonic', return_value=10):
            self.assertEqual(runner.measure_after_crouch(),
                             [(obs.stamp, *obs.xy) for obs in observations])


class CrouchActionTests(unittest.TestCase):
    def test_dense_format_and_exact_source_prefix_fingerprints(self):
        for side, (count, _, fingerprint) in CROUCH.items():
            with self.subTest(side=side):
                columns, rows, integrity = read_action(ACTIONS / ('clamp_' + side + '_crouch.d6a'))
                self.assertEqual(integrity, [('ok',)])
                self.assertEqual(columns, ['Index', 'Time'] + ['Servo%d' % i for i in range(1, 23)])
                self.assertEqual([row[0] for row in rows], list(range(1, count + 1)))
                self.assertEqual([row[1] for row in rows], [1000] * count)
                self.assertTrue(all(type(value) is int for row in rows for value in row))
                digest = hashlib.sha256(json.dumps(rows, separators=(',', ':')).encode()).hexdigest()
                self.assertEqual(digest, fingerprint)

    @unittest.skipUnless(all((SOURCE / ('clamp_' + side + '.d6a')).is_file()
                             for side in CROUCH), 'Ignored robot snapshot is not installed')
    def test_exact_prefix_comparison_and_originals_unchanged(self):
        for side, (count, source_digest, _) in CROUCH.items():
            with self.subTest(side=side):
                path = SOURCE / ('clamp_' + side + '.d6a')
                before = path.read_bytes()
                self.assertEqual(hashlib.sha256(before).hexdigest(), source_digest)
                source_columns, source_rows, _ = read_action(path)
                columns, rows, _ = read_action(ACTIONS / ('clamp_' + side + '_crouch.d6a'))
                self.assertEqual(columns, source_columns)
                self.assertEqual(rows, source_rows[:count])
                self.assertEqual(path.read_bytes(), before)

    @unittest.skipUnless(MOTION_MANAGER.is_file(), 'Ignored vendor MotionManager is not installed')
    def test_real_parser_guard_and_frames_only_after_crouch_and_settle(self):
        # Import definitions with inert ROS modules; never create ROS nodes,
        # publishers, services, a real manager or servo output.
        modules = {name: types.ModuleType(name) for name in (
            'rospy', 'ros_robot_controller', 'ros_robot_controller.srv',
            'ros_robot_controller.msg')}
        modules['ros_robot_controller.srv'].GetBusServosPosition = object
        modules['ros_robot_controller.msg'].SetBusServosPosition = object
        modules['ros_robot_controller.msg'].BusServoPosition = object
        spec = importlib.util.spec_from_file_location('tag_grab_offline_motion_manager', MOTION_MANAGER)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        for target, side in enumerate(('left', 'right')):
            with self.subTest(side=side):
                name = 'clamp_' + side + '_crouch'
                path = ACTIONS / (name + '.d6a')
                before = path.read_bytes()
                _, rows, _ = read_action(path)
                runner = grab.Runner.__new__(grab.Runner)
                runner.args = grab.arguments(['--execute', '--ids', str(target)])
                runner.target = target
                runner.frames = grab.FrameBuffer()
                runner.frames.reset(target, 100_000_000_000)
                now = [100_000_000_000]
                runner.ros = SimpleNamespace(
                    is_shutdown=lambda: False, loginfo=lambda *args: None,
                    Time=SimpleNamespace(now=lambda: SimpleNamespace(to_nsec=lambda: now[0])))
                occupied_ids = (14, 16, 18, 20, 22) if target == 0 else (13, 15, 17, 19, 21)
                runner.held_servos = {sid: 400 + sid for sid in occupied_ids}
                held = dict(runner.held_servos)
                runner.head_locked = False
                manager = module.MotionManager.__new__(module.MotionManager)
                manager.action_path = str(ACTIONS)
                manager.servo_position = {}
                runner.motion = manager
                calls, events = [], []

                def frame(stamp, xy=(255, 410)):
                    return SimpleNamespace(
                        header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda: stamp)),
                        detections=[DetectionTests.detection(target, *xy)])

                def send(duration, positions):
                    calls.append((duration, positions))
                    manager.servo_position.update({str(sid): pulse for sid, pulse in positions})
                    now[0] += duration * 1_000_000
                    runner.frames.push(frame(now[0]), 10)
                    events.append('motion')

                def settle(seconds):
                    self.assertEqual(seconds, runner.args.settle)
                    now[0] += int(seconds * 1e9)
                    runner.frames.push(frame(now[0]), 10)
                    events.append('settle')

                runner.original_set_positions = send
                manager.set_servos_position = runner.guarded_positions
                runner.sleep = settle
                with patch.object(module.time, 'sleep') as sleep:
                    runner.action(name)
                self.assertEqual(calls, [(row[1], [[sid, held.get(sid, row[sid + 1])]
                                                 for sid in range(1, 23)]) for row in rows])
                self.assertEqual([call.args[0] for call in sleep.call_args_list], [1.0] * len(rows))
                self.assertEqual(events, ['motion'] * len(rows) + ['settle'])
                self.assertEqual(runner.held_servos, held)
                self.assertIsNone(runner.frames.observation)
                self.assertEqual(runner.frames.cutoff, now[0])
                for stamp in (now[0] - 1_000_000_000, now[0]):
                    runner.frames.push(frame(stamp), 10)
                    self.assertIsNone(runner.frames.observation)

                original_wait = runner.frames.wait_new
                stamps = iter(now[0] + i for i in (1, 2, 3))

                def fresh_sample(serial):
                    stamp = next(stamps)
                    runner.frames.push(frame(stamp), 10)
                    obs = runner.frames.observation
                    runner.frames.push(frame(stamp), 10)  # duplicate cannot add a sample
                    self.assertIs(runner.frames.observation, obs)
                    return original_wait(serial)

                runner.frames.wait_new = fresh_sample
                with patch.object(grab.time, 'monotonic', return_value=10):
                    measurements = runner.measure_after_crouch()
                self.assertEqual(measurements, [(now[0] + i, 255, 410) for i in (1, 2, 3)])
                self.assertEqual(runner.held_servos, held)
                self.assertFalse(manager.runningAction)
                self.assertEqual(path.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
