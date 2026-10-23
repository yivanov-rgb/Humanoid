"""Offline regressions. Importing tag_grab does not import ROS or create a Runner."""
import importlib.util
import io
from contextlib import redirect_stderr
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'tag_grab.py'
spec = importlib.util.spec_from_file_location('tag_grab_under_test', SCRIPT)
grab = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = grab
spec.loader.exec_module(grab)


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
    def runner(self, argv, fail_target=None, fail_action=None):
        # Bypass __init__: no ROS imports, subscriptions, services or hardware.
        runner = grab.Runner.__new__(grab.Runner)
        runner.args = grab.arguments(['--execute'] + argv)
        runner.ros = SimpleNamespace(loginfo=lambda *args: None)
        runner.motion = SimpleNamespace(servo_position={str(i): 500 for i in range(1, 25)})
        runner.left_arm_ids = (13, 15, 17, 19, 21)
        runner.right_arm_ids = (14, 16, 18, 20, 22)
        runner.held_servos = {}
        runner.head_locked = False
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

        runner.approach = approach
        runner.action = action
        return runner, events

    def test_both_clamps_precede_single_placement(self):
        runner, events = self.runner(['--ids', '0', '1'])
        runner.run()
        names = [event[0] for event in events]
        self.assertLess(names.index('clamp_left'), names.index('clamp_right'))
        self.assertLess(names.index('clamp_right'), names.index('place_block'))
        self.assertEqual(names.count('place_block'), 1)
        self.assertNotIn('turn_right', names)
        self.assertNotIn('forward', names)

    def test_demo_order_counts_and_turn_pause(self):
        runner, events = self.runner(['--ids', '0', '1', '--demo-transfer'])
        runner.run()
        names = [event[0] for event in events]
        self.assertEqual(events.count(('turn_right', 0.1)), 8)
        self.assertEqual(events.count(('forward', 0.0)), 12)
        self.assertLess(names.index('clamp_right'), names.index('turn_right'))
        self.assertLess(names.index('turn_right'), names.index('forward'))
        self.assertLess(names.index('forward'), names.index('place_block'))
        self.assertEqual(names.count('place_block'), 1)
        important = [name for name in names if name in {
            'clamp_left', 'clamp_right', 'turn_right', 'forward', 'place_block'}]
        self.assertEqual(important, ['clamp_left', 'clamp_right']
                         + ['turn_right'] * 8 + ['forward'] * 12 + ['place_block'])

    def test_single_grasp_does_not_place(self):
        runner, events = self.runner(['--ids', '1'])
        runner.run()
        self.assertIn(('clamp_right', None), events)
        self.assertNotIn(('place_block', None), events)

    def test_second_grasp_failure_prevents_transfer_and_placement(self):
        for failure in ({'fail_target': 1}, {'fail_action': 'clamp_right'}):
            with self.subTest(failure=failure):
                runner, events = self.runner(['--ids', '0', '1', '--demo-transfer'], **failure)
                with self.assertRaises(RuntimeError):
                    runner.run()
                self.assertTrue({'turn_right', 'forward', 'place_block'}.isdisjoint(
                    event[0] for event in events))


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


if __name__ == '__main__':
    unittest.main()
