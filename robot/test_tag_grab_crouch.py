"""Офлайн-проверки второго этапа: python3 -m unittest discover -s robot."""
import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import tag_grab_approach as approach
import tag_grab_crouch as crouch


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def sample(x=None, y=240, age=0, delay=0.1):
    return SimpleNamespace(x=x, y=y, age=age, delay=delay)


class Frames:
    def __init__(self, clock, samples):
        self.clock = clock
        self.samples = iter(samples)
        self.serial = 0
        self.cutoffs = []

    def reset(self, cutoff):
        self.cutoffs.append(cutoff)

    def wait_new(self, previous_serial):
        item = next(self.samples)
        self.clock.advance(item.delay)
        self.serial += 1
        target = None if item.x is None else approach.Target(item.x, item.y, 0)
        return approach.Observation(self.serial, self.clock(),
                                    int((self.clock() - item.age) * 1e9), target)


class CrouchTests(unittest.TestCase):
    def runner(self, samples, *options, execute=True, hand='left'):
        runner = crouch.Runner.__new__(crouch.Runner)
        runner.args = crouch.arguments(['--hand', hand]
                                      + (['--execute'] if execute else []) + list(options))
        clock = Clock()
        runner.frames = Frames(clock, samples)
        runner.ros = Mock()
        runner.ros.is_shutdown.return_value = False
        runner.ros.Time.now.side_effect = lambda: SimpleNamespace(
            to_nsec=lambda: int(clock() * 1e9))
        runner.motion = Mock()
        actions = []
        runner.motion.run_action.side_effect = lambda name: actions.append(
            (name, runner.frames.serial))
        runner.sleep = clock.advance
        return runner, clock, actions

    def run_sequence(self, runner, clock):
        with patch('tag_grab_crouch.time.monotonic', clock):
            runner.run()

    def test_both_directions_then_grasp_without_y_correction(self):
        for hand in ('left', 'right'):
            with self.subTest(hand=hand):
                runner, clock, actions = self.runner(
                    [sample(300, y=0)] * 3 + [sample(340, y=479)] * 3
                    + [sample(320, y=0)] * 3, hand=hand)
                self.run_sequence(runner, clock)
                self.assertEqual(actions, [('step_to_left', 3), ('step_to_right', 6),
                                           ('clamp_' + hand + '_from_crouch', 9)])
                # action() сбрасывает буфер после каждого движения и выдержки.
                self.assertEqual(len(runner.frames.cutoffs), 4)
                self.assertTrue(all(a < b for a, b in zip(
                    runner.frames.cutoffs, runner.frames.cutoffs[1:])))

    def test_missing_and_stale_targets_reset_streak(self):
        runner, clock, actions = self.runner(
            [sample(300)] * 2 + [sample()] + [sample(300)] * 3
            + [sample(320)] * 2 + [sample(320, age=2)] + [sample(320)] * 3)
        self.run_sequence(runner, clock)
        self.assertEqual(actions, [('step_to_left', 6), ('clamp_left_from_crouch', 12)])

    def test_gap_in_fresh_frames_resets_ready_streak(self):
        runner, clock, actions = self.runner(
            [sample(320)] * 2 + [sample(320, delay=1.1)] + [sample(320)] * 2)
        self.run_sequence(runner, clock)
        self.assertEqual(actions, [('clamp_left_from_crouch', 5)])

    def test_lost_target_and_step_limit_do_not_grasp(self):
        cases = (([sample(300), sample(340)], ['--max-moves', '1'], 'лимит'),
                 ([sample(300), sample(delay=11)], [], 'Потеряна'),
                 ([sample(300), sample(delay=2)], ['--timeout', '1'], 'время'))
        for samples, options, message in cases:
            with self.subTest(message=message):
                runner, clock, actions = self.runner(samples, '--stable-frames', '1', *options)
                with self.assertRaisesRegex(RuntimeError, message):
                    self.run_sequence(runner, clock)
                self.assertEqual(actions, [('step_to_left', 1)])

    def test_observation_does_not_step_or_grasp(self):
        runner, clock, actions = self.runner(
            [sample(300)] * 3 + [sample(320)] * 3 + [sample(delay=11)], execute=False)
        with self.assertRaisesRegex(RuntimeError, 'Потеряна'):
            self.run_sequence(runner, clock)
        self.assertEqual(actions, [])

    def test_observation_does_not_setup_motion(self):
        ros = Mock()
        ros.get_param.return_value = approach.TAG_FAMILY
        ros.wait_for_message.return_value = SimpleNamespace(width=640, height=480)
        modules = {'apriltag_ros.msg': SimpleNamespace(AprilTagDetectionArray=object),
                   'sensor_msgs.msg': SimpleNamespace(CameraInfo=object)}
        with patch.dict('sys.modules', modules), patch.object(crouch.Runner, 'setup_motion') as setup:
            runner = crouch.Runner(crouch.arguments(['--hand', 'left']), ros)
        setup.assert_not_called()
        self.assertIsNone(runner.motion)

    def test_shutdown_before_grasp_prevents_action(self):
        runner, clock, actions = self.runner([])
        runner.align = Mock(side_effect=lambda: setattr(runner.ros.is_shutdown, 'return_value', True))
        with self.assertRaises(InterruptedError):
            self.run_sequence(runner, clock)
        self.assertEqual(actions, [])

    def test_centered_defaults_inclusive_bounds_and_independent_hands(self):
        args = crouch.arguments(['--hand', 'left'])
        for hand in ('left', 'right'):
            self.assertEqual(crouch.bounds_for(hand, args), (310, 330))
        self.assertEqual([crouch.decide(x, (310, 330)) for x in (309, 310, 320, 330, 331)],
                         ['step_to_left', 'READY', 'READY', 'READY', 'step_to_right'])
        args = crouch.arguments(['--hand', 'right', '--right-x-min', '200', '--right-x-max', '220'])
        self.assertEqual(crouch.bounds_for('left', args), (310, 330))
        self.assertEqual(crouch.bounds_for('right', args), (200, 220))

    def test_invalid_arguments_rejected(self):
        for options in ([], ['--hand', 'left', '--left-x-min', 'nan'],
                        ['--hand', 'right', '--right-x-max', '640'],
                        ['--hand', 'left', '--left-x-min', '331'],
                        ['--hand', 'left', '--stable-frames', '0'],
                        ['--hand', 'left', '--max-age', 'inf']):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    crouch.arguments(options)

    def test_motion_action_sets_preserve_first_stage(self):
        for hand in ('left', 'right'):
            runner, _, _ = self.runner([], hand=hand)
            self.assertEqual(runner.motion_actions(),
                             {'step_to_left', 'step_to_right', 'clamp_' + hand + '_from_crouch'})
        runner = approach.Runner.__new__(approach.Runner)
        runner.args = approach.arguments(['--execute', '--prepare-only'])
        self.assertEqual(runner.motion_actions(), {'stand'})
        runner.args = approach.arguments([])
        self.assertEqual(runner.motion_actions(),
                         {'stand', 'move_left', 'move_right', 'forward_one_step', 'back_step',
                          'crouch', 'clamp_right_crouch'})


if __name__ == '__main__':
    unittest.main()
