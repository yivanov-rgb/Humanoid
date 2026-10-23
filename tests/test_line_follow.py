"""Standalone line follower regressions; fixtures never move a robot."""
from contextlib import redirect_stderr
import importlib.util
import io
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('line_follow_under_test',
                                             Path(__file__).resolve().parents[1] / 'line_follow.py')
line = importlib.util.module_from_spec(spec)
spec.loader.exec_module(line)

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


def arguments(*argv):
    return line.arguments(list(argv))


def follower(**overrides):
    runner = line.Follower.__new__(line.Follower)
    runner.args = arguments()
    for name, value in overrides.items():
        setattr(runner.args, name, value)
    runner.now = 10_000_000_000
    runner.ros = NS(is_shutdown=lambda: False, Time=NS(now=lambda: NS(to_nsec=lambda: runner.now)),
                    loginfo_throttle=Mock())
    runner.lock, runner.stopped = threading.Lock(), threading.Event()
    runner.image = (runner.now, time.monotonic(), 'forward_one_step', line.LineObservation(0, 0))
    runner.clock_reset, runner.backend = False, None
    runner.started = time.monotonic()
    return runner


class LineTests(unittest.TestCase):
    def test_robot_observation_is_default_and_execution_is_explicit(self):
        args = arguments()
        self.assertFalse(args.execute)
        self.assertEqual(args.backend, 'robot')
        self.assertIsNone(args.image_orientation)
        for argv in (['--execute'], ['--execute', '--empty-hands'],
                     ['--execute', '--image-orientation', 'standard'],
                     ['--frame-timeout', 'nan']):
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                arguments(*argv)

    def test_single_copied_file_help_needs_no_companion_or_ros_installation(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'line_follow.py'
            shutil.copyfile(Path(line.__file__).resolve(), target)
            result = subprocess.run([sys.executable, '-I', str(target), '--help'],
                                    cwd=directory, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--execute', result.stdout)
            self.assertNotIn('--servo-limits', result.stdout)
            self.assertEqual(list(Path(directory).iterdir()), [target])


    def test_mirrored_and_standard_images_turn_in_opposite_directions(self):
        standard, mirrored = arguments('--image-orientation', 'standard'), arguments('--image-orientation', 'mirrored')
        self.assertEqual(line.choose_action(line.LineObservation(0, 25), standard), 'turn_right')
        self.assertEqual(line.choose_action(line.LineObservation(0, 25), mirrored), 'turn_left')
        self.assertEqual(line.choose_action(line.LineObservation(0, -25), standard), 'turn_left')
        self.assertEqual(line.choose_action(line.LineObservation(0, 0), standard), 'forward_one_step')
        self.assertIsNone(line.choose_action(None, standard))

    def test_parallel_offset_uses_sidesteps_and_heading_takes_priority(self):
        args = arguments('--image-orientation', 'standard')
        for offset, expected in ((-0.1, 'move_left'), (0.1, 'move_right')):
            self.assertEqual(line.choose_action(line.LineObservation(offset, 0), args), expected)
        args.image_orientation = 'mirrored'
        self.assertEqual(line.choose_action(line.LineObservation(0.1, 0), args), 'move_left')
        args.image_orientation = 'standard'
        self.assertEqual(line.choose_action(line.LineObservation(-0.1, 25), args), 'turn_right')

    def test_observation_creates_no_motion_backend(self):
        runner = follower()
        runner.check = Mock(side_effect=(None, None, TimeoutError('TIME_LIMIT')))
        with patch.object(line, 'Backend') as backend, patch.object(line.time, 'sleep'):
            with self.assertRaisesRegex(TimeoutError, 'TIME_LIMIT'):
                runner.run()
        backend.assert_not_called()
        self.assertIsNone(runner.backend)
        runner.ros.loginfo_throttle.assert_called_once()

    def test_camera_receipt_and_source_stamp_must_both_be_fresh(self):
        runner = follower()
        for stamp, received in ((runner.now - 3_000_000_000, time.monotonic()),
                                (runner.now, time.monotonic() - 3), (0, time.monotonic()),
                                (runner.now + 200_000_000, time.monotonic())):
            runner.image = (stamp, received, 'forward_one_step', None)
            with self.subTest(stamp=stamp), self.assertRaisesRegex(RuntimeError, 'CAMERA_STALE'):
                runner.check()

    def test_stop_cancels_backend_and_prevents_next_action(self):
        runner = follower()
        runner.backend = NS(cancel=Mock(), play=Mock())
        runner.stop()
        runner.backend.cancel.assert_called_once()
        with self.assertRaises(InterruptedError):
            runner.check()
        runner.backend.play.assert_not_called()

    def test_image_clock_reset_inhibits_movement(self):
        runner = follower()
        runner.clock_reset = True
        with self.assertRaisesRegex(RuntimeError, 'timestamps went backwards'):
            runner.check()

    def test_previous_action_frame_cannot_start_the_next_step(self):
        runner = follower(frame_timeout=0.03)
        with self.assertRaisesRegex(RuntimeError, 'NO_STABLE_LINE'):
            runner.wait_decision(runner.now)

    def test_pre_window_receipt_cannot_be_reused_even_with_a_future_stamp(self):
        runner = follower(frame_timeout=0.03, stable_frames=1)
        runner.image = (runner.now + 1, time.monotonic() - 0.001, 'forward_one_step', None)
        with self.assertRaisesRegex(RuntimeError, 'NO_STABLE_LINE'):
            runner.wait_decision(runner.now)

    def test_three_distinct_new_stationary_frames_confirm_one_decision(self):
        runner = follower(frame_timeout=0.1)
        counter = [0]
        def receive(_delay):
            counter[0] += 1
            runner.image = (runner.now + counter[0], time.monotonic(), 'move_right', None)
        with patch.object(line.time, 'sleep', side_effect=receive):
            self.assertEqual(runner.wait_decision(runner.now), 'move_right')
        self.assertEqual(counter[0], runner.args.stable_frames)

    def test_no_stable_line_produces_no_motion(self):
        runner = follower(frame_timeout=0.03)
        runner.image = (runner.now + 1, time.monotonic(), None, None)
        runner.backend = NS(play=Mock())
        with self.assertRaisesRegex(RuntimeError, 'NO_STABLE_LINE'):
            runner.wait_decision(runner.now)
        runner.backend.play.assert_not_called()


    def test_tested_actions_execute_without_servo_configuration(self):
        args = arguments('--execute', '--empty-hands', '--image-orientation', 'standard')
        self.assertTrue(args.execute)
        self.assertFalse(hasattr(args, 'servo_limits'))

    def test_native_motion_manager_receives_action_and_unchanged_positions(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in line.ACTIONS:
                (Path(directory)/(name+'.d6a')).touch()
            args = arguments('--action-path', directory)
            native_send = Mock()
            motion = NS(set_servos_position=native_send, stop_action_group=Mock(),
                        servo_state_pub=NS(get_num_connections=lambda: 1))
            motion.run_action = Mock(side_effect=lambda name:
                motion.set_servos_position(200, [[1, 50], [2, 950]]))  # synthetic fixture
            factory = Mock(return_value=motion)
            ros = NS(get_param=lambda *a: False, wait_for_service=Mock(),
                     ServiceProxy=Mock(return_value=lambda cmd: NS(result=True)))
            modules = {'ainex_interfaces': NS(), 'ainex_interfaces.srv': NS(SetWalkingCommand=object()),
                       'ainex_kinematics': NS(),
                       'ainex_kinematics.motion_manager': NS(MotionManager=factory)}
            with patch.dict(sys.modules, modules):
                backend = line.Backend(args, ros)
                backend.play('move_left')
            motion.run_action.assert_called_once_with('move_left')
            native_send.assert_called_once_with(200, [[1, 50], [2, 950]])
            backend.cancel()
            with self.assertRaises(InterruptedError):
                motion.set_servos_position(200, [[1, 50]])
            native_send.assert_called_once()
            motion.stop_action_group.assert_called_once()

    def test_camera_error_cancels_native_action(self):
        backend = line.Backend.__new__(line.Backend)
        backend.abort, backend.check = threading.Event(), Mock()
        backend.timeout = 1
        backend.motion = NS(run_action=Mock(side_effect=RuntimeError('CAMERA_STALE')),
                            stop_action_group=Mock())
        with self.assertRaisesRegex(RuntimeError, 'CAMERA_STALE'):
            backend.play('forward_one_step')
        backend.motion.stop_action_group.assert_called_once()
        self.assertTrue(backend.abort.is_set())

    def test_only_tested_line_actions_can_be_requested(self):
        backend = line.Backend.__new__(line.Backend)
        backend.motion = NS(run_action=Mock())
        with self.assertRaisesRegex(ValueError, 'Unsupported line action'):
            backend.play('unlisted_action')
        backend.motion.run_action.assert_not_called()


@unittest.skipIf(cv2 is None, 'OpenCV/NumPy are provided by the simulation image')
class VisionTests(unittest.TestCase):
    def image(self, x=None, white=False):
        image = np.full((480, 640, 3), 0 if white else 255, dtype=np.uint8)
        if x is not None:
            cv2.rectangle(image, (x - 12, 320), (x + 12, 479),
                          (255, 255, 255) if white else (0, 0, 0), -1)
        return image

    def test_center_left_right_black_lines(self):
        args = arguments('--image-orientation', 'standard')
        for x, expected in ((320, 'forward_one_step'), (260, 'move_left'), (380, 'move_right')):
            with self.subTest(x=x):
                error = line.detect_line(self.image(x), args, cv2)
                self.assertEqual(line.choose_action(error, args), expected)

    def test_white_line(self):
        args = arguments('--line-color', 'white')
        self.assertAlmostEqual(line.detect_line(self.image(320, white=True), args, cv2).offset, 0, delta=0.02)

    def test_missing_line_and_solid_dark_roi_are_rejected(self):
        args = arguments()
        for image in (self.image(), self.image(white=True)):
            self.assertIsNone(line.detect_line(image, args, cv2))

    def test_two_competing_tracks_are_rejected(self):
        image = self.image(280)
        cv2.rectangle(image, (348, 320), (372, 479), (0, 0, 0), -1)
        self.assertIsNone(line.detect_line(image, arguments(), cv2))

    def test_sloping_line_turns_instead_of_sidestepping(self):
        args = arguments('--image-orientation', 'standard')
        for far_x, expected in ((250, 'turn_left'), (390, 'turn_right')):
            image = self.image()
            cv2.line(image, (far_x, 320), (320, 479), (0, 0, 0), 20)
            detected = line.detect_line(image, args, cv2)
            self.assertIsNotNone(detected)
            self.assertEqual(line.choose_action(detected, args), expected)

    def test_curve_has_a_heading_signal_and_short_fragment_is_rejected(self):
        image = self.image()
        cv2.polylines(image, [np.array([[285, 370], [300, 405], [320, 445], [320, 479]])],
                      False, (0, 0, 0), 20)
        detected = line.detect_line(image, arguments(), cv2)
        self.assertIsNotNone(detected)
        self.assertLess(detected.heading, 0)
        fragment = self.image()
        cv2.rectangle(fragment, (308, 390), (332, 410), (0, 0, 0), -1)
        self.assertIsNone(line.detect_line(fragment, arguments(), cv2))

    def test_upper_image_is_ignored_while_near_feet_line_is_followed(self):
        args = arguments('--image-orientation', 'standard')
        upper_only = self.image()
        cv2.line(upper_only, (250, 150), (320, 300), (0, 0, 0), 20)
        self.assertIsNone(line.detect_line(upper_only, args, cv2))
        both = self.image(320)
        cv2.line(both, (250, 150), (320, 300), (0, 0, 0), 20)
        self.assertEqual(line.choose_action(line.detect_line(both, args, cv2), args),
                         'forward_one_step')


if __name__ == '__main__':
    unittest.main()
