"""Offline regression checks for the two single-arm, dense AiNex ActionGroups."""
from contextlib import closing
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import sys
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
ACTIONS = ROOT / 'robot/ActionGroups'
SOURCE = (ROOT / 'simulation/artifacts/d6a_from_robot/home/ubuntu/software'
          '/ainex_controller/ActionGroups/place_block.d6a')
MOTION_MANAGER = (ROOT / 'simulation/vendor/ainex/src/ainex_driver'
                  '/ainex_kinematics/src/ainex_kinematics/motion_manager.py')
SOURCE_SHA256 = '902702310bc4784490098f8ee9f9c9c024f4e1bcef493dd2e605cde9f9fa4422'
INACTIVE = {
    'left': {14: 165, 16: 170, 18: 520, 20: 930, 22: 570},
    'right': {13: 835, 15: 830, 17: 480, 19: 70, 21: 430},
}
# Fingerprints of Index, Time and ALL retained servo columns from the source.
# These checks also run in a clean checkout without the ignored robot snapshot.
RETAINED_SHA256 = {
    'left': 'fd51a5676f70d71d0623078d35d7e5b9c9aa32ea6554893f42ed81f8b5f8a07c',
    'right': 'ae4dac930b7f1b1166ebc6107c60380e993839927b3dac26e068a89f1f3c6976',
}


def read_action(path):
    with closing(sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)) as db:
        columns = [column[1] for column in db.execute('PRAGMA table_info(ActionGroup)')]
        rows = db.execute('SELECT * FROM ActionGroup').fetchall()
        integrity = db.execute('PRAGMA integrity_check').fetchall()
    return columns, rows, integrity


class PlaceBlockTests(unittest.TestCase):
    def test_dense_format_and_all_retained_source_values(self):
        for side, inactive in INACTIVE.items():
            with self.subTest(side=side):
                columns, rows, integrity = read_action(ACTIONS / ('place_block_' + side + '.d6a'))
                self.assertEqual(integrity, [('ok',)])
                self.assertEqual(columns, ['Index', 'Time'] + ['Servo%d' % i for i in range(1, 23)])
                self.assertEqual([row[0] for row in rows], list(range(1, 25)))
                self.assertEqual(sum(row[1] for row in rows), 10520)
                self.assertTrue(all(type(value) is int for row in rows for value in row))
                self.assertTrue(all(row[1] > 0 for row in rows))
                retained = [i for i, name in enumerate(columns)
                            if name not in {'Servo%d' % sid for sid in inactive}]
                projection = [[row[i] for i in retained] for row in rows]
                digest = hashlib.sha256(json.dumps(projection, separators=(',', ':')).encode()).hexdigest()
                self.assertEqual(digest, RETAINED_SHA256[side])

    def test_opposite_arm_never_changes_from_source_initial_pose(self):
        for side, inactive in INACTIVE.items():
            with self.subTest(side=side):
                _, rows, _ = read_action(ACTIONS / ('place_block_' + side + '.d6a'))
                for row in rows:
                    self.assertEqual({sid: row[sid + 1] for sid in inactive}, inactive)

    @unittest.skipUnless(SOURCE.is_file(), 'Ignored robot snapshot is not installed')
    def test_original_unchanged_and_exact_row_comparison(self):
        before = SOURCE.read_bytes()
        self.assertEqual(hashlib.sha256(before).hexdigest(), SOURCE_SHA256)
        source_columns, source_rows, _ = read_action(SOURCE)
        for side, inactive in INACTIVE.items():
            with self.subTest(side=side):
                columns, rows, _ = read_action(ACTIONS / ('place_block_' + side + '.d6a'))
                self.assertEqual(columns, source_columns)
                self.assertEqual(len(rows), len(source_rows))
                for original, actual in zip(source_rows, rows):
                    expected = list(original)
                    for sid in inactive:
                        expected[sid + 1] = source_rows[0][sid + 1]
                    self.assertEqual(actual, tuple(expected))
        self.assertEqual(SOURCE.read_bytes(), before)

    @unittest.skipUnless(MOTION_MANAGER.is_file(), 'Ignored vendor MotionManager is not installed')
    def test_real_motion_manager_parser_with_mocked_output_and_time(self):
        # Import definitions with inert ROS modules; never construct a manager,
        # publisher, service proxy, robot driver or ROS node.
        modules = {name: types.ModuleType(name) for name in (
            'rospy', 'ros_robot_controller', 'ros_robot_controller.srv',
            'ros_robot_controller.msg')}
        modules['ros_robot_controller.srv'].GetBusServosPosition = object
        modules['ros_robot_controller.msg'].SetBusServosPosition = object
        modules['ros_robot_controller.msg'].BusServoPosition = object
        spec = importlib.util.spec_from_file_location('offline_motion_manager', MOTION_MANAGER)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        for side in INACTIVE:
            with self.subTest(side=side):
                name = 'place_block_' + side
                path = ACTIONS / (name + '.d6a')
                before = path.read_bytes()
                _, rows, _ = read_action(path)
                calls = []
                manager = module.MotionManager.__new__(module.MotionManager)
                manager.action_path = str(ACTIONS)
                manager.set_servos_position = lambda duration, data: calls.append((duration, data))
                with patch.object(module.time, 'sleep') as sleep:
                    manager.run_action(name)
                self.assertEqual(calls, [(row[1], [[sid, row[sid + 1]] for sid in range(1, 23)])
                                         for row in rows])
                self.assertEqual([call.args[0] for call in sleep.call_args_list],
                                 [row[1] / 1000 for row in rows])
                self.assertFalse(manager.runningAction)
                self.assertEqual(path.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
