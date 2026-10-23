"""Pure tests: no ROS imports, no hardware and no genuine ActionGroups needed."""
import importlib.util
from contextlib import closing
from pathlib import Path
import sqlite3
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

MODULE_PATH = (Path(__file__).resolve().parents[1] / 'simulation/ros_ws/src/robotrio_sim'
               / 'src/robotrio_sim/action_runtime.py')
spec = importlib.util.spec_from_file_location('robotrio_action_runtime_test', MODULE_PATH)
runtime = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runtime
spec.loader.exec_module(runtime)


def load_server_without_ros():
    """Only import definitions; constructors, ROS transport and robot stay absent."""
    modules = {}
    for name in ('rospy', 'yaml', 'ainex_kinematics', 'ainex_kinematics.motion_manager',
                 'ros_robot_controller', 'ros_robot_controller.msg', 'ros_robot_controller.srv',
                 'rosgraph_msgs', 'rosgraph_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
                 'std_msgs', 'std_msgs.msg', 'robotrio_sim', 'robotrio_sim.srv'):
        modules[name] = types.ModuleType(name)
    ros = modules['rospy']
    ros.is_shutdown = lambda: False
    ros.get_name = lambda: '/test_sim_server'
    ros.logerr = lambda *args: None
    ros.logwarn = lambda *args: None
    modules['ainex_kinematics'].motion_manager = modules['ainex_kinematics.motion_manager']
    for module, names in (
            ('ros_robot_controller.msg', ('BusServoPosition', 'SetBusServosPosition')),
            ('ros_robot_controller.srv', ('GetBusServosPosition', 'GetBusServosPositionResponse')),
            ('rosgraph_msgs.msg', ('Clock',)), ('sensor_msgs.msg', ('JointState',)),
            ('std_msgs.msg', ('Float64',)),
            ('robotrio_sim.srv', ('ActionCommand', 'ActionCommandResponse'))):
        for name in names:
            setattr(modules[module], name, type(name, (), {
                '__init__': lambda self, **kwargs: self.__dict__.update(kwargs)}))
    modules['robotrio_sim.action_runtime'] = runtime
    path = MODULE_PATH.parents[2] / 'scripts/action_server.py'
    definition = importlib.util.spec_from_file_location('robotrio_action_server_test', path)
    server = importlib.util.module_from_spec(definition)
    with patch.dict(sys.modules, modules):
        definition.loader.exec_module(server)
    return server


server = load_server_without_ros()


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.config = {'controllers': {
            str(i): {'joint_name': 'joint_' + str(i),
                     'servo': {'id': i, 'init': 500, 'min': 1000 if i == 2 else 0,
                               'max': 0 if i == 2 else 1000}}
            for i in range(1, 25)}}
        self.xml = '<robot>' + ''.join(
            '<joint name="joint_%d" type="revolute"><limit lower="-2" upper="2"/></joint>' % i
            for i in range(1, 25)) + '</robot>'
        self.servos = runtime.ServoMap(self.config, self.xml)

    def test_exact_conversion_and_flipped_inverse(self):
        targets = self.servos.targets(((1, 600), (2, 600)), 0.1)
        self.assertAlmostEqual(targets['joint_1'], 100 / runtime.ENCODER_TICKS_PER_RADIAN)
        self.assertAlmostEqual(targets['joint_2'], -targets['joint_1'])
        self.assertEqual(self.servos.pulses(targets, (1, 2)), [(1, 600), (2, 600)])

    def test_invalid_calibration_cannot_silently_choose_a_direction(self):
        for field, value in [('min', float('nan')), ('max', '1000'),
                             ('init', -1), ('min', 1000)]:
            with self.subTest(field=field, value=value):
                config = {'controllers': {name: {**entry, 'servo': entry['servo'].copy()}
                                          for name, entry in self.config['controllers'].items()}}
                config['controllers']['1']['servo'][field] = value
                with self.assertRaises(runtime.ValidationError):
                    runtime.ServoMap(config, self.xml)

    def test_spawn_pose_offsets_preserve_original_physical_angles(self):
        import xml.etree.ElementTree as ET
        robot = ET.fromstring(self.xml)
        shifts = {1: 100 / runtime.ENCODER_TICKS_PER_RADIAN,
                  2: -100 / runtime.ENCODER_TICKS_PER_RADIAN}
        for sid, shift in shifts.items():
            joint = robot.find("joint[@name='joint_%d']" % sid)
            joint.set('robotrio_initial_angle', str(shift))
            joint.find('limit').set('lower', str(-2 - shift))
            joint.find('limit').set('upper', str(2 - shift))
        rebased = runtime.ServoMap(self.config, ET.tostring(robot, encoding='unicode'))
        for sid, shift in shifts.items():
            self.assertAlmostEqual(rebased.targets(((sid, 600),), 0)['joint_' + str(sid)], 0)
            for pulse in (400, 500, 600, 700):
                original = self.servos.targets(((sid, pulse),), 0)['joint_' + str(sid)]
                shifted = rebased.targets(((sid, pulse),), 0)['joint_' + str(sid)]
                self.assertAlmostEqual(original, shift + shifted)
        self.assertEqual(self.config['controllers']['1']['servo']['init'], 500)

    def test_atomic_validation_rejects_bad_entries(self):
        for positions, duration in [(((1, 500), (25, 500)), 1),
                                    (((1, 500), (1, 600)), 1),
                                    (((1, float('nan')),), 1),
                                    (((1, 1000),), 1),  # beyond URDF limit
                                    (((1, 500),), float('nan'))]:
            with self.subTest(positions=positions, duration=duration):
                with self.assertRaises(runtime.ValidationError):
                    self.servos.targets(positions, duration)

    def fixture(self, directory, rows):
        path = Path(directory) / 'synthetic_validation_only.d6a'
        with closing(sqlite3.connect(str(path))) as db:
            db.execute('CREATE TABLE ActionGroup ("Index" INT, Time INT, Servo1 INT)')
            db.executemany('INSERT INTO ActionGroup VALUES (?, ?, ?)', rows)
            db.commit()
        return path

    def test_reads_all_rows_in_explicit_index_order_without_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.fixture(directory, [(2, 120, 550), (1, 100, 500)])
            before = path.read_bytes()
            plan = runtime.load_action(directory, path.stem, self.servos)
            self.assertEqual([f.index for f in plan.frames], [1, 2])
            self.assertEqual(plan.summary()['duration_seconds'], 0.22)
            self.assertEqual(before, path.read_bytes())
            self.assertFalse(plan.summary()['grasp_verified'])

    def test_invalid_later_frame_cannot_pass_preflight(self):
        for rows in ([(1, 100, 500), (2, -1, 500)],
                     [(1, 100, 500), (1, 100, 550)],
                     [(1, 100, 500), (2, 100, 1001)]):
            with tempfile.TemporaryDirectory() as directory:
                path = self.fixture(directory, rows)
                with self.assertRaises(runtime.ValidationError):
                    runtime.load_action(directory, path.stem, self.servos)

    def test_path_traversal_and_external_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryDirectory() as outside:
            path = self.fixture(outside, [(1, 100, 500)])
            (Path(directory) / 'external.d6a').symlink_to(path)
            for name in ('../synthetic_validation_only', 'external'):
                with self.assertRaises((runtime.ValidationError, FileNotFoundError)):
                    runtime.load_action(directory, name, self.servos)

    def test_one_owner_and_stop_only_current_generation(self):
        gate = runtime.ActionGate()
        self.assertFalse(gate.stop())
        self.assertTrue(gate.acquire())
        self.assertFalse(gate.acquire())
        self.assertTrue(gate.stop())
        self.assertTrue(gate.stop_event.is_set())
        gate.release()
        self.assertTrue(gate.acquire())
        self.assertFalse(gate.stop_event.is_set())
        gate.release()

    def bridge(self):
        bridge = server.Bridge.__new__(server.Bridge)
        bridge.gate = runtime.ActionGate()
        bridge.gate.acquire()
        bridge.servos = self.servos
        bridge.lock = threading.RLock()
        bridge.watchdog = 2.0
        bridge.clock = 0.0
        bridge.clock_wall = time.monotonic()
        bridge.joints = {'joint_1': 0.05}
        bridge.joint_wall = {'joint_1': time.monotonic()}
        values = []
        bridge.publishers = {'joint_1': types.SimpleNamespace(
            publish=lambda msg: values.append(msg.data))}
        receipt = server.Receipt((1.0, ((1, 500),)))
        bridge.pending = receipt
        bridge.job = {'receipt': receipt, 'targets': {'joint_1': 0.5},
                      'start': {'joint_1': 0.0}, 'time': 0.0, 'duration': 1.0}
        return bridge, receipt, values

    def test_interpolation_completion_follows_simulated_clock(self):
        bridge, receipt, values = self.bridge()
        bridge.clock = 0.25
        bridge.tick()
        self.assertAlmostEqual(values[-1], 0.125)
        self.assertFalse(receipt.done.is_set())
        bridge.clock = 1.0
        bridge.tick()
        self.assertAlmostEqual(values[-1], 0.5)
        self.assertTrue(receipt.done.is_set())
        self.assertIsNone(receipt.error)

    def test_paused_clock_aborts_and_holds_latest_measured_position(self):
        bridge, receipt, values = self.bridge()
        bridge.clock_wall -= 3.0
        bridge.tick()
        self.assertTrue(receipt.done.is_set())
        self.assertIn('paused/stale', receipt.error)
        self.assertAlmostEqual(values[-1], 0.05)
        self.assertIsNone(bridge.job)

    def test_stop_and_clock_reset_latch_until_next_playback(self):
        bridge, receipt, values = self.bridge()
        bridge.gate.stop()
        bridge.tick()
        self.assertIn('stopped', receipt.error)
        self.assertAlmostEqual(values[-1], 0.05)
        bridge.gate.stop_event.clear()
        bridge.clock = 1.0
        bridge.on_clock(types.SimpleNamespace(clock=types.SimpleNamespace(to_sec=lambda: 0.0)))
        self.assertTrue(bridge.gate.stop_event.is_set())

    def test_sleep_facade_has_no_second_duration_and_honors_stop(self):
        bridge = types.SimpleNamespace(health=lambda: None)
        gate = runtime.ActionGate()
        facade = server.PlaybackSleep(bridge, gate)
        with patch.object(server.time, 'sleep') as sleep:
            facade.sleep(123.0)
            sleep.assert_not_called()
        gate.acquire()
        gate.stop()
        with self.assertRaises(RuntimeError):
            facade.sleep(123.0)
        gate.release()

    def test_frame_ack_is_required_before_completion_counter(self):
        player = server.ActionServer.__new__(server.ActionServer)
        player.gate, player.servos = runtime.ActionGate(), self.servos
        player.plan = types.SimpleNamespace(frames=(runtime.Frame(1, 100, ((1, 500),)),))
        player.cursor, player.acknowledged_total = 0, 0
        player.original_positions = lambda *_args: None

        def rejected(_receipt):
            raise RuntimeError('interpolation did not complete')

        player.bridge = types.SimpleNamespace(reserve=lambda _value: object(), wait_receipt=rejected)
        with self.assertRaises(RuntimeError):
            player.guarded_positions(100, [[1, 500]])
        self.assertEqual((player.cursor, player.acknowledged_total), (0, 0))


if __name__ == '__main__':
    unittest.main()
