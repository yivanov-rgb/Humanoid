#!/usr/bin/env python3
"""Simulation-only bus-servo adapter and single-owner .d6a playback server."""
import json
from collections import deque
import math
from pathlib import Path
import sqlite3
import threading
import time

import rospy
import yaml
from ainex_kinematics import motion_manager as vendor_motion
from ros_robot_controller.msg import BusServoPosition, SetBusServosPosition
from ros_robot_controller.srv import GetBusServosPosition, GetBusServosPositionResponse
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from robotrio_sim.action_runtime import ActionGate, ServoMap, ValidationError, load_action, quoted_identifier
from robotrio_sim.srv import ActionCommand, ActionCommandResponse


def signature(duration, positions):
    return round(float(duration), 9), tuple((int(sid), int(pulse)) for sid, pulse in positions)


class Receipt:
    def __init__(self, expected):
        self.expected = expected
        self.done = threading.Event()
        self.error = None
        self.received = False


class Bridge:
    def __init__(self, servos, gate, watchdog=2.0):
        self.servos, self.gate, self.watchdog = servos, gate, watchdog
        self.lock = threading.RLock()
        self.clock = None
        self.clock_wall = 0.0
        self.joints = {}
        self.joint_wall = {}
        self.pending = None
        self.job = None
        self.publishers = {servo.name: rospy.Publisher('/' + servo.name + '_controller/command',
                                                      Float64, queue_size=1)
                           for servo in servos.servos.values()}
        self.subscribers = [
            rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1),
            rospy.Subscriber('/joint_states', JointState, self.on_joints, queue_size=1),
            rospy.Subscriber('/ros_robot_controller/bus_servo/set_position', SetBusServosPosition,
                             self.on_command, queue_size=1),
        ]
        self.position_service = rospy.Service('/ros_robot_controller/bus_servo/get_position',
                                              GetBusServosPosition, self.get_positions)
        threading.Thread(target=self.worker, daemon=True).start()

    def on_clock(self, msg):
        value = msg.clock.to_sec()
        with self.lock:
            if self.clock is None or value > self.clock:
                self.clock_wall = time.monotonic()
            elif value < self.clock:
                self.gate.stop()  # latch a reset even if it falls between frames
                self.cancel('Simulation clock reset during command')
                self.clock_wall = time.monotonic()
            self.clock = value

    def on_joints(self, msg):
        now = time.monotonic()
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                if name in self.publishers and math.isfinite(value):
                    self.joints[name] = value
                    self.joint_wall[name] = now

    def health(self, names=None):
        with self.lock:
            now = time.monotonic()
            if self.clock is None or now - self.clock_wall > self.watchdog:
                raise RuntimeError('Simulated clock paused/stale; command aborted')
            for name in names if names is not None else self.publishers:
                if name not in self.joints or now - self.joint_wall[name] > self.watchdog:
                    raise RuntimeError('Joint feedback unavailable/stale: ' + name)
            if rospy.is_shutdown():
                raise RuntimeError('ROS shutdown')

    def ready(self):
        try:
            self.health()
            return all(pub.get_num_connections() > 0 for pub in self.publishers.values())
        except RuntimeError:
            return False

    def reserve(self, expected):
        with self.lock:
            self.health()
            if self.job or self.pending:
                raise RuntimeError('Bus-servo bridge is busy with another frame')
            self.pending = Receipt(expected)
            return self.pending

    def on_command(self, msg):
        receipt = None
        try:
            positions = tuple((p.id, p.position) for p in msg.position)
            targets = self.servos.targets(positions, msg.duration)
            with self.lock:
                self.health(targets)
                caller = (getattr(msg, '_connection_header', None) or {}).get('callerid', '')
                if self.gate.active and caller != rospy.get_name():
                    raise RuntimeError('Another publisher cannot move joints during server playback')
                if self.job:
                    raise RuntimeError('A servo frame is already active')
                if self.pending:
                    if caller != rospy.get_name():
                        raise RuntimeError('Reserved playback frame rejects another publisher')
                    receipt = self.pending
                    if signature(msg.duration, positions) != receipt.expected:
                        raise RuntimeError('Published frame differs from validated playback frame')
                else:
                    receipt = Receipt(signature(msg.duration, positions))
                receipt.received = True
                self.job = {'receipt': receipt, 'targets': targets,
                            'start': {name: self.joints[name] for name in targets},
                            'time': self.clock, 'duration': msg.duration}
        except Exception as error:
            with self.lock:
                # An unrelated rejected publisher must not abort the owner's frame.
                own = (getattr(msg, '_connection_header', None) or {}).get('callerid', '') == rospy.get_name()
                if own and self.pending:
                    self.pending.error = str(error)
                    self.pending.done.set()
                    self.pending = None
            rospy.logerr('SIM_FRAME_REJECTED: %s', error)

    def hold(self, names):
        for name in names:
            if name in self.joints:
                self.publishers[name].publish(Float64(data=self.joints[name]))

    def cancel(self, reason):
        with self.lock:
            if self.job:
                self.hold(self.job['targets'])
                self.job['receipt'].error = reason
                self.job['receipt'].done.set()
                self.job = None
            if self.pending:
                self.pending.error = reason
                self.pending.done.set()
                self.pending = None

    def worker(self):
        while not rospy.is_shutdown():
            self.tick()
            time.sleep(0.01)
        self.cancel('ROS shutdown')

    def tick(self):
        with self.lock:
            if self.job:
                try:
                    job = self.job
                    self.health(job['targets'])
                    if self.gate.active and self.gate.stop_event.is_set():
                        raise RuntimeError('Playback stopped by request')
                    elapsed = self.clock - job['time']
                    if elapsed < 0:
                        raise RuntimeError('Simulation clock moved backwards')
                    fraction = 1.0 if job['duration'] == 0 else min(1.0, elapsed / job['duration'])
                    for name, target in job['targets'].items():
                        value = job['start'][name] + fraction * (target - job['start'][name])
                        self.publishers[name].publish(Float64(data=value))
                    if fraction >= 1.0:
                        job['receipt'].done.set()
                        self.job = None
                        self.pending = None
                except Exception as error:
                    self.cancel(str(error))

    def wait_receipt(self, receipt):
        sent_at = time.monotonic()
        while not receipt.done.wait(0.01):
            self.health()
            if self.gate.stop_event.is_set():
                self.cancel('Playback stopped by request')
            if not receipt.received and time.monotonic() - sent_at > self.watchdog:
                self.cancel('No acknowledgment from bus-servo topic subscriber')
        if receipt.error:
            raise RuntimeError(receipt.error)

    def wait_sim(self, duration):
        self.health()
        with self.lock:
            start = self.clock
        while True:
            self.health()
            if self.gate.stop_event.is_set():
                raise RuntimeError('Playback stopped by request')
            with self.lock:
                if self.clock < start:
                    raise RuntimeError('Simulation clock moved backwards')
                if self.clock - start >= duration:
                    return
            time.sleep(0.01)

    def get_positions(self, req):
        try:
            with self.lock:
                names = [self.servos.servos[sid].name for sid in req.id]
                self.health(names)
                positions = self.servos.pulses(self.joints, req.id)
            return GetBusServosPositionResponse(
                success=True, position=[BusServoPosition(id=sid, position=pulse)
                                        for sid, pulse in positions])
        except Exception as error:
            rospy.logwarn('SIM_POSITION_READ_REJECTED: %s', error)
            return GetBusServosPositionResponse(success=False, position=[])


class ReadOnlyCursor:
    def __init__(self, cursor):
        self.cursor = cursor

    def execute(self, sql):
        if sql.strip().lower() == 'select * from actiongroup':
            columns = self.cursor.execute('PRAGMA table_info(ActionGroup)').fetchall()
            sql += ' ORDER BY ' + quoted_identifier(columns[0][1])
        return self.cursor.execute(sql)

    def fetchone(self):
        return self.cursor.fetchone()

    def close(self):
        return self.cursor.close()


class ReadOnlyConnection:
    def __init__(self, path):
        self.connection = sqlite3.connect(Path(path).resolve().as_uri() + '?mode=ro', uri=True)

    def cursor(self):
        return ReadOnlyCursor(self.connection.cursor())

    def close(self):
        self.connection.close()

    def __del__(self):
        try:
            self.connection.close()
        except Exception:
            pass


class PlaybackSleep:
    """Frame wrapper already waited simulated duration; never double-sleep."""
    def __init__(self, bridge, gate):
        self.bridge, self.gate = bridge, gate

    def sleep(self, _seconds):
        self.bridge.health()
        if self.gate.stop_event.is_set():
            raise RuntimeError('Playback stopped by request')


class ActionServer:
    def __init__(self):
        self.gate = ActionGate()
        self.owner_lock = threading.Lock()
        self.owner_token = None
        self.cancelled_tokens = deque(maxlen=256)
        self.default_path = rospy.get_param('~action_path', '/actions')
        startup_deadline = time.monotonic() + float(rospy.get_param('~startup_timeout', 120.0))
        while not rospy.is_shutdown():
            mode = rospy.get_param('/robotrio_sim/mode', '')
            if (mode in ('anchored_inspection', 'dynamic_experiment')
                    and rospy.get_param('/use_sim_time', False)
                    and rospy.has_param('/robot_description')):
                break
            if time.monotonic() >= startup_deadline:
                raise RuntimeError('Refusing non-simulation ROS master or missing simulation model')
            time.sleep(0.05)
        config = Path(rospy.get_param('~servo_config'))
        self.servos = ServoMap(yaml.safe_load(config.read_text()), rospy.get_param('/robot_description'))
        self.bridge = Bridge(self.servos, self.gate, float(rospy.get_param('~watchdog', 2.0)))
        self.motion = vendor_motion.MotionManager(action_path=self.default_path)
        self.original_positions = self.motion.set_servos_position
        self.motion.set_servos_position = self.guarded_positions
        # Isolated module facades leave Python's stdlib time/sqlite3 unchanged.
        vendor_motion.time = PlaybackSleep(self.bridge, self.gate)
        vendor_motion.sql = type('ReadOnlySQLite', (), {'connect': staticmethod(ReadOnlyConnection)})
        self.plan, self.cursor, self.acknowledged_total = None, 0, 0
        while not self.bridge.ready():
            if rospy.is_shutdown() or time.monotonic() >= startup_deadline:
                raise RuntimeError('Simulator clock, 24 joint feedback/controllers are not ready')
            time.sleep(0.05)
        self.service = rospy.Service('/robotrio_sim/actions', ActionCommand, self.command)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('SIM_ACTIONS_READY: path=%s, no actions run on startup', self.default_path)

    def shutdown(self):
        self.gate.stop()
        self.motion.stop_action_group()
        self.bridge.cancel('ROS shutdown')

    def guarded_positions(self, duration_ms, *positions):
        if self.gate.stop_event.is_set():
            raise RuntimeError('Playback stopped by request')
        if self.plan is None or self.cursor >= len(self.plan.frames):
            raise RuntimeError('Unexpected/unvalidated ActionGroup frame')
        frame = self.plan.frames[self.cursor]
        actual = tuple(tuple(p) for p in positions[0])
        if duration_ms != frame.duration_ms or actual != frame.positions:
            raise RuntimeError('ActionGroup changed after validation or playback order differs')
        self.servos.targets(actual, duration_ms / 1000.0)
        receipt = self.bridge.reserve(signature(duration_ms / 1000.0, actual))
        self.original_positions(duration_ms, positions[0])
        self.bridge.wait_receipt(receipt)
        self.cursor += 1
        self.acknowledged_total += 1

    def response(self, success, message, details=None):
        return ActionCommandResponse(success=success, message=message,
                                     json_result=json.dumps(details or {}, ensure_ascii=False))

    def command(self, req):
        directory = req.action_path or self.default_path
        try:
            if req.command == 'stop':
                with self.owner_lock:
                    if req.token and req.token != self.owner_token:
                        self.cancelled_tokens.append(req.token)
                        return self.response(True, 'Cancellation recorded; no matching active playback')
                    active = self.gate.stop()
                    self.motion.stop_action_group()
                    self.bridge.cancel('Playback stopped by request')
                return self.response(True, 'Stop requested' if active else 'No active server playback',
                                     {'active': active})
            if req.command == 'list':
                root = Path(directory).expanduser().resolve(strict=True)
                if not root.is_dir():
                    raise ValidationError('Action path is not a directory')
                return self.response(True, 'Action files',
                                     {'action_path': str(root),
                                      'actions': sorted(p.stem for p in root.glob('*.d6a') if p.is_file())})
            if req.command == 'info':
                return self.response(True, 'Validated ActionGroup',
                                     load_action(directory, req.name, self.servos).summary())
            if req.command != 'play':
                raise ValidationError('Unknown command: ' + req.command)
            if (rospy.get_param('/robotrio_sim/mode', '') == 'dynamic_experiment'
                    and not rospy.get_param('/robotrio_sim/initial_pose_ready', False)):
                raise ValidationError('Initial standing pose is not ready; actions are inhibited')
            if not req.token:
                raise ValidationError('Playback requires a client ownership token')
            if not 1 <= req.repeat <= 1000 or not math.isfinite(req.pause) or req.pause < 0:
                raise ValidationError('repeat must be 1..1000; pause finite and nonnegative')
            with self.owner_lock:
                if req.token in self.cancelled_tokens:
                    return self.response(False, 'Playback token was cancelled before starting')
                if not self.gate.acquire():
                    return self.response(False, 'Another action is already playing')
                self.owner_token = req.token
            completed = 0
            self.cursor, self.acknowledged_total = 0, 0
            try:
                self.plan = load_action(directory, req.name, self.servos)
                summary = self.plan.summary()
                self.motion.action_path = str(self.plan.path.parent)
                for index in range(req.repeat):
                    if self.gate.stop_event.is_set():
                        raise RuntimeError('Playback stopped by request')
                    self.cursor = 0
                    self.motion.run_action(self.plan.path.stem)
                    if self.cursor != len(self.plan.frames) or self.gate.stop_event.is_set():
                        raise RuntimeError('MotionManager returned without full acknowledged playback')
                    completed += 1
                    if index + 1 < req.repeat and req.pause:
                        self.bridge.wait_sim(req.pause)
                return self.response(True, 'All frames acknowledged in simulator; grasp is not verified',
                                     {**summary, 'completed_repeats': completed,
                                      'acknowledged_frames': self.acknowledged_total,
                                      'grasp_verified': False})
            except Exception as error:
                self.motion.stop_action_group()
                self.bridge.cancel(str(error))
                return self.response(False, str(error),
                                     {'completed_repeats': completed,
                                      'acknowledged_frames': self.acknowledged_total,
                                      'grasp_verified': False})
            finally:
                self.plan = None
                self.motion.runningAction = False  # upstream lacks exception cleanup
                with self.owner_lock:
                    self.owner_token = None
                    self.gate.release()
        except Exception as error:
            return self.response(False, str(error))


if __name__ == '__main__':
    rospy.init_node('robotrio_sim_actions')
    try:
        ActionServer()
        rospy.spin()
    except Exception as error:
        rospy.logfatal('SIM_ACTION_SERVER_FAILED: %s', error)
        raise SystemExit(1)
