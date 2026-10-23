"""Pure validation/conversion used by the simulation ActionGroup server.

ActionGroups contain sequential servo columns, not arbitrary servo-ID columns:
the third SQL column maps to servo 1, as in Hiwonder MotionManager.run_action.
"""
from dataclasses import dataclass
from contextlib import closing
import hashlib
import math
from pathlib import Path
import sqlite3
import threading
import xml.etree.ElementTree as ET


# Exact expression from upstream ainex_controller.Controller, not a servo guess.
ENCODER_TICKS_PER_RADIAN = 180 / 3.1415926 / 240 * 1000


class ValidationError(ValueError):
    pass


def integer(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(label + ' must be an integer')
    if not math.isfinite(value) or int(value) != value:
        raise ValidationError(label + ' must be a finite integer')
    return int(value)


@dataclass(frozen=True)
class Servo:
    name: str
    initial: float
    coefficient: float
    lower: float
    upper: float


class ServoMap:
    def __init__(self, config, robot_xml):
        robot = ET.fromstring(robot_xml)
        limits = {}
        initial_angles = {}
        for joint in robot.findall('joint'):
            limit = joint.find('limit')
            if joint.get('type') == 'revolute' and limit is not None:
                limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
                initial_angles[joint.get('name')] = float(joint.get('robotrio_initial_angle', '0'))
        self.servos = {}
        names = set()
        for controller in config.get('controllers', {}).values():
            if not isinstance(controller, dict) or 'servo' not in controller:
                continue
            raw = controller['servo']
            sid = integer(raw['id'], 'servo ID')
            name = controller.get('joint_name')
            if sid in self.servos or name in names or name not in limits:
                raise ValidationError('Duplicate or unmapped servo/joint: %s/%s' % (sid, name))
            initial = integer(raw['init'], 'servo initial pulse')
            raw_min = integer(raw['min'], 'servo calibration min')
            raw_max = integer(raw['max'], 'servo calibration max')
            if not all(0 <= value <= 1000 for value in (initial, raw_min, raw_max)) or raw_min == raw_max:
                raise ValidationError('Calibration pulses must be 0..1000 with distinct min/max')
            coefficient = ENCODER_TICKS_PER_RADIAN * (-1 if raw_min > raw_max else 1)
            initial += initial_angles[name] * coefficient
            lower, upper = limits[name]
            if not all(math.isfinite(v) for v in (initial, lower, upper)) or lower >= upper:
                raise ValidationError('Invalid calibration/limits for ' + str(name))
            self.servos[sid] = Servo(name, initial, coefficient, lower, upper)
            names.add(name)
        if set(self.servos) != set(range(1, 25)):
            raise ValidationError('Simulation requires exactly servo IDs 1..24')

    def targets(self, positions, duration):
        if not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration < 0:
            raise ValidationError('Duration must be finite and nonnegative (seconds)')
        if not positions:
            raise ValidationError('Empty servo frame')
        result = {}
        seen = set()
        for sid, pulse in positions:
            sid = integer(sid, 'servo ID')
            pulse = integer(pulse, 'servo pulse')
            if sid not in self.servos or sid in seen:
                raise ValidationError('Unknown/duplicate servo ID: ' + str(sid))
            if not 0 <= pulse <= 1000:
                raise ValidationError('Pulse outside 0..1000 for servo ' + str(sid))
            servo = self.servos[sid]
            angle = (pulse - servo.initial) / servo.coefficient
            if angle < servo.lower - 1e-7 or angle > servo.upper + 1e-7:
                raise ValidationError('Servo %d (%s): pulse %d -> %.6f rad outside [%.6f, %.6f]' %
                                      (sid, servo.name, pulse, angle, servo.lower, servo.upper))
            result[servo.name] = angle
            seen.add(sid)
        return result  # no command is emitted until every entry has passed

    def pulses(self, joint_positions, ids):
        result = []
        for sid in ids:
            if sid not in self.servos:
                raise ValidationError('Unknown servo ID: ' + str(sid))
            servo = self.servos[sid]
            if servo.name not in joint_positions:
                raise ValidationError('No measured position for ' + servo.name)
            pulse = int(round(servo.initial + joint_positions[servo.name] * servo.coefficient))
            if not 0 <= pulse <= 1000:
                raise ValidationError('Measured servo pulse outside 0..1000 for ' + servo.name)
            result.append((sid, pulse))
        return result


@dataclass(frozen=True)
class Frame:
    index: int
    duration_ms: int
    positions: tuple


@dataclass(frozen=True)
class ActionPlan:
    path: Path
    order_column: str
    frames: tuple
    sha256: str

    def summary(self):
        return {'name': self.path.stem, 'path': str(self.path), 'frames': len(self.frames),
                'duration_seconds': sum(f.duration_ms for f in self.frames) / 1000.0,
                'servo_columns': len(self.frames[0].positions), 'sha256': self.sha256,
                'order_column': self.order_column, 'grasp_verified': False}


def action_path(directory, name):
    if not name or '\x00' in name or Path(name).name != name or name in ('.', '..'):
        raise ValidationError('Action name must be a basename without directory components')
    if name.endswith('.d6a'):
        name = name[:-4]
    root = Path(directory).expanduser().resolve(strict=True)
    path = (root / (name + '.d6a')).resolve(strict=True)
    if path.parent != root or not path.is_file():
        raise ValidationError('Action must be a regular .d6a inside the selected directory')
    return path


def quoted_identifier(value):
    return '"' + value.replace('"', '""') + '"'


def load_action(directory, name, servos):
    path = action_path(directory, name)
    with closing(sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)) as db:
        columns = db.execute('PRAGMA table_info(ActionGroup)').fetchall()
        if not 3 <= len(columns) <= 26:
            raise ValidationError('ActionGroup must contain index, milliseconds and 1..24 servo columns')
        order_column = columns[0][1]
        rows = db.execute('SELECT * FROM ActionGroup ORDER BY ' + quoted_identifier(order_column)).fetchall()
    if not rows:
        raise ValidationError('Empty ActionGroup')
    frames = []
    previous = -1
    for row in rows:
        index = integer(row[0], 'frame index')
        duration_ms = integer(row[1], 'frame duration')
        if index < 0 or index <= previous or duration_ms <= 0:
            raise ValidationError('Frame indices must increase uniquely; durations must be positive milliseconds')
        positions = tuple((sid, integer(pulse, 'servo pulse')) for sid, pulse in enumerate(row[2:], 1))
        servos.targets(positions, duration_ms / 1000.0)
        frames.append(Frame(index, duration_ms, positions))
        previous = index
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return ActionPlan(path, order_column, tuple(frames), digest)


class ActionGate:
    """One playback owner, with stop visible to the currently active action."""
    def __init__(self):
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.state_lock = threading.Lock()
        self.active = False

    def acquire(self):
        with self.state_lock:
            if not self.lock.acquire(blocking=False):
                return False
            self.stop_event.clear()
            self.active = True
            return True

    def release(self):
        with self.state_lock:
            self.active = False
            self.lock.release()

    def stop(self):
        with self.state_lock:
            if self.active:
                self.stop_event.set()
            return self.active
