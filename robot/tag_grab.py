#!/usr/bin/env python3
"""Robotrio/AiNex ROS1: подход и измерение блока после подсаживания.

Среда: существующий контейнер ainex, ROS Noetic, детектор HiWonder с corners.
По умолчанию только наблюдение: MotionManager даже не создаётся.
Голова при исполнении: pan=500, tilt=330. Подход по X/Y, затем
forward_one_step x2, родной присед и новое измерение.
После подшагов видимость метки не требуется: она возвращается после приседа.
ID 0 -> кадр 1 clamp_left; ID 1 -> кадры 1–2 clamp_right.
Затем --execute останавливается с CALIBRATION_REQUIRED: подтверждённого
критерия финального захвата пока нет. Свежие x/y записываются с timestamp,
но не разрешают захват и не являются TF-коррекцией или калибровкой.
При --ids 0 1 остановка происходит на первой цели; второй захват,
--demo-transfer и размещение пока недоступны. Робот остаётся в приседе.
IDs сервоприводов читаются из установленного servo_controller.yaml.
Во время подхода и двух финальных шагов голова зафиксирована в 500/330.
Для каждого ID используются только его явно выбранные пороги.
Возврат run_action означает конец воспроизведения, не обратную связь о позе.

Примеры внутри контейнера:
  python3 tag_grab.py --ids 0
  python3 tag_grab.py --execute --prepare-only
  python3 tag_grab.py --execute --ids 0 --approach-only
  python3 tag_grab.py --execute --ids 0
  python3 tag_grab.py --execute --ids 1 --approach-only
  python3 tag_grab.py --execute --ids 0 1
  python3 tag_grab.py --execute --ids 0 1 --demo-transfer

Настройка: --left-x-min 280 --left-x-max 310
          --right-x-min 240 --right-x-max 270
          --left-y-min 400 --left-y-max 420
          --right-y-min 400 --right-y-max 420
          --final-steps 2 --max-moves 40 --lost-timeout 10
NO_TARGET -> ожидание, затем выход; слепых шагов назад нет.
Ctrl+C запрещает следующие команды и просит остановку между кадрами .d6a.
Прежний сценарий двух захватов пользователь подтвердил с этими порогами.
Новая последовательность с паузой в приседе на роботе не проверена.
По проверке пользователя больший X соответствует положению левее:
для правой руки используется меньший X, чем для левой.
"""
import argparse
import math
import sqlite3
import threading
import time
from dataclasses import dataclass
from pathlib import Path

SCRIPT_VERSION = 'tag-grab-2026-10-23-post-crouch-measurement'


@dataclass(frozen=True)
class Observation:
    serial: int
    received: float
    stamp: int
    xy: object


def target_center(detections, target):
    """Одиночный ID; не берём первое обнаружение и не выбираем bundle."""
    matches = [d for d in detections if list(d.id) == [target]]
    if len(matches) != 1:
        return None
    corners = matches[0].corners
    if len(corners) != 4:
        return None
    x = (corners[0].x + corners[2].x) / 2.0
    y = (corners[0].y + corners[2].y) / 2.0
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


def bounds_for(target, args):
    if target not in (0, 1):
        raise ValueError('Неизвестный ID цели: ' + str(target))
    prefix = {0: 'left', 1: 'right'}[target]
    return tuple(getattr(args, prefix + '_' + axis + '_' + bound)
                 for axis, bound in (('x', 'min'), ('x', 'max'),
                                     ('y', 'min'), ('y', 'max')))


def decide(x, y, bounds):
    # Сохранены направления действий проверенного цветового скрипта.
    # X проверяется на каждой итерации, включая подход по Y.
    # Пороги передаются явно: не существует цели/руки по умолчанию.
    x_min, x_max, y_min, y_max = bounds
    if x > x_max:
        return "move_right"
    if x < x_min:
        return "move_left"
    if y < y_min:
        return "forward_one_step"
    if y > y_max:
        return "back_step"
    return "READY"


def protected_positions(positions, held):
    return [[servo_id, held.get(servo_id, pulse)]
            for servo_id, pulse in positions]


def arm_ids(config, side):
    if side not in ('left', 'right'):
        raise ValueError('Неизвестная рука: ' + str(side))
    joints = {}
    for controller in config['controllers'].values():
        if isinstance(controller, dict) and 'servo' in controller:
            joints[controller.get('joint_name')] = int(controller['servo']['id'])
    prefix = 'l' if side == 'left' else 'r'
    names = tuple(prefix + '_' + name for name in
                  ('sho_pitch', 'sho_roll', 'el_pitch', 'el_yaw', 'gripper'))
    missing = [name for name in names if name not in joints]
    if missing:
        raise RuntimeError('Нет суставов %s в servo_controller.yaml: %s'
                           % (side, missing))
    ids = tuple(joints[name] for name in names)
    if len(set(ids)) != 5 or any(i < 1 or i > 22 for i in ids):
        raise RuntimeError('Неверная карта сервоприводов %s: %s' % (side, ids))
    return ids


class FrameBuffer:
    """Храним последний кадр, включая пустой; исключаем кадры до движения."""
    def __init__(self):
        self.condition = threading.Condition()
        self.target = 0
        self.cutoff = 0
        self.last_stamp = 0
        self.serial = 0
        self.observation = None

    def reset(self, target, cutoff):
        with self.condition:
            self.target = target
            self.cutoff = cutoff
            self.last_stamp = cutoff
            self.observation = None

    def push(self, msg, received):
        stamp = msg.header.stamp.to_nsec()
        with self.condition:
            if stamp <= self.last_stamp:  # нулевые, старые и повторные кадры
                return
            self.last_stamp = stamp
            self.serial += 1
            self.observation = Observation(
                self.serial, received, stamp,
                target_center(msg.detections, self.target))
            self.condition.notify_all()

    def wait_new(self, previous_serial):
        with self.condition:
            if self.observation is None or self.observation.serial <= previous_serial:
                self.condition.wait(0.1)
            obs = self.observation
            return obs if obs and obs.serial > previous_serial else None


class Runner:
    def __init__(self, args, ros):
        from apriltag_ros.msg import AprilTagDetectionArray
        from sensor_msgs.msg import CameraInfo
        self.args = args
        self.ros = ros
        self.frames = FrameBuffer()
        self.motion = None
        self.held_servos = {}
        self.head_locked = False
        self.left_arm_ids = ()
        self.right_arm_ids = ()
        self.target = args.ids[0]
        self.sub = ros.Subscriber(
            args.topic, AprilTagDetectionArray, self.callback, queue_size=1)
        info = ros.wait_for_message(args.camera_info, CameraInfo, timeout=8)
        if (info.width, info.height) != (640, 480):
            raise RuntimeError(
                "Пороги заданы для 640x480; CameraInfo сообщает %sx%s."
                % (info.width, info.height))
        ros.on_shutdown(self.stop)
        if args.execute:
            self.setup_motion()

    def callback(self, msg):
        if msg.header.stamp.to_nsec() == 0:
            self.ros.logwarn_throttle(3, "ZERO_IMAGE_STAMP: кадр не используется")
        self.frames.push(msg, time.monotonic())

    def stop(self):
        if self.motion is not None:
            self.motion.stop_action_group()

    def check_running(self):
        if self.ros.is_shutdown():
            raise InterruptedError("Остановка запрошена")

    def sleep(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check_running()
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def fresh_frames(self):
        self.frames.reset(self.target, self.ros.Time.now().to_nsec())

    def setup_motion(self):
        from ainex_kinematics.motion_manager import MotionManager
        from ainex_interfaces.srv import SetWalkingCommand

        actions = {"stand", "move_left", "move_right",
                   "forward_one_step", "back_step"}
        # Пока исполняется только первая цель: после приседа нет
        # подтверждённого критерия продолжения финального захвата.
        actions.add('clamp_left_crouch' if self.args.ids[0] == 0
                    else 'clamp_right_crouch')
        if len(self.args.ids) == 2:
            import rospkg
            import yaml
            config_path = (Path(rospkg.RosPack().get_path('ainex_kinematics'))
                           / 'config' / 'servo_controller.yaml')
            with config_path.open() as stream:
                config = yaml.safe_load(stream)
            self.left_arm_ids = arm_ids(config, 'left')
            self.ros.loginfo('LEFT_ARM_SERVO_IDS: %s', self.left_arm_ids)
            if self.args.demo_transfer:
                self.right_arm_ids = arm_ids(config, 'right')
                if set(self.left_arm_ids) & set(self.right_arm_ids):
                    raise RuntimeError('Карты сервоприводов рук пересекаются')
                self.ros.loginfo('RIGHT_ARM_SERVO_IDS: %s', self.right_arm_ids)
        for action in actions:
            path = Path(self.args.action_path) / (action + ".d6a")
            if not path.is_file():
                raise RuntimeError("Нет ActionGroup: " + str(path))
            with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
                row = db.execute("SELECT * FROM ActionGroup LIMIT 1").fetchone()
                if row is None or len(row) < 3:
                    raise RuntimeError("Пустая/неверная ActionGroup: " + str(path))

        self.ros.wait_for_service("/walking/command", timeout=8)
        # Как в tutorial HiWonder: отключаем процедурную походку перед .d6a.
        self.ros.ServiceProxy("/walking/command", SetWalkingCommand)("disable")
        self.motion = MotionManager(action_path=self.args.action_path)
        self.original_set_positions = self.motion.set_servos_position
        self.motion.set_servos_position = self.guarded_positions
        deadline = time.monotonic() + 5
        while self.motion.servo_state_pub.get_num_connections() == 0:
            self.check_running()
            if time.monotonic() >= deadline:
                raise RuntimeError("Нет подписчика на команды bus_servo")
            time.sleep(0.05)

    def guarded_positions(self, duration, *positions):
        protected = dict(self.held_servos)
        if self.head_locked:
            # ActionGroup не должен смещать голову относительно калибровки
            # в процессе наблюдения и подхода к любой из двух целей.
            protected.update({23: 500, 24: 330})
        return self.original_set_positions(
            duration, protected_positions(positions[0], protected))

    def hold_left_arm(self):
        if len(self.left_arm_ids) != 5:
            raise RuntimeError('Не определены пять сервоприводов левой руки')
        cache = self.motion.servo_position
        if any(str(i) not in cache for i in self.left_arm_ids):
            raise RuntimeError('Нет командной позы левой руки после clamp_left')
        self.held_servos = {i: cache[str(i)] for i in self.left_arm_ids}
        self.ros.loginfo('LEFT_ARM_HOLD: %s', self.held_servos)

    def demo_transfer(self):
        # Снимок обеих командных поз после clamp_right. Удержание проходит
        # через guarded_positions для каждого кадра turn_right и forward.
        ids = self.left_arm_ids + self.right_arm_ids
        if len(self.left_arm_ids) != 5 or len(self.right_arm_ids) != 5 or len(set(ids)) != 10:
            raise RuntimeError('Не определены десять разных сервоприводов рук')
        cache = self.motion.servo_position
        if any(str(i) not in cache for i in ids):
            raise RuntimeError('Нет командной позы обеих рук после захватов')
        self.held_servos = {i: cache[str(i)] for i in ids}
        self.ros.loginfo('BOTH_ARMS_HOLD: %s', self.held_servos)
        self.ros.loginfo('DEMO_TRANSFER: turn_right x%d, pause=0.1 s, forward x12',
                         self.args.demo_turns)
        for index in range(self.args.demo_turns):
            self.ros.loginfo('DEMO_TURN: %d/%d', index + 1, self.args.demo_turns)
            self.action('turn_right', settle=0.1)
        for index in range(12):
            self.ros.loginfo('DEMO_FORWARD: %d/12', index + 1)
            self.action('forward', settle=0.0)

    def action(self, name, settle=None):
        self.check_running()
        self.ros.loginfo("ACTION: %s", name)
        self.motion.run_action(name)  # синхронный вызов
        self.check_running()
        self.sleep(self.args.settle if settle is None else settle)
        self.fresh_frames()

    def prepare(self):
        self.action("stand")
        self.set_head()

    def set_head(self):
        self.motion.set_servos_position(
            300, [[23, 500], [24, 330]])
        self.sleep(0.3 + self.args.settle)
        self.fresh_frames()
        self.ros.loginfo("HEAD: pan=500 tilt=330")

    def observation_xy(self, obs):
        image_age = (self.ros.Time.now().to_nsec() - obs.stamp) / 1e9
        if (time.monotonic() - obs.received > self.args.max_age
                or image_age < -0.2 or image_age > self.args.max_age
                or obs.xy is None):
            return None
        x, y = obs.xy
        return (x, y) if 0 <= x < 640 and 0 <= y < 480 else None

    def measure_after_crouch(self):
        # action() уже сбросил буфер ПОСЛЕ приседа и выдержки. Пиксельные
        # пороги стойки здесь неприменимы; серия кадров не доказывает,
        # что блок находится в области захвата или неподвижен.
        serial, measurements, previous_received = 0, [], None
        deadline = time.monotonic() + min(self.args.timeout, self.args.lost_timeout)
        while not self.ros.is_shutdown():
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "ID %d: нет серии свежих измерений после подсаживания"
                    % self.target)
            obs = self.frames.wait_new(serial)
            if obs is None:
                continue
            serial = obs.serial
            xy = self.observation_xy(obs)
            if xy is None:
                measurements, previous_received = [], None
                continue
            if (previous_received is not None
                    and obs.received - previous_received > self.args.max_age):
                measurements = []
            previous_received = obs.received
            measurements.append((obs.stamp, *xy))
            self.ros.loginfo(
                "POST_CROUCH_MEASUREMENT ID=%d stamp=%d x=%.1f y=%.1f sample=%d/%d",
                self.target, obs.stamp, *xy, len(measurements), self.args.stable_frames)
            if len(measurements) >= self.args.stable_frames:
                return measurements
        self.check_running()

    def approach(self, target):
        self.target = target
        self.fresh_frames()
        serial, streak, previous, moves = 0, 0, None, 0
        start = last_seen = time.monotonic()
        hand = 'LEFT' if target == 0 else 'RIGHT'
        bounds = bounds_for(target, self.args)
        self.ros.loginfo("TARGET ID=%d HAND=%s X=[%.1f, %.1f] Y=[%.1f, %.1f]",
                         target, hand, *bounds)
        while not self.ros.is_shutdown():
            now = time.monotonic()
            if now - start > self.args.timeout:
                raise RuntimeError("ID %d: истекло время подхода" % target)
            if now - last_seen > self.args.lost_timeout:
                raise RuntimeError("ID %d: нет свежего наблюдения цели" % target)
            obs = self.frames.wait_new(serial)
            if obs is None:
                continue
            serial = obs.serial
            now = time.monotonic()
            xy = self.observation_xy(obs)
            if xy is None:
                streak, previous = 0, None
                self.ros.loginfo_throttle(1, "ID=%d WAIT_TARGET", target)
                continue
            x, y = xy
            last_seen = now
            decision = decide(x, y, bounds)
            streak = streak + 1 if decision == previous else 1
            previous = decision
            self.ros.loginfo_throttle(
                0.5, "ID=%d HAND=%s x=%.1f y=%.1f X=[%.1f,%.1f] Y=[%.1f,%.1f] -> %s [%d/%d]",
                target, hand, x, y, *bounds, decision, streak, self.args.stable_frames)
            if not self.args.execute:
                continue
            if streak < self.args.stable_frames:
                continue
            if decision == "READY":
                return
            if moves >= self.args.max_moves:
                raise RuntimeError("ID %d: достигнут лимит шагов" % target)
            self.action(decision)
            moves += 1
            # Длительность самого движения не считаем потерей цели.
            last_seen = time.monotonic()
            streak, previous = 0, None
        self.check_running()

    def run(self):
        if not self.args.execute:
            self.ros.loginfo("DRY_RUN: команды приводам не отправляются")
            self.approach(self.args.ids[0])
            return
        if not (self.args.prepare_only or self.args.approach_only):
            self.ros.logwarn(
                "CALIBRATION_REQUIRED: после подшагов, штатного приседа и измерения "
                "сценарий остановится в приседе; финальный захват, второй захват "
                "и перенос недоступны без подтверждённого критерия")
        self.prepare()
        if self.args.prepare_only:
            self.ros.loginfo("PREPARED: можно измерять пиксели при голове 500/330")
            return
        target = self.args.ids[0]
        self.head_locked = True
        self.approach(target)
        if self.args.approach_only:
            self.ros.loginfo("APPROACH_ONLY: остановка до финальных шагов и захвата")
            return
        for _ in range(self.args.final_steps):
            self.action("forward_one_step")
        # Потеря видимости после подшагов ожидаема. Измеряем только после
        # родного приседа; уже занятая рука остаётся защищена guard-ом.
        self.head_locked = False
        self.action('clamp_left_crouch' if target == 0 else 'clamp_right_crouch')
        measurements = self.measure_after_crouch()
        raise RuntimeError(
            "CALIBRATION_REQUIRED ID=%d: измерения после подсаживания "
            "(stamp_ns, x, y)=%s; критерий положения блока и коррекция "
            "не откалиброваны, финальный захват не выполнен" % (target, measurements))


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--version', action='version', version=SCRIPT_VERSION)
    p.add_argument("--ids", nargs="+", type=int, default=[0])
    p.add_argument("--execute", action="store_true",
                   help="реальные движения до приседа/измерения; затем остановка "
                        "CALIBRATION_REQUIRED без финального захвата")
    p.add_argument('--demo-transfer', action='store_true',
                   help='сохранённый интерфейс переноса; сейчас недоступен: '
                        'сценарий остановится после первого приседа/измерения')
    p.add_argument('--demo-turns', type=int, default=8,
                   help='сохранённое число turn_right в --demo-transfer '
                        '(по умолчанию 8); перенос сейчас недоступен')
    p.add_argument("--prepare-only", action="store_true", help="только stand и голова 500/330")
    p.add_argument("--approach-only", action="store_true", help="остановиться при READY, без захвата")
    p.add_argument("--topic", default="/tag_detections")
    p.add_argument("--camera-info", default="/camera/camera_info")
    p.add_argument("--action-path", default="/home/ubuntu/software/ainex_controller/ActionGroups")
    p.add_argument('--left-x-min', '--x-min', dest='left_x_min', type=float, default=280)
    p.add_argument('--left-x-max', '--x-max', dest='left_x_max', type=float, default=310)
    p.add_argument('--right-x-min', type=float, default=240)
    p.add_argument('--right-x-max', type=float, default=270)
    p.add_argument('--left-y-min', '--y-min', dest='left_y_min', type=float, default=400)
    p.add_argument('--left-y-max', '--y-max', dest='left_y_max', type=float, default=420)
    p.add_argument('--right-y-min', type=float, default=400)
    p.add_argument('--right-y-max', type=float, default=420)
    p.add_argument("--final-steps", type=int, default=2)
    p.add_argument("--stable-frames", type=int, default=3)
    p.add_argument("--max-moves", type=int, default=40)
    p.add_argument("--max-age", type=float, default=1.0)
    p.add_argument("--settle", type=float, default=0.5)
    p.add_argument("--lost-timeout", type=float, default=10)
    p.add_argument("--timeout", type=float, default=180)
    a = p.parse_args(argv)
    if len(a.ids) > 2 or len(a.ids) != len(set(a.ids)) or any(i not in (0, 1) for i in a.ids):
        p.error("--ids: один или два разных ID из 0, 1")
    if len(a.ids) == 2 and a.ids != [0, 1]:
        p.error('Два захвата выполняются в порядке --ids 0 1: левая, затем правая')
    if (a.prepare_only or a.approach_only) and not a.execute:
        p.error("Режимы с движением требуют --execute")
    if a.approach_only and len(a.ids) != 1:
        p.error("--approach-only используется с одним ID")
    if a.prepare_only and a.approach_only:
        p.error("Выбери --prepare-only или --approach-only")
    if a.demo_turns < 0:
        p.error('--demo-turns должен быть неотрицательным')
    if a.demo_transfer and (not a.execute or a.ids != [0, 1]
                            or a.prepare_only or a.approach_only):
        p.error('--demo-transfer требует --execute --ids 0 1 без режимов подготовки/подхода')
    for target in (0, 1):
        x_min, x_max, y_min, y_max = bounds_for(target, a)
        if not (0 <= x_min <= x_max < 640 and 0 <= y_min <= y_max < 480):
            p.error('Пороги каждой руки должны быть внутри кадра 640x480')
    if (not all(math.isfinite(v) for v in
                (a.max_age, a.settle, a.lost_timeout, a.timeout))
            or a.max_age <= 0 or a.settle < 0
            or a.lost_timeout <= 0 or a.timeout <= 0
            or a.stable_frames < 1 or a.max_moves < 1
            or not 0 <= a.final_steps <= 2):
        p.error("Неверный таймаут/лимит; --final-steps допустим от 0 до 2")
    a.action_path = str(Path(a.action_path).resolve())
    return a


def main():
    a = arguments()
    import rospy
    rospy.init_node("robotrio_tag_grab")
    rospy.loginfo('SCRIPT_VERSION=%s FILE=%s', SCRIPT_VERSION,
                   str(Path(__file__).resolve()))
    for target in (0, 1):
        rospy.loginfo('CONFIG ID=%d HAND=%s X=[%.1f, %.1f] Y=[%.1f, %.1f]',
                       target, 'LEFT' if target == 0 else 'RIGHT',
                       *bounds_for(target, a))
    try:
        Runner(a, rospy).run()
        return 0
    except (KeyboardInterrupt, InterruptedError, rospy.ROSInterruptException):
        return 130
    except Exception as error:
        rospy.logerr("STOP: %s", error)
        return 1
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("tag_grab finished")


if __name__ == "__main__":
    raise SystemExit(main())
