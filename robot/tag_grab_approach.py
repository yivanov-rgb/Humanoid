#!/usr/bin/env python3
"""AiNex / ROS 1: первый этап захвата — ориентация, рука, подход, присед.

Самостоятельный скрипт; tag_grab.py не импортируется и не изменяется.
Читает готовые обнаружения HiWonder tag36h11, только одиночный ID 0.
Угол рисунка вычисляется по декодированному порядку corners, без сортировки:
0 градусов — ребро 0->1 направлено вправо, положительный угол — по часовой.
Соответствие углов рукам задаётся --left-angle и --right-angle; встроенной
привязки нет. Без --execute только выводит измерения, MotionManager не создаёт.

При --execute: голова 500/330 -> stand -> выбор руки по свежим кадрам ->
подход по прежним порогам выбранной руки -> два forward_one_step ->
crouch или clamp_right_crouch. Голова остаётся в 500/330
весь этап, выбранная рука больше не меняется. Финальное прицеливание,
захват и подъём относятся ко второму этапу и здесь не выполняются.

Примеры внутри аппаратного контейнера ainex:
  python3 tag_grab_approach.py
  python3 tag_grab_approach.py --execute --prepare-only
  python3 tag_grab_approach.py --help

--left-angle/--right-angle нужно сначала измерить для реальных поз блока.
При нескольких видимых ID 0 цель неоднозначна; шаги не выполняются.
Ctrl+C останавливает дальнейшие команды без автоматического вставания.
"""
import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time


SCRIPT_VERSION = 'tag-grab-approach-2026-11-15'
TAG_ID = 0
TAG_FAMILY = 'tag36h11'
HEAD_POSITION = {23: 500, 24: 330}


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    angle: float


@dataclass(frozen=True)
class Observation:
    serial: int
    received: float
    stamp: int
    target: object


def angle_distance(first, second):
    return abs((first - second + 180.0) % 360.0 - 180.0)


def target_from_detections(detections):
    """Один ID 0; corners сохраняют ориентацию декодированного рисунка."""
    matches = [item for item in detections if list(item.id) == [TAG_ID]]
    if len(matches) != 1 or len(matches[0].corners) != 4:
        return None
    points = [(p.x, p.y) for p in matches[0].corners]
    if not all(math.isfinite(value) for point in points for value in point):
        return None
    edges = [(points[(i + 1) % 4][0] - points[i][0],
              points[(i + 1) % 4][1] - points[i][1]) for i in range(4)]
    crosses = [edges[i][0] * edges[(i + 1) % 4][1]
               - edges[i][1] * edges[(i + 1) % 4][0] for i in range(4)]
    if not (all(value > 1e-6 for value in crosses)
            or all(value < -1e-6 for value in crosses)):
        return None
    dx, dy = edges[0][0] - edges[2][0], edges[0][1] - edges[2][1]
    if math.hypot(dx, dy) < 1e-6:
        return None
    # Центр сохранён как в tag_grab.py: диагональ 0--2 и прежние пороги.
    x, y = (points[0][0] + points[2][0]) / 2, (points[0][1] + points[2][1]) / 2
    if not (0 <= x < 640 and 0 <= y < 480):
        return None
    angle = (math.degrees(math.atan2(dy, dx)) + 180.0) % 360.0 - 180.0
    return Target(x, y, angle)


def hand_for_angle(angle, args):
    if args.left_angle is None or args.right_angle is None:
        return None
    matches = [side for side in ('left', 'right')
               if angle_distance(angle, getattr(args, side + '_angle'))
               <= args.angle_tolerance]
    return matches[0] if len(matches) == 1 else None


def bounds_for(hand, args):
    if hand not in ('left', 'right'):
        raise ValueError('Неизвестная рука: ' + str(hand))
    return tuple(getattr(args, hand + '_' + axis + '_' + bound)
                 for axis, bound in (('x', 'min'), ('x', 'max'),
                                     ('y', 'min'), ('y', 'max')))


def decide(x, y, bounds):
    # Направления и приоритет X сохранены из tag_grab.py.
    x_min, x_max, y_min, y_max = bounds
    if x > x_max:
        return 'move_right'
    if x < x_min:
        return 'move_left'
    if y < y_min:
        return 'forward_one_step'
    if y > y_max:
        return 'back_step'
    return 'READY'


class FrameBuffer:
    """Только новые изображения, включая кадры без однозначного ID 0."""
    def __init__(self):
        self.condition = threading.Condition()
        self.last_stamp = 0
        self.serial = 0
        self.observation = None

    def reset(self, cutoff):
        with self.condition:
            self.last_stamp = cutoff
            self.observation = None

    def push(self, msg, received):
        stamp = msg.header.stamp.to_nsec()
        with self.condition:
            if stamp <= self.last_stamp:
                return
            self.last_stamp = stamp
            self.serial += 1
            self.observation = Observation(
                self.serial, received, stamp, target_from_detections(msg.detections))
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
        self.args, self.ros = args, ros
        self.frames = FrameBuffer()
        self.motion = None
        ros.on_shutdown(self.stop)
        family = ros.get_param(args.family_param, None)
        if family != TAG_FAMILY:
            raise RuntimeError('%s должен быть %s, получено %s'
                               % (args.family_param, TAG_FAMILY, family))
        self.sub = ros.Subscriber(args.topic, AprilTagDetectionArray,
                                  self.callback, queue_size=1)
        info = ros.wait_for_message(args.camera_info, CameraInfo, timeout=8)
        if (info.width, info.height) != (640, 480):
            raise RuntimeError('Пороги заданы для 640x480; CameraInfo сообщает %sx%s'
                               % (info.width, info.height))
        if args.execute:
            self.setup_motion()

    def callback(self, msg):
        self.frames.push(msg, time.monotonic())

    def stop(self):
        if self.motion is not None:
            self.motion.stop_action_group()

    def check_running(self):
        if self.ros.is_shutdown():
            raise InterruptedError('Остановка запрошена')

    def sleep(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check_running()
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def fresh_frames(self):
        self.frames.reset(self.ros.Time.now().to_nsec())

    def setup_motion(self):
        from ainex_kinematics.motion_manager import MotionManager
        from ainex_interfaces.srv import SetWalkingCommand
        if self.ros.get_param('/use_sim_time', False):
            raise RuntimeError('Этот скрипт предназначен для физического робота')
        actions = {'stand'}
        if not self.args.prepare_only:
            actions.update(('move_left', 'move_right', 'forward_one_step', 'back_step',
                            'crouch', 'clamp_right_crouch'))
        for name in sorted(actions):
            path = Path(self.args.action_path) / (name + '.d6a')
            if not path.is_file():
                raise RuntimeError('Нет ActionGroup: ' + str(path))
        self.ros.wait_for_service('/walking/command', timeout=8)
        self.ros.ServiceProxy('/walking/command', SetWalkingCommand)('disable')
        self.motion = MotionManager(action_path=self.args.action_path)
        self.original_set_positions = self.motion.set_servos_position
        self.motion.set_servos_position = self.guarded_positions
        deadline = time.monotonic() + 5
        while self.motion.servo_state_pub.get_num_connections() == 0:
            self.check_running()
            if time.monotonic() >= deadline:
                raise RuntimeError('Нет подписчика на команды bus_servo')
            self.sleep(0.05)

    def guarded_positions(self, duration, *positions):
        self.check_running()
        return self.original_set_positions(
            duration, [[sid, HEAD_POSITION.get(sid, pulse)] for sid, pulse in positions[0]])

    def action(self, name):
        self.check_running()
        self.ros.loginfo('ACTION: %s', name)
        self.motion.run_action(name)
        self.check_running()
        self.sleep(self.args.settle)
        self.fresh_frames()

    def prepare(self):
        # Фиксация камеры предшествует stand и любому выбору руки/подходу.
        self.check_running()
        self.motion.set_servos_position(300, sorted(HEAD_POSITION.items()))
        self.sleep(0.3 + self.args.settle)
        self.ros.loginfo('HEAD_LOCKED: pan=500 tilt=330')
        self.action('stand')

    def fresh_target(self, obs):
        image_age = (self.ros.Time.now().to_nsec() - obs.stamp) / 1e9
        if (time.monotonic() - obs.received > self.args.max_age
                or image_age < -0.2 or image_age > self.args.max_age):
            return None
        return obs.target

    def select_hand(self):
        self.fresh_frames()
        serial, previous, streak, previous_received = 0, None, 0, None
        start = last_seen = time.monotonic()
        while not self.ros.is_shutdown():
            now = time.monotonic()
            if now - start > self.args.timeout:
                raise RuntimeError('Истекло время наблюдения/выбора руки')
            if now - last_seen > self.args.lost_timeout:
                raise RuntimeError('Нет свежего однозначного ID 0')
            obs = self.frames.wait_new(serial)
            if obs is None:
                continue
            serial = obs.serial
            target = self.fresh_target(obs)
            if target is None:
                previous, streak, previous_received = None, 0, None
                self.ros.loginfo_throttle(1, 'WAIT_TARGET: нужен один свежий ID 0')
                continue
            last_seen = time.monotonic()
            hand = hand_for_angle(target.angle, self.args)
            continuous = (previous_received is not None
                          and obs.received - previous_received <= self.args.max_age)
            streak = streak + 1 if hand and hand == previous and continuous else int(hand is not None)
            previous, previous_received = hand, obs.received
            state = hand.upper() if hand else ('UNCALIBRATED' if self.args.left_angle is None
                                               else 'OUTSIDE_ANGLE_RANGES')
            self.ros.loginfo_throttle(
                0.5, 'TAG ID=0 x=%.1f y=%.1f angle=%.1f HAND=%s [%d/%d]',
                target.x, target.y, target.angle, state, streak, self.args.stable_frames)
            if self.args.execute and hand and streak >= self.args.stable_frames:
                self.ros.loginfo('HAND_SELECTED: %s angle=%.1f', hand.upper(), target.angle)
                return hand
        self.check_running()

    def approach(self, hand):
        self.fresh_frames()
        bounds = bounds_for(hand, self.args)
        self.ros.loginfo('APPROACH HAND=%s X=[%.1f,%.1f] Y=[%.1f,%.1f]', hand.upper(), *bounds)
        serial, previous, streak, moves, previous_received = 0, None, 0, 0, None
        start = last_seen = time.monotonic()
        while not self.ros.is_shutdown():
            now = time.monotonic()
            if now - start > self.args.timeout:
                raise RuntimeError('Истекло время подхода')
            if now - last_seen > self.args.lost_timeout:
                raise RuntimeError('Потеряна цель ID 0; следующие шаги запрещены')
            obs = self.frames.wait_new(serial)
            if obs is None:
                continue
            serial = obs.serial
            target = self.fresh_target(obs)
            if target is None:
                previous, streak, previous_received = None, 0, None
                continue
            last_seen = time.monotonic()
            decision = decide(target.x, target.y, bounds)
            continuous = (previous_received is not None
                          and obs.received - previous_received <= self.args.max_age)
            streak = streak + 1 if decision == previous and continuous else 1
            previous, previous_received = decision, obs.received
            self.ros.loginfo_throttle(
                0.5, 'ID=0 HAND=%s x=%.1f y=%.1f angle=%.1f -> %s [%d/%d]',
                hand.upper(), target.x, target.y, target.angle, decision,
                streak, self.args.stable_frames)
            if streak < self.args.stable_frames:
                continue
            if decision == 'READY':
                return
            if moves >= self.args.max_moves:
                raise RuntimeError('Достигнут лимит шагов')
            self.action(decision)
            moves += 1
            last_seen = time.monotonic()
            previous, streak, previous_received = None, 0, None
        self.check_running()

    def run(self):
        if not self.args.execute:
            self.ros.loginfo('OBSERVE: только измерения ID 0; камера и приводы не перемещаются')
            self.select_hand()
            return
        self.prepare()
        if self.args.prepare_only:
            self.ros.loginfo('PREPARED: stand, камера зафиксирована в 500/330')
            return
        hand = self.select_hand()
        self.approach(hand)
        # Как в исходном сценарии, после READY видимость может пропасть до приседа.
        for _ in range(self.args.final_steps):
            self.action('forward_one_step')
        self.action('crouch' if hand == 'left' else 'clamp_right_crouch')
        self.ros.loginfo('CROUCHED: ID=0 HAND=%s HEAD=500/330; первый этап завершён', hand.upper())


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--version', action='version', version=SCRIPT_VERSION)
    p.add_argument('--execute', action='store_true', help='разрешить движения до приседа')
    p.add_argument('--prepare-only', action='store_true', help='только камера 500/330 и stand')
    p.add_argument('--left-angle', type=float, help='измеренный угол рисунка для левой руки, градусы')
    p.add_argument('--right-angle', type=float, help='измеренный угол рисунка для правой руки, градусы')
    p.add_argument('--angle-tolerance', type=float, default=15, help='допуск каждого угла, градусы')
    p.add_argument('--topic', default='/tag_detections')
    p.add_argument('--camera-info', default='/camera/camera_info')
    p.add_argument('--family-param', default='/apriltag_ros_continuous_node/tag_family')
    p.add_argument('--action-path', default='/home/ubuntu/software/ainex_controller/ActionGroups')
    p.add_argument('--left-x-min', type=float, default=280)
    p.add_argument('--left-x-max', type=float, default=310)
    p.add_argument('--right-x-min', type=float, default=240)
    p.add_argument('--right-x-max', type=float, default=270)
    p.add_argument('--left-y-min', type=float, default=400)
    p.add_argument('--left-y-max', type=float, default=420)
    p.add_argument('--right-y-min', type=float, default=400)
    p.add_argument('--right-y-max', type=float, default=420)
    p.add_argument('--final-steps', type=int, default=2)
    p.add_argument('--stable-frames', type=int, default=3)
    p.add_argument('--max-moves', type=int, default=40)
    p.add_argument('--max-age', type=float, default=1.0)
    p.add_argument('--settle', type=float, default=0.5)
    p.add_argument('--lost-timeout', type=float, default=10)
    p.add_argument('--timeout', type=float, default=180)
    a = p.parse_args(argv)
    if a.prepare_only and not a.execute:
        p.error('--prepare-only требует --execute')
    if (a.left_angle is None) != (a.right_angle is None):
        p.error('Задайте оба ориентира --left-angle и --right-angle либо ни одного')
    if not math.isfinite(a.angle_tolerance) or not 0 < a.angle_tolerance < 90:
        p.error('--angle-tolerance должен быть больше 0 и меньше 90 градусов')
    if a.left_angle is not None:
        if not all(math.isfinite(v) for v in (a.left_angle, a.right_angle)):
            p.error('Углы должны быть конечными числами')
        a.left_angle = (a.left_angle + 180) % 360 - 180
        a.right_angle = (a.right_angle + 180) % 360 - 180
        if angle_distance(a.left_angle, a.right_angle) <= 2 * a.angle_tolerance:
            p.error('Диапазоны углов левой и правой руки не должны пересекаться')
    if a.execute and not a.prepare_only and a.left_angle is None:
        p.error('Сначала измерьте углы, затем задайте --left-angle и --right-angle')
    for side in ('left', 'right'):
        x_min, x_max, y_min, y_max = bounds_for(side, a)
        if not (0 <= x_min <= x_max < 640 and 0 <= y_min <= y_max < 480):
            p.error('Пороги каждой руки должны быть внутри кадра 640x480')
    if (not all(math.isfinite(v) for v in (a.max_age, a.settle, a.lost_timeout, a.timeout))
            or a.max_age <= 0 or a.settle < 0 or a.lost_timeout <= 0 or a.timeout <= 0
            or a.stable_frames < 1 or a.max_moves < 1 or not 0 <= a.final_steps <= 2):
        p.error('Неверный таймаут/лимит; --final-steps допустим от 0 до 2')
    a.action_path = str(Path(a.action_path).resolve())
    return a


def main():
    args = arguments()
    import rospy
    rospy.init_node('robotrio_tag_grab_approach')
    rospy.loginfo('SCRIPT_VERSION=%s FILE=%s', SCRIPT_VERSION, str(Path(__file__).resolve()))
    try:
        Runner(args, rospy).run()
        return 0
    except (KeyboardInterrupt, InterruptedError, rospy.ROSInterruptException):
        return 130
    except Exception as error:
        rospy.logerr('STOP: %s', error)
        return 1
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown('tag_grab_approach finished')


if __name__ == '__main__':
    raise SystemExit(main())
