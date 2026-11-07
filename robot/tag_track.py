#!/usr/bin/env python3
"""AiNex / ROS 1 Noetic: центрирование AprilTag движениями головы.

По умолчанию только наблюдение; --execute разрешает команды ID 23 и 24.
Диапазоны pulse подтверждены пользователем: pan 300..700, tilt 350..650.
Начальная поза считывается с робота; при потере цели голова остаётся на месте.
"""
import argparse
from dataclasses import dataclass
import math
import threading
import time


PAN_LIMITS = (300, 700)
TILT_LIMITS = (350, 650)


@dataclass(frozen=True)
class Observation:
    stamp: int
    received: float
    xy: object


def target_center(detections, target):
    """Один экземпляр выбранного ID; bundle и неоднозначные цели исключены."""
    matches = [item for item in detections if list(item.id) == [target]]
    if len(matches) != 1 or len(matches[0].corners) != 4:
        return None
    corners = matches[0].corners
    if not all(math.isfinite(p.x) and math.isfinite(p.y) for p in corners):
        return None
    return (sum(p.x for p in corners) / 4, sum(p.y for p in corners) / 4)


def axis_step(position, error, limits, args, inverted=False):
    """P-коррекция: зона покоя, предел шага и ограничение конечного pulse."""
    if abs(error) <= args.deadband:
        return position
    # Штатный AprilTag-трекер HiWonder использует target - current для обеих осей.
    delta = args.gain * error * (1 if inverted else -1)
    delta = max(-args.max_step, min(args.max_step, delta))
    return max(limits[0], min(limits[1], round(position + delta)))


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='разрешить движения головы')
    parser.add_argument('--id', type=int, default=0, help='ID единственной цели, по умолчанию 0')
    parser.add_argument('--topic', default='/tag_detections')
    parser.add_argument('--camera-info', default='/camera/camera_info')
    parser.add_argument('--pan-range', type=int, nargs=2, default=PAN_LIMITS,
                        metavar=('MIN', 'MAX'), help='рабочий диапазон ID 23 внутри 300..700')
    parser.add_argument('--tilt-range', type=int, nargs=2, default=TILT_LIMITS,
                        metavar=('MIN', 'MAX'), help='рабочий диапазон ID 24 внутри 350..650')
    parser.add_argument('--deadband', type=float, default=15, help='зона покоя каждой оси, пиксели')
    parser.add_argument('--gain', type=float, default=0.05, help='pulse на пиксель ошибки за кадр')
    parser.add_argument('--max-step', type=int, default=8, help='максимальная коррекция каждой оси, pulse')
    parser.add_argument('--move-ms', type=int, default=150, help='длительность команды, миллисекунды')
    parser.add_argument('--settle', type=float, default=0.05, help='пауза после движения, секунды')
    parser.add_argument('--max-age', type=float, default=0.5, help='максимальный возраст кадра, секунды')
    parser.add_argument('--invert-pan', action='store_true', help='инвертировать коррекцию по X')
    parser.add_argument('--invert-tilt', action='store_true', help='инвертировать коррекцию по Y')
    args = parser.parse_args(argv)
    for name, limits in (('pan', PAN_LIMITS), ('tilt', TILT_LIMITS)):
        low, high = getattr(args, name + '_range')
        if not limits[0] <= low < high <= limits[1]:
            parser.error('--%s-range должен быть внутри %s..%s, MIN < MAX'
                         % (name, *limits))
    if (args.id < 0 or args.max_step < 1 or args.move_ms < 1
            or not math.isfinite(args.deadband) or args.deadband < 0
            or not math.isfinite(args.gain) or args.gain <= 0
            or not math.isfinite(args.settle) or args.settle < 0
            or not math.isfinite(args.max_age) or args.max_age <= 0):
        parser.error('Неверный ID, зона покоя, коэффициент, шаг или время')
    return args


class Tracker:
    def __init__(self, args, ros):
        from apriltag_ros.msg import AprilTagDetectionArray
        from sensor_msgs.msg import CameraInfo
        self.args = args
        self.ros = ros
        self.motion = None
        self.lock = threading.Lock()
        self.observation = None
        self.last_stamp = 0
        self.cutoff = 0
        info = ros.wait_for_message(args.camera_info, CameraInfo, timeout=8)
        self.width, self.height = info.width, info.height
        if min(self.width, self.height) <= 0 or args.deadband >= min(self.width, self.height) / 2:
            raise RuntimeError('Неверный размер CameraInfo или слишком большая --deadband')
        if args.execute:
            self.setup_motion()
        self.fresh_frames()
        self.sub = ros.Subscriber(args.topic, AprilTagDetectionArray, self.callback, queue_size=1)

    def check_running(self):
        if self.ros.is_shutdown():
            raise InterruptedError('Остановка запрошена')

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check_running()
            time.sleep(min(0.02, max(0, deadline - time.monotonic())))

    def setup_motion(self):
        from ainex_kinematics.motion_manager import MotionManager
        if self.ros.get_param('/use_sim_time', False):
            raise RuntimeError('Этот скрипт управляет физическим роботом; /use_sim_time включён')
        self.ros.wait_for_service('/ros_robot_controller/bus_servo/get_position', timeout=8)
        self.motion = MotionManager()
        deadline = time.monotonic() + 5
        while self.motion.servo_state_pub.get_num_connections() == 0:
            self.check_running()
            if time.monotonic() >= deadline:
                raise RuntimeError('Нет подписчика на команды bus_servo')
            self.wait(0.05)
        positions = self.motion.get_servos_position(23, 24)
        if len(positions) != 2 or {item[0] for item in positions} != {23, 24}:
            raise RuntimeError('Не удалось прочитать положения ID 23 и 24')
        measured = dict(positions)
        for servo_id, limits in ((23, self.args.pan_range), (24, self.args.tilt_range)):
            value = measured[servo_id]
            if not isinstance(value, int) or not limits[0] <= value <= limits[1]:
                raise RuntimeError('ID %d: прочитано %s вне рабочего диапазона %s; '
                                   'команды движения не отправлены' % (servo_id, value, limits))
        self.pan, self.tilt = measured[23], measured[24]
        self.ros.loginfo('HEAD_START: pan=%d tilt=%d', self.pan, self.tilt)

    def fresh_frames(self):
        with self.lock:
            self.cutoff = self.ros.Time.now().to_nsec()
            self.observation = None

    def fresh(self, observation):
        age = (self.ros.Time.now().to_nsec() - observation.stamp) / 1e9
        return (observation.stamp > 0 and -0.2 <= age <= self.args.max_age
                and time.monotonic() - observation.received <= self.args.max_age)

    def callback(self, msg):
        observation = Observation(msg.header.stamp.to_nsec(), time.monotonic(),
                                  target_center(msg.detections, self.args.id))
        # Не даём ошибочному timestamp из будущего заблокировать последующие кадры.
        if not self.fresh(observation):
            self.ros.logwarn_throttle(2, 'STALE_FRAME: неверный timestamp или старое изображение')
            return
        with self.lock:
            if observation.stamp <= max(self.cutoff, self.last_stamp):
                return
            self.last_stamp = observation.stamp
            self.observation = observation  # Пустое обнаружение тоже заменяет предыдущее.

    def process(self, observation):
        if not self.fresh(observation) or observation.xy is None:
            self.ros.loginfo_throttle(2, 'NO_TARGET: ожидание свежего обнаружения выбранного ID')
            return
        x, y = observation.xy
        if not (0 <= x < self.width and 0 <= y < self.height):
            self.ros.logwarn_throttle(2, 'INVALID_TARGET: центр вне изображения CameraInfo')
            return
        error_x, error_y = x - self.width / 2, y - self.height / 2
        if not self.args.execute:
            self.ros.loginfo_throttle(1, 'OBSERVE: ID=%d xy=(%.1f, %.1f) error=(%.1f, %.1f) px',
                                      self.args.id, x, y, error_x, error_y)
            return
        # В tag_grab.py пользователь подтвердил: больший X находится левее.
        # ID 23 при увеличении pulse поворачивается влево (SERVO_CHECKLIST_RU.md).
        pan = axis_step(self.pan, error_x, self.args.pan_range, self.args, not self.args.invert_pan)
        tilt = axis_step(self.tilt, error_y, self.args.tilt_range, self.args, self.args.invert_tilt)
        centered = max(abs(error_x), abs(error_y)) <= self.args.deadband
        unchanged = (pan, tilt) == (self.pan, self.tilt)
        state = 'CENTERED' if centered else ('LIMIT_OR_SMALL_STEP' if unchanged else 'TRACK')
        self.ros.loginfo_throttle(1, '%s: ID=%d error=(%.1f, %.1f) pan=%d tilt=%d',
                                  state, self.args.id, error_x, error_y, pan, tilt)
        if unchanged:
            return
        self.check_running()
        self.motion.set_servos_position(self.args.move_ms, [[23, pan], [24, tilt]])
        self.pan, self.tilt = pan, tilt
        self.wait(self.args.move_ms / 1000 + self.args.settle)
        # Следующая коррекция разрешена только по изображению после окончания движения.
        self.fresh_frames()

    def run(self):
        self.ros.loginfo('TAG_TRACK: mode=%s ID=%d image=%dx%d pan=%s tilt=%s',
                         'EXECUTE' if self.args.execute else 'OBSERVE', self.args.id,
                         self.width, self.height, self.args.pan_range, self.args.tilt_range)
        while not self.ros.is_shutdown():
            with self.lock:
                observation, self.observation = self.observation, None
            if observation is None:
                self.ros.loginfo_throttle(2, 'WAIT_FRAME: ожидание нового кадра; новых команд нет')
                self.wait(0.02)
                continue
            self.process(observation)


def main():
    args = arguments()
    import rospy
    rospy.init_node('robotrio_tag_track')
    try:
        Tracker(args, rospy).run()
        return 0
    except (KeyboardInterrupt, InterruptedError, rospy.ROSInterruptException):
        return 130
    except Exception as error:
        rospy.logerr('STOP: %s', error)
        return 1
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown('tag_track finished')


if __name__ == '__main__':
    raise SystemExit(main())
