#!/usr/bin/env python3
"""AiNex/ROS Noetic: follow one contrasting line using recorded ActionGroups.

Copy this file only. Default: camera observation. --execute enables movement.
The current head pose is preserved. Stop with Ctrl+C or ~stop.
"""
import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import queue
import signal
import threading
import time

ACTIONS = ('forward_one_step', 'turn_left', 'turn_right', 'move_left', 'move_right')
# Three narrower bands touching the bottom edge: 80..100% of image height.
ROIS = ((96/120, 104/120), (104/120, 112/120), (112/120, 1.0))


@dataclass(frozen=True)
class LineObservation:
    offset: float
    heading: float


def bounded(operation, seconds, check=None):
    """Wall-time bound for read/service calls, even if ROS stops responding."""
    result = queue.Queue(maxsize=1)
    def worker():
        try:
            result.put((True, operation()))
        except Exception as error:
            result.put((False, error))
    threading.Thread(target=worker, daemon=True).start()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if check:
            check()
        try:
            ok, value = result.get(timeout=0.02)
        except queue.Empty:
            continue
        if not ok:
            raise value
        return value
    raise TimeoutError('ROS service timeout')


def wait(seconds, check):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        check()
        time.sleep(min(0.02, max(0, deadline-time.monotonic())))


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true', help='enable walking; default is observation')
    p.add_argument('--empty-hands', action='store_true', help='confirm both hands are empty')
    p.add_argument('--image-topic', default='/camera/image_raw')
    p.add_argument('--image-orientation', choices=('standard', 'mirrored'))
    p.add_argument('--action-path', help='existing robot ActionGroups directory')
    p.add_argument('--threshold', type=int, default=80)
    p.add_argument('--line-color', choices=('black', 'white'), default='black')
    p.add_argument('--deadband', type=float, default=0.04)
    p.add_argument('--heading-deadband', type=float, default=15)
    p.add_argument('--max-actions', type=int, default=0, help='0: until stopped or line lost')
    p.add_argument('--max-seconds', type=float, default=0, help='0: no time limit')
    # Kept for local simulation/diagnostic commands; no simulation dependency on hardware.
    p.add_argument('--backend', choices=('robot', 'sim'), default='robot', help=argparse.SUPPRESS)
    p.add_argument('--keep-head', action='store_true', help=argparse.SUPPRESS)
    p.add_argument('--frame-timeout', type=float, default=2, help=argparse.SUPPRESS)
    p.add_argument('--settle', type=float, default=0.25, help=argparse.SUPPRESS)
    p.set_defaults(min_area=3, max_line_width=0.35, center_offset=0, action_timeout=30,
                   stable_frames=3, debug_topic='/line_follow/image')
    return p


def arguments(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if (not 0 <= args.threshold <= 255 or not math.isfinite(args.deadband)
            or not 0 < args.deadband < 0.25 or not math.isfinite(args.heading_deadband)
            or not 0 < args.heading_deadband < 90 or args.max_actions < 0
            or any(not math.isfinite(v) or v < 0 for v in (args.max_seconds, args.settle))
            or not math.isfinite(args.frame_timeout) or args.frame_timeout <= 0):
        p.error('invalid threshold, deadband, angle, timeout or action limit')
    if args.execute and (not args.empty_hands or args.image_orientation is None):
        p.error('--execute requires --empty-hands and --image-orientation standard|mirrored')
    args.action_path = args.action_path or ('/actions' if args.backend == 'sim' else
        '/home/ubuntu/software/ainex_controller/ActionGroups')
    return args


def detect_line(image, args, cv2):
    """Hiwonder's 160x120 processing, with three ROIs moved to the image bottom.

    For a contrasting monochrome track use a grayscale mask instead of the
    vendor LAB color service. Keep all usable ROIs: one X coordinate cannot
    distinguish a parallel offset from a curve or heading error.
    """
    small = cv2.resize(image, (160, 120))
    gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    kind = cv2.THRESH_BINARY_INV if args.line_color == 'black' else cv2.THRESH_BINARY
    mask = cv2.threshold(gray, args.threshold, 255, kind)[1]
    center = 0.5 + args.center_offset
    h, w = image.shape[:2]
    points = []
    cv2.line(image, (int(center * w), 0), (int(center * w), h - 1), (255, 0, 0), 1)
    for low, high in ROIS:
        x0, x1, y0, y1 = 40, 120, int(low * 120), int(high * 120)
        cv2.rectangle(image, (int(x0 / 160 * w), int(low * h)),
                      (int(x1 / 160 * w), min(int(high * h), h - 1)), (0, 180, 0), 1)
        contours = cv2.findContours(mask[y0:y1, x0:x1], cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, cw, ch = cv2.boundingRect(contour)
            # Reject an ROI filled by shadow/background, and lateral clipping.
            if (area >= args.min_area and cw / 160 <= args.max_line_width
                    and x > 0 and x + cw < x1 - x0 and ch >= (y1 - y0) / 2):
                candidates.append((area, x + cw / 2 + x0))
        candidates.sort(reverse=True)
        if not candidates:
            continue
        if len(candidates) > 1 and candidates[1][0] >= candidates[0][0] * 0.5:
            return None  # ambiguous track; this controller does not choose forks
        x, y = candidates[0][1], (y0 + y1) / 2
        points.append((x, y))
        cv2.circle(image, (int(x / 160 * w), int(y / 120 * h)), 5, (0, 0, 255), -1)
    if len(points) < 2:
        return None  # insufficient geometry: do not guess a turn or a sidestep
    far, near = points[0], points[-1]
    cv2.line(image, (int(far[0]/160*w), int(far[1]/120*h)),
             (int(near[0]/160*w), int(near[1]/120*h)), (255, 180, 0), 2)
    return LineObservation(near[0] / 160 - center,
                           math.degrees(math.atan2(far[0] - near[0], near[1] - far[1])))


def choose_action(line, args):
    if line is None:
        return None
    if abs(line.heading) > args.heading_deadband:
        action, error = 'turn', line.heading
    elif abs(line.offset) > args.deadband:
        action, error = 'move', line.offset
    else:
        return 'forward_one_step'
    if args.image_orientation is None:
        return 'ORIENTATION_REQUIRED'
    right = (error > 0) != (args.image_orientation == 'mirrored')
    return action + ('_right' if right else '_left')


class Backend:
    """Run only the five native ActionGroups tested by the operator."""
    def __init__(self, args, ros, check=None):
        if ros.get_param('/use_sim_time', False):
            raise RuntimeError('Robot backend refuses simulation time')
        root = Path(args.action_path).expanduser().resolve(strict=True)
        for name in ACTIONS:
            if not (root / (name+'.d6a')).is_file():
                raise FileNotFoundError('Missing ActionGroup: '+name)
        self.abort = threading.Event()
        self.check = check or (lambda: None)
        self.timeout = args.action_timeout
        from ainex_interfaces.srv import SetWalkingCommand
        from ainex_kinematics.motion_manager import MotionManager
        self.check()
        ros.wait_for_service('/walking/command', timeout=8)
        disabled = bounded(lambda: ros.ServiceProxy('/walking/command', SetWalkingCommand)('disable'), 8)
        if not disabled.result:
            raise RuntimeError('Could not disable procedural walking')
        self.motion = MotionManager(action_path=str(root))
        send = self.motion.set_servos_position
        def guarded_send(duration, *positions):
            # Cancellation/fresh-camera guard only; native pulses pass unchanged.
            self.guard()
            send(duration, *positions)
        self.motion.set_servos_position = guarded_send
        deadline = time.monotonic()+5
        while self.motion.servo_state_pub.get_num_connections() == 0:
            self.guard()
            if ros.is_shutdown() or time.monotonic() >= deadline:
                raise RuntimeError('No bus servo command subscriber')
            time.sleep(0.02)

    def guard(self):
        if self.abort.is_set():
            raise InterruptedError('Motion cancelled')
        self.check()

    def play(self, name):
        if name not in ACTIONS:
            raise ValueError('Unsupported line action: '+name)
        self.guard()
        try:
            bounded(lambda: self.motion.run_action(name), self.timeout, check=self.guard)
        except BaseException:
            self.cancel()
            raise

    def cancel(self):
        self.abort.set()
        self.motion.stop_action_group()


class Follower:
    def __init__(self, args, ros):
        import cv2
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from std_srvs.srv import Trigger, TriggerResponse
        self.args, self.ros, self.cv2 = args, ros, cv2
        self.bridge = CvBridge()
        self.lock, self.stopped = threading.Lock(), threading.Event()
        self.image, self.backend, self.last_stamp = None, None, 0
        self.clock_reset, self.started = False, time.monotonic()
        self.debug = ros.Publisher(args.debug_topic, Image, queue_size=1)
        self.sub = ros.Subscriber(args.image_topic, Image, self.on_image, queue_size=1, buff_size=2**24)
        ros.on_shutdown(self.stop)
        def stop_request(_request):
            self.stop()
            return TriggerResponse(success=True, message='Further frames cancelled')
        self.stop_service = ros.Service('~stop', Trigger, stop_request)

    def fresh_stamp(self, stamp):
        age = (self.ros.Time.now().to_nsec()-stamp)/1e9
        return stamp > 0 and -0.1 <= age <= self.args.frame_timeout

    def on_image(self, msg):
        stamp = msg.header.stamp.to_nsec()
        with self.lock:
            if stamp < self.last_stamp:
                self.clock_reset = True
            if stamp <= self.last_stamp or not self.fresh_stamp(stamp):
                return
            self.last_stamp = stamp
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
            detected = detect_line(image, self.args, self.cv2)
            action = choose_action(detected, self.args)
            with self.lock:
                self.image = (stamp, time.monotonic(), action, detected)
            label = str(action or 'NO_LINE')
            if detected:
                label += ' offset=%.3f heading=%.1f' % (detected.offset, detected.heading)
            self.cv2.putText(image, label, (8, 25), self.cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            debug = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            debug.header = msg.header
            self.debug.publish(debug)
        except Exception as error:
            self.ros.logerr_throttle(2, 'Camera processing: %s', error)

    def stop(self):
        self.stopped.set()
        if self.backend is not None:
            self.backend.cancel()

    def check(self):
        if self.stopped.is_set() or self.ros.is_shutdown():
            raise InterruptedError('Stopped')
        if self.args.max_seconds and time.monotonic()-self.started >= self.args.max_seconds:
            raise TimeoutError('TIME_LIMIT')
        with self.lock:
            image = self.image
        if self.clock_reset:
            raise RuntimeError('Camera timestamps went backwards')
        if image is None or time.monotonic()-image[1] > self.args.frame_timeout or not self.fresh_stamp(image[0]):
            raise RuntimeError('CAMERA_STALE')

    def wait_decision(self, cutoff):
        deadline = time.monotonic()+self.args.frame_timeout
        window_received = time.monotonic()
        previous, selected, count = cutoff, None, 0
        while time.monotonic() < deadline:
            self.check()
            with self.lock:
                image = self.image
            if image[0] > previous and image[1] > window_received:
                previous = image[0]
                count = count+1 if image[2] is not None and image[2] == selected else 1
                selected = image[2]
                if selected is not None and count >= self.args.stable_frames:
                    return selected
            time.sleep(0.02)
        raise RuntimeError('NO_STABLE_LINE (no blind search)')

    def run(self):
        deadline = time.monotonic()+self.args.frame_timeout
        while self.image is None and time.monotonic() < deadline and not self.ros.is_shutdown():
            time.sleep(0.02)
        self.check()
        if not self.args.execute:
            while True:
                self.check()
                with self.lock:
                    image = self.image
                self.ros.loginfo_throttle(1, 'OBSERVE action=%s line=%s', image[2], image[3])
                time.sleep(0.05)
        if self.args.backend == 'sim':
            from robotrio_sim.line_backend import SimBackend
            self.backend = SimBackend(self.args, self.ros, self.check)
        else:
            self.wait_decision(self.ros.Time.now().to_nsec())  # reject lost line before any command setup
            self.backend = Backend(self.args, self.ros, self.check)
        count, cutoff = 0, self.ros.Time.now().to_nsec()
        while not self.args.max_actions or count < self.args.max_actions:
            action = self.wait_decision(cutoff)
            self.check()
            self.ros.loginfo('ACTION %d/%s: %s', count+1, self.args.max_actions or 'unlimited', action)
            self.backend.play(action)
            wait(self.args.settle, self.check)
            cutoff = self.ros.Time.now().to_nsec()  # discard every image taken during motion
            count += 1
        self.ros.loginfo('ACTION_LIMIT: stopped after %d actions', count)


def main():
    args = arguments()  # --help and validation need no ROS installation
    import rospy
    import rosnode
    runner = None
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    try:
        if '/robotrio_line_follow' in bounded(rosnode.get_node_names, 8):
            raise RuntimeError('This node is already running; stop it explicitly first')
        bounded(lambda: rospy.init_node('robotrio_line_follow', disable_signals=True), 8)
        runner = Follower(args, rospy)
        runner.run()
        return 0
    except (KeyboardInterrupt, InterruptedError):
        return 130
    except TimeoutError as error:
        if str(error) == 'TIME_LIMIT':
            rospy.loginfo('TIME_LIMIT: stopped')
            return 0
        rospy.logerr('%s', error)
        return 1
    except Exception as error:
        rospy.logerr('LINE_FOLLOW_STOP: %s', error)
        return 1
    finally:
        if runner is not None:
            runner.stop()
        if not rospy.is_shutdown():
            rospy.signal_shutdown('Line follower finished')


if __name__ == '__main__':
    raise SystemExit(main())
