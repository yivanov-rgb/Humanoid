#!/usr/bin/env python3
"""Check an anchored AiNex Gazebo instance; never connect this to a real robot.

This checks simulator transport and one joint, not walking, AprilTag detection,
ActionGroup playback, or successful physical grasp. All deadlines use wall time
so a paused or stalled simulated /clock produces a bounded failure.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import sys
import subprocess
import threading
import time

import cv2
import rospy
from controller_manager_msgs.srv import ListControllers
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetWorldProperties
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64


JOINTS = tuple(
    side + '_' + joint
    for side in ('l', 'r')
    for joint in ('hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ank_roll',
                  'ank_pitch', 'sho_roll', 'sho_pitch', 'el_pitch', 'el_yaw',
                  'gripper')
) + ('head_pan', 'head_tilt')


class SmokeFailure(RuntimeError):
    pass


class Samples:
    def __init__(self):
        self.lock = threading.Lock()
        self.first_clock = None
        self.clock = None
        self.clock_received = 0.0
        self.joints = {}
        self.joints_received = 0.0
        self.image = None
        self.image_received = 0.0
        self.camera_info = None
        self.camera_frame = None
        self.bridge = CvBridge()

    def on_clock(self, msg):
        with self.lock:
            value = msg.clock.to_sec()
            if self.first_clock is None:
                self.first_clock = value
            self.clock = value
            self.clock_received = time.monotonic()

    def on_joints(self, msg):
        with self.lock:
            self.joints.update(zip(msg.name, msg.position))
            self.joints_received = time.monotonic()

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Downsampling suppresses the upstream sensor's Gaussian noise, so
            # a noisy but spatially uniform image does not pass the render check.
            reduced = cv2.resize(frame, (80, 60), interpolation=cv2.INTER_AREA)
            luminance = cv2.cvtColor(reduced, cv2.COLOR_BGR2GRAY)
            stats = {'luminance_min': int(luminance.min()),
                     'luminance_max': int(luminance.max()),
                     'luminance_stddev': round(float(luminance.std()), 4)}
        except Exception:
            return
        with self.lock:
            self.image = {'width': msg.width, 'height': msg.height,
                          'encoding': msg.encoding, 'render_stats': stats}
            self.camera_frame = frame.copy()
            self.image_received = time.monotonic()

    def on_camera_info(self, msg):
        with self.lock:
            self.camera_info = {'width': msg.width, 'height': msg.height,
                                'fx': msg.K[0], 'fy': msg.K[4]}


def bounded_call(operation, deadline):
    """ROS service calls have no response timeout; keep a wall-time bound."""
    result = queue.Queue(maxsize=1)

    def worker():
        try:
            result.put((True, operation()))
        except Exception as error:  # deliver the original service failure
            result.put((False, error))

    threading.Thread(target=worker, daemon=True).start()
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise SmokeFailure('Wall-time deadline reached')
    try:
        ok, value = result.get(timeout=remaining)
    except queue.Empty:
        raise SmokeFailure('ROS service response exceeded wall-time deadline')
    if not ok:
        raise SmokeFailure(str(value))
    return value


def wait_until(predicate, deadline, description):
    while time.monotonic() < deadline:
        if rospy.is_shutdown():
            raise SmokeFailure('ROS shutdown while waiting for ' + description)
        value = predicate()
        if value:
            return value
        time.sleep(0.05)
    raise SmokeFailure('Timed out waiting for ' + description)


def wait_service(name, deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise SmokeFailure('Deadline before service ' + name)
    bounded_call(lambda: rospy.wait_for_service(name, timeout=remaining), deadline)


def run_checks(report, deadline, report_path):
    samples = Samples()
    subscribers = [
        rospy.Subscriber('/clock', Clock, samples.on_clock, queue_size=1),
        rospy.Subscriber('/joint_states', JointState, samples.on_joints, queue_size=1),
        rospy.Subscriber('/camera/image_raw', Image, samples.on_image, queue_size=1,
                         buff_size=2 * 1024 * 1024),
        rospy.Subscriber('/camera/camera_info', CameraInfo, samples.on_camera_info,
                         queue_size=1),
    ]
    publisher = rospy.Publisher('/head_pan_controller/command', Float64, queue_size=1)
    baseline = None

    def passed(name, **details):
        report['checks'].append({'name': name, 'ok': True, **details})

    try:
        # A marker prevents accidentally treating a hardware ROS master as sim.
        def simulation_mode_ready():
            mode = bounded_call(lambda: rospy.get_param('/robotrio_sim/mode', ''), deadline)
            use_sim_time = bounded_call(lambda: rospy.get_param('/use_sim_time', False), deadline)
            return mode == 'anchored_inspection' and use_sim_time

        wait_until(simulation_mode_ready, deadline,
                   'robotrio_sim anchored_inspection and use_sim_time=true')
        passed('simulation_mode', mode='anchored_inspection')

        def clock_ready():
            with samples.lock:
                return (samples.clock is not None and samples.first_clock is not None
                        and samples.clock > samples.first_clock
                        and time.monotonic() - samples.clock_received < 2.0)

        wait_until(clock_ready, deadline, 'progressing /clock (unpause Gazebo if paused)')
        with samples.lock:
            passed('clock_progresses', first=samples.first_clock, latest=samples.clock)

        wait_service('/gazebo/get_world_properties', deadline)
        get_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

        def model_ready():
            world = bounded_call(get_world, deadline)
            return world if world.success and 'ainex' in world.model_names else None

        world = wait_until(model_ready, deadline, 'Gazebo model ainex to finish spawning')
        passed('model_exists', model='ainex', world_models=list(world.model_names))

        wait_service('/controller_manager/list_controllers', deadline)
        expected = {name + '_controller' for name in JOINTS} | {'joint_state_controller'}
        list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers',
                                             ListControllers)
        controller_states = {}

        def controllers_ready():
            response = bounded_call(list_controllers, deadline)
            controller_states.clear()
            controller_states.update((c.name, c.state) for c in response.controller)
            return all(controller_states.get(name) == 'running' for name in expected)

        wait_until(controllers_ready, deadline, '24 joint controllers and joint_state_controller')
        passed('controllers_running', joint_controller_count=len(JOINTS),
               total_expected=len(expected), states=controller_states.copy())

        def joints_ready():
            with samples.lock:
                return (all(name in samples.joints and math.isfinite(samples.joints[name])
                            for name in JOINTS)
                        and time.monotonic() - samples.joints_received < 2.0)

        wait_until(joints_ready, deadline, 'fresh finite /joint_states for all 24 joints')
        passed('joint_states', joint_count=len(JOINTS))

        def camera_ready():
            with samples.lock:
                image, info = samples.image, samples.camera_info
                return (image and info and (image['width'], image['height']) == (640, 480)
                        and (info['width'], info['height']) == (640, 480)
                        and info['fx'] > 0 and info['fy'] > 0
                        and time.monotonic() - samples.image_received < 2.0)

        wait_until(camera_ready, deadline, '640x480 camera image and CameraInfo')
        with samples.lock:
            passed('camera', image=samples.image.copy(), camera_info=samples.camera_info.copy())

        snapshot_path = report_path.parent / (report_path.stem + '_camera.png')
        snapshot_path.parent.mkdir(parents=True, exist_ok=True)

        def save_snapshot():
            with samples.lock:
                frame = samples.camera_frame.copy()
                stats = samples.image['render_stats'].copy()
            if not cv2.imwrite(str(snapshot_path), frame):
                raise SmokeFailure('Could not save rendered camera snapshot: ' + str(snapshot_path))
            report['camera_snapshot'] = str(snapshot_path)
            return stats

        # Retain diagnostic evidence even when a uniform frame later fails.
        save_snapshot()

        def rendered_scene_ready():
            with samples.lock:
                if not samples.image:
                    return False
                stats = samples.image['render_stats']
                return (stats['luminance_stddev'] > 3.0
                        and stats['luminance_max'] - stats['luminance_min'] >= 12
                        and time.monotonic() - samples.image_received < 2.0)

        wait_until(rendered_scene_ready, deadline, 'spatially non-uniform rendered camera frame')
        stats = save_snapshot()
        passed('rendered_scene', snapshot=str(snapshot_path), **stats)

        wait_until(lambda: publisher.get_num_connections() > 0, deadline,
                   'head_pan controller subscriber')
        with samples.lock:
            baseline = samples.joints['head_pan']
        target = baseline + 0.12 if baseline <= 1.8 else baseline - 0.12

        def moved():
            publisher.publish(Float64(data=target))
            with samples.lock:
                actual = samples.joints.get('head_pan', float('nan'))
                return (abs(actual - baseline) >= 0.04 and abs(actual - target) <= 0.04
                        and time.monotonic() - samples.joints_received < 2.0)

        wait_until(moved, min(deadline, time.monotonic() + 10.0), 'measured head_pan movement')
        with samples.lock:
            actual = samples.joints['head_pan']
        passed('head_joint_response', baseline_rad=baseline, target_rad=target,
               measured_rad=actual)

        def restored():
            publisher.publish(Float64(data=baseline))
            with samples.lock:
                return abs(samples.joints['head_pan'] - baseline) <= 0.04

        wait_until(restored, min(deadline, time.monotonic() + 10.0), 'head_pan restored')
        passed('head_joint_restored', target_rad=baseline)
        baseline = None
    finally:
        if baseline is not None:
            publisher.publish(Float64(data=baseline))
        for subscriber in subscribers:
            subscriber.unregister()
        publisher.unregister()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/workspace/simulation/artifacts/smoke.json')
    parser.add_argument('--timeout', type=float, default=90.0,
                        help='total wall-time budget in seconds')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    report = {
        'schema_version': 1,
        'mode': 'anchored_inspection',
        'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'ok': False,
        'checks': [],
        'limitations': [
            'Robot is fixed to world; this is not a walking or balance test.',
            'Upstream camera is attached to body, not the moving head.',
            'Inspection camera pitch is a simulation fixture of 0.45 rad, not hardware calibration.',
            'No ActionGroups, servo bridge, AprilTags or grasp verification are tested.',
        ],
    }
    start = time.monotonic()
    exit_code = 1
    try:
        report['runtime'] = {
            'python': sys.version,
            'python_executable': sys.executable,
            'python_prefix': sys.prefix,
            'python_base_prefix': sys.base_prefix,
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'gazebo_package': subprocess.check_output(
                ['dpkg-query', '-W', '-f=${Version}', 'gazebo11'], timeout=5).decode().strip(),
            'vendor_lock': Path('/opt/ainex-vendor.lock').read_text().strip(),
        }
        # disable_signals permits initialization in this bounded worker thread.
        bounded_call(lambda: rospy.init_node('robotrio_sim_smoke', anonymous=True,
                                             disable_signals=True),
                     start + args.timeout)
        run_checks(report, start + args.timeout, Path(args.output))
        report['ok'] = True
        exit_code = 0
    except Exception as error:
        report['error'] = str(error)
    finally:
        report['elapsed_wall_seconds'] = round(time.monotonic() - start, 3)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(output.name + '.tmp')
        temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n',
                             encoding='utf-8')
        temporary.replace(output)
        print(json.dumps({'ok': report['ok'], 'report': str(output),
                          'error': report.get('error')}, ensure_ascii=False))
        if not rospy.is_shutdown():
            rospy.signal_shutdown('smoke finished')
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
