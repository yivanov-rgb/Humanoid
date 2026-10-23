"""Real Gazebo camera + transferred robot ActionGroups, anchored integration.

Run only inside the isolated simulation container. No synthetic image publisher
is used: line loss/correction is induced by moving the existing Gazebo line.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from ros_robot_controller.msg import SetBusServosPosition
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Empty, Trigger
from robotrio_sim.srv import ActionCommand

ROOT = Path('/workspace')
ARTIFACTS = ROOT / 'simulation/artifacts'
sys.path.insert(0, str(ROOT))
import line_follow as line


def wait_until(condition, seconds=15):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.03)
    raise TimeoutError('Condition did not become true within %.1f seconds' % seconds)


class Checks:
    def __init__(self):
        self.bus = []
        self.images = deque(maxlen=1)
        self.debug = 0
        self.joints = []
        self.children = []
        self.report = {'started_at_utc': datetime.now(timezone.utc).isoformat(),
                       'ok': False, 'mode': rospy.get_param('/robotrio_sim/mode', ''),
                       'real_robot_actions': True, 'camera': '/camera/image_raw',
                       'synthetic_images': False, 'route_A_to_B_verified': False,
                       'apriltag_integration_verified': False, 'checks': []}
        self.subs = [
            rospy.Subscriber('/ros_robot_controller/bus_servo/set_position', SetBusServosPosition,
                             lambda msg: self.bus.append((time.monotonic(), rospy.Time.now().to_sec(), msg)), queue_size=100),
            rospy.Subscriber('/camera/image_raw', Image, lambda msg: self.images.append(msg), queue_size=1),
            rospy.Subscriber('/line_follow/image', Image, self.on_debug, queue_size=1),
            rospy.Subscriber('/joint_states', JointState, lambda msg: self.joints.append(msg), queue_size=100)]

    def on_debug(self, msg):
        self.debug += 1
        if self.debug == 1:
            import cv2
            from cv_bridge import CvBridge
            cv2.imwrite(str(ARTIFACTS / 'line_debug.png'), CvBridge().imgmsg_to_cv2(msg, 'bgr8'))

    def verify(self, name, condition, details=None):
        self.report['checks'].append({'name': name, 'ok': bool(condition), 'details': details})
        print(name + ': ' + ('PASS' if condition else 'FAIL'), flush=True)
        if not condition:
            raise AssertionError(name + ': ' + str(details))

    def call(self, command, name=''):
        return line.bounded(lambda: rospy.ServiceProxy('/robotrio_sim/actions', ActionCommand)(
            command=command, action_path='/actions', name=name, repeat=1, pause=0,
            token=uuid.uuid4().hex if command == 'play' else ''), 30)

    def start(self, name, *arguments):
        log_path = ARTIFACTS / ('line_' + name + '.log')
        output = log_path.open('w')
        process = subprocess.Popen([sys.executable, '-u', str(ROOT / 'line_follow.py'),
                                    '--backend', 'sim', '--image-orientation', 'standard',
                                    '--max-seconds', '45'] + list(arguments),
                                   stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.children.append((process, output))
        return process, log_path

    def finish(self, process, log_path, seconds=25):
        try:
            process.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)
            raise TimeoutError('Follower did not finish: ' + str(log_path))
        return log_path.read_text()

    def stop_follower(self):
        rospy.wait_for_service('/robotrio_line_follow/stop', timeout=10)
        return line.bounded(lambda: rospy.ServiceProxy('/robotrio_line_follow/stop', Trigger)(), 4)

    def move_line(self, y):
        state = ModelState(model_name='line_2m_30mm', reference_frame='world')
        state.pose.position.y = y
        state.pose.orientation.w = 1
        response = line.bounded(lambda: rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state), 5)
        if not response.success:
            raise RuntimeError(response.status_message)
        time.sleep(0.5)  # let the rendering thread publish the updated scene

    def run(self):
        self.verify('isolated_anchored_simulation', self.report['mode'] == 'anchored_inspection'
                    and rospy.get_param('/use_sim_time', False))
        rospy.wait_for_service('/robotrio_sim/actions', timeout=30)
        wait_until(lambda: self.images and self.joints, 30)
        selected = json.loads((ROOT / 'simulation/actiongroups_selection.json').read_text())['actions']
        digests = {entry['name']: entry['sha256'] for entry in selected}
        playback = []
        for entry in selected:
            name = entry['name']
            info = self.call('info', name)
            if not info.success:
                raise AssertionError('Action validation failed: ' + name + ': ' + info.message)
            details = json.loads(info.json_result)
            if details['sha256'] != entry['sha256']:
                raise AssertionError('Transferred file changed: ' + name)
            start = len(self.joints)
            played = self.call('play', name)
            result = json.loads(played.json_result)
            if not played.success or result['acknowledged_frames'] != details['frames']:
                raise AssertionError('Incomplete action: ' + name + ': ' + played.message)
            if len(self.joints) <= start:
                raise AssertionError('No joint feedback during action: ' + name)
            playback.append({'name': name, 'frames': details['frames'],
                             'duration_seconds': details['duration_seconds'], 'sha256': details['sha256'],
                             'acknowledged_frames': result['acknowledged_frames']})
        self.verify('all_nine_real_actions_validated_and_replayed', len(playback) == 9, playback)

        start_bus, start_debug = len(self.bus), self.debug
        process, path = self.start('observe')
        wait_until(lambda: self.debug >= start_debug + 4 or process.poll() is not None)
        stopped = self.stop_follower()
        log = self.finish(process, path)
        self.verify('observation_has_debug_frames_and_no_motion', stopped.success
                    and self.debug >= start_debug + 4 and len(self.bus) == start_bus
                    and 'OBSERVE' in log, {'log': str(path), 'new_bus_frames': len(self.bus) - start_bus})

        start_bus = len(self.bus)
        process, path = self.start('two_steps', '--execute', '--empty-hands', '--max-actions', '2')
        log = self.finish(process, path)
        self.verify('camera_to_two_real_actions_then_limit', process.returncode == 0
                    and log.count('ACTION ') == 2 and 'forward_one_step' in log
                    and 'ACTION_LIMIT' in log and len(self.bus) - start_bus == 12,
                    {'log': str(path), 'new_bus_frames': len(self.bus) - start_bus})

        # The physical scene, not a fake ROS image, causes a lateral error.
        self.move_line(0.08)
        process, path = self.start('left_correction', '--execute', '--empty-hands', '--max-actions', '1')
        log = self.finish(process, path)
        self.verify('shifted_gazebo_line_causes_left_turn', 'ACTION 1/1: turn_left' in log
                    and 'ACTION_LIMIT' in log, {'log': str(path)})
        self.move_line(-0.08)
        process, path = self.start('right_correction', '--execute', '--empty-hands', '--max-actions', '1')
        log = self.finish(process, path)
        self.verify('shifted_gazebo_line_causes_right_turn', 'ACTION 1/1: turn_right' in log
                    and 'ACTION_LIMIT' in log, {'log': str(path)})

        self.move_line(1.0)
        start_bus = len(self.bus)
        process, path = self.start('lost_line', '--execute', '--empty-hands')
        log = self.finish(process, path)
        self.verify('missing_line_stops_without_blind_steps', 'NO_STABLE_LINE' in log
                    and len(self.bus) == start_bus, {'log': str(path)})
        self.move_line(0.0)

        start_bus = len(self.bus)
        process, path = self.start('missing_camera', '--execute', '--empty-hands',
                                    '--image-topic', '/line_check/no_camera', '--frame-timeout', '0.7')
        log = self.finish(process, path)
        self.verify('missing_camera_prevents_motion', 'CAMERA_STALE' in log
                    and len(self.bus) == start_bus, {'log': str(path)})

        start_bus = len(self.bus)
        process, path = self.start('stop_between_steps', '--execute', '--empty-hands',
                                    '--max-actions', '10', '--settle', '3')
        wait_until(lambda: len(self.bus) >= start_bus + 6 or process.poll() is not None)
        final_frame = self.bus[-1]
        wait_until(lambda: rospy.Time.now().to_sec() > final_frame[1] + final_frame[2].duration + 0.05)
        stopped = self.stop_follower()
        log = self.finish(process, path)
        self.verify('route_stop_between_actions_prevents_next_step', stopped.success
                    and log.count('ACTION ') == 1 and len(self.bus) - start_bus == 6,
                    {'log': str(path), 'new_bus_frames': len(self.bus) - start_bus})

        start_bus = len(self.bus)
        process, path = self.start('paused_camera', '--execute', '--empty-hands',
                                    '--max-actions', '10', '--frame-timeout', '0.8')
        wait_until(lambda: len(self.bus) > start_bus or process.poll() is not None)
        rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
        try:
            log = self.finish(process, path, seconds=8)
            self.verify('paused_camera_cancels_current_action', process.returncode != 0
                        and log.count('ACTION ') == 1 and 'CAMERA_STALE' in log,
                        {'log': str(path), 'new_bus_frames': len(self.bus) - start_bus})
        finally:
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
        self.verify('source_copies_unchanged', all(
            hashlib.sha256((Path('/actions') / (name + '.d6a')).read_bytes()).hexdigest() == digest
            for name, digest in digests.items()))
        self.report['ok'] = True

    def cleanup(self):
        for process, output in self.children:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            output.close()
        try:
            self.call('stop')
            self.move_line(0)
        finally:
            for sub in self.subs:
                sub.unregister()


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument('--report', type=Path, default=ARTIFACTS / 'line_follow_integration.json')
    args = args.parse_args()
    rospy.init_node('line_follow_integration', anonymous=True)
    checks = Checks()
    try:
        checks.run()
        return 0
    except Exception as error:
        checks.report['error'] = str(error)
        print('FAILED: ' + str(error), flush=True)
        return 1
    finally:
        try:
            checks.cleanup()
        except Exception as error:
            checks.report['cleanup_error'] = str(error)
            checks.report['ok'] = False
        checks.report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        args.report.write_text(json.dumps(checks.report, indent=2, ensure_ascii=False) + '\n')
        rospy.signal_shutdown('line checks finished')


if __name__ == '__main__':
    raise SystemExit(main())
