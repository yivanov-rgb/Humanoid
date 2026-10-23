"""Free-body camera-to-action test, with measured motion and bounded cleanup.

Only the actual line_follow.py issues movement commands. ACTION_LIMIT is an
expected end of this bounded experiment, never proof of arrival at AprilTag B.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelProperties, GetWorldProperties, SetModelState
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, Trigger
from robotrio_sim.srv import ActionCommand
from robotrio_sim.track import load_track, nearest_track

sys.path.insert(0, '/workspace')
from line_follow import bounded

ARTIFACTS = Path('/workspace/simulation/artifacts')


def pose():
    result = bounded(lambda: rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)('ainex', 'world'), 3)
    if not result.success:
        raise RuntimeError(result.status_message)
    q, p = result.pose.orientation, result.pose.position
    return {'x': p.x, 'y': p.y, 'z': p.z,
            'yaw_degrees': math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))),
            'tilt_degrees': math.degrees(math.acos(max(-1, min(1, 1-2*(q.x*q.x + q.y*q.y)))))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-actions', type=int, default=10)
    parser.add_argument('--max-seconds', type=float, default=60)
    parser.add_argument('--deadband', type=float, default=0.04)
    parser.add_argument('--line-only', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--line-y', type=float, default=0.0, help='Parallel line offset for sidestep tests, metres')
    parser.add_argument('--view', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.max_actions <= 100 or not 0 < args.max_seconds <= 300:
        parser.error('Use 1..100 actions and 0..300 seconds')
    if not math.isfinite(args.deadband) or not 0 < args.deadband < 0.25:
        parser.error('deadband must be finite and between 0 and 0.25')
    if not math.isfinite(args.line_y) or abs(args.line_y) > 0.05:
        parser.error('line-y must be within +/-0.05 metres')
    rospy.init_node('line_free_probe', anonymous=True, disable_rostime=True)
    report = {'started_at_utc': datetime.now(timezone.utc).isoformat(), 'ok': False,
              'attempted_walk': False, 'route_A_to_B_verified': False,
              'requested_actions': args.max_actions, 'synthetic_images': False,
              'complete_line_verified': False, 'snapshots': []}
    report['deadband'] = args.deadband
    report['line_only_scene'] = True
    report['line_y'] = args.line_y
    process, viewer = None, None
    images = {}
    subscribers = [rospy.Subscriber(topic, Image, lambda msg, key=key: images.update({key: msg}), queue_size=1)
                   for key, topic in (('raw', '/camera/image_raw'), ('debug', '/line_follow/image'))]
    unsafe = False
    try:
        report['mode'] = rospy.get_param('/robotrio_sim/mode', '')
        if report['mode'] != 'dynamic_experiment' or not rospy.get_param('/use_sim_time', False):
            raise RuntimeError('Only the isolated FREE simulation is allowed')
        rospy.wait_for_service('/robotrio_sim/actions', timeout=30)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
        deadline = time.monotonic() + 40
        while not rospy.get_param('/robotrio_sim/initial_pose_ready', False) or 'raw' not in images:
            error = rospy.get_param('/robotrio_sim/initial_pose_error', '')
            if error or time.monotonic() > deadline:
                raise RuntimeError(error or 'Standing pose/camera not ready')
            time.sleep(0.1)
        robot = ET.fromstring(rospy.get_param('/robot_description'))
        props = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)('ainex')
        if props.is_static or robot.find("joint[@name='inspection_world_mount']") is not None:
            raise RuntimeError('Refusing to test walking on an anchored/static model')
        report['fixed_to_world'] = False
        world = bounded(lambda: rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)(), 3)
        report['cube_models'] = [name for name in world.model_names if name.startswith('cube_')]
        if report['cube_models']:
            raise RuntimeError('Cube fixtures must be removed from the default world')
        if args.line_only:
            rospy.logwarn('--line-only is obsolete: cubes were permanently removed from the world')
        if args.line_y:
            state = ModelState(model_name='line_2m_30mm', reference_frame='world')
            state.pose.position.y, state.pose.orientation.w = args.line_y, 1
            response = bounded(lambda: rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state), 3)
            if not response.success:
                raise RuntimeError(response.status_message)
        line_pose = bounded(lambda: rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)(
            'line_2m_30mm', 'world'), 3)
        if not line_pose.success:
            raise RuntimeError(line_pose.status_message)
        p, q = line_pose.pose.position, line_pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        segments = load_track(rospy.get_param('/robotrio_sim/world_file'), p.x, p.y, yaw)
        report['track'] = {'start': segments[0][0], 'end': segments[-1][1],
                           'length_m': sum(math.hypot(bx-ax, by-ay) for (ax, ay), (bx, by) in segments),
                           'world_file': rospy.get_param('/robotrio_sim/world_file')}

        def measured_pose():
            current = pose()
            distance, progress, heading = nearest_track(current['x'], current['y'], segments)
            current.update(track_distance_m=distance, track_progress_m=progress,
                           track_heading_degrees=heading)
            return current

        initial = measured_pose()
        report['snapshots'].append({'phase': 'initial', **initial})
        if initial['tilt_degrees'] > 15:
            raise RuntimeError('Initial standing pose is not stable')
        if args.view:
            viewer = subprocess.Popen(['rosrun', 'rqt_image_view', 'rqt_image_view', '/line_follow/image'],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with (ARTIFACTS / 'line_free.log').open('w') as output:
            process = subprocess.Popen([sys.executable, '-u', '/workspace/robot/line_follow.py',
                                        '--backend', 'sim', '--execute', '--empty-hands',
                                        '--image-orientation', 'standard',
                                        '--deadband', str(args.deadband),
                                        '--max-actions', str(args.max_actions),
                                        '--max-seconds', str(args.max_seconds)],
                                       stdout=output, stderr=subprocess.STDOUT)
            report['attempted_walk'] = True
            deadline = time.monotonic() + args.max_seconds + 10
            while True:
                current = measured_pose()
                report['snapshots'].append({'phase': 'follower', **current})
                faults = []
                if current['tilt_degrees'] > 25:
                    faults.append('tilt %.2f > 25 degrees' % current['tilt_degrees'])
                if current['z'] < initial['z'] - 0.04:
                    faults.append('body height dropped more than 0.04 m')
                if current['track_distance_m'] > 0.08:
                    faults.append('distance to track %.5f > 0.08 m' % current['track_distance_m'])
                if faults:
                    unsafe = True
                    report['safety_trigger'] = faults
                    raise RuntimeError('; '.join(faults) + '; physics paused')
                if process.poll() is not None:
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError('Free follower exceeded wall time')
                time.sleep(0.1)
        log = (ARTIFACTS / 'line_free.log').read_text()
        actions = re.findall(r'ACTION \d+/\d+: (\w+)', log)
        report.update(exit_code=process.returncode, actions=actions,
                      maximum_tilt_degrees=max(s['tilt_degrees'] for s in report['snapshots']),
                      maximum_lateral_error_m=max(s['track_distance_m'] for s in report['snapshots']))
        final = report['snapshots'][-1]
        report['forward_displacement_m'] = final['x'] - initial['x']
        report['final_lateral_displacement_m'] = final['y'] - initial['y']
        report['along_track_progress_m'] = final['track_progress_m'] - initial['track_progress_m']
        report['complete_line_verified'] = False  # bounded probe, no end/AprilTag test
        if (len(actions) != args.max_actions or 'ACTION_LIMIT' not in log
                or report['forward_displacement_m'] < min(0.05, 0.003 * args.max_actions)
                or report['maximum_lateral_error_m'] > 0.05):
            raise RuntimeError('Line following was not established: inspect camera, motion and follower log')
        report['ok'] = True
        return 0
    except Exception as error:
        report['error'] = str(error)
        return 1
    finally:
        if unsafe:
            try:
                bounded(lambda: rospy.ServiceProxy('/gazebo/pause_physics', Empty)(), 3)
            except Exception:
                pass
        if process is not None and process.poll() is None:
            try:
                bounded(lambda: rospy.ServiceProxy('/robotrio_line_follow/stop', Trigger)(), 2)
            except Exception:
                pass
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        if viewer is not None and viewer.poll() is None:
            viewer.terminate()
            try:
                viewer.wait(timeout=5)
            except subprocess.TimeoutExpired:
                viewer.kill()
                viewer.wait(timeout=2)
        try:
            bounded(lambda: rospy.ServiceProxy('/robotrio_sim/actions', ActionCommand)(command='stop'), 3)
        except Exception:
            pass
        for sub in subscribers:
            sub.unregister()
        import cv2
        for key, msg in images.items():
            cv2.imwrite(str(ARTIFACTS / ('line_free_' + key + '.png')), CvBridge().imgmsg_to_cv2(msg, 'bgr8'))
        if process is not None and 'actions' not in report and (ARTIFACTS / 'line_free.log').exists():
            report['actions'] = re.findall(r'ACTION \d+/\d+: (\w+)', (ARTIFACTS / 'line_free.log').read_text())
        if process is not None:
            report['exit_code'] = process.returncode
        # Preserve measurements when the safety monitor aborts an unfinished action.
        samples = report['snapshots']
        if samples:
            report['maximum_tilt_degrees'] = max(s['tilt_degrees'] for s in samples)
            report['maximum_lateral_error_m'] = max(s['track_distance_m'] for s in samples)
            report['forward_displacement_m'] = samples[-1]['x'] - samples[0]['x']
            report['final_lateral_displacement_m'] = samples[-1]['y'] - samples[0]['y']
            report['along_track_progress_m'] = samples[-1]['track_progress_m'] - samples[0]['track_progress_m']
        report['actions_started'] = len(report.get('actions', []))
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        (ARTIFACTS / 'line_free_probe.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'snapshots'}, ensure_ascii=False), flush=True)
        rospy.signal_shutdown('free line probe finished')


if __name__ == '__main__':
    raise SystemExit(main())
