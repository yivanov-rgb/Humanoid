"""One free-model step with measured heading/displacement, never a route claim."""
import json
import math
from pathlib import Path
import sys
import threading
import time
import uuid

import rospy
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from std_srvs.srv import Empty
from robotrio_sim.srv import ActionCommand

sys.path.insert(0, '/workspace')
from line_follow import bounded


def main():
    rospy.init_node('check_free_step', anonymous=True, disable_rostime=True)
    report = {'ok': False, 'action': 'forward_one_step', 'repeat': 1,
              'route_A_to_B_verified': False, 'samples': []}
    token = uuid.uuid4().hex
    started = False
    call = lambda **kw: rospy.ServiceProxy('/robotrio_sim/actions', ActionCommand)(**kw)
    try:
        if (rospy.get_param('/robotrio_sim/mode', '') != 'dynamic_experiment'
                or not rospy.get_param('/robotrio_sim/initial_pose_ready', False)):
            raise RuntimeError('A verified free standing pose is required')
        get = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

        def sample():
            value = bounded(lambda: get('ainex', 'world'), 3)
            if not value.success:
                raise RuntimeError(value.status_message)
            p, q = value.pose.position, value.pose.orientation
            return {'x': p.x, 'y': p.y, 'z': p.z,
                    'yaw': math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)),
                    'tilt_degrees': math.degrees(math.acos(max(-1, min(1, 1-2*(q.x*q.x + q.y*q.y)))))}

        before = sample()
        report['before'] = before
        result, errors = [], []

        def play():
            try:
                result.append(call(command='play', action_path='/actions', name=report['action'],
                                   repeat=1, pause=0, token=token))
            except Exception as error:
                errors.append(str(error))

        started = True
        worker = threading.Thread(target=play, daemon=True)
        worker.start()
        start = time.monotonic()
        completed_sim = None
        while True:
            if time.monotonic() - start > 25:
                raise TimeoutError('Single-step probe exceeded wall time')
            current = sample()
            turn = math.atan2(math.sin(current['yaw']-before['yaw']), math.cos(current['yaw']-before['yaw']))
            current.update(wall_seconds=time.monotonic()-start, turn_degrees=math.degrees(turn))
            report['samples'].append(current)
            if current['tilt_degrees'] > 25 or current['z'] < before['z'] - 0.04 or abs(current['turn_degrees']) > 10:
                raise AssertionError('Step loses balance or turns instead of moving straight')
            if not worker.is_alive():
                if errors or not result or not result[0].success:
                    raise RuntimeError(errors or (result[0].message if result else 'Missing response'))
                if completed_sim is None:
                    completed_sim = world().sim_time
                    report['playback'] = json.loads(result[0].json_result)
                if world().sim_time - completed_sim >= 5:
                    break
            time.sleep(0.05)
        after = report['samples'][-1]
        dx, dy = after['x']-before['x'], after['y']-before['y']
        report.update(forward_m=dx*math.cos(before['yaw'])+dy*math.sin(before['yaw']),
                      lateral_m=-dx*math.sin(before['yaw'])+dy*math.cos(before['yaw']),
                      maximum_turn_degrees=max(abs(s['turn_degrees']) for s in report['samples']),
                      maximum_tilt_degrees=max(s['tilt_degrees'] for s in report['samples']))
        if report['forward_m'] < 0.003 or abs(report['lateral_m']) > 0.015 or report['maximum_turn_degrees'] > 5:
            raise AssertionError('Measured motion does not confirm a straight forward step')
        report['ok'] = True
        print('Single free step: PASS', flush=True)
        return 0
    except Exception as error:
        report['error'] = str(error)
        print('Single free step: FAIL: ' + str(error), flush=True)
        return 1
    finally:
        if started:
            try:
                bounded(lambda: call(command='stop', token=token), 3)
            except Exception:
                pass
        if not report['ok']:
            try:
                bounded(lambda: rospy.ServiceProxy('/gazebo/pause_physics', Empty)(), 3)
            except Exception as error:
                report['pause_error'] = str(error)
        Path('/workspace/simulation/artifacts/free_step_check.json').write_text(json.dumps(report, indent=2)+'\n')
        rospy.signal_shutdown('single step probe finished')


if __name__ == '__main__':
    raise SystemExit(main())
