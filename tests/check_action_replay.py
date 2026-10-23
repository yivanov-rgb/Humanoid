"""Integration check using explicitly synthetic .d6a files, never robot actions.

Generate fixtures with --make-fixtures DIR, then mount DIR at /actions in an
isolated simulation and run with --report FILE. This is not a grasp test.
"""
import argparse
from contextlib import closing
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
import threading
import time
import uuid


def make_fixtures(directory):
    import yaml
    import rospkg
    config = Path(rospkg.RosPack().get_path('ainex_kinematics')) / 'config/servo_controller.yaml'
    controllers = yaml.safe_load(config.read_text())['controllers'].values()
    pulses = {c['servo']['id']: c['servo']['init'] for c in controllers
              if isinstance(c, dict) and 'servo' in c}
    directory.mkdir(parents=True, exist_ok=True)
    for name, duration, invalid in [('inspection_head', 400, False),
                                     ('inspection_long', 4000, False),
                                     ('inspection_invalid', 400, True)]:
        path = directory / (name + '.d6a')
        if path.exists():
            raise RuntimeError('Refusing to overwrite fixture: ' + str(path))
        with closing(sqlite3.connect(str(path))) as db:
            db.execute('CREATE TABLE ActionGroup ("Index" INTEGER PRIMARY KEY, Time INTEGER, '
                       + ', '.join('Servo%d INTEGER' % sid for sid in range(1, 25)) + ')')
            for index, head in ((1, 530), (2, 500)):
                row = [pulses[sid] for sid in range(1, 25)]
                row[22] = 1001 if invalid and index == 2 else head
                db.execute('INSERT INTO ActionGroup VALUES (' + ','.join(['?'] * 26) + ')',
                           [index, duration] + row)
            db.commit()
    print('Created synthetic inspection fixtures in ' + str(directory))


def check_replay(report_path):
    import rospy
    from robotrio_sim.srv import ActionCommand
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Empty
    rospy.init_node('robotrio_action_integration', anonymous=True)
    rospy.wait_for_service('/robotrio_sim/actions', timeout=60)
    samples = []
    def observe(msg):
        if 'head_pan' in msg.name:
            samples.append(msg.position[msg.name.index('head_pan')])
    sub = rospy.Subscriber('/joint_states', JointState, observe, queue_size=50)
    report = {'started_at_utc': datetime.now(timezone.utc).isoformat(),
              'synthetic_actions_only': True, 'grasp_verified': False, 'ok': False, 'checks': []}
    def call(command, name='', repeat=1, token=None):
        return rospy.ServiceProxy('/robotrio_sim/actions', ActionCommand)(
            command=command, action_path='/actions', name=name, repeat=repeat,
            pause=0.05, token=token or (uuid.uuid4().hex if command == 'play' else ''))
    def verify(name, condition, details=None):
        if not condition:
            raise AssertionError(name + ': ' + str(details))
        report['checks'].append({'name': name, 'ok': True, 'details': details})
    def async_play():
        output = []
        thread = threading.Thread(target=lambda: output.append(call('play', 'inspection_long')), daemon=True)
        thread.start()
        time.sleep(0.4)
        return thread, output
    try:
        listed = call('list')
        verify('list_fixtures', listed.success and 'inspection_head' in listed.json_result)
        info = call('info', 'inspection_head')
        verify('validate_all_frames', info.success and json.loads(info.json_result)['frames'] == 2)
        result = call('play', 'inspection_head', repeat=2)
        details = json.loads(result.json_result)
        verify('replay_four_acknowledged_frames', result.success and details['acknowledged_frames'] == 4
               and details['completed_repeats'] == 2 and details['grasp_verified'] is False, details)
        verify('measured_head_motion', bool(samples) and max(samples) > 0.04,
               {'max_rad': max(samples) if samples else None})
        invalid = call('play', 'inspection_invalid')
        verify('invalid_later_frame_rejected_before_motion', not invalid.success
               and json.loads(invalid.json_result).get('acknowledged_frames') == 0, invalid.message)
        thread, output = async_play()
        second = call('play', 'inspection_head')
        verify('concurrent_playback_rejected', not second.success and 'already' in second.message, second.message)
        stopped = call('stop')
        thread.join(6)
        verify('stop_interrupts_playback', stopped.success and not thread.is_alive()
               and bool(output) and not output[0].success, output[0].message if output else None)
        thread, output = async_play()
        rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
        try:
            thread.join(6)
            verify('paused_clock_aborts_in_bounded_time', not thread.is_alive() and bool(output)
                   and not output[0].success, output[0].message if output else None)
        finally:
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
        report['ok'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        call('stop')
        sub.unregister()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        rospy.signal_shutdown('integration complete')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--make-fixtures', type=Path)
    parser.add_argument('--report', type=Path, default=Path('/workspace/simulation/artifacts/action_replay.json'))
    args = parser.parse_args()
    if args.make_fixtures:
        make_fixtures(args.make_fixtures)
    else:
        check_replay(args.report)
