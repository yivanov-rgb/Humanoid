"""Measure uncommanded gripper oscillation at rest and through one step."""
import json
import math
from pathlib import Path
import threading
import time
import uuid

import rospy
from gazebo_msgs.srv import GetWorldProperties
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from robotrio_sim.srv import ActionCommand


def main():
    rospy.init_node('check_grippers', anonymous=True, disable_rostime=True)
    report = {'ok': False, 'samples': []}
    joints = {}
    names = ('l_gripper', 'r_gripper')
    subscriber = rospy.Subscriber('/joint_states', JointState,
                                  lambda msg: joints.update(zip(msg.name, msg.position)), queue_size=1)
    try:
        deadline = time.monotonic() + 40
        while (not rospy.get_param('/robotrio_sim/initial_pose_ready', False)
               or not all(name in joints for name in names)):
            error = rospy.get_param('/robotrio_sim/initial_pose_error', '')
            if error or time.monotonic() > deadline:
                raise RuntimeError(error or 'Standing pose or gripper feedback not ready')
            time.sleep(0.1)
        world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        start = world().sim_time
        result = []
        worker = None
        while world().sim_time - start < 9:
            if time.monotonic() > deadline + 15:
                raise TimeoutError('Gripper observation timed out')
            elapsed = world().sim_time - start
            if elapsed >= 3 and worker is None:
                def play():
                    result.append(rospy.ServiceProxy('/robotrio_sim/actions', ActionCommand)(
                        command='play', action_path='/actions', name='forward_one_step',
                        repeat=1, pause=0, token=uuid.uuid4().hex))
                worker = threading.Thread(target=play, daemon=True)
                worker.start()
            if not all(name in joints for name in names):
                raise RuntimeError('Missing gripper feedback')
            report['samples'].append({'simulation_seconds': elapsed,
                                      **{name: joints[name] for name in names}})
            time.sleep(0.02)
        if not result or not result[0].success:
            raise RuntimeError('Step did not complete')
        report['maximum_error_degrees'] = {name: math.degrees(max(abs(s[name]) for s in report['samples']))
                                           for name in names}
        report['peak_to_peak_degrees'] = {name: math.degrees(max(s[name] for s in report['samples'])
                                         - min(s[name] for s in report['samples'])) for name in names}
        if max(report['maximum_error_degrees'].values()) > 2:
            raise AssertionError('Constant 500-pulse target deviates by more than 2 degrees')
        # Also prove that damping has not turned fingers into fixed joints.
        publishers = {name: rospy.Publisher('/'+name+'_controller/command', Float64,
                                           queue_size=1, latch=True) for name in names}
        report['commanded_positions'] = []
        for target in (0.25, -0.25, 0.0):
            until = world().sim_time + 1
            while world().sim_time < until:
                if time.monotonic() > deadline + 15:
                    raise TimeoutError('Commanded gripper check timed out')
                for publisher in publishers.values():
                    publisher.publish(Float64(target))
                time.sleep(0.02)
            measured = {name: joints[name] for name in names}
            report['commanded_positions'].append({'target': target, 'measured': measured})
            if any(abs(value-target) > math.radians(2) for value in measured.values()):
                raise AssertionError('Gripper did not follow an explicit position command')
        report['ok'] = True
        return 0
    except Exception as error:
        report['error'] = str(error)
        return 1
    finally:
        subscriber.unregister()
        Path('/workspace/simulation/artifacts/gripper_check.json').write_text(json.dumps(report, indent=2)+'\n')
        print({k: v for k, v in report.items() if k != 'samples'}, flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
