"""Check free standing, foot contact and absence of a world fixture. No gait."""
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rospy
from gazebo_msgs.srv import GetLinkState, GetModelProperties, GetModelState, GetPhysicsProperties, GetWorldProperties
from tf.transformations import quaternion_matrix

from robotrio_sim.initial_pose import origin


def main():
    rospy.init_node('check_initial_pose', anonymous=True, disable_rostime=True)
    report = {'ok': False, 'walking_commands_sent': False,
              'started_at_utc': datetime.now(timezone.utc).isoformat(), 'samples': []}
    try:
        end = time.monotonic() + 40
        while time.monotonic() < end:
            error = rospy.get_param('/robotrio_sim/initial_pose_error', '')
            if error:
                raise RuntimeError(error)
            if rospy.get_param('/robotrio_sim/initial_pose_ready', False):
                break
            time.sleep(0.1)
        else:
            raise TimeoutError('Initial pose was not confirmed')
        robot = ET.fromstring(rospy.get_param('/robot_description'))
        if robot.find("joint[@name='inspection_world_mount']") is not None:
            raise AssertionError('World fixture still present')
        properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)('ainex')
        physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)()
        if properties.is_static or physics.pause or physics.gravity.z > -9:
            raise AssertionError('Expected a dynamic, unpaused model with normal gravity')
        report.update({'fixed_to_world': False, 'is_static': properties.is_static,
                       'gravity_z': physics.gravity.z})
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        get_link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        get_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        report['arms'] = {}
        for side in ('l', 'r'):
            shoulder = get_link('ainex::' + side + '_sho_pitch_link', 'world').link_state.pose.position
            hand = get_link('ainex::' + side + '_gripper_link', 'world').link_state.pose.position
            report['arms'][side] = {'shoulder_z': shoulder.z, 'hand_z': hand.z}
            if hand.z > shoulder.z - 0.03:
                raise AssertionError('Hand is not lowered below shoulder: ' + side)
        start = time.monotonic()
        # disable_rostime keeps startup responsive while Gazebo is paused; read
        # simulation time explicitly rather than accidentally timing wall time.
        start_sim = get_world().sim_time
        reference = None
        while time.monotonic() - start < 5 or get_world().sim_time - start_sim < 5:
            if time.monotonic() - start > 40:
                raise TimeoutError('Standing observation did not reach five simulation seconds')
            state = get_model('ainex', 'world')
            q, p = state.pose.orientation, state.pose.position
            tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x*q.x + q.y*q.y)))))
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            if reference is None:
                reference = (p.x, p.y, yaw)
            drift = math.hypot(p.x - reference[0], p.y - reference[1])
            turn = math.degrees(math.atan2(math.sin(yaw - reference[2]), math.cos(yaw - reference[2])))
            sample = {'x': p.x, 'y': p.y, 'z': p.z, 'yaw_degrees': math.degrees(yaw),
                      'drift_m': drift, 'turn_degrees': turn, 'tilt_degrees': tilt, 'soles': {}}
            for name in ('l_ank_roll_link', 'r_ank_roll_link'):
                collision = robot.find("link[@name='%s']/collision" % name)
                size = np.array([float(v) for v in collision.find('geometry/box').get('size').split()])
                vertices = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5)
                                     for z in (-0.5, 0.5)]) * size
                pose = get_link('ainex::' + name, 'world').link_state.pose
                p, orientation = pose.position, pose.orientation
                transform = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
                transform[:3, 3] = [p.x, p.y, p.z]
                transform = transform @ origin(collision.find('origin'))
                world = vertices @ transform[:3, :3].T + transform[:3, 3]
                sample['soles'][name] = float(world[:, 2].min())
            report['samples'].append(sample)
            if tilt > 15 or any(abs(z) > 0.005 for z in sample['soles'].values()):
                raise AssertionError('Body tilt or sole height outside the standing tolerance')
            if drift > 0.005 or abs(turn) > 1:
                raise AssertionError('Uncommanded stand motion exceeds 5 mm or 1 degree')
            time.sleep(0.2)
        report['observed_simulation_seconds'] = get_world().sim_time - start_sim
        report['maximum_drift_m'] = max(s['drift_m'] for s in report['samples'])
        report['maximum_turn_degrees'] = max(abs(s['turn_degrees']) for s in report['samples'])
        report['maximum_tilt_degrees'] = max(s['tilt_degrees'] for s in report['samples'])
        report['height_range'] = [min(s['z'] for s in report['samples']),
                                   max(s['z'] for s in report['samples'])]
        if report['height_range'][1] - report['height_range'][0] > 0.01:
            raise AssertionError('Body does not hold its initial height')
        report['ok'] = True
        print('Initial stand: PASS; lowered arms, foot contact, no uncommanded translation/rotation')
        return 0
    except Exception as error:
        report['error'] = str(error)
        print('Initial stand: FAIL: ' + str(error))
        return 1
    finally:
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        Path('/workspace/simulation/artifacts/initial_stand_check.json').write_text(
            json.dumps(report, indent=2) + '\n')
        rospy.signal_shutdown('stand check finished')


if __name__ == '__main__':
    raise SystemExit(main())
