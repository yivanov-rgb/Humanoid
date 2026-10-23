#!/usr/bin/env python3
"""Initialize a free model while paused, hold stand, then verify stability."""
import json
import math
from pathlib import Path
import time

import rospkg
import rospy
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelProperties, SetModelConfiguration, SetModelState
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
from gazebo_msgs.srv import GetJointProperties, GetLinkState
from geometry_msgs.msg import Vector3
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from robotrio_sim.initial_pose import prepare


def main():
    rospy.init_node('prepare_start')
    rospy.set_param('/robotrio_sim/initial_pose_ready', False)
    rospy.set_param('/robotrio_sim/initial_pose_error', '')
    report = {'ok': False, 'fixed_to_world': False, 'samples': []}
    pause = None
    physics = None
    set_physics = None
    try:
        if rospy.get_param('/robotrio_sim/mode', '') != 'dynamic_experiment':
            raise RuntimeError('Initial pose setup is only for the isolated free model')
        targets, height, metadata = prepare(
            rospy.get_param('/robot_description'),
            yaml.safe_load(Path(rospy.get_param('~servo_config')).read_text()),
            rospy.get_param('~action_path', '/actions'), rospkg.RosPack().get_path)
        report.update(metadata)
        rospy.wait_for_service('/gazebo/set_model_configuration', timeout=30)
        pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause()
        deadline = time.monotonic() + 30
        properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        while not rospy.is_shutdown() and not properties('ainex').success:
            if time.monotonic() > deadline:
                raise TimeoutError('Model was not spawned')
            time.sleep(0.05)
        response = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)(
            'ainex', 'robot_description', list(targets), list(targets.values()))
        if not response.success:
            raise RuntimeError(response.status_message)
        state = ModelState(model_name='ainex', reference_frame='world')
        state.pose.position.z = height
        state.pose.orientation.y = math.sin(metadata['base_pitch'] / 2)
        state.pose.orientation.w = math.cos(metadata['base_pitch'] / 2)
        response = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state)
        if not response.success:
            raise RuntimeError(response.status_message)
        publishers = {name: rospy.Publisher('/' + name + '_controller/command', Float64,
                                           queue_size=1, latch=True) for name in targets}
        # Latched targets are also received by controllers loaded during pause.
        for name, publisher in publishers.items():
            publisher.publish(Float64(targets[name]))
        # Controllers finish switching only when Gazebo updates. Warm them up
        # without gravity, then reset the pose before enabling normal physics.
        physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)()
        set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        set_physics(physics.time_step, physics.max_update_rate, Vector3(), physics.ode_config)
        rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
        rospy.wait_for_service('/controller_manager/list_controllers', timeout=20)
        controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        deadline = time.monotonic() + 20
        while True:
            states = {c.name: c.state for c in controllers().controller}
            for name, publisher in publishers.items():
                publisher.publish(Float64(targets[name]))
            if all(states.get(name + '_controller') == 'running' for name in targets):
                break
            if rospy.is_shutdown() or time.monotonic() > deadline:
                raise TimeoutError('Joint controllers did not start')
            time.sleep(0.05)
        for _ in range(20):
            for name, publisher in publishers.items():
                publisher.publish(Float64(targets[name]))
            time.sleep(0.05)
        pause()
        response = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)(
            'ainex', 'robot_description', list(targets), list(targets.values()))
        if not response.success:
            raise RuntimeError(response.status_message)
        response = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state)
        if not response.success:
            raise RuntimeError(response.status_message)
        set_physics(physics.time_step, physics.max_update_rate, physics.gravity, physics.ode_config)
        physics = None
        report['prepared_feet'] = {}
        for name in ('l_ank_roll_link', 'r_ank_roll_link'):
            value = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)('ainex::' + name, 'world')
            p = value.link_state.pose.position
            report['prepared_feet'][name] = [p.x, p.y, p.z]
        rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
        get = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        start = time.monotonic()
        start_sim = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and (time.monotonic() - start < 8
                                           or rospy.Time.now().to_sec() - start_sim < 3):
            if time.monotonic() - start > 30:
                raise TimeoutError('Simulator is too slow or paused during stand verification')
            for name, publisher in publishers.items():
                publisher.publish(Float64(targets[name]))
            result = get('ainex', 'world')
            if not result.success:
                raise RuntimeError(result.status_message)
            q, p = result.pose.orientation, result.pose.position
            tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y)))))
            yaw = math.degrees(math.atan2(2 * (q.w*q.z + q.x*q.y),
                                          1 - 2 * (q.y*q.y + q.z*q.z)))
            report['samples'].append({'wall_seconds': time.monotonic() - start,
                                      'x': p.x, 'y': p.y, 'z': p.z,
                                      'yaw_degrees': yaw, 'tilt_degrees': tilt})
            if tilt > 15 or p.z < height - 0.03:
                raise RuntimeError('Initial stand is unstable; physics paused, actions inhibited')
            if math.hypot(p.x, p.y) > 0.01 or abs(yaw) > 3:
                raise RuntimeError('Stand drifts (>10 mm) or rotates (>3 degrees) without commands')
            time.sleep(0.05)
        if rospy.is_shutdown():
            raise InterruptedError('Initialization interrupted')
        report['held_wall_seconds'] = time.monotonic() - start
        report['held_simulation_seconds'] = rospy.Time.now().to_sec() - start_sim
        report['maximum_tilt_degrees'] = max(sample['tilt_degrees'] for sample in report['samples'])
        report['maximum_drift_m'] = max(math.hypot(s['x'], s['y']) for s in report['samples'])
        report['maximum_yaw_degrees'] = max(abs(s['yaw_degrees']) for s in report['samples'])
        report['base_z_range'] = [min(sample['z'] for sample in report['samples']),
                                  max(sample['z'] for sample in report['samples'])]
        report['ok'] = True
        rospy.set_param('/robotrio_sim/initial_pose_ready', True)
        rospy.loginfo('INITIAL_POSE_READY: stand, base height %.4f m, no world fixture', height)
        return 0
    except Exception as error:
        report['error'] = str(error)
        rospy.set_param('/robotrio_sim/initial_pose_error', str(error))
        if pause is not None:
            pause()
            report['joint_diagnostics'] = {}
            for name in ('l_hip_pitch', 'l_knee', 'l_ank_pitch', 'r_hip_pitch', 'r_knee', 'r_ank_pitch'):
                value = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)('ainex::' + name)
                report['joint_diagnostics'][name] = {'target': targets.get(name),
                                                    'position': list(value.position), 'rate': list(value.rate)}
            report['feet_diagnostics'] = {}
            for name in ('l_ank_roll_link', 'r_ank_roll_link'):
                value = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)('ainex::' + name, 'world')
                report['feet_diagnostics'][name] = str(value.link_state.pose)
        if physics is not None and set_physics is not None:
            set_physics(physics.time_step, physics.max_update_rate, physics.gravity, physics.ode_config)
        rospy.logerr('INITIAL_POSE_FAILED: %s', error)
        return 1
    finally:
        Path('/workspace/simulation/artifacts/initial_pose.json').write_text(
            json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
