#!/usr/bin/env python3
"""Expand the model, align its servos and attach the camera to the moving head.

The camera mount/optical reference is an explicit simulation fixture, not a
measurement of real hardware. Upstream and hardware calibration stay intact.
"""
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import struct
import numpy as np
import rospkg
import yaml
from robotrio_sim.action_runtime import ServoMap, load_action
from robotrio_sim.initial_pose import align_shoulder_axes, mount_camera_on_head, origin, rotation

arguments = sys.argv[1:]
settings = {}
for key in ('init_actions', 'init_servo_config'):
    matches = [arg for arg in arguments if arg.startswith(key + ':=')]
    if matches:
        settings[key] = matches[0].split(':=', 1)[1]
        arguments.remove(matches[0])
xml = subprocess.check_output(['xacro'] + arguments, universal_newlines=True)
robot = ET.fromstring(xml)
align_shoulder_axes(robot)
mount_camera_on_head(robot)
# Tiny finger inertias cannot use the arm's large, discontinuous dry friction.
# ODE joint damping is implicit; retain a controllable revolute joint.
for name in ('l_gripper', 'r_gripper'):
    dynamics = robot.find("joint[@name='%s']/dynamics" % name)
    dynamics.set('damping', '0.002')
    dynamics.set('friction', '0.0')
xml = ET.tostring(robot, encoding='unicode')
poses = robot.findall("./gazebo[@reference='camera_link']/sensor[@name='camera']/camera/pose")
if len(poses) != 1:
    raise RuntimeError('Expected exactly one upstream camera pose')
if robot.find("joint[@name='inspection_world_mount']") is None:
    # Spawn the joint tree already in stand rather than teleporting an assembled
    # dynamic mechanism through large joint angles after controller startup.
    config = yaml.safe_load(Path(settings['init_servo_config']).read_text())
    servos = ServoMap(config, xml)
    plan = load_action(settings['init_actions'], 'stand', servos)
    angles = servos.targets(plan.frames[-1].positions, 0)
    for joint in robot.findall('joint'):
        if joint.get('name') not in angles:
            continue
        angle = angles[joint.get('name')]
        transform = origin(joint.find('origin'))
        axis = [float(v) for v in joint.find('axis').get('xyz').split()]
        transform[:3, :3] = transform[:3, :3] @ rotation(axis, angle)
        matrix = transform[:3, :3]
        rpy = [np.arctan2(matrix[2, 1], matrix[2, 2]),
               np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0])),
               np.arctan2(matrix[1, 0], matrix[0, 0])]
        joint.find('origin').set('rpy', ' '.join(str(float(v)) for v in rpy))
        joint.set('robotrio_initial_angle', str(angle))
        limit = joint.find('limit')
        for key in ('lower', 'upper'):
            limit.set(key, str(float(limit.get(key)) - angle))
    # Retain upstream EffortJointInterface: a finite-torque servo must react to
    # contact forces. Direct SetPosition actuation can inject motion into the
    # floating body even when targets are constant.
    # Flat sole contacts avoid the dense CAD mesh acting as many small contact
    # edges. Visual meshes stay intact; the collision sole uses their bounds.
    for name in ('l_ank_roll_link', 'r_ank_roll_link'):
        collision = robot.find("link[@name='%s']/collision" % name)
        mesh = collision.find('geometry/mesh')
        package, relative = mesh.get('filename')[len('package://'):].split('/', 1)
        data = (Path(rospkg.RosPack().get_path(package)) / relative).read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        points = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
        low, high = points.min(axis=0), points.max(axis=0)
        center = (low + high) / 2
        center[2] = low[2] + 0.003
        collision.find('origin').set('xyz', ' '.join(str(float(v)) for v in center))
        geometry = collision.find('geometry')
        geometry.remove(mesh)
        ET.SubElement(geometry, 'box', size='%s %s 0.006' % (float(high[0]-low[0]), float(high[1]-low[1])))
print(ET.tostring(robot, encoding='unicode'))
