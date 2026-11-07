"""Place the unmodified stand ActionGroup on its actual STL sole geometry."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from .action_runtime import ServoMap, load_action


def align_servo_model(robot):
    """Apply the operator's physical direction/pose checks to the CAD model.

    Joint names and servo IDs stay unchanged: *_el_pitch rotates the forearm,
    while *_el_yaw bends the elbow in this CAD chain. Only simulation axes and
    pulse zero references change; hardware calibration and actions stay intact.
    """
    axes = (
        ('l_ank_roll', '1 0 0', '-1 0 0'),
        ('l_sho_roll', '1 0 0', '-1 0 0'),
        ('r_sho_roll', '1 0 0', '-1 0 0'),
        ('r_sho_pitch', '0 -1 0', '0 1 0'),
        ('l_el_pitch', '0 -1 0', '0 1 0'),
        ('head_pan', '0 0 -1', '0 0 1'),
    )
    for name, upstream, corrected in axes:
        joint = robot.find("joint[@name='%s']" % name)
        axis = joint.find('axis')
        if axis.get('xyz') != upstream:
            raise ValueError('Unexpected upstream servo axis: ' + name)
        axis.set('xyz', corrected)
    # Shoulder zero: upper arm parallel to body in the forward/back plane.
    # Elbow references: left 148 / right 850 are 90 degrees; 375 pulses = 90 deg.
    for name, pulse in (
            ('l_sho_pitch', 725), ('r_sho_pitch', 275),
            ('l_el_yaw', 148 + 375), ('r_el_yaw', 850 - 375)):
        robot.find("joint[@name='%s']" % name).set('robotrio_servo_init', str(pulse))


def mount_camera_on_head(robot):
    """Simulation fixture: preserve old view at pulse 330, tilt with the head.

    Upstream mounts camera on body, inside the visual head shell. Put its lens
    in front of that shell and parent it to head_tilt_link. This fixture's
    optical pitch reference is not hardware calibration.
    """
    camera = robot.find("joint[@name='camera']")
    head = origin(robot.find("joint[@name='head_pan']/origin")) @ origin(
        robot.find("joint[@name='head_tilt']/origin"))
    local = np.linalg.inv(head) @ origin(camera.find('origin'))
    # Pinned head_tilt STL extends to X=0.04864 m in its local frame.
    # A 3 mm clearance prevents the CAD shell from occluding the whole image.
    local[0, 3] = max(local[0, 3], 0.052)
    camera.find('parent').set('link', 'head_tilt_link')
    camera.find('origin').set('xyz', ' '.join(str(float(v)) for v in local[:3, 3]))
    # All three upstream fixed joint origins have identity rotation.
    if not np.allclose(local[:3, :3], np.eye(3)):
        raise ValueError('Unexpected upstream camera mounting rotation')
    pitch = 0.45 - (500 - 330) / (180 / 3.1415926 / 240 * 1000)
    # Put optical alignment on the mount itself; keep the nested rendering
    # camera pose zero, so URDF->SDF fixed-joint reduction has one transform.
    camera.find('origin').set('rpy', '0 %s 0' % pitch)
    robot.find("./gazebo[@reference='camera_link']/sensor[@name='camera']/camera/pose").text = (
        '0 0 0 0 0 0')


def origin(element):
    result = np.eye(4)
    if element is None:
        return result
    result[:3, 3] = [float(v) for v in element.get('xyz', '0 0 0').split()]
    roll, pitch, yaw = [float(v) for v in element.get('rpy', '0 0 0').split()]
    for axis, angle in (([0, 0, 1], yaw), ([0, 1, 0], pitch), ([1, 0, 0], roll)):
        result[:3, :3] = result[:3, :3] @ rotation(axis, angle)
    return result


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * (cross @ cross)


def prepare(robot_xml, calibration, action_path, package_path):
    servos = ServoMap(calibration, robot_xml)
    plan = load_action(action_path, 'stand', servos)
    targets = servos.targets(plan.frames[-1].positions, 0)
    for servo in servos.servos.values():
        targets.setdefault(servo.name, 0.0)
    robot = ET.fromstring(robot_xml)
    transforms = {'base_link': np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            local = origin(joint.find('origin'))
            if joint.get('type') in ('revolute', 'continuous'):
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                local[:3, :3] = local[:3, :3] @ rotation(axis, targets[joint.get('name')])
            transforms[joint.find('child').get('link')] = transforms[parent] @ local
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('Initial pose requires a free URDF rooted at base_link')
    soles = {}
    foot_points = []
    sole_pitch = float(np.arctan2(transforms['l_ank_roll_link'][0, 2],
                                 transforms['l_ank_roll_link'][2, 2]))
    base_pitch = -sole_pitch
    base_rotation = rotation([0, 1, 0], base_pitch)
    for name in ('l_ank_roll_link', 'r_ank_roll_link'):
        collision = robot.find("link[@name='%s']/collision" % name)
        mesh = collision.find('geometry/mesh')
        box = collision.find('geometry/box')
        if box is not None:
            size = np.array([float(v) for v in box.get('size').split()])
            vertices = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5)
                                 for z in (-0.5, 0.5)]) * size
            transform = transforms[name] @ origin(collision.find('origin'))
            world = (vertices @ transform[:3, :3].T + transform[:3, 3]) @ base_rotation.T
            foot_points.append(world)
            soles[name] = float(world[:, 2].min())
            continue
        uri = mesh.get('filename')
        if not uri.startswith('package://'):
            raise ValueError('Expected a package STL for ' + name)
        package, relative = uri[len('package://'):].split('/', 1)
        data = (Path(package_path(package)) / relative).read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) != 84 + count * 50:
            raise ValueError('Expected binary STL: ' + uri)
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        vertices = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
        vertices = vertices * np.array([float(v) for v in mesh.get('scale', '1 1 1').split()])
        transform = transforms[name] @ origin(collision.find('origin'))
        world = (vertices @ transform[:3, :3].T + transform[:3, 3]) @ base_rotation.T
        foot_points.append(world)
        soles[name] = float(world[:, 2].min())
    if abs(soles['l_ank_roll_link'] - soles['r_ank_roll_link']) > 0.005:
        raise ValueError('Stand pose does not put the two soles on the same plane')
    height = -min(soles.values()) + 0.001
    mass, weighted = 0.0, np.zeros(3)
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is not None:
            value = float(inertial.find('mass').get('value'))
            center = (transforms[link.get('name')] @ origin(inertial.find('origin')))[:3, 3]
            weighted += value * (base_rotation @ center)
            mass += value
    feet = np.concatenate(foot_points)
    return targets, height, {'action': 'stand', 'sha256': plan.sha256,
                             'base_height': height, 'base_pitch': base_pitch,
                             'com': (weighted / mass).tolist(),
                             'foot_bounds_min': feet.min(axis=0).tolist(),
                             'foot_bounds_max': feet.max(axis=0).tolist(),
                             'foot_origins_expected': {
                                 name: ((base_rotation @ transforms[name][:3, 3]) + [0, 0, height]).tolist()
                                 for name in ('l_ank_roll_link', 'r_ank_roll_link')},
                             'sole_heights_in_base': soles}
