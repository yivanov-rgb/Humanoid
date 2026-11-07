"""Simulation-only placement geometry and short, fixed-foot squat planning.

Joint positions use the supplied URDF coordinates, including the stand offsets
in robot_description.py. The block is an ideal rigid grasp, without payload
mass, contact, release, perception or hardware servo commands.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

from .initial_pose import origin, rotation


BLOCK_EDGE = 0.035
TONGUE_HEIGHT = 0.020


def orientation_error(target, actual):
    matrix = target @ actual.T
    angle = math.acos(float(np.clip((np.trace(matrix) - 1) / 2, -1, 1)))
    if angle < 1e-8:
        return np.zeros(3)
    if math.pi - angle < 1e-5:
        _, vectors = np.linalg.eigh((matrix + np.eye(3)) / 2)
        return vectors[:, -1] * angle
    return angle / (2 * math.sin(angle)) * np.array([
        matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1]])


def hand_rotation(side):
    """Fingers forward, palm inward; columns are wrist axes in level body axes."""
    if side not in ('left', 'right'):
        raise ValueError('hand must be left or right')
    sign = 1 if side == 'left' else -1
    return np.array([[0., sign, 0.], [0., 0., sign], [1., 0., 0.]])


def block_in_wrist(side):
    result = np.eye(4)
    result[:3, :3] = hand_rotation(side).T
    # Ideal 35 mm block between the fingers. This is a simulation fixture,
    # not measured hardware grasp geometry.
    result[:3, 3] = [0., 0.112 if side == 'left' else -0.112, -0.020]
    return result


def block_points(wrist, side):
    block = wrist @ block_in_wrist(side)
    bottom = block[:3, 3] - block[:3, 2] * BLOCK_EDGE / 2
    tip = bottom - block[:3, 2] * TONGUE_HEIGHT
    return block, bottom, tip


class Kinematics:
    def __init__(self, robot_xml):
        robot = ET.fromstring(robot_xml)
        self.joints = {joint.get('name'): joint for joint in robot.findall('joint')}
        body_mount = self.joints.get('base_link_to_body')
        if (body_mount is None or body_mount.get('type') != 'fixed'
                or not np.allclose(origin(body_mount.find('origin')), np.eye(4))):
            raise ValueError('Expected body_link fixed at the base_link origin')
        self.parents = {joint.find('child').get('link'): joint
                        for joint in self.joints.values()}
        self.limits = {name: (float(j.find('limit').get('lower')),
                              float(j.find('limit').get('upper')))
                       for name, j in self.joints.items() if j.get('type') == 'revolute'}
        self.origins = {name: origin(j.find('origin')) for name, j in self.joints.items()}
        self.axes = {name: [float(v) for v in j.find('axis').get('xyz').split()]
                     for name, j in self.joints.items() if j.get('type') == 'revolute'}
        self.chains = {}

    def chain(self, link):
        if link not in self.chains:
            chain, child = [], link
            while child != 'body_link':
                joint = self.parents[child]
                chain.append(joint.get('name'))
                child = joint.find('parent').get('link')
            self.chains[link] = list(reversed(chain))
        return self.chains[link]

    def pose(self, link, positions):
        result = np.eye(4)
        for name in self.chain(link):
            transform = self.origins[name].copy()
            if name in self.axes:
                transform[:3, :3] = transform[:3, :3] @ rotation(self.axes[name], positions[name])
            result = result @ transform
        return result

    def solve(self, names, seed, residual, tolerance=0.001, iterations=100):
        positions = dict(seed)
        low, high = np.array([self.limits[name] for name in names]).T
        q = np.clip([positions[name] for name in names], low, high)
        for _ in range(iterations):
            positions.update(zip(names, q))
            error = residual(positions)
            if np.linalg.norm(error) < tolerance:
                return positions
            jacobian = np.empty((len(error), len(names)))
            for column, name in enumerate(names):
                perturbed = dict(positions)
                perturbed[name] += 1e-5
                jacobian[:, column] = (residual(perturbed) - error) / 1e-5
            step = np.linalg.lstsq(
                np.vstack((jacobian, 0.001 * np.eye(len(names)))),
                np.concatenate((-error, np.zeros(len(names)))), rcond=None)[0]
            step *= min(1., 0.10 / max(1e-12, float(np.max(np.abs(step)))))
            best, best_norm = q, np.linalg.norm(error)
            for fraction in (1., 0.5, 0.25, 0.125):
                candidate = np.clip(q + step * fraction, low, high)
                trial = dict(positions)
                trial.update(zip(names, candidate))
                norm = np.linalg.norm(residual(trial))
                if norm < best_norm:
                    best, best_norm = candidate, norm
            if np.max(np.abs(best - q)) < 1e-8:
                break
            q = best
        raise ValueError('Target is unreachable within URDF limits; residual=%.6f' % best_norm)

    def arm_at_entry(self, side, seed, world_body, entry_height, heading, observed_wrist=None):
        prefix = 'l' if side == 'left' else 'r'
        names = [prefix + '_' + name for name in ('sho_pitch', 'sho_roll', 'el_pitch', 'el_yaw')]
        wrist = prefix + '_el_yaw_link'
        desired = rotation([0, 0, 1], heading) @ hand_rotation(side)
        correction = np.eye(4)
        if observed_wrist is not None:
            # Re-anchor this local IK step to measured task pose. ODE link
            # constraints need not coincide exactly with ideal URDF FK.
            # This is refreshed each iteration, not a changed grasp transform.
            nominal = world_body @ self.pose(wrist, seed)
            correction = observed_wrist @ np.linalg.inv(nominal)
            if (np.linalg.norm(observed_wrist[:3, 3] - nominal[:3, 3]) > 0.03
                    or np.linalg.norm(orientation_error(observed_wrist[:3, :3], nominal[:3, :3])) > 0.1):
                raise ValueError('Measured wrist differs too far from URDF FK for local correction')

        def residual(q):
            pose = correction @ world_body @ self.pose(wrist, q)
            _, _, tip = block_points(pose, side)
            return np.concatenate((orientation_error(desired, pose[:3, :3]),
                                   [(tip[2] - entry_height) * 10]))

        seeds = [dict(seed)]
        for pitch in (0.6, 1.0, -0.6):
            candidate = dict(seed)
            sign = 1 if side == 'left' else -1
            physical = [pitch, sign * math.pi / 2, 0., sign * (pitch - math.pi / 2)]
            for name, value in zip(names, physical):
                candidate[name] = value - float(self.joints[name].get('robotrio_initial_angle', '0'))
            seeds.append(candidate)
        for candidate in seeds:
            try:
                return self.solve(names, candidate, residual)
            except ValueError:
                pass
        raise ValueError('Entry height %.3f m is unreachable with fingers forward and palm inward' % entry_height)

    def squat(self, seed, world_body, world_feet, distance=TONGUE_HEIGHT, count=41):
        """Hold both feet fixed and lower the body vertically, preserving its rotation."""
        if not math.isfinite(distance) or not 0 < distance <= 0.05:
            raise ValueError('Squat distance must be in (0, 0.05] m')
        positions, frames = dict(seed), []
        for depth in np.linspace(0, distance, count):
            body = world_body.copy()
            body[2, 3] -= depth
            for prefix in ('l', 'r'):
                foot = prefix + '_ank_roll_link'
                names = [name for name in self.chain(foot) if name in self.axes]
                target = np.linalg.inv(body) @ world_feet[foot]

                def residual(q):
                    actual = self.pose(foot, q)
                    return np.concatenate(((actual[:3, 3] - target[:3, 3]) * 10,
                                           orientation_error(target[:3, :3], actual[:3, :3])))

                positions = self.solve(names, positions, residual, tolerance=0.0002)
            frames.append(dict(positions))
        return frames
