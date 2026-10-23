"""Geometry of an initial floor pose, without ROS or a running simulator."""
from contextlib import closing
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'simulation/ros_ws/src/robotrio_sim/src'))
try:
    from robotrio_sim.initial_pose import align_shoulder_axes, mount_camera_on_head, origin, prepare, rotation
except ImportError:
    prepare = None


@unittest.skipIf(prepare is None, 'NumPy is supplied by the simulation image')
class InitialPoseTests(unittest.TestCase):
    def test_camera_follows_head_and_lower_pulse_points_further_down(self):
        import math
        import numpy as np
        path = Path(__file__).resolve().parents[1] / 'simulation/vendor/ainex/src/ainex_simulations/ainex_description/urdf/ainex.urdf.xacro'
        robot = ET.fromstring(path.read_text())
        ET.SubElement(ET.SubElement(ET.SubElement(ET.SubElement(robot, 'gazebo', reference='camera_link'),
            'sensor', name='camera'), 'camera'), 'pose').text = '0 0 0 0 0.45 0'
        old = origin(robot.find("joint[@name='camera']/origin"))
        mount_camera_on_head(robot)
        self.assertEqual(robot.find("joint[@name='camera']/parent").get('link'), 'head_tilt_link')
        head = origin(robot.find("joint[@name='head_pan']/origin")) @ origin(robot.find("joint[@name='head_tilt']/origin"))
        mounted = head @ origin(robot.find("joint[@name='camera']/origin"))
        np.testing.assert_allclose(mounted[1:3, 3], old[1:3, 3])
        self.assertGreater(mounted[0, 3], old[0, 3])
        pitch = float(robot.find("joint[@name='camera']/origin").get('rpy').split()[1])
        views = []
        for pulse in (330, 260):
            angle = (pulse-500)/(180/3.1415926/240*1000)
            direction = rotation([0, -1, 0], angle) @ rotation([0, 1, 0], pitch) @ [1, 0, 0]
            views.append(math.atan2(-direction[2], direction[0]))
        self.assertAlmostEqual(views[0], 0.45)
        self.assertGreater(views[1], views[0])

    def test_hardware_stand_shoulder_roll_lowers_both_arms(self):
        # Actual CAD axes and hardware stand roll pulses, evaluated independently
        # of rebasing: distal arm points outward along +/-Y in the zero pose.
        path = Path(__file__).resolve().parents[1] / 'simulation/vendor/ainex/src/ainex_simulations/ainex_description/urdf/ainex.urdf.xacro'
        robot = ET.fromstring(path.read_text())
        align_shoulder_axes(robot)
        for name, pulse, direction in (('l_sho_roll', 830, 1), ('r_sho_roll', 170, -1)):
            axis = [float(v) for v in robot.find("joint[@name='%s']/axis" % name).get('xyz').split()]
            angle = (pulse - 500) / (180 / 3.1415926 / 240 * 1000)
            distal = rotation(axis, angle) @ [0, direction * 0.1, 0]
            self.assertLess(distal[2], -0.09)

    def model(self):
        parts = ['<robot><link name="base_link"><inertial><mass value="1"/></inertial></link>']
        for sid in range(1, 25):
            name = {1: 'l_ank_roll_link', 2: 'r_ank_roll_link'}.get(sid, 'link_%d' % sid)
            parts.append('<link name="%s"><collision><geometry><box size="0.08 0.04 0.01"/>'
                         '</geometry></collision></link>' % name)
            parts.append('<joint name="joint_%d" type="revolute"><parent link="base_link"/>'
                         '<child link="%s"/><origin xyz="0 0 -0.2"/><axis xyz="0 1 0"/>'
                         '<limit lower="-2" upper="2"/></joint>' % (sid, name))
        return ''.join(parts) + '</robot>'

    def check(self, anchored=False):
        config = {'controllers': {str(sid): {'joint_name': 'joint_%d' % sid,
                  'servo': {'id': sid, 'init': 500, 'min': 0, 'max': 1000}} for sid in range(1, 25)}}
        xml = self.model()
        if anchored:
            xml = xml.replace('</robot>', '<link name="world"/><joint name="mount" type="fixed">'
                              '<parent link="world"/><child link="base_link"/></joint></robot>')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'stand.d6a'
            with closing(sqlite3.connect(str(path))) as db:
                db.execute('CREATE TABLE ActionGroup (Id INT, Time INT, '
                           + ','.join('Servo%d INT' % sid for sid in range(1, 25)) + ')')
                db.execute('INSERT INTO ActionGroup VALUES (' + ','.join('?' for _ in range(26)) + ')',
                           [1, 500] + [500] * 24)
                db.commit()
            before = path.read_bytes()
            result = prepare(xml, config, directory, lambda _name: directory)
            self.assertEqual(before, path.read_bytes())
            return result

    def test_sole_geometry_places_feet_on_floor_without_a_world_joint(self):
        targets, height, metadata = self.check()
        self.assertEqual(len(targets), 24)
        self.assertAlmostEqual(height, 0.206)
        self.assertAlmostEqual(metadata['base_pitch'], 0)

    def test_world_fixture_cannot_be_reported_as_a_free_standing_pose(self):
        with self.assertRaisesRegex(ValueError, 'free URDF'):
            self.check(anchored=True)
