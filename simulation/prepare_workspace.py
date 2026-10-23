"""Normalize empty upstream maintainer names in image-local package manifests.

The pinned vendor checkout stays unchanged. No model/controller data is altered.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path('/opt/ainex_ws/src/ainex_simulations')
for package in ('ainex_description', 'ainex_gazebo'):
    manifest = root / package / 'package.xml'
    tree = ET.parse(manifest)
    changed = False
    for maintainer in tree.getroot().findall('maintainer'):
        if not (maintainer.text or '').strip():
            maintainer.text = 'Hiwonder'
            changed = True
    if changed:
        tree.write(manifest, encoding='utf-8', xml_declaration=True)

# The published checkout omits package markers; catkin's Python relay expects
# them. These empty files affect packaging only and import no hardware drivers.
for package in ('ainex_kinematics', 'ros_robot_controller'):
    marker = Path('/opt/ainex_ws/src') / package / 'src' / package / '__init__.py'
    if not marker.exists():
        marker.write_text('')
