from xml.etree import ElementTree as ET
from pathlib import Path

src = Path('robot_mjcf.xml')
dst = Path('actuators_auto.xml')

tree = ET.parse(src); root = tree.getroot()

joints = []
for j in root.findall('.//joint'):
    name = j.attrib.get('name')
    if not name:
        continue
    if j.attrib.get('type') == 'free':
        continue
    rng = j.attrib.get('range')
    joints.append((name, rng))

out = ET.Element('mujoco')
act = ET.SubElement(out, 'actuator')
for name, rng in joints:
    a = ET.SubElement(act, 'position', dict(joint=name, kp='300'))
    if rng:
        a.set('ctrllimited','true')
        a.set('ctrlrange', rng)

ET.ElementTree(out).write(dst, encoding='utf-8', xml_declaration=True)
print(f"Wrote {dst} with {len(joints)} position actuators")
