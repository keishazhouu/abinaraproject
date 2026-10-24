#!/usr/bin/env python3
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ARMS = [("arm1", -0.4), ("arm2", 0.4)]


def add_prefix(elem, prefix):
    for e in elem.iter():
        for attr in ("name", "joint"):
            if attr in e.attrib:
                e.set(attr, prefix + e.get(attr))


src = ET.parse(ROOT / "models" / "arm_final.xml").getroot()
world = src.find("worldbody")
shared = [e for e in world if e.tag == "light" or (e.tag == "geom" and e.get("type") == "plane")]
arm_parts = [e for e in world if e not in shared]

out = ET.Element("mujoco", model="abinara_dual")
for e in src:
    if e.tag not in ("worldbody", "actuator", "sensor"):
        out.append(copy.deepcopy(e))

out_world = ET.SubElement(out, "worldbody")
for e in shared:
    out_world.append(copy.deepcopy(e))
for name, x in ARMS:
    wrapper = ET.SubElement(out_world, "body", name=f"{name}_base", pos=f"{x} 0 0")
    for e in arm_parts:
        part = copy.deepcopy(e)
        add_prefix(part, f"{name}_")
        wrapper.append(part)

for section in ("actuator", "sensor"):
    block = ET.SubElement(out, section)
    for name, _ in ARMS:
        for e in src.find(section):
            item = copy.deepcopy(e)
            add_prefix(item, f"{name}_")
            block.append(item)

ET.indent(out, space="    ")
ET.ElementTree(out).write(ROOT / "models" / "dual_arm.xml", encoding="unicode")
print("OK -> models/dual_arm.xml")