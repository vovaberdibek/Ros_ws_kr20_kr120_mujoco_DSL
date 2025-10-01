import xml.etree.ElementTree as ET
from math import isfinite

IN  = "robot.urdf"       # use your current URDF here (robot_abs.urdf is fine too)
OUT = "robot_inertia_fixed.urdf"

def pd3(ixx,iyy,izz,ixy,ixz,iyz):
    # Sylvester's criterion: all leading principal minors > 0
    if ixx <= 0: return False
    det2 = ixx*iyy - ixy*ixy
    if det2 <= 0: return False
    det3 = (ixx*(iyy*izz - iyz*iyz)
           -ixy*(ixy*izz - iyz*ixz)
           +ixz*(ixy*iyz - iyy*ixz))
    return det3 > 0

def sanitize_inertia(inertial):
    # mass
    m_el = inertial.find("mass")
    if m_el is None:
        m_el = ET.SubElement(inertial, "mass")
        m_el.set("value", "1.0")
    try:
        m = float(m_el.get("value","0"))
    except Exception:
        m = 0.0
    if not isfinite(m) or m <= 0.0:
        m = 1.0
        m_el.set("value", f"{m}")

    # inertia
    I = inertial.find("inertia")
    if I is None:
        I = ET.SubElement(inertial, "inertia")
        I.set("ixx","0"); I.set("iyy","0"); I.set("izz","0")
        I.set("ixy","0"); I.set("ixz","0"); I.set("iyz","0")

    def f(attr, default=0.0):
        try: return float(I.get(attr, default))
        except Exception: return default

    ixx,iyy,izz = f("ixx"), f("iyy"), f("izz")
    ixy,ixz,iyz = f("ixy"), f("ixz"), f("iyz")

    if not pd3(ixx,iyy,izz,ixy,ixz,iyz):
        # replace with tiny diagonal inertia; off-diagonals to 0
        # use small sphere model: I = 2/5 m r^2 (choose r=0.01 m)
        I_diag = 0.4 * m * (0.01**2)  # = 4e-5 * m
        if I_diag <= 0: I_diag = 1e-6
        I.set("ixx", f"{I_diag}")
        I.set("iyy", f"{I_diag}")
        I.set("izz", f"{I_diag}")
        I.set("ixy", "0.0"); I.set("ixz", "0.0"); I.set("iyz", "0.0")
        return True
    return False

tree = ET.parse(IN)
root = tree.getroot()

fixed = []
for link in root.findall(".//link"):
    inertial = link.find("inertial")
    if inertial is None:
        # Create a minimal inertial if completely missing
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin").set("xyz", "0 0 0")
        ET.SubElement(inertial, "mass").set("value", "1.0")
        I = ET.SubElement(inertial, "inertia")
        I.set("ixx","1e-6"); I.set("iyy","1e-6"); I.set("izz","1e-6")
        I.set("ixy","0"); I.set("ixz","0"); I.set("iyz","0")
        fixed.append(link.get("name","(unnamed)"))
        continue

    if sanitize_inertia(inertial):
        fixed.append(link.get("name","(unnamed)"))

tree.write(OUT)
print(f"Sanitized {len(fixed)} link(s): {fixed}")
print(f"Wrote {OUT}")
