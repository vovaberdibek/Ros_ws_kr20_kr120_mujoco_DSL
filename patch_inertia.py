import sys, xml.etree.ElementTree as ET

LINKS_TO_FIX = {"rl_base", "rl_lifter", "rl_slider", "rl_plate"}

def ensure_tag(parent, tag):
    el = parent.find(tag)
    if el is None:
        el = ET.SubElement(parent, tag)
    return el

def fix_file(in_path, out_path):
    ET.register_namespace('', "")  # keep tags clean
    tree = ET.parse(in_path)
    root = tree.getroot()
    n = 0
    for link in root.findall(".//link"):
        name = link.get("name", "")
        if name in LINKS_TO_FIX:
            inertial = ensure_tag(link, "inertial")
            mass = ensure_tag(inertial, "mass")
            mass.set("value", "0.001")  # small, non-zero
            origin = ensure_tag(inertial, "origin")
            origin.set("xyz", "0 0 0")
            origin.set("rpy", "0 0 0")
            inertia = ensure_tag(inertial, "inertia")
            inertia.set("ixx", "1e-6")
            inertia.set("ixy", "0.0")
            inertia.set("ixz", "0.0")
            inertia.set("iyy", "1e-6")
            inertia.set("iyz", "0.0")
            inertia.set("izz", "1e-6")
            n += 1
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print(f"Patched {n} links -> {out_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 patch_inertia.py IN_URDF OUT_URDF")
        sys.exit(1)
    fix_file(sys.argv[1], sys.argv[2])
