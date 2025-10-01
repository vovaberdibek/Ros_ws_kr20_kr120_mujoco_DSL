from pathlib import Path
import xml.etree.ElementTree as ET

URDF_IN  = Path("robot.urdf")
URDF_OUT = Path("robot_abs.urdf")

PKG_ROOT = Path("/home/user/Desktop/ros2_ws/src/kr20_kr120_description")
VIS = PKG_ROOT / "visual"
COL = PKG_ROOT / "collision"

def resolve(p: str) -> str:
    if not p: return p
    if p.startswith("/"):                      # already absolute
        return p
    if p.startswith("package://kr20_kr120_description/"):
        return str(PKG_ROOT / p.split("package://kr20_kr120_description/")[1])
    # relative paths
    if p.startswith("visual/"):
        return str(VIS / p.split("visual/")[1])
    if p.startswith("collision/"):
        return str(COL / p.split("collision/")[1])
    # bare filenames -> route by extension
    low = p.lower()
    if low.endswith(".stl"):
        return str(COL / Path(p).name)
    if low.endswith(".dae"):
        return str(VIS / Path(p).name)
    return p

tree = ET.parse(URDF_IN)
root = tree.getroot()

changed, missing = 0, []

def fix_attr(elem, attr):
    global changed
    v = elem.get(attr)
    if v is None: return
    new = resolve(v)
    if new != v:
        elem.set(attr, new); changed += 1
    if new.lower().endswith((".stl",".dae")) and not Path(new).is_file():
        missing.append(new)

# handle <mesh filename="...">
for mesh in root.iter("mesh"):
    fix_attr(mesh, "filename")
    # also handle nested <filename> / <uri> children if present
    for child in list(mesh):
        if child.tag in ("filename","uri") and child.text:
            new = resolve(child.text.strip())
            if new != child.text:
                child.text = new; changed += 1
            if new.lower().endswith((".stl",".dae")) and not Path(new).is_file():
                missing.append(new)

# write out
tree.write(URDF_OUT, encoding="utf-8", xml_declaration=True)

print(f"Rewrote {changed} mesh references → {URDF_OUT}")
if missing:
    print("\nMISSING FILES (create/copy these or adjust names):")
    for m in sorted(set(missing)):
        print("  -", m)
