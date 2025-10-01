import re
from pathlib import Path

SRC = Path("robot.urdf").read_text()
OUT = Path("robot_abs.urdf")

PKG = Path("/home/user/Desktop/ros2_ws/src/kr20_kr120_description")
VIS = PKG / "visual"
COL = PKG / "collision"

def resolve(p: str) -> str:
    if not p: return p
    if p.startswith("package://kr20_kr120_description/"):
        return str(PKG / p.split("package://kr20_kr120_description/",1)[1])
    if p.startswith(("model://","/")):
        return p
    if p.startswith(("./","../")):
        p = p.split("/",1)[1] if "/" in p else p
    if p.startswith("visual/"):
        return str(VIS / p.split("visual/",1)[1])
python3 fix_mesh_paths_text.pylized ->", OUT)frag}" for ln,frag in bad[:20]], se
All mesh refs normalized -> robot_abs.urdf
user@user-Precision-3581:~/Desktop/mujoco_cell$ source ~/venvs/mj/bin/activate
python3 convert_urdf_to_mjcf.py robot_abs.urdf robot_mjcf.xml
deactivate
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
WARNING: Geom with duplicate name '' encountered in URDF, creating an unnamed geom.
MuJoCo could not load the URDF.
Common cause: unresolved 'package://' URIs. Replace them with absolute mesh paths.
Error: Error: Error opening file '/home/user/Desktop/mujoco_cell/kr20_asse4.stl': No such file or directory
user@user-Precision-3581:~/Desktop/mujoco_cell$ ~/venvs/mj/bin/python -m mujoco.viewer ~/Desktop/mujoco_cell/robot_mjcf.xml
user@user-Precision-3581:~/Desktop/mujoco_cell$ grep -nE '(\.stl|\.dae)' robot_abs.urdf | head -n 40
77:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_basement.stl" scale="0.001 0.001 0.001"/>
83:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_basement.stl" scale="0.001 0.001 0.001"/>
106:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_base.stl" scale="0.001 0.001 0.001"/>
112:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_base.stl" scale="0.001 0.001 0.001"/>
125:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse1.stl" scale="0.001 0.001 0.001"/>
131:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse1.stl" scale="0.001 0.001 0.001"/>
152:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse2.stl" scale="0.001 0.001 0.001"/>
158:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse2.stl" scale="0.001 0.001 0.001"/>
179:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse3.stl" scale="0.001 0.001 0.001"/>
185:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse3.stl" scale="0.001 0.001 0.001"/>
206:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse4.stl" scale="0.001 0.001 0.001"/>
212:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse4.stl" scale="0.001 0.001 0.001"/>
233:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse5.stl" scale="0.001 0.001 0.001"/>
239:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_asse5.stl" scale="0.001 0.001 0.001"/>
262:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_flange.stl" scale="0.001 0.001 0.001"/>
268:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_flange.stl" scale="0.001 0.001 0.001"/>
289:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_polso.stl" scale="0.001 0.001 0.001"/>
295:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_polso.stl" scale="0.001 0.001 0.001"/>
316:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_fixed.stl" scale="0.001 0.001 0.001"/>
322:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_fixed.stl" scale="0.001 0.001 0.001"/>
335:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_slide.stl" scale="0.001 0.001 0.001"/>
341:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr20_slide.stl" scale="0.001 0.001 0.001"/>
446:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_basement.stl" scale="0.001 0.001 0.001"/>
452:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_basement.stl" scale="0.001 0.001 0.001"/>
475:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_base.stl" scale="0.001 0.001 0.001"/>
481:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_base.stl" scale="0.001 0.001 0.001"/>
494:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse1.stl" scale="0.001 0.001 0.001"/>
500:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse1.stl" scale="0.001 0.001 0.001"/>
521:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse2.stl" scale="0.001 0.001 0.001"/>
527:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse2.stl" scale="0.001 0.001 0.001"/>
548:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse3.stl" scale="0.001 0.001 0.001"/>
554:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse3.stl" scale="0.001 0.001 0.001"/>
575:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse4.stl" scale="0.001 0.001 0.001"/>
581:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse4.stl" scale="0.001 0.001 0.001"/>
602:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse5.stl" scale="0.001 0.001 0.001"/>
608:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_asse5.stl" scale="0.001 0.001 0.001"/>
631:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_flange.stl" scale="0.001 0.001 0.001"/>
637:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_flange.stl" scale="0.001 0.001 0.001"/>
658:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_slider.stl" scale="0.001 0.001 0.001"/>
664:        <mesh filename="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/collision/kr120_slider.stl" scale="0.001 0.001 0.001"/>
user@user-Precision-3581:~/Desktop/mujoco_cell$ 


cat > fix_mesh_paths_all.py <<'PY'
import re
from pathlib import Path

URDF_IN  = Path("robot_abs.urdf") if Path("robot_abs.urdf").exists() else Path("robot.urdf")
URDF_OUT = Path("robot_abs.urdf")

PKG = Path("/home/user/Desktop/ros2_ws/src/kr20_kr120_description")
VIS = PKG / "visual"
COL = PKG / "collision"

def resolve(p: str) -> str:
    if not p: return p
    if p.startswith(("model://","/")):             # leave absolute/model:// as-is
        return p
    if p.startswith("package://kr20_kr120_description/"):
        return str(PKG / p.split("package://kr20_kr120_description/",1)[1])
    # normalize ./ or ../ at start
    if p.startswith(("./","../")):
        p = p.split("/",1)[1] if "/" in p else p
    # folder hints
    if p.startswith("visual/"):
        return str(VIS / p.split("visual/",1)[1])
    if p.startswith("collision/"):
        return str(COL / p.split("collision/",1)[1])
    # default by extension
    low = p.lower()
    if low.endswith(".stl"):
        return str(COL / Path(p).name)
    if low.endswith(".dae"):
        return str(VIS / Path(p).name)
    return p

src = URDF_IN.read_text()

def sub_attr(m):
    key,q,val = m.group("key","q","val")
    return f'{key}={q}{resolve(val.strip())}{q}'

# filename="..." and uri="..."
txt = re.sub(r'(?P<key>\b(?:filename|uri))\s*=\s*(?P<q>["\'])(?P<val>.*?)(?P=q)',
             sub_attr, src, flags=re.I)

# <filename>...</filename> and <uri>...</uri>
def sub_tag(m):
    tag,val = m.group("tag","val")
    return f'<{tag}>{resolve(val.strip())}</{tag}>'
txt = re.sub(r'<(?P<tag>filename|uri)>(?P<val>.*?)</(?P=tag)>',
             sub_tag, txt, flags=re.I|re.S)

# bare tokens between > and < (e.g., >kr20_asse4.stl<)
def sub_bare(m):
    inner = m.group("val").strip()
    if re.search(r'(?i)\.(stl|dae)\b', inner) and "/" not in inner and " " not in inner:
        return ">" + resolve(inner) + "<"
    return m.group(0)

txt = re.sub(r'>(?P<val>[^<>]{1,200})<', sub_bare, txt)

URDF_OUT.write_text(txt)

# Report leftovers
bad = []
for i,line in enumerate(txt.splitlines(),1):
    if re.search(r'(?i)(filename|uri)\s*=\s*["\'](?!/|package://|model://)[^"\']+\.(stl|dae)["\']', line) \
    or re.search(r'(?i)<(filename|uri)>(?!/|package://|model://)[^<]+\.(stl|dae)</\1>', line):
        bad.append((i,line.strip()))
print("Wrote", URDF_OUT)
if bad:
    print("Still relative paths remain:")
    for ln,frag in bad[:20]: print(f"  line {ln}: {frag}")
else:
    print("All mesh refs absolute.")
