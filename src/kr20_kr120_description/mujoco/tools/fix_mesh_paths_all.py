import re
from pathlib import Path

URDF_IN  = Path("robot_abs.urdf") if Path("robot_abs.urdf").exists() else Path("robot.urdf")
URDF_OUT = Path("robot_abs.urdf")

PKG = Path("/home/user/Desktop/ros2_ws/src/kr20_kr120_description")
VIS = PKG / "visual"
COL = PKG / "collision"

def resolve(p: str) -> str:
    if not p: return p
    if p.startswith(("model://","/")):             # already absolute or model://
        return p
    if p.startswith("package://kr20_kr120_description/"):
        return str(PKG / p.split("package://kr20_kr120_description/",1)[1])
    # normalize './' or '../' prefixes
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

# filename="..." and uri="..." attributes
def sub_attr(m):
    key,q,val = m.group("key","q","val")
    return f'{key}={q}{resolve(val.strip())}{q}'

txt = re.sub(r'(?P<key>\b(?:filename|uri))\s*=\s*(?P<q>["\'])(?P<val>.*?)(?P=q)',
             sub_attr, src, flags=re.I)

# <filename>…</filename> and <uri>…</uri> tags
def sub_tag(m):
    tag,val = m.group("tag","val")
    return f'<{tag}>{resolve(val.strip())}</{tag}>'

txt = re.sub(r'<(?P<tag>filename|uri)>(?P<val>.*?)</(?P=tag)>',
             sub_tag, txt, flags=re.I|re.S)

# Bare tokens like >kr20_asse2.stl< with no attribute/tag
def sub_bare(m):
    inner = m.group("val").strip()
    if re.search(r'(?i)\.(stl|dae)\b', inner) and "/" not in inner and " " not in inner:
        return ">" + resolve(inner) + "<"
    return m.group(0)

txt = re.sub(r'>(?P<val>[^<>]{1,200})<', sub_bare, txt)

URDF_OUT.write_text(txt)

# Report any leftovers (should be none)
left = []
for i,line in enumerate(txt.splitlines(),1):
    if re.search(r'(?i)(filename|uri)\s*=\s*["\'](?!/|package://|model://)[^"\']+\.(stl|dae)["\']', line) \
    or re.search(r'(?i)<(filename|uri)>(?!/|package://|model://)[^<]+\.(stl|dae)</\1>', line) \
    or re.search(r'>([^/<> ]+)\.(stl|dae)<', line):
        left.append((i, line.strip()[:200]))
print("Wrote", URDF_OUT)
if left:
    print("Still relative mesh refs:")
    for ln,frag in left[:40]:
        print(f"  line {ln}: {frag}")
else:
    print("All mesh refs absolute.")
