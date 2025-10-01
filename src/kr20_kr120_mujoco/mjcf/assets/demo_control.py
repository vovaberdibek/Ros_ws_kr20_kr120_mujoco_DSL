import time
import mujoco as mj
try:
    from mujoco import viewer
except Exception:
    viewer = None

MODEL = "/home/user/Desktop/mujoco_cell/scene_wrapper_safe.xml"

m = mj.MjModel.from_xml_path(MODEL)
d = mj.MjData(m)

def aid(name): return mj.mj_name2id(m, mj.mjtObj.mjOBJ_ACTUATOR, name)

# set a few targets (radians/meters)
d.ctrl[aid("servo_kr20_base_link__link1")] = 0.5
d.ctrl[aid("servo_kr120_base_link__link1")] = 0.5
d.ctrl[aid("servo_rl_base__slider")]       = -0.6
d.ctrl[aid("servo_rl_slider__plate")]      = 2.57  # 90°

if viewer:
    with viewer.launch_passive(m, d) as v:
        while v.is_running():
            mj.mj_step(m, d)
            v.sync()
            time.sleep(0.002)
else:
    for _ in range(5000):
        mj.mj_step(m, d)
