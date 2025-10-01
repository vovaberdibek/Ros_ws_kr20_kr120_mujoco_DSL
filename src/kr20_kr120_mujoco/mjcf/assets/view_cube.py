import time, mujoco
import mujoco.viewer as viewer

xml = """
<mujoco model="hello">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.9 0.9 0.95 1"/>
    <body pos="0 0 0.1">
      <geom type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
    </body>
    <light diffuse="1 1 1" pos="1 1 3"/>
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
with viewer.launch_passive(m, d) as v:
    v.cam.lookat[:] = [0, 0, 0.1]
    v.cam.distance  = 1.2
    v.cam.azimuth   = 45
    v.cam.elevation = -20
    t0 = time.time()
    while time.time() - t0 < 10:  # keep open ~10s
        mujoco.mj_step(m, d)
        time.sleep(1/60)
