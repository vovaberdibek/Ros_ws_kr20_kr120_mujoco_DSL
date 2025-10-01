# kuka_stabilize_test.py
# One-shot stabilizer + hold-position test for your KUKA scene.

import os
import math
import time
import numpy as np
import mujoco as mj

# Set this to your wrapper XML (the one that includes robot_mjcf.xml + actuators)
MODEL_PATH = "/home/user/Desktop/mujoco_cell/scene_wrap_g.xml"

# Show the viewer? (requires glfw). Set False for headless stepping.
USE_VIEWER = True

def _try_name2id(m, objtype, name):
    try:
        return mj.mj_name2id(m, objtype, name)
    except Exception:
        return -1

def apply_runtime_stabilizers(m):
    """
    Make conservative, simulation-stable tweaks at runtime WITHOUT editing your XML:
      - soften contacts globally
      - add minimum damping & armature on all DOFs
      - zero joint frictionloss
      - lower too-high position actuator gains (kp)
      - disable collisions for heavy base meshes (basement/base only)
    """
    # 1) Contact softening (global per-geom)
    # friction: (slide, spin, roll)
    m.geom_friction[:, 0] = 0.6
    m.geom_friction[:, 1] = 0.01
    m.geom_friction[:, 2] = 0.001
    # solimp has 5 slots; we just set the first three like in your XML
    m.geom_solimp[:, 0] = 0.90
    m.geom_solimp[:, 1] = 0.995
    m.geom_solimp[:, 2] = 0.0005
    # solref: (timeconst, dampratio)
    m.geom_solref[:, 0] = 0.015
    m.geom_solref[:, 1] = 1.0

    # 2) Joint-level damping & armature (minimums)
    if m.dof_damping is not None and m.dof_damping.size:
        m.dof_damping[:] = np.maximum(m.dof_damping, 8.0)
    if m.dof_armature is not None and m.dof_armature.size:
        m.dof_armature[:] = np.maximum(m.dof_armature, 2.0)

    # 3) Remove joint Coulomb friction losses (these can spike torques)
    if hasattr(m, "dof_frictionloss") and m.dof_frictionloss is not None:
        m.dof_frictionloss[:] = 0.0

    # 4) Position actuator kp guard (if encoded in gainprm[0])
    #    Only lower values that are clearly set (>0).
    if hasattr(m, "actuator_gainprm") and m.actuator_gainprm.size:
        kp = m.actuator_gainprm[:, 0].copy()
        mask = kp > 0.0
        kp[mask] = np.minimum(kp[mask], 80.0)  # cap at ~80 Nm/rad (tune later)
        m.actuator_gainprm[:, 0] = kp

    # 5) Disable collision for heavy bases so they don't fight contacts with the arms.
    for gname in ["kr20_basement", "kr20_base", "kr120_basement", "kr120_base"]:
        gid = _try_name2id(m, mj.mjtObj.mjOBJ_GEOM, gname)
        if gid != -1:
            m.geom_contype[gid] = 0
            m.geom_conaffinity[gid] = 0

def set_hold_position_targets(m, d):
    """
    For every actuator that targets a joint, set ctrl to the CURRENT joint qpos.
    This makes the robot hold still at whatever pose it starts in.
    """
    for a in range(m.nu):
        j = m.actuator_trnid[a, 0]
        if j >= 0:
            qadr = m.jnt_qposadr[j]
            d.ctrl[a] = d.qpos[qadr]

def run():
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"MODEL_PATH does not exist: {MODEL_PATH}")

    m = mj.MjModel.from_xml_path(MODEL_PATH)
    d = mj.MjData(m)

    # Optionally bump solver options for stability if they look weak.
    # (If your wrapper XML already set these, this won't do harm.)
    m.opt.integrator = mj.mjtIntegrator.mjINT_IMPLICIT
    m.opt.timestep   = max(m.opt.timestep, 0.001)
    m.opt.iterations = max(m.opt.iterations, 200)
    m.opt.tolerance  = min(m.opt.tolerance, 1e-9)

    apply_runtime_stabilizers(m)

    # BEFORE stepping: send hold-position setpoints like Panda does
    set_hold_position_targets(m, d)

    # Let constraints settle a bit
    for _ in range(200):
        mj.mj_step(m, d)

    def unstable(d):
        return (not np.isfinite(d.qacc).all()) or (not np.isfinite(d.qvel).all())

    if USE_VIEWER:
        try:
            from mujoco import viewer
            with viewer.launch_passive(m, d) as v:
                last = time.time()
                while v.is_running():
                    mj.mj_step(m, d)
                    if unstable(d):
                        print("WARNING: Detected non-finite state (qacc/qvel). Stopping.")
                        break
                    # refresh at ~60Hz
                    now = time.time()
                    if now - last > 1/60:
                        v.sync()
                        last = now
        except Exception as e:
            print(f"Viewer failed ({e}). Falling back to headless.")
            for _ in range(5000):
                mj.mj_step(m, d)
                if unstable(d):
                    print("WARNING: Detected non-finite state (qacc/qvel). Stopping.")
                    break
    else:
        # Headless stepping
        for i in range(8000):
            mj.mj_step(m, d)
            if unstable(d):
                print(f"WARNING: Detected non-finite state at step {i}.")
                break
        print("Headless stepping finished OK.")

if __name__ == "__main__":
    run()
