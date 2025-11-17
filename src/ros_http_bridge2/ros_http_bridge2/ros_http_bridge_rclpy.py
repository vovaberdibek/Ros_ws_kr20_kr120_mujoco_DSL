#!/usr/bin/env python3
import asyncio
import io
import sys
import threading
import json
import subprocess
import traceback
from typing import Dict, List, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from starlette.responses import StreamingResponse

from lark import Lark
from lark.exceptions import UnexpectedInput
import importlib.resources as pkg_res

# -------------- Import ROS 2 service types --------------
from custom_interfaces.srv import (
    BringTray                      as BringTraySrv,
    PositionTray                   as PositionTraySrv,
    PickTray                       as PickTraySrv,
    AddTray                        as AddTraySrv,
    AddTray2                       as AddTray2Srv,
    InitParameters                 as InitParametersSrv,
    InternalScrewingSequence       as InternalScrewingSrv,
    InternalScrewByNum             as InternalScrewSrv,
    ExternalScrewingSequence       as ExternalScrewingSrv,
    PlaceTray                      as PlaceTraySrv,
    OperatorPositionTray           as OperatorPositionTraySrv,
    RechargeSequence               as RechargeSequenceSrv,
)

# ---------- FastAPI app ----------
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# --------- Models ----------
class InitParamsModel(BaseModel):
    params: dict

class BringTrayModel(BaseModel):
    tray_name: str
    location: str
    agent: str

class IndexModel(BaseModel):
    index: int

class AddTray2Model(BaseModel):
    tray_name: str
    object_name: str

class ConfirmModel(BaseModel):
    task_description: str

class ConfirmResponseModel(BaseModel):
    ok: bool

class RunModel(BaseModel):
    dsl: str
    screwHoles: Optional[Dict[str, List[Dict[str,Any]]]] = None

class InternalScrewByNumModel(BaseModel):
    index: int
    ScrewNum: int

# ---- Confirmation state (shared with RobotManager monkey-patch) ----
confirm_event = threading.Event()
confirm_result: bool = False

# ---------- ROS2 Node wrapper ----------
class BridgeClient(Node):
    def __init__(self):
        super().__init__('ros_http_bridge')
        self._client_cache = {}  # (name, srv_type) -> client

    def _get_client(self, service_name: str, srv_type):
        key = (service_name, srv_type)
        if key in self._client_cache:
            return self._client_cache[key]
        cli = self.create_client(srv_type, service_name)
        self._client_cache[key] = cli
        return cli

    def call_service(self, service_name: str, srv_type, request, timeout_sec: float = 30.0):
        cli = self._get_client(service_name, srv_type)
        # Wait for service
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Timeout waiting for ROS 2 service '{service_name}'")
        future: Future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"Call to '{service_name}' timed out after {timeout_sec}s")
        if future.result() is None:
            raise RuntimeError(f"Service '{service_name}' returned None (exception likely).")
        return future.result()

# Make a single global node (initialized below)
_bridge_node: Optional[BridgeClient] = None

# --------------- /init_parameters ---------------
@app.post("/init_parameters")
def init_parameters(req: InitParamsModel):
    n = _bridge_node
    assert n is not None, "ROS2 node not initialized"

    p = req.params
    ros_req = InitParametersSrv.Request()
    ros_req.tray_step_poses = [v for pose in p.get("tray_step_poses", []) for v in pose]

    main_poses = p.get("main_poses", {})
    ros_req.table_pose       = main_poses.get("Table",       [0.0]*6)
    ros_req.tray_init_pose   = main_poses.get("TrayInit",    [0.0]*6)
    ros_req.bottom_init_pose = main_poses.get("BottomPanelInit",[0.0]*6)
    ros_req.bottom_pick_pose = main_poses.get("BottomPick",  [0.0]*6)
    ros_req.recharge_pose    = main_poses.get("Recharge",    [0.0]*6)
    ros_req.rest_pose        = main_poses.get("Rest",        [0.0]*6)
    ros_req.pick_pose        = main_poses.get("Pick",        [0.0]*6)
    ros_req.pre_pick_pose    = main_poses.get("PrePick",     [0.0]*6)
    ros_req.post_pick_pose   = main_poses.get("PostPick",    [0.0]*6)
    ros_req.end_pose         = main_poses.get("End",         [0.0]*6)
    ros_req.pre_end_pose     = main_poses.get("PreEnd",      [0.0]*6)

    ros_req.tray_final_pose    = p.get("tray_final_pose",   [0.0]*6)
    ros_req.tray_pose_operator = p.get("tray_pose_operator",[0.0]*6)
    ros_req.tip_offset         = p.get("tip_offset", [0.0, 0.0, 0.0])
    ros_req.tray_down_steps    = p.get("tray_down_steps", []) or []
    ros_req.operator_steps     = p.get("operator_steps", []) or []
    ros_req.rotation_steps     = p.get("rotation_steps", []) or []
    ros_req.new_tray_steps     = p.get("new_tray_steps", []) or []

    tray_angles = p.get("tray_angles", [0.0, 0.0])
    ros_req.tray_angle1 = float(tray_angles[0])
    ros_req.tray_angle2 = float(tray_angles[1])

    ros_req.tray_heights      = p.get("tray_heights", [])
    ros_req.origin_to_bottom  = p.get("origin_to_bottom", [])
    ros_req.tray_init_offset  = p.get("tray_init_offset", [])
    ros_req.new_dummy         = p.get("new_dummy", "")

    units, pose_indices = [], []
    for tray in p.get("trays", {}).values():
        for unit in tray.get("units", []):
            units.append(unit["name"])
            pose_indices.append(int(unit["pose_index"]))
    ros_req.tray_unit_names        = units
    ros_req.tray_unit_pose_indices = pose_indices

    screw_quantities, screw_types = [], []
    for tray in p.get("trays", {}).values():
        for screw in tray.get("screws", []):
            screw_quantities.append(int(screw["quantity"]))
            screw_types.append(screw["type"])
    ros_req.screw_quantities = screw_quantities
    ros_req.screw_types      = screw_types

    try:
        resp = n.call_service("init_parameters", InitParametersSrv, ros_req, timeout_sec=60.0)
        return {"success": bool(resp.success)}
    except RuntimeError as e:
        raise HTTPException(status_code=500, detail=str(e))

# --------------- Single-index helpers ---------------
def _index_call(service_name: str, srv_type, idx: int):
    n = _bridge_node
    assert n is not None, "ROS2 node not initialized"
    req = srv_type.Request()
    # all your index-based srvs have 'index' field
    if hasattr(req, 'index'):
        req.index = int(idx)
    else:
        raise HTTPException(status_code=500, detail=f"Service '{service_name}' has no 'index' field")
    resp = n.call_service(service_name, srv_type, req)
    return {"success": bool(resp.success)} if hasattr(resp, 'success') else {"ok": True}

@app.post("/bring_tray")
def bring_tray(req: BringTrayModel):
    n = _bridge_node
    assert n is not None, "ROS2 node not initialized"
    ros_req = BringTraySrv.Request()
    ros_req.tray_name = req.tray_name
    ros_req.location  = req.location
    try:
        resp = n.call_service("bring_tray", BringTraySrv, ros_req)
        return {"success": bool(resp.success)}
    except RuntimeError as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/add_tray")
def add_tray(req: IndexModel):
    return _index_call("add_tray", AddTraySrv, req.index)

@app.post("/add_tray2")
def add_tray2(req: AddTray2Model):
    n = _bridge_node
    assert n is not None, "ROS2 node not initialized"
    ros_req = AddTray2Srv.Request()
    ros_req.tray_name   = req.tray_name
    ros_req.object_name = req.object_name
    try:
        resp = n.call_service("add_tray2", AddTray2Srv, ros_req)
        return {"success": bool(resp.success)}
    except RuntimeError as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/pick_tray")
def pick_tray(req: IndexModel):
    return _index_call("pick_tray", PickTraySrv, req.index)

@app.post("/position_tray")
def position_tray(req: IndexModel):
    return _index_call("position_tray", PositionTraySrv, req.index)

@app.post("/operator_position_tray")
def operator_position_tray(req: IndexModel):
    return _index_call("operator_position_tray", OperatorPositionTraySrv, req.index)

@app.post("/recharge_sequence")
def recharge_sequence(req: IndexModel):
    return _index_call("recharge_sequence", RechargeSequenceSrv, req.index)

@app.post("/internal_screwing_sequence")
def internal_screwing_sequence(req: IndexModel):
    return _index_call("internal_screwing_sequence", InternalScrewingSrv, req.index)

@app.post("/internal_screw_by_num")
def internal_screw_by_num(req: InternalScrewByNumModel):
    n = _bridge_node
    assert n is not None, "ROS2 node not initialized"
    ros_req = InternalScrewSrv.Request()
    ros_req.index     = int(req.index)
    ros_req.screw_num = int(req.ScrewNum) if hasattr(ros_req, 'screw_num') else int(req.ScrewNum)
    try:
        resp = n.call_service("internal_screw_by_num", InternalScrewSrv, ros_req)
        out = {"success": bool(getattr(resp, 'success', True))}
        if hasattr(resp, 'message'):
            out["message"] = resp.message
        return out
    except RuntimeError as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/external_screwing_sequence")
def external_screwing_sequence(req: IndexModel):
    return _index_call("external_screwing_sequence", ExternalScrewingSrv, req.index)

@app.post("/place_tray")
def place_tray(req: IndexModel):
    return _index_call("place_tray", PlaceTraySrv, req.index)

# --------------- Manual confirmation ---------------
@app.post("/confirm_workflow")
def confirm_workflow(req: ConfirmModel):
    # Keep simple console prompt (like your original)
    ans = input(f"{req.task_description} [y/N]? ").strip().lower()
    return {"success": (ans == "y")}

@app.post("/confirm_response")
def confirm_response(resp: ConfirmResponseModel):
    global confirm_result
    confirm_result = bool(resp.ok)
    confirm_event.set()
    return {"received": True}

# --------------- DSL parsing & streaming endpoint ---------------
# Load grammar from package data
with pkg_res.files(__package__).joinpath("combined_dsl.lark").open("r") as f:
    _grammar = f.read()
_parser = Lark(_grammar, start="start", propagate_positions=True)

@app.post("/run_workflow")
async def run_workflow(req: RunModel):
    if req.screwHoles is not None:
        # ROS2 equivalent of rosrun:
        # Ensure your C++ node is installed as an executable with the same name
        proc = subprocess.Popen(
            ['ros2', 'run', 'kr20_kr120', 'kr120_pick_placeTest'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        proc.stdin.write(json.dumps(req.screwHoles))
        proc.stdin.close()

        async def stream():
            while True:
                line = proc.stdout.readline()
                if line == '' and proc.poll() is not None:
                    break
                if line:
                    yield line
                await asyncio.sleep(0.01)
        return StreamingResponse(stream(), media_type="text/plain")

    # Parse DSL
    try:
        tree = _parser.parse(req.dsl)
    except UnexpectedInput as e:
        detail = (
            f"DSL parse error at line {e.line}, column {e.column}: {e.get_context(req.dsl)}"
        )
        raise HTTPException(status_code=400, detail=detail)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"DSL parse error: {e}")

    # Capture stdout/stderr
    buf = io.StringIO()
    old_stdout, old_stderr = sys.stdout, sys.stderr
    sys.stdout = buf
    sys.stderr = buf

    # Monkey-patch confirmation to use /confirm_response path
    from .robot_manager import RobotHTTPClient
    def local_confirm(self, task_desc: str) -> bool:
        print(f"▶️ NEED_CONFIRM {task_desc}")
        confirm_event.clear()
        confirm_event.wait()
        return confirm_result
    RobotHTTPClient.confirm_workflow = local_confirm

    # Start RobotManager in a background thread
    from .robot_manager import RobotManager
    def _run():
        try:
            mgr = RobotManager(tree)
            mgr.execute_workflow()
        except Exception:
            traceback.print_exc()
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr

    t = threading.Thread(target=_run, daemon=True)
    t.start()

    async def stream():
        last = 0
        while t.is_alive() or last < buf.tell():
            await asyncio.sleep(0.1)
            data = buf.getvalue()
            if len(data) > last:
                chunk = data[last:]
                last = len(data)
                yield chunk
        data = buf.getvalue()
        if len(data) > last:
            yield data[last:]
    return StreamingResponse(stream(), media_type="text/plain")

# --------------- Entrypoint (uvicorn) ---------------
def main():
    global _bridge_node
    rclpy.init()
    _bridge_node = BridgeClient()

    import uvicorn
    # Run FastAPI (this blocks). rclpy node is used synchronously in handlers.
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()
