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
from rclpy.action import ActionClient

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from starlette.responses import StreamingResponse

from lark import Lark
from lark.exceptions import UnexpectedInput
import importlib.resources as pkg_res

# -------------- Import ROS 2 service types --------------
from custom_interfaces.srv import InitParameters as InitParametersSrv
from custom_interfaces.action import ExecuteAction
from diagnostic_msgs.msg import KeyValue

# ---------- FastAPI app ----------
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# --------- Models ----------
from .dsl_models import (
    InitParamsModel,
    BringTrayModel,
    IndexModel,
    AddTrayModel,
    ConfirmModel,
    ConfirmResponseModel,
    RunModel,
    InternalScrewByNumModel,
    ExecuteActionModel,
)

# ---- Confirmation state (shared with RobotManager monkey-patch) ----
confirm_event = threading.Event()
confirm_result: bool = False

# ---------- ROS2 Node wrapper ----------
class BridgeClient(Node):
    def __init__(self):
        super().__init__('ros_http_bridge')
        self._client_cache = {}  # (name, srv_type) -> client
        self._execute_action_client = ActionClient(self, ExecuteAction, "execute_action")

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

    def _encode_param_value(self, value: Any) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, (int, float)):
            return str(value)
        if value is None:
            return ""
        if isinstance(value, (dict, list)):
            return json.dumps(value)
        return str(value)

    def call_execute_action(self, action_name: str, params: Dict[str, Any]):
        if not self._execute_action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("ExecuteAction server not available.")
        goal = ExecuteAction.Goal()
        goal.action_name = action_name
        goal.params = []
        for key, value in (params or {}).items():
            kv = KeyValue()
            kv.key = str(key)
            kv.value = self._encode_param_value(value)
            goal.params.append(kv)

        send_future = self._execute_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"success": False, "message": f"Action '{action_name}' was rejected."}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not result_future.done():
            return {"success": False, "message": f"Action '{action_name}' timed out."}
        result = result_future.result().result
        return {
            "success": bool(result.success),
            "message": result.message,
            "result": result.result_json,
        }

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
    tray_step_pose_names = p.get("tray_step_pose_names", [])
    pose_name_lookup = {}
    if isinstance(tray_step_pose_names, list):
        pose_name_lookup = {name: idx for idx, name in enumerate(tray_step_pose_names) if isinstance(name, str) and name}

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
            unit_name = unit.get("name")
            units.append(unit_name)
            pose_idx = unit.get("pose_index")
            if pose_idx is None:
                pose_name = unit.get("pose_name")
                if pose_name:
                    pose_idx = pose_name_lookup.get(pose_name)
            if pose_idx is None:
                raise HTTPException(status_code=400, detail=f"Unit '{unit_name}' missing pose_index/pose_name mapping.")
            pose_indices.append(int(pose_idx))
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


@app.post("/execute_action")
def execute_action(req: ExecuteActionModel):
    return _dispatch_action(req.action, req.params)

def _dispatch_action(action: str, params: Optional[Dict[str, Any]] = None):
    n = _bridge_node
    assert n is not None, "ROS2 node not initialized"
    try:
        return n.call_execute_action(action, params or {})
    except RuntimeError as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/bring_tray")
def bring_tray(req: BringTrayModel):
    return _dispatch_action("BringTray", {"tray": req.tray_name, "location": req.location})

@app.post("/add_tray")
def add_tray2(req: AddTrayModel):
    return _dispatch_action("AddTray", {"tray": req.tray_name, "object": req.object_name})

@app.post("/pick_tray")
def pick_tray(req: IndexModel):
    return _dispatch_action("PickTray", {"index": req.index})

@app.post("/position_tray")
def position_tray(req: IndexModel):
    return _dispatch_action("PositionTray", {"index": req.index})

@app.post("/operator_position_tray")
def operator_position_tray(req: IndexModel):
    return _dispatch_action("OperatorPositionTray", {"index": req.index})

@app.post("/recharge_sequence")
def recharge_sequence(req: IndexModel):
    return _dispatch_action("RechargeSequence", {"index": req.index})

@app.post("/internal_screwing_sequence")
def internal_screwing_sequence(req: IndexModel):
    return _dispatch_action("InternalScrewingSequence", {"index": req.index})

@app.post("/internal_screw_by_num")
def internal_screw_by_num(req: InternalScrewByNumModel):
    return _dispatch_action("InternalScrew", {"index": req.index, "screw_num": req.ScrewNum})

@app.post("/external_screwing_sequence")
def external_screwing_sequence(req: IndexModel):
    return _dispatch_action("ExternalScrewingSequence", {"index": req.index})

@app.post("/place_tray")
def place_tray(req: IndexModel):
    return _dispatch_action("PlaceTray", {"index": req.index})

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
        # Log to server console for immediate visibility
        print(f"❌ {detail}")
        raise HTTPException(status_code=400, detail=detail)
    except Exception as e:
        detail = f"DSL parse error: {e}"
        print(f"❌ {detail}")
        raise HTTPException(status_code=400, detail=detail)

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
