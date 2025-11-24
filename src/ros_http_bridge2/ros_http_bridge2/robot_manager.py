import queue
import threading
import requests
import urllib3
import json
import re
import ast
from typing import Any, Dict, List, Optional, Callable

from pydantic import ValidationError

from .dsl_models import TrayModel, DSLValidationError
from .dsl_parser import DSLParser, WorkflowArtifacts


BASE_URL = "http://localhost:8000"
BUILDING_BLOCKS = {
    "depositTray",
    "gyroGrpRot",
    "loadTray",
    "mrHoming",
    "mrTrolleyVCheck",
    "mrTrolleyVCheckErrorCheck",
    "pickUpTray",
    "positionerRotate",
    "present2Op",
    "presentToScrew",
    "screwPickup",
    "screwTight",
    "setScrewBayState",
    "srHoming",
    "stackTray",
}


def resolve_block_name(name: str) -> Optional[str]:
    if not name:
        return None
    lowered = name.strip().lower()
    for item in BUILDING_BLOCKS:
        if item.lower() == lowered:
            return item
    return None

class RobotHTTPClient:
    def __init__(self, base_url=BASE_URL):
        self.base_url = base_url

  
    def send_init_parameters(self, params):
        url = f"{self.base_url}/init_parameters"
        try:
            res = requests.post(url, json={"params": params}, timeout=30.0)
            return res.json().get("success", False)
        except requests.RequestException as e:
            print(f"❌ InitParameters HTTP call failed: {e}")
            return False

    def confirm_workflow(self, task_description):
        url = f"{self.base_url}/confirm_workflow"
        try:
            # no timeout ⇒ wait until user types Y/N in the FastAPI terminal
            res = requests.post(url, json={"task_description": task_description})
            return res.json().get("success", False)
        except requests.RequestException as e:
            print(f"❌ confirm_workflow HTTP call failed: {e}")
            return False

    def execute_action(self, action: str, params: Dict[str, Any]):
        url = f"{self.base_url}/execute_action"
        payload = {"action": action, "params": params}
        try:
            res = requests.post(url, json=payload, timeout=60.0)
            return res.json()
        except requests.RequestException as e:
            print(f"❌ execute_action '{action}' HTTP call failed: {e}")
            return {}


class PLCClient:
    _ros_imports = None
    _rclpy_initialized = False

    BLOCK_PARAM_SCHEMA = {
        "loadTray": [("bool_param1", "load", "bool")],
        "positionerRotate": [("bool_param1", "clockwise", "bool")],
        "stackTray": [("int_param1", "number", "int")],
        "gyroGrpRot": [("int_param1", "direction", "int")],
        "screwPickup": [("int_param1", "screw", "int")],
        "present2Op": [("int_param1", "side", "int"), ("int_param2", "face", "int")],
        "presentToScrew": [("int_param1", "side", "int"), ("int_param2", "face", "int")],
        "screwTight": [
            ("float_param1", "offset_x", "float"),
            ("float_param2", "offset_y", "float"),
            ("float_param3", "offset_z", "float"),
            ("int_param1", "target", "int"),
            ("int_param2", "focal_plane", "int"),
            ("int_param3", "recipe_id", "int"),
            ("bool_param1", "inside_area", "bool"),
        ],
    }

    def __init__(self, logger: Optional[Callable[[str], None]] = None, ats_ip: str = "10.10.10.100"):
        self.logger = logger or (lambda msg: print(f"[PLC] {msg}"))
        modules = self._load_ros_dependencies()
        self.rclpy = modules["rclpy"]
        self.ActionClient = modules["ActionClient"]
        self.CallFunctionBlock = modules["CallFunctionBlock"]
        self.SetScrewBayState = modules["SetScrewBayState"]
        self.ScrewSlot = modules["ScrewSlot"]
        self.Offset = modules["Offset"]
        self.Empty = modules["Empty"]
        self.msgType = modules["msgType"]

        if not PLCClient._rclpy_initialized:
            try:
                self.rclpy.init(args=None)
            except RuntimeError as exc:
                if "Context.init() must only be called once" not in str(exc):
                    raise
            PLCClient._rclpy_initialized = True

        self.node = self.rclpy.create_node("dsl_plc_client")
        self._action_client = self.ActionClient(self.node, self.CallFunctionBlock, "CallFunctionBlock")
        self._error_ack_pub = self.node.create_publisher(self.Empty, "errorCheckAck", 10)
        self._offset_pub = self.node.create_publisher(self.Offset, "offset", 10)
        self._screw_state_client = self.node.create_client(self.SetScrewBayState, "setScrewBayState")
        self._lock = threading.Lock()
        self._active_context: Dict[str, Any] = {}
        self.ats_ip = ats_ip
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

    @classmethod
    def _load_ros_dependencies(cls):
        if cls._ros_imports:
            return cls._ros_imports
        try:
            import rclpy  # type: ignore
            from rclpy.action import ActionClient  # type: ignore
            from prometheus_req_interfaces.action import CallFunctionBlock  # type: ignore
            from prometheus_req_interfaces.srv import SetScrewBayState  # type: ignore
            from prometheus_req_interfaces.msg import ScrewSlot, Offset  # type: ignore
            from std_msgs.msg import Empty  # type: ignore
            from prometheus_req_py.ADS.utils import msgType  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "Real mode requires the prometheus_req_interfaces and prometheus_req_py packages to be installed."
            ) from exc

        cls._ros_imports = {
            "rclpy": rclpy,
            "ActionClient": ActionClient,
            "CallFunctionBlock": CallFunctionBlock,
            "SetScrewBayState": SetScrewBayState,
            "ScrewSlot": ScrewSlot,
            "Offset": Offset,
            "Empty": Empty,
            "msgType": msgType,
        }
        return cls._ros_imports

    def _log(self, message: str):
        if self.logger:
            self.logger(message)

    def call_block(self, block_name: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        params = params or {}
        resolved = resolve_block_name(block_name)
        if not resolved:
            raise ValueError(f"'{block_name}' is not a valid PLC building block.")

        self._active_context = dict(params)
        self._log(f"🚧 Calling PLC block '{resolved}' with params {self._active_context}")

        if resolved == "setScrewBayState":
            return self._call_set_screw_bay_state(self._active_context)

        goal = self._build_goal(resolved, self._active_context)
        return self._send_goal(goal)

    def _build_goal(self, block_name: str, params: Dict[str, Any]):
        goal = self.CallFunctionBlock.Goal()
        goal.function_block_name = block_name
        schema = self.BLOCK_PARAM_SCHEMA.get(block_name, [])
        for field_name, param_name, kind in schema:
            if param_name not in params:
                raise ValueError(f"Missing required parameter '{param_name}' for block '{block_name}'.")
            value = self._coerce_value(params[param_name], param_name, kind)
            setattr(goal, field_name, value)
        return goal

    def _coerce_value(self, value: Any, key: str, kind: str):
        if kind == "int":
            return self._coerce_int(value, key)
        if kind == "float":
            return self._coerce_float(value, key)
        if kind == "bool":
            return self._coerce_bool(value, key)
        return value

    def _coerce_int(self, value: Any, key: str) -> int:
        if value is None:
            raise ValueError(f"Parameter '{key}' cannot be None.")
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, (int, float)):
            return int(value)
        if isinstance(value, str) and value.strip():
            return int(float(value.strip()))
        raise ValueError(f"Parameter '{key}' must be an integer-compatible value (got {value!r}).")

    def _coerce_float(self, value: Any, key: str) -> float:
        if value is None:
            raise ValueError(f"Parameter '{key}' cannot be None.")
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, str) and value.strip():
            return float(value.strip())
        raise ValueError(f"Parameter '{key}' must be a float-compatible value (got {value!r}).")

    def _coerce_bool(self, value: Any, key: str) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in {"true", "yes", "y", "1"}:
                return True
            if lowered in {"false", "no", "n", "0"}:
                return False
        raise ValueError(f"Parameter '{key}' must be boolean-compatible (got {value!r}).")

    def _send_goal(self, goal):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("CallFunctionBlock action server is not available.")

        send_future = self._action_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        self.rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"success": False, "state": "REJECTED", "msg": "PLC rejected the goal."}

        result_future = goal_handle.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, result_future)
        result_data = result_future.result().result
        payload = {
            "success": bool(result_data.success),
            "state": result_data.state,
            "msg": result_data.msg,
        }
        self._log(f"🏁 PLC block '{goal.function_block_name}' finished: {payload}")
        return payload

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        msg_type = feedback.msg_type
        self._log(f"ℹ️ PLC feedback ({msg_type}): {feedback.msg}")

        if msg_type == self.msgType.ERROR_CHECK:
            self._log("↩️ Auto-acknowledging PLC error check.")
            self._error_ack_pub.publish(self.Empty())
        elif msg_type in (self.msgType.ASK_PICTURE_SCREW, self.msgType.ASK_PICTURE_VCHECK):
            self._handle_picture_request(msg_type)

    def _handle_picture_request(self, msg_type):
        focal_plane = self._coerce_int(self._active_context.get("focal_plane", 0), "focal_plane")
        roi_id = self._coerce_int(self._active_context.get("roi_id", 0), "roi_id")
        find_screw = self._coerce_bool(self._active_context.get("find_screw", True), "find_screw")
        data_valid, x_off, y_off, theta = self.calculate_picture_offset(msg_type, focal_plane, roi_id, find_screw)
        offset_msg = self.Offset()
        offset_msg.data_valid = bool(data_valid)
        offset_msg.x = float(x_off)
        offset_msg.y = float(y_off)
        offset_msg.theta = float(theta)
        self._offset_pub.publish(offset_msg)
        self._log(f"📡 Published offset for msg_type {msg_type}: {offset_msg}")

    def calculate_picture_offset(self, asked_action, calibration_plane=0, roi_id=0, find_screw=False):
        parameters = {
            "calibrationPlane": int(calibration_plane),
            "roiId": int(roi_id),
            "findScrew": bool(find_screw),
        }
        command = "GetScrewCorrection" if asked_action == self.msgType.ASK_PICTURE_SCREW else "GetTrayCorrection"
        url = f"https://{self.ats_ip}/{command}"
        self._log(f"🔍 Requesting picture offset from ATS: {url} params={parameters}")
        try:
            response = requests.get(url, params=parameters, verify=False, timeout=15.0).json()
        except Exception as exc:
            self._log(f"❌ Picture offset request failed: {exc}")
            return False, 0.0, 0.0, 0.0

        if isinstance(response, str):
            try:
                response = ast.literal_eval(response.replace("false", "False").replace("true", "True"))
            except Exception as exc:
                self._log(f"❌ Could not parse ATS response: {exc}")
                return False, 0.0, 0.0, 0.0

        data_valid = bool(response.get("DataValid", False))
        if not data_valid:
            self._log("❌ ATS returned invalid data for picture offset.")
            return False, 0.0, 0.0, 0.0

        translation_x = float(response.get("TranslationX", 0.0))
        translation_y = float(response.get("TranslationY", 0.0))
        rotation = float(response.get("Rotation", 0.0))
        self._log(
            f"✅ ATS offset response: data_valid={data_valid}, "
            f"translation=({translation_x}, {translation_y}), rotation={rotation}"
        )
        return data_valid, translation_x, translation_y, rotation

    def _call_set_screw_bay_state(self, params: Dict[str, Any]) -> Dict[str, Any]:
        slots = params.get("slots")
        if not isinstance(slots, list) or not slots:
            raise ValueError("SetScrewBayState requires a non-empty 'slots' list.")

        if not self._screw_state_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("setScrewBayState service is not available.")

        request = self.SetScrewBayState.Request()
        request.bay_number = int(params.get("bay_number", len(slots)))
        request.screw_bays = []
        for idx, entry in enumerate(slots, start=1):
            normalized = self._normalize_slot_entry(entry, idx)
            slot_msg = self.ScrewSlot()
            slot_msg.max_idx_x = self._coerce_int(normalized["max_idx_x"], f"slots[{idx}].max_idx_x")
            slot_msg.max_idx_y = self._coerce_int(normalized["max_idx_y"], f"slots[{idx}].max_idx_y")
            slot_msg.next_idx_x = self._coerce_int(normalized["next_idx_x"], f"slots[{idx}].next_idx_x")
            slot_msg.next_idx_y = self._coerce_int(normalized["next_idx_y"], f"slots[{idx}].next_idx_y")
            request.screw_bays.append(slot_msg)

        future = self._screw_state_client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        payload = {"success": bool(getattr(response, "success", True)), "state": "SERVICE", "msg": "setScrewBayState"}
        self._log(f"🔧 Updated screw bay state: {payload}")
        return payload

    def _normalize_slot_entry(self, entry: Any, index: int) -> Dict[str, Any]:
        if isinstance(entry, dict):
            required = {"max_idx_x", "max_idx_y", "next_idx_x", "next_idx_y"}
            if not required.issubset(entry.keys()):
                missing = required - set(entry.keys())
                raise ValueError(f"Slot #{index} missing keys: {missing}")
            return entry
        if isinstance(entry, (list, tuple)) and len(entry) == 4:
            return {
                "max_idx_x": entry[0],
                "max_idx_y": entry[1],
                "next_idx_x": entry[2],
                "next_idx_y": entry[3],
            }
        raise ValueError(
            "Slot entries must be dicts with keys "
            "['max_idx_x','max_idx_y','next_idx_x','next_idx_y'] or 4-item lists/tuples."
        )

class RobotManager:
    UNIT_INDEX_ACTIONS = {"PositionTray", "RechargeSequence", "InternalScrewingSequence", "OperatorPositionTray"}
    ACTION_PARAM_SCHEMAS = {
        "AddTray": {"required": {"tray": str}},
        "InternalScrewUnitHole": {"required": {"unit": str, "hole": int}},
        "CallBlock": {"required": {"name": str}},
    }

    def __init__(self, parsed_spec):
        self.http_client = RobotHTTPClient()
        self.mode = "simulation"
        self.plc_client: Optional[PLCClient] = None

        # The same lists and dicts you had originally:
        self.tip_offset = []
        self.tray_angles = []
        self.tray_down_steps = []
        self.operator_steps = []
        self.rotation_steps = []
        self.new_tray_steps = []
        self.vector3 = []
        self.vector2 = []
        self.int_list = []
        self.tray_step_poses = []
        self.current_tray_step = None
        self.location_order = []
        self.location_name_to_index = {}
        self.main_poses = {}
        self.named_poses = {}
        self.parameters = {}

        parser = DSLParser()
        if isinstance(parsed_spec, WorkflowArtifacts):
            artifacts = parsed_spec
        else:
            artifacts = parser.build(parsed_spec)

        self.mode = artifacts.mode or "simulation"
        self.agents = artifacts.agents
        self.workflow = artifacts.workflow
        self.trays = artifacts.trays
        self.locations = artifacts.locations
        self.tray_step_poses = artifacts.tray_step_poses
        self.tray_step_pose_names = getattr(artifacts, "tray_step_pose_names", [None] * len(self.tray_step_poses))
        self.tray_step_pose_name_to_index = {
            name: idx for idx, name in enumerate(self.tray_step_pose_names) if name is not None
        }
        self.main_poses = artifacts.main_poses
        self.named_poses = artifacts.named_poses
        self.parameters = artifacts.parameters
        self.location_order = artifacts.location_order
        self.location_name_to_index = artifacts.location_name_to_index

        # Construct the same "params" dictionary you used in ROS version
        tray_pose_operator = (
            self.parameters.get('tray_pose_operator')
            or self.parameters.get('TrayPoseOperator')
            or self.named_poses.get('TrayPoseOperator', [])
        )
        tray_final_pose = (
            self.parameters.get('tray_final_pose')
            or self.parameters.get('TrayFinalPose')
            or self.named_poses.get('TrayFinalPose', [])
        )

        params = {
            'tray_step_poses':     self.tray_step_poses,
            'tray_step_pose_names': self.tray_step_pose_names,
            'named_poses':         self.named_poses,
            'main_poses':          self.main_poses,
            'trays':               self.trays,

            'tip_offset':          self.parameters.get('tip_offset'),
            'tray_angles':         self.parameters.get('tray_angles'),
            'tray_down_steps':     self.parameters.get('tray_down_steps'),
            'operator_steps':      self.parameters.get('operator_steps'),
            'rotation_steps':      self.parameters.get('rotation_steps'),
            'new_tray_steps':      self.parameters.get('new_tray_steps'),

            'tray_heights':        self.parameters.get('tray_heights', []),
            'origin_to_bottom':    self.parameters.get('origin_to_bottom', []),
            'new_dummy':           self.parameters.get('new_dummy', ''),
            'tray_init_offset':    self.parameters.get('tray_init_offset', []),
            'tray_pose_operator':  tray_pose_operator,
            'tray_final_pose':     tray_final_pose,
        }

        if self.mode == "simulation":
            self._validate_trays(self.trays)
            self._validate_workflow_symbols(self.workflow)

            print("📤 Sending InitParameters to HTTP bridge (and thus to ROS)…")
            success = self.http_client.send_init_parameters(params)
            if not success:
                raise RuntimeError("InitParameters call failed (returned False)")

            print("✅ InitParameters succeeded. RobotManager ready to run workflow.")

            table = params['main_poses'].get('Table', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            tip   = params.get('tip_offset', [0.0, 0.0, 0.0])
            recharge = [
                table[0] - 0.55160 + tip[0],
                table[1] - 0.17248 + tip[1],
                table[2] + 0.82025 + 0.051 + tip[2],
                0.0, 0.0, 0.0
            ]
            params['main_poses']['Recharge'] = recharge

            print("📤 Sending DSL runtime parameters via HTTP...")
            print(json.dumps(params, indent=2))
            self.http_client.send_init_parameters(params)
        else:
            print("🛠️ Real robot mode selected. Skipping HTTP init and creating PLC client…")
            self.tray_models = {}
            self.unit_lookup = {}
            self.tray_names = set()
            self.unit_names = set()
            self.plc_client = PLCClient(logger=self._plc_log)

    def to_camel_case(self, snake_str):
        if '_' not in snake_str:
            return snake_str
        components = snake_str.split('_')
        return ''.join(x.capitalize() for x in components)

    def build_workflow(self, tree):
        parser = DSLParser()
        artifacts = parser.build(tree)
        self.mode = artifacts.mode or self.mode
        self.tray_step_poses = artifacts.tray_step_poses
        self.main_poses = artifacts.main_poses
        self.named_poses = artifacts.named_poses
        self.parameters = artifacts.parameters
        self.location_order = artifacts.location_order
        self.location_name_to_index = artifacts.location_name_to_index
        return artifacts.workflow, artifacts.trays, artifacts.locations



    def _get_named_param(
        self,
        named_params: Dict[str, Any],
        key: str,
        action: str,
        *,
        required: bool = True,
        default: Any = None,
    ):
        if key not in named_params:
            if required:
                print(f"❌ Action '{action}' missing required parameter '{key}'.")
                return None
            return default
        return named_params[key]

    def _extract_index_param(self, action: str, named_params: Dict[str, Any], positional_params: List[Any]):
        # For operator positioning we only use the location/index, not the unit's tray-step.
        if action != "OperatorPositionTray" and "unit" in named_params:
            pose_index = self._resolve_unit_pose_index(str(named_params["unit"]), action)
            if pose_index is not None:
                return pose_index
        if "index" in named_params:
            return int(named_params["index"])
        if "location" in named_params:
            loc_name = str(named_params["location"])
            idx = self.location_name_to_index.get(loc_name)
            if idx is None:
                print(f"❌ Action '{action}' references unknown location '{loc_name}'.")
                return None
            print(f"   ↳ Resolved location '{loc_name}' → index {idx}")
            return idx
        if positional_params:
            return int(positional_params[0])
        print(f"❌ Action '{action}' requires an 'index' or 'location' parameter.")
        return None

    def _log_tray_pose(self, idx: int, context: str):
        if not self.tray_step_poses:
            print(f"⚠️ No tray_step_poses available while handling {context}.")
            return
        if 0 <= idx < len(self.tray_step_poses):
            pose = self.tray_step_poses[idx]
            name = None
            if 0 <= idx < len(self.tray_step_pose_names):
                name = self.tray_step_pose_names[idx]
            label = name or f"index {idx}"
            print(f"🧭 {context}: tray_step_pose '{label}' = {pose}")
        else:
            print(f"⚠️ {context}: tray_step_pose index {idx} out of range (len={len(self.tray_step_poses)}).")

    def _list_to_named_params(self, param_list: List[Any]) -> Dict[str, Any]:
        if not isinstance(param_list, list) or len(param_list) < 2:
            return {}
        result: Dict[str, Any] = {}
        if len(param_list) % 2 != 0:
            return {}
        it = iter(param_list)
        for key in it:
            value = next(it, None)
            if not isinstance(key, str):
                return {}
            if isinstance(value, str) and value.isdigit():
                value = int(value)
            result[key] = value
        return result

    def _resolve_unit_pose_index(self, unit_name: str, action: str):
        entry = self.unit_lookup.get(unit_name)
        if not entry:
            print(f"❌ Action '{action}' references unknown unit '{unit_name}'.")
            return None
        pose_index = entry.get("pose_index")
        if pose_index is None:
            print(f"❌ Unit '{unit_name}' does not define a pose_index.")
            return None
        return int(pose_index)

    def _maybe_inject_unit_index(self, action: str, named_params: Dict[str, Any]):
        if action not in self.UNIT_INDEX_ACTIONS or not isinstance(named_params, dict):
            return named_params
        if "index" in named_params or "unit" not in named_params:
            return named_params
        unit_name = str(named_params["unit"])
        pose_index = self._resolve_unit_pose_index(unit_name, action)
        if pose_index is None:
            return None
        clone = dict(named_params)
        clone["index"] = pose_index
        return clone

    def _plc_log(self, message: str):
        print(f"[PLC] {message}")

    def _print_operator_guidance(self, target_location: Optional[str] = None, target_unit: Optional[str] = None):
        trays = getattr(self, "trays", {}) or {}
        if not trays:
            print("ℹ️ Operator guidance unavailable (no tray metadata).")
            return
        if not target_unit:
            print("ℹ️ OperatorPositionTray: add 'unit <name>' to show screw info for that unit.")
            return
        print("👷 Operator manual screw guide:")
        any_printed = False
        for tray_name, tray in trays.items():
            units = tray.get("units") or []
            for unit in units:
                unit_name = unit.get("name", "<unnamed>")
                if target_unit and unit_name != target_unit:
                    continue
                screws = unit.get("screws") or {}
                manual = screws.get("manual_indices") or []
                auto = screws.get("auto_indices") or []
                pose_idx = unit.get("pose_index")
                pose_name = unit.get("pose_name") or self.unit_lookup.get(unit_name, {}).get("pose_name")
                if not manual and not auto:
                    continue
                parts = []
                if manual:
                    parts.append(f"manual {sorted(int(v) for v in manual)}")
                if auto:
                    parts.append(f"auto {sorted(int(v) for v in auto)}")
                if pose_name:
                    pose_str = f"pose='{pose_name}'"
                elif pose_idx is not None:
                    pose_str = f"pose_index={pose_idx}"
                else:
                    pose_str = "pose_index=?"
                print(f"   • Tray '{tray_name}' unit '{unit_name}' ({pose_str}) → {', '.join(parts)}")
                any_printed = True
        if target_unit and not any_printed:
            print(f"   • No screw data found for requested unit '{target_unit}'.")
        if target_location:
            loc = self.locations.get(target_location)
            if loc:
                pos = loc.get("position", [])
                orient = loc.get("orientation", [])
                print(
                    f"📍 Target location '{target_location}': position={pos}, orientation={orient}"
                )
            else:
                print(f"📍 Target location '{target_location}' is not defined in Locations.")

    def _execute_real_task(self, action: str, named_params: Dict[str, Any], positional_params: List[Any]):
        if not self.plc_client:
            print("❌ PLC client is not initialized; cannot execute real-mode workflow.")
            return

        payload = dict(named_params)
        if not payload and positional_params:
            payload = self._list_to_named_params(positional_params)

        if action == "CallBlock":
            block_name = payload.pop("name", None)
            if block_name is None:
                print("❌ CallBlock requires a 'name' parameter.")
                return
        else:
            block_name = action

        payload.pop("location", None)
        payload.pop("target_location", None)

        resolved = resolve_block_name(str(block_name))
        if not resolved:
            print(f"❌ Unknown PLC building block '{block_name}'. Allowed blocks: {sorted(BUILDING_BLOCKS)}")
            return

        confirm_text = f"Execute PLC block '{resolved}' with parameters {payload}?"
        if not self.http_client.confirm_workflow(confirm_text):
            print(f"Skipped '{resolved}' due to user rejection.")
            return

        try:
            result = self.plc_client.call_block(resolved, payload)
        except Exception as exc:
            print(f"❌ PLC CallBlock '{resolved}' failed: {exc}")
            return

        if not result.get("success", False):
            print(f"⚠️ PLC reported failure for '{resolved}': {result}")

    def _ensure_simulation_sections(self, seen_sections: set[str]):
        missing = []
        for section in ("Agents", "Locations", "Trays", "Assembly"):
            if section not in seen_sections:
                missing.append(section)
        if missing:
            raise DSLValidationError([f"Section(s) {', '.join(missing)} are required in simulation mode."])

    def _validate_trays(self, trays):
        errors = []
        self.tray_models = {}
        self.unit_lookup = {}

        for name, payload in trays.items():
            try:
                payload_copy = dict(payload)
                payload_copy.pop("name", None)
                tray_model = TrayModel(name=name, **payload_copy)
            except ValidationError as exc:
                errors.append(f"Tray '{name}' invalid:\n{exc}")
                continue

            self.tray_models[name] = tray_model
            for unit in tray_model.units:
                if unit.name in self.unit_lookup:
                    errors.append(
                        f"Unit '{unit.name}' defined multiple times "
                        f"(already used in tray '{self.unit_lookup[unit.name]['tray']}')"
                    )
                    continue

                pose_index = unit.pose_index
                pose_name = getattr(unit, "pose_name", None)
                if pose_index is None and pose_name:
                    pose_index = self.tray_step_pose_name_to_index.get(pose_name)
                    if pose_index is None:
                        errors.append(
                            f"Unit '{unit.name}' in tray '{name}' references unknown pose '{pose_name}'."
                        )
                        continue
                if pose_index is None:
                    errors.append(
                        f"Unit '{unit.name}' in tray '{name}' must define either 'pose_name' or 'pose_index'."
                    )
                    continue

                screw_meta = unit.screws
                allowed = set()
                if screw_meta:
                    if screw_meta.manual_indices:
                        allowed.update(int(v) for v in screw_meta.manual_indices)
                    if screw_meta.auto_indices:
                        allowed.update(int(v) for v in screw_meta.auto_indices)

                self.unit_lookup[unit.name] = {
                    "tray": name,
                    "pose_index": int(pose_index),
                    "pose_name": pose_name,
                    "allowed_screws": allowed,
                    "definition": unit.dict(),
                }

        if errors:
            raise DSLValidationError(errors)

        self.tray_names = set(self.tray_models.keys())
        self.unit_names = set(self.unit_lookup.keys())

    def _validate_workflow_symbols(self, workflow):
        known_symbols = set(getattr(self, "tray_names", set()))
        known_symbols |= set(getattr(self, "unit_names", set()))
        known_symbols |= set(self.named_poses.keys())
        known_symbols |= set(self.parameters.keys())

        errors: List[str] = []

        for command in workflow:
            action = command["action"]
            params_obj = command.get("params", {})

            schema = self.ACTION_PARAM_SCHEMAS.get(action)
            if schema and isinstance(params_obj, dict):
                required = schema.get("required", {})
                for key, expected in required.items():
                    if key not in params_obj:
                        errors.append(f"Action '{action}' is missing required parameter '{key}'.")
                        continue
                    if expected is int and not isinstance(params_obj[key], int):
                        errors.append(
                            f"Parameter '{key}' in action '{action}' must be an integer (got {params_obj[key]!r})."
                        )
                    if expected is str and not isinstance(params_obj[key], str):
                        errors.append(
                            f"Parameter '{key}' in action '{action}' must be a string (got {params_obj[key]!r})."
                        )

            if action == "InternalScrewUnitHole" and isinstance(params_obj, dict):
                unit_name = params_obj.get("unit")
                hole = params_obj.get("hole")
                if unit_name not in self.unit_lookup:
                    errors.append(f"InternalScrew references unknown unit '{unit_name}'.")
                    continue
                allowed = self.unit_lookup[unit_name]["allowed_screws"]
                if allowed and hole not in allowed:
                    allowed_str = ", ".join(str(v) for v in sorted(allowed))
                    errors.append(
                        f"Hole {hole} is not defined for unit '{unit_name}'. Allowed holes: [{allowed_str}]"
                    )
                continue

            if isinstance(params_obj, list):
                for param in params_obj:
                    if isinstance(param, dict):
                        for value in param.values():
                            if isinstance(value, str) and value not in known_symbols:
                                print(
                                    f"⚠️ Parameter '{value}' in action '{action}' does not match any "
                                    "defined tray, unit, named pose, or parameter."
                                )
                        continue
                    if isinstance(param, str):
                        stripped = param.strip('"')
                        if stripped.isdigit():
                            continue
                        if stripped in known_symbols:
                            continue
                        print(
                            f"⚠️ Parameter '{param}' in action '{action}' does not match any "
                            "defined tray, unit, named pose, or parameter."
                        )

        if errors:
            raise DSLValidationError(errors)

    def execute_task(self, task, module_name="my_python_pkg.functions"):
        action = task["action"]
        params = task.get("params")
        robot_name = task.get("robot")
        from_location = task.get("from")
        to_location = task.get("to")
        control_type = task.get("control_type", "automated")
        task["control_type"] = control_type
        if isinstance(params, dict):
            named_params = params
            positional_params: List[Any] = []
        elif isinstance(params, list):
            named_params = self._list_to_named_params(params)
            positional_params = [] if named_params else params
        else:
            named_params = {}
            positional_params = []

        if isinstance(named_params, dict) and named_params:
            injected = self._maybe_inject_unit_index(action, named_params)
            if injected is None:
                return
            named_params = injected

        if robot_name and isinstance(named_params, dict):
            if from_location and "location" not in named_params:
                named_params["location"] = from_location
            elif to_location and "location" not in named_params:
                named_params["location"] = to_location
            if to_location and "target_location" not in named_params:
                named_params["target_location"] = to_location

        # … code that replaces tray names with objects, digits → ints, etc. …

        # 1) Manual confirmation? (unchanged)
        if control_type == "manual" and self.mode == "simulation":
            task_desc = f"Execute '{action}' with parameters {params}?"
            confirmed = self.http_client.confirm_workflow(task_desc)
            if not confirmed:
                print(f"Skipped '{action}' as it was not confirmed.")
                return

        if self.mode == "real":
            self._execute_real_task(action, named_params, positional_params)
            return

        response_data = {}
        http_action = action
        http_payload: Optional[Dict[str, Any]] = None

        if action == "AddTray":
            tray_name = self._get_named_param(named_params, "tray", action)
            if tray_name is None:
                return
            object_name = self._get_named_param(
                named_params,
                "object",
                action,
                required=False,
                default=tray_name,
            )
            if object_name is None:
                return
            tray_name = str(tray_name).strip('"')
            object_name = str(object_name).strip('"')
            print(f"📡 [HTTP] {action}: tray='{tray_name}', object='{object_name}'")
            http_payload = {"tray": tray_name, "object": object_name}

        elif action in ["PickTray", "PositionTray", "OperatorPositionTray",
                        "RechargeSequence", "InternalScrewingSequence",
                        "ExternalScrewingSequence", "PlaceTray"]:
            idx = self._extract_index_param(action, named_params, positional_params)
            if idx is None:
                return
            if action == "OperatorPositionTray":
                target_unit = named_params.get("unit")
                target_hint = (
                    named_params.get("target_location")
                    or named_params.get("location")
                    or to_location
                )
                self._print_operator_guidance(
                    None if target_hint in {"unit", "units"} else target_hint,
                    target_unit=target_unit,
                )
            target_desc = (
                named_params.get("location")
                or named_params.get("target_location")
                or named_params.get("unit")
                or named_params.get("tray")
                or (f"index {idx}")
            )
            print(f"📡 [HTTP] {action}: target='{target_desc}'")
            location_hint = named_params.get("location")
            if location_hint:
                print(f"   ↳ Resolved location '{location_hint}' → '{target_desc}'")
            if action in {"PositionTray", "InternalScrewingSequence"}:
                self._log_tray_pose(idx, action)
            http_payload = {"index": int(idx)}

        elif action in ("InternalScrewUnitHole", "InternalScrew"):
            unit_name = self._get_named_param(named_params, "unit", action)
            hole = self._get_named_param(named_params, "hole", action)
            if unit_name is None or hole is None:
                return
            try:
                hole = int(hole)
            except (TypeError, ValueError):
                print(f"❌ Action '{action}' hole parameter must be an integer (got {hole!r}).")
                return
            unit_entry = self.unit_lookup.get(unit_name)
            if not unit_entry:
                print(f"❌ Unknown unit '{unit_name}' referenced in InternalScrew.")
                response_data = {"success": False, "error": "unknown_unit"}
            else:
                allowed = unit_entry.get("allowed_screws", set())
                if allowed and hole not in allowed:
                    print(
                        f"❌ Hole {hole} is not defined for unit '{unit_name}'. Allowed: {sorted(allowed)}"
                    )
                    response_data = {"success": False, "error": "invalid_hole"}
                else:
                    tray_index = unit_entry.get("pose_index")
                    if tray_index is None:
                        response_data = {"success": False, "error": "missing_pose_index"}
                    else:
                        # Do not auto-position here; assume the user already aligned the tray (or will do it explicitly).
                        if self.current_tray_step != tray_index:
                            print(
                                f"ℹ️ Tray is currently aligned to step {self.current_tray_step}, "
                                f"but unit '{unit_name}' uses step {tray_index}. "
                                f"Run PositionTray first if alignment is required."
                            )
                        self._log_tray_pose(tray_index, f"InternalScrew({unit_name})")
                        print(f"🧷 InternalScrew unit '{unit_name}' → tray_index {tray_index}, hole {hole}")
                        print(f"📡 [HTTP] InternalScrew unit={unit_name} hole={hole}")
                        http_action = "InternalScrew"
                        http_payload = {"index": int(tray_index), "screw_num": int(hole)}

        if http_payload is None:
            if response_data:
                print(f"Response for '{action}': {response_data}")
                return
            print(f"❌ Action '{action}' is not supported by the simulation HTTP bridge.")
            return

        result_index = None
        if isinstance(http_payload, dict) and "index" in http_payload:
            result_index = http_payload["index"]

        response_data = self.http_client.execute_action(http_action, http_payload)

        if action == "PositionTray" and result_index is not None and response_data.get("success", True):
            self.current_tray_step = int(result_index)
        elif action == "InternalScrewingSequence" and result_index is not None and response_data.get("success", True):
            self.current_tray_step = int(result_index)
        elif action == "PlaceTray" and response_data.get("success", True):
            self.current_tray_step = None

        print(f"Response for '{action}': {response_data}")

    def execute_workflow(self):
        task_queue = queue.Queue()
        for task in self.workflow:
            task_queue.put(task)

        while not task_queue.empty():
            task = task_queue.get()
            self.execute_task(task)
            task_queue.task_done()

        print("✅ Workflow completed successfully.")
