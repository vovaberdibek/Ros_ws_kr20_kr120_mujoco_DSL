from lark import Tree, Token
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


BASE_URL = "http://localhost:8000"
BUILDING_BLOCKS = {
    "depositTray",
    "gyroGrpRot",
    "loadTray",
    "mrHoming",
    "mrTrolleyVCheck",
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

    def add_tray2(self, tray_name: str, object_name: str):
        url = f"{self.base_url}/add_tray2"
        payload = {"tray_name": tray_name, "object_name": object_name}
        try:
            res = requests.post(url, json=payload, timeout=30.0)
            return res.json()
        except requests.RequestException as e:
            print(f"❌ AddTray2 HTTP call failed: {e}")
            return {}

    def index_action(self, endpoint: str, index: int):
        """
        For all endpoints that expect {'index': <int>},
        like /pick_tray, /position_tray, etc.
        """
        url = f"{self.base_url}/{endpoint}"
        payload = {"index": index}
        try:
            res = requests.post(url, json=payload, timeout=30.0)
            return res.json()
        except requests.RequestException as e:
            print(f"❌ {endpoint} HTTP call failed: {e}")
            return {}

    def generic_params_action(self, action: str, params_list):
        """
        For any endpoint that expects {"params": [...]}, e.g. some custom endpoints.
        We still include the CamelCase → snake_case transform.
        """
        snake = re.sub(r'(?<!^)(?=[A-Z])', '_', action).lower()
        url = f"{self.base_url}/{snake}"
        try:
            res = requests.post(url, json={"params": params_list}, timeout=30.0)
            return res.json()
        except requests.RequestException as e:
            print(f"❌ HTTP call to {url} failed: {e}")
            return {}

    def internal_screw_by_num(self, index: int, screw_num: int):
        url = f"{self.base_url}/internal_screw_by_num"
        payload = {"index": int(index), "ScrewNum": int(screw_num)}
        try:
            res = requests.post(url, json=payload, timeout=60.0)
            return res.json()
        except requests.RequestException as e:
            print(f"❌ internal_screw_by_num HTTP call failed: {e}")
            return {}


    # def execute_action(self, action, data):
    #     try:
    #         response = requests.post(
    #             f"{self.base_url}/{action.lower()}",
    #             json=data,
    #             timeout=5.0
    #         )
    #         return response.json()
    #     except requests.RequestException as e:
    #         print(f"❌ {action} HTTP call failed: {e}")
    #         return {}
        
    def execute_action(self, action, data):
        """
        Convert CamelCase (e.g. "PositionTray") → snake_case (e.g. "position_tray"),
        then do POST /<that> with JSON payload.
        """
        # 1) insert underscore before each capital letter (except the very first),
        #    then lowercase everything.
        snake = re.sub(r'(?<!^)(?=[A-Z])', '_', action).lower()
        url = f"{self.base_url}/{snake}"
        response = requests.post(url, json=data, timeout=30.0)
        return response.json()


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
    ACTION_PARAM_SCHEMAS = {
        "AddTray": {"required": {"tray": str, "object": str}},
        "InternalScrewUnitHole": {"required": {"unit": str, "hole": int}},
        "CallBlock": {"required": {"name": str}},
    }

    def __init__(self, parsed_tree):
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

        # Build workflow, trays, locations exactly as before
        self.workflow, self.trays, self.locations = self.build_workflow(parsed_tree)

        # Construct the same "params" dictionary you used in ROS version
        params = {
            'tray_step_poses':     self.tray_step_poses,
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
            'tray_pose_operator':  self.parameters.get('tray_pose_operator', []),
            'tray_final_pose':     self.parameters.get('tray_final_pose', []),
        }

        if self.mode == "simulation":
            # 2) Immediately send them over HTTP
            print("📤 Sending InitParameters to HTTP bridge (and thus to ROS)…")
            success = self.http_client.send_init_parameters(params)
            if not success:
                raise RuntimeError("InitParameters call failed (returned False)")

            # 3) Now it’s safe to continue; tasks will be executed later
            print("✅ InitParameters succeeded. RobotManager ready to run workflow.")

            # Recompute Recharge pose exactly as you did in C++/ROS version:
            table = params['main_poses'].get('Table', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            tip   = params.get('tip_offset', [0.0, 0.0, 0.0])
            recharge = [
                table[0] - 0.55160 + tip[0],
                table[1] - 0.17248 + tip[1],
                table[2] + 0.82025 + 0.051 + tip[2],
                0.0, 0.0, 0.0
            ]
            params['main_poses']['Recharge'] = recharge

            # Debug print just like before
            print("📤 Sending DSL runtime parameters via HTTP...")
            print(json.dumps(params, indent=2))

            # HTTP call replaces rospy.ServiceProxy('init_parameters', ...)
            self.http_client.send_init_parameters(params)
        else:
            print("🛠️ Real robot mode selected. Skipping HTTP init and creating PLC client…")
            self.plc_client = PLCClient(logger=self._plc_log)

    def to_camel_case(self, snake_str):
        components = snake_str.split('_')
        return ''.join(x.capitalize() for x in components)

    def build_workflow(self, tree):
        """
        Replicate your full DSL parsing logic exactly:
        - locations_definition
        - trays_definition (with units, screws, height, initial/operator/final poses)
        - tray_step_poses_definition
        - main_poses_definition
        - parameters_definition (tip_offset, tray_angles, named_pose_entry, vector3, int_list, etc.)
        - assembly_definition (control_type, action, params, repeat, speed)
        """
        if tree is None:
            print("No tree provided, returning empty workflow, trays, and locations.")
            return [], {}, {}

        workflow = []
        trays = {}
        locations = {}

        # Reset storage
        self.tray_step_poses = []
        self.main_poses = {}
        self.named_poses = {}
        self.parameters = {}
        seen_sections: set[str] = set()

        for child in tree.children:
            # ————————————————
            # 0) Mode header
            if child.data == "mode_definition":
                mode_token = child.children[0]
                self.mode = mode_token.value if isinstance(mode_token, Token) else "simulation"
                print(f"🛠️ DSL mode set to '{self.mode}'.")
                continue

            # ————————————————
            # 1) Agents (ignored but tracked for validation)
            if child.data == "agents_definition":
                seen_sections.add("Agents")
                continue

            # ————————————————
            # 2) Locations
            if child.data == "locations_definition":
                seen_sections.add("Locations")
                for loc in child.children:
                    loc_name = loc.children[0].value
                    position = [float(n.value) for n in loc.children[1].children]
                    orientation = [float(n.value) for n in loc.children[2].children]
                    locations[loc_name] = {
                        "position": position,
                        "orientation": orientation
                    }
                    if loc_name not in self.location_name_to_index:
                        idx = len(self.location_order)
                        self.location_order.append(loc_name)
                        self.location_name_to_index[loc_name] = idx

            # ————————————————
            # 3) Trays (with all nested attributes)
            elif child.data == "trays_definition":
                seen_sections.add("Trays")
                for tray in child.children:
                    tray_name = tray.children[0].value
                    tray_data = {
                        "name":         tray_name,
                        "units":        [],
                        "screws":       [],
                        "tray":         None,
                        "height":       None,
                        "initial_pose": None,
                        "operator_pose":None,
                        "final_pose":   None
                    }
                    for attr in tray.children[1:]:
                        if isinstance(attr, Tree) and attr.data == "tray_attribute":
                            inner = attr.children[0]
                            if inner.data == "tray_line":
                                tray_data["tray"] = inner.children[0].value
                            elif inner.data == "units_def":
                                tray_data["units"] = [
                                    self._parse_unit_tree(unit)
                                    for unit in inner.children
                                    if unit.data == "unit"
                                ]
                            elif inner.data == "screws_def":
                                screws_list = [
                                    {
                                        "quantity": int(screw.children[0].value),
                                        "type":     screw.children[1].value
                                    }
                                    for screw in inner.children
                                ]
                                tray_data["screws"] = screws_list
                            elif inner.data == "height_def":
                                tray_data["height"] = float(inner.children[0].value)
                            elif inner.data == "initial_pose":
                                tray_data["initial_pose"] = [float(n.value) for n in inner.children[0].children]
                            elif inner.data == "operator_pose":
                                tray_data["operator_pose"] = [float(n.value) for n in inner.children[0].children]
                            elif inner.data == "final_pose":
                                tray_data["final_pose"] = [float(n.value) for n in inner.children[0].children]
                    trays[tray_name] = tray_data

            # ————————————————
            # 4) Tray Step Poses
            elif child.data == "tray_step_poses_definition":
                seen_sections.add("TrayStepPoses")
                self.tray_step_poses = [
                    [float(n.value) for n in entry.children[0].children]
                    for entry in child.children
                ]

            # ————————————————
            # 5) Main Poses
            elif child.data == "main_poses_definition":
                seen_sections.add("MainPoses")
                self.main_poses = {
                    pose.children[0].value: [float(n.value) for n in pose.children[1].children]
                    for pose in child.children
                }

            # ————————————————
            # 6) Parameters (tip_offset, tray_angles, named_pose_entry, vector3/int_list/float_list, etc.)
            elif child.data == "parameters_definition":
                seen_sections.add("Parameters")
                self.parameters = {}
                self.named_poses = {}
                for param in child.children:
                    if len(param.children) == 0:
                        continue
                    param_entry = param.children[0]
                    # Named pose entries
                    if param_entry.data == "named_pose_entry":
                        pose_name   = param_entry.children[0].value
                        pose_values = [float(n.value) for n in param_entry.children[1].children]
                        self.named_poses[pose_name] = pose_values
                        continue

                    # Otherwise, vector3, vector2, vector6, int_list, float_list
                    param_name = param_entry.data
                    inner = param_entry.children[0]
                    if inner.data in {"vector3", "vector2", "vector6"}:
                        values = [float(n.value) for n in inner.children]
                    elif inner.data == "int_list":
                        values = [int(n.value) for n in inner.children]
                    elif inner.data == "float_list":
                        values = [float(n.value) for n in inner.children]
                    else:
                        print(f"⚠️ Unhandled parameter type '{inner.data}' for param '{param_name}'")
                        continue
                    self.parameters[param_name] = values

                print("✅ Extracted Parameters:", self.parameters)
                print("✅ Extracted Named Poses:", self.named_poses)

            # ————————————————
            # 7) Assembly (workflow) definitions
            elif child.data == "assembly_definition":
                seen_sections.add("Assembly")
                for command in child.children:
                    control_type = command.children[0].value
                    action_node = command.children[1]
                    if not isinstance(action_node, Tree):
                        print(f"⚠️ Unexpected action node type: {type(action_node).__name__}")
                        continue

                    action = self.to_camel_case(action_node.data)

                    named_params = self._named_args_to_dict(action_node)
                    if action in self.ACTION_PARAM_SCHEMAS:
                        params_obj = named_params
                    else:
                        if named_params:
                            params_obj = named_params
                        else:
                            params_obj = []
                            for p in getattr(action_node, "children", []):
                                if p is None:
                                    continue
                                params_obj.append(self._literal_from_node(p))

                    workflow.append({
                        "control_type": control_type,
                        "action":       action,
                        "params":       params_obj,
                    })

        if self.mode == "simulation":
            self._validate_trays(trays)
            self._validate_workflow_symbols(workflow)
        else:
            self.tray_models = {}
            self.unit_lookup = {}
            self.tray_names = set()
            self.unit_names = set()

        print("Extracted Trays:", trays)
        print("Extracted Locations:", locations)
        print("Extracted Tray Step Poses:", self.tray_step_poses)
        print("Extracted Main Poses:", self.main_poses)
        print("Extracted Named Poses:", self.named_poses)

        return workflow, trays, locations

    def _parse_unit_tree(self, unit_node: Tree) -> Dict[str, Any]:
        unit_info: Dict[str, Any] = {}
        for child in unit_node.children:
            if not isinstance(child, Tree) or child.data != "unit_field":
                continue
            key_token = child.children[0]
            value_node = child.children[1]
            value = self._parse_unit_value(value_node)
            unit_info[key_token.value] = value
        return unit_info

    def _parse_unit_value(self, node):
        if isinstance(node, Token):
            if node.type == "INT":
                return int(node.value)
            return node.value
        if isinstance(node, Tree):
            if node.data == "unit_value" and node.children:
                return self._parse_unit_value(node.children[0])
            if node.data == "screw_block":
                return self._parse_screw_block(node)
            if node.data == "int_list":
                return [int(tok.value) for tok in node.children if isinstance(tok, Token)]
            if node.data == "float_list":
                return [float(tok.value) for tok in node.children if isinstance(tok, Token)]
            if node.data == "vector6":
                return [float(tok.value) for tok in node.children]
        return str(node)

    def _parse_screw_block(self, block_node: Tree) -> Dict[str, Any]:
        data: Dict[str, Any] = {}
        for field in block_node.children:
            if not isinstance(field, Tree):
                continue
            inner = field.children[0] if field.children else None
            if not isinstance(inner, Tree):
                continue

            if inner.data == "manual_field":
                int_list = inner.children[0] if inner.children else None
                data["manual_indices"] = [
                    int(tok.value) for tok in getattr(int_list, "children", []) if isinstance(tok, Token)
                ]
            elif inner.data == "auto_field":
                int_list = inner.children[0] if inner.children else None
                data["auto_indices"] = [
                    int(tok.value) for tok in getattr(int_list, "children", []) if isinstance(tok, Token)
                ]
            elif inner.data == "positions_field":
                vector_group = inner.children[0] if inner.children else None
                poses = []
                for vec in getattr(vector_group, "children", []):
                    if isinstance(vec, Tree) and vec.data == "vector6":
                        poses.append([float(tok.value) for tok in vec.children])
                data["positions"] = poses
        return data

    def _literal_from_node(self, node):
        if isinstance(node, Token):
            if node.type == "INT":
                return int(node.value)
            if node.type == "NUMBER":
                return float(node.value)
            if node.type == "STRING":
                return node.value.strip("\"")
            if node.type == "FLOAT":
                return float(node.value)
            return node.value
        if isinstance(node, Tree):
            if hasattr(node, "value"):
                return node.value
            if getattr(node, "children", None):
                return self._literal_from_node(node.children[0])
        return str(node)

    def _named_args_to_dict(self, action_node: Tree) -> Dict[str, Any]:
        params: Dict[str, Any] = {}
        for child in getattr(action_node, "children", []):
            if not isinstance(child, Tree) or child.data != "named_arg":
                continue
            if len(child.children) < 2:
                continue
            key_token = child.children[0]
            value_node = child.children[1]
            key = key_token.value
            if key in params:
                raise DSLValidationError([
                    f"Duplicate parameter '{key}' provided for action '{self.to_camel_case(action_node.data)}'."
                ])
            params[key] = self._literal_from_node(value_node)
        return params

    def _get_named_param(self, named_params: Dict[str, Any], key: str, action: str):
        if key not in named_params:
            print(f"❌ Action '{action}' missing required parameter '{key}'.")
            return None
        return named_params[key]

    def _extract_index_param(self, action: str, named_params: Dict[str, Any], positional_params: List[Any]):
        if "index" in named_params:
            return int(named_params["index"])
        if "location" in named_params:
            loc_name = str(named_params["location"])
            idx = self.location_name_to_index.get(loc_name)
            if idx is None:
                print(f"❌ Action '{action}' references unknown location '{loc_name}'.")
                return None
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
            print(f"🧭 {context}: tray_step_pose[{idx}] = {pose}")
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

    def _plc_log(self, message: str):
        print(f"[PLC] {message}")

    def _execute_real_task(self, action: str, named_params: Dict[str, Any], positional_params: List[Any]):
        if not self.plc_client:
            print("❌ PLC client is not initialized; cannot execute real-mode workflow.")
            return

        if action != "CallBlock":
            print(f"⚠️ Skipping '{action}' in real mode. Only CallBlock actions are executed.")
            return

        payload = dict(named_params)
        if not payload and positional_params:
            payload = self._list_to_named_params(positional_params)

        block_name = payload.pop("name", None)
        if block_name is None:
            print("❌ CallBlock requires a 'name' parameter.")
            return

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

                if unit.pose_index is None:
                    errors.append(f"Unit '{unit.name}' in tray '{name}' is missing 'pose_index'.")
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
                    "pose_index": int(unit.pose_index),
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
        if isinstance(params, dict):
            named_params = params
            positional_params: List[Any] = []
        elif isinstance(params, list):
            named_params = self._list_to_named_params(params)
            positional_params = [] if named_params else params
        else:
            named_params = {}
            positional_params = []

        # … code that replaces tray names with objects, digits → ints, etc. …

        # 1) Manual confirmation? (unchanged)
        if task["control_type"] == "manual" and self.mode == "simulation":
            task_desc = f"Execute '{action}' with parameters {params}?"
            confirmed = self.http_client.confirm_workflow(task_desc)
            if not confirmed:
                print(f"Skipped '{action}' as it was not confirmed.")
                return

        if self.mode == "real":
            self._execute_real_task(action, named_params, positional_params)
            return

        # 2) Now choose the correct HTTP call:
        #    - AddTray uses named parameters (tray/object)
        #    - Single-index actions
        #    - Everything else fall back to a {"params": […]} body

        response_data = {}
        if action == "AddTray":
            tray_name = self._get_named_param(named_params, "tray", action)
            object_name = self._get_named_param(named_params, "object", action)
            if tray_name is None or object_name is None:
                return
            tray_name = str(tray_name).strip('"')
            object_name = str(object_name).strip('"')
            print(f"📡 [HTTP] AddTray: tray='{tray_name}', object='{object_name}'")
            response_data = self.http_client.add_tray2(tray_name, object_name)

        elif action in ["PickTray", "PositionTray", "OperatorPositionTray",
                        "RechargeSequence", "InternalScrewingSequence",
                        "ExternalScrewingSequence", "PlaceTray"]:
            idx = self._extract_index_param(action, named_params, positional_params)
            if idx is None:
                return
            snake_endpoint = re.sub(r'(?<!^)(?=[A-Z])', '_', action).lower()
            print(f"📡 [HTTP] {action}: index={idx} → /{snake_endpoint}")
            response_data = self.http_client.index_action(snake_endpoint, idx)
            location_hint = named_params.get("location")
            if location_hint:
                print(f"   ↳ Resolved location '{location_hint}' → index {idx}")
            self._log_tray_pose(idx, action)
            if action == "PositionTray" and response_data.get("success", True):
                self.current_tray_step = idx
            elif action == "InternalScrewingSequence" and response_data.get("success", True):
                self.current_tray_step = idx
            elif action == "PlaceTray" and response_data.get("success", True):
                self.current_tray_step = None

        elif action == "InternalScrewUnitHole":
            unit_name = self._get_named_param(named_params, "unit", action)
            hole = self._get_named_param(named_params, "hole", action)
            if unit_name is None or hole is None:
                return
            hole = int(hole)
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
                        # Auto-position the tray if it isn't already aligned to this unit.
                        if self.current_tray_step != tray_index:
                            print(
                                f"↪️ Auto PositionTray: aligning tray step {tray_index} for unit '{unit_name}'."
                            )
                            auto_resp = self.http_client.index_action("position_tray", tray_index)
                            print(f"Response for auto PositionTray: {auto_resp}")
                            if not auto_resp.get("success", True):
                                response_data = {
                                    "success": False,
                                    "error": "position_failed",
                                    "detail": auto_resp,
                                }
                            else:
                                self.current_tray_step = tray_index
                        if not response_data or response_data.get("success", True):
                            self._log_tray_pose(tray_index, f"InternalScrew({unit_name})")
                            print(f"🧷 InternalScrew unit '{unit_name}' → tray_index {tray_index}, hole {hole}")
                            print(
                                f"📡 [HTTP] InternalScrew unit={unit_name} hole={hole} (tray step {tray_index})"
                            )
                            response_data = self.http_client.internal_screw_by_num(tray_index, hole)

        else:
            # Fallback: send everything under {"params": […]}
            # e.g. if there is any other action that expects a "params" list
            fallback_params = []
            if isinstance(params, list):
                fallback_params = params
            elif isinstance(params, dict) and params:
                fallback_params = [params]
            print(f"📡 [HTTP] Fallback for '{action}', data={{'params': {fallback_params}}}")
            response_data = self.http_client.generic_params_action(action, fallback_params)

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
