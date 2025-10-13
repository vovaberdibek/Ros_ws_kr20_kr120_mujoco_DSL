from lark import Tree, Token
import queue
import threading
import requests
import json
import re


BASE_URL = "http://localhost:8000"

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
    

class RobotManager:
    def __init__(self, parsed_tree):
        self.http_client = RobotHTTPClient()

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

        for child in tree.children:
            # ————————————————
            # 1) Locations
            if child.data == "locations_definition":
                for loc in child.children:
                    loc_name = loc.children[0].value
                    position = [float(n.value) for n in loc.children[1].children]
                    orientation = [float(n.value) for n in loc.children[2].children]
                    locations[loc_name] = {
                        "position": position,
                        "orientation": orientation
                    }

            # ————————————————
            # 2) Trays (with all nested attributes)
            elif child.data == "trays_definition":
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
                                units_list = []
                                for unit in inner.children:
                                    if unit.data == "unit":
                                        unit_info = {}
                                        for pair in unit.children:
                                            key_token = pair.children[0]
                                            val_token = pair.children[1]
                                            key = key_token.value
                                            if isinstance(val_token, Token):
                                                value = int(val_token.value) if val_token.type == "INT" else val_token.value
                                            elif isinstance(val_token, Tree):
                                                inner_token = val_token.children[0]
                                                value = int(inner_token.value) if inner_token.type == "INT" else inner_token.value
                                            else:
                                                value = str(val_token)
                                            unit_info[key] = value
                                        units_list.append(unit_info)
                                tray_data["units"] = units_list
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
            # 3) Tray Step Poses
            elif child.data == "tray_step_poses_definition":
                self.tray_step_poses = [
                    [float(n.value) for n in entry.children[0].children]
                    for entry in child.children
                ]

            # ————————————————
            # 4) Main Poses
            elif child.data == "main_poses_definition":
                self.main_poses = {
                    pose.children[0].value: [float(n.value) for n in pose.children[1].children]
                    for pose in child.children
                }

            # ————————————————
            # 5) Parameters (tip_offset, tray_angles, named_pose_entry, vector3/int_list/float_list, etc.)
            elif child.data == "parameters_definition":
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
            # 6) Assembly (workflow) definitions
            elif child.data == "assembly_definition":
                for command in child.children:
                    control_type = command.children[0].value

                    # action node must be a Tree
                    action_node = command.children[1]
                    if not isinstance(action_node, Tree):
                        print(f"⚠️ Unexpected action node type: {type(action_node).__name__}")
                        continue

                    action = self.to_camel_case(action_node.data)

                    # collect params from action_node children, skip Nones
                    params_list = []
                    for p in getattr(action_node, "children", []):
                        if p is None:
                            continue
                        params_list.append(getattr(p, "value", str(p)))

                    # no repeat/speed anymore
                    workflow.append({
                        "control_type": control_type,
                        "action":       action,
                        "params":       params_list,
                    })


        print("Extracted Trays:", trays)
        print("Extracted Locations:", locations)
        print("Extracted Tray Step Poses:", self.tray_step_poses)
        print("Extracted Main Poses:", self.main_poses)
        print("Extracted Named Poses:", self.named_poses)

        return workflow, trays, locations

    def execute_task(self, task, module_name="my_python_pkg.functions"):
        action = task["action"]
        params = task["params"]

        # … code that replaces tray names with objects, digits → ints, etc. …

        # 1) Manual confirmation? (unchanged)
        if task["control_type"] == "manual":
            task_desc = f"Execute '{action}' with parameters {params}?"
            confirmed = self.http_client.confirm_workflow(task_desc)
            if not confirmed:
                print(f"Skipped '{action}' as it was not confirmed.")
                return

        # 2) Now choose the correct HTTP call:
        #    - AddTray2 needs two strings
        #    - Single-index actions
        #    - Everything else fall back to a {"params": […]} body

        response_data = {}
        if action == "AddTray2":
            # params list should be [tray_name, object_name], possibly with quotes around them
            tray_name = params[0].strip('"')    # remove any stray quotes
            object_name = params[1].strip('"')
            print(f"📡 [HTTP] AddTray2: tray='{tray_name}', object='{object_name}'")
            response_data = self.http_client.add_tray2(tray_name, object_name)

        elif action in ["AddTray", "PickTray", "PositionTray", "OperatorPositionTray",
                        "RechargeSequence", "InternalScrewingSequence", 
                        "ExternalScrewingSequence", "PlaceTray"]:
            # All of these expect {"index": <int>}
            idx = int(params[0])  # safe, because you ensured param is digit above
            # map CamelCase → snake_case endpoint name
            snake_endpoint = re.sub(r'(?<!^)(?=[A-Z])', '_', action).lower()
            print(f"📡 [HTTP] {action}: index={idx} → /{snake_endpoint}")
            response_data = self.http_client.index_action(snake_endpoint, idx)

        else:
            # Fallback: send everything under {"params": […]}
            # e.g. if there is any other action that expects a "params" list
            print(f"📡 [HTTP] Fallback for '{action}', data={{'params': {params}}}")
            response_data = self.http_client.generic_params_action(action, params)

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

