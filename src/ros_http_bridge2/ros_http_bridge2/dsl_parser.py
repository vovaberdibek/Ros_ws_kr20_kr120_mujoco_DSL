from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from lark import Tree, Token
from .dsl_models import DSLValidationError


ACTION_PARAM_ORDER = {
    "AddTray": ["tray", "object"],
    "PickTray": ["location"],
    "OperatorPositionTray": ["location"],
    "PositionTray": ["unit"],
    "RechargeSequence": ["unit"],
    "InternalScrewingSequence": ["unit"],
    "InternalScrew": ["unit", "hole"],
    "PlaceTray": [],
    "ScrewPickup": ["screw"],
    "LoadTray": ["load"],
    "DepositTray": [],
    "MrHoming": [],
    "SrHoming": [],
    "PositionerRotate": ["clockwise"],
    "GyroGrpRot": ["direction"],
    "Present2Op": ["side", "face"],
    "PresentToScrew": ["side", "face"],
    "MrTrolleyVCheck": [],
    "MrTrolleyVCheckErrorCheck": [],
    "StackTray": ["number"],
    "PickUpTray": [],
}


@dataclass
class WorkflowArtifacts:
    """Structured representation of the DSL input."""

    mode: str = "simulation"
    agents: List[str] = field(default_factory=list)
    workflow: List[Dict[str, Any]] = field(default_factory=list)
    trays: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    locations: Dict[str, Dict[str, List[float]]] = field(default_factory=dict)
    tray_step_poses: List[List[float]] = field(default_factory=list)
    tray_step_pose_names: List[Optional[str]] = field(default_factory=list)
    main_poses: Dict[str, List[float]] = field(default_factory=dict)
    named_poses: Dict[str, List[float]] = field(default_factory=dict)
    parameters: Dict[str, Any] = field(default_factory=dict)
    location_order: List[str] = field(default_factory=list)
    location_name_to_index: Dict[str, int] = field(default_factory=dict)


class DSLParser:
    """Parses the Lark tree into structured workflow artifacts."""

    def build(self, tree: Tree) -> WorkflowArtifacts:
        if tree is None:
            raise DSLValidationError(["Empty DSL tree provided."])

        self.mode = "simulation"
        self.agents: List[str] = []
        self.workflow: List[Dict[str, Any]] = []
        self.trays: Dict[str, Dict[str, Any]] = {}
        self.locations: Dict[str, Dict[str, List[float]]] = {}
        self.tray_step_poses: List[List[float]] = []
        self.tray_step_pose_names: List[Optional[str]] = []
        self.tray_step_pose_names: List[Optional[str]] = []
        self.main_poses: Dict[str, List[float]] = {}
        self.named_poses: Dict[str, List[float]] = {}
        self.parameters: Dict[str, Any] = {}
        self.location_order: List[str] = []
        self.location_name_to_index: Dict[str, int] = {}
        self._tray_step_entries: List[tuple[str, Any]] = []

        for child in tree.children:
            if not isinstance(child, Tree):
                continue
            handler = getattr(self, f"_handle_{child.data}", None)
            if handler:
                handler(child)

        self._finalize_tray_step_poses()

        return WorkflowArtifacts(
            mode=self.mode,
            agents=self.agents,
            workflow=self.workflow,
            trays=self.trays,
            locations=self.locations,
            tray_step_poses=self.tray_step_poses,
            tray_step_pose_names=self.tray_step_pose_names,
            main_poses=self.main_poses,
            named_poses=self.named_poses,
            parameters=self.parameters,
            location_order=self.location_order,
            location_name_to_index=self.location_name_to_index,
        )

    # ---------------------------------------
    # Section handlers
    # ---------------------------------------
    def _handle_mode_definition(self, node: Tree):
        token = node.children[0]
        self.mode = token.value if isinstance(token, Token) else "simulation"

    def _handle_agents_definition(self, node: Tree):
        for agent in node.children:
            if isinstance(agent, Token):
                self.agents.append(agent.value)

    def _handle_locations_definition(self, node: Tree):
        for loc in node.children:
            if not isinstance(loc, Tree):
                continue
            name = loc.children[0].value
            position = [float(tok.value) for tok in loc.children[1].children]
            orientation = [float(tok.value) for tok in loc.children[2].children]
            self.locations[name] = {"position": position, "orientation": orientation}
            pose_vec = list(position) + list(orientation)
            self.named_poses[name] = pose_vec
            self.main_poses[name] = pose_vec
            if name not in self.location_name_to_index:
                idx = len(self.location_order)
                self.location_order.append(name)
                self.location_name_to_index[name] = idx

    def _handle_trays_definition(self, node: Tree):
        for tray in node.children:
            name = tray.children[0].value
            payload: Dict[str, Any] = {
                "name": name,
                "units": [],
                "screws": [],
                "tray": None,
                "height": None,
                "initial_pose": None,
                "operator_pose": None,
                "final_pose": None,
            }
            for attribute in tray.children[1:]:
                if not isinstance(attribute, Tree) or attribute.data != "tray_attribute":
                    continue
                inner = attribute.children[0]
                if inner.data == "tray_line":
                    payload["tray"] = inner.children[0].value
                elif inner.data == "units_def":
                    payload["units"] = [
                        self._parse_unit_tree(unit)
                        for unit in inner.children
                        if isinstance(unit, Tree) and unit.data == "unit"
                    ]
                elif inner.data == "screws_def":
                    payload["screws"] = [
                        {
                            "quantity": int(s.children[0].value),
                            "type": s.children[1].value,
                        }
                        for s in inner.children
                    ]
                elif inner.data == "height_def":
                    payload["height"] = float(inner.children[0].value)
                elif inner.data == "initial_pose":
                    payload["initial_pose"] = [float(tok.value) for tok in inner.children[0].children]
                elif inner.data == "operator_pose":
                    payload["operator_pose"] = [float(tok.value) for tok in inner.children[0].children]
                elif inner.data == "final_pose":
                    payload["final_pose"] = [float(tok.value) for tok in inner.children[0].children]
            self.trays[name] = payload

    def _handle_tray_step_poses_definition(self, node: Tree):
        for entry in node.children:
            self._tray_step_entries.append(self._parse_tray_step_entry(entry))

    def _handle_poses_definition(self, node: Tree):
        for pose in node.children:
            if not isinstance(pose, Tree):
                continue
            name = pose.children[0].value
            values = [float(tok.value) for tok in pose.children[1].children]
            self.main_poses[name] = values
            self.named_poses[name] = values

    def _handle_parameters_definition(self, node: Tree):
        for parameter in node.children:
            if not isinstance(parameter, Tree) or not parameter.children:
                continue
            inner = parameter.children[0]
            if not isinstance(inner, Tree):
                continue
            rule = str(inner.data)
            value_tree = inner.children[0] if inner.children else None
            if not isinstance(value_tree, Tree):
                continue
            camel = self._rule_to_name(rule)
            snake = rule
            if value_tree.data == "vector3":
                values = [float(tok.value) for tok in value_tree.children]
            elif value_tree.data == "vector2":
                values = [float(tok.value) for tok in value_tree.children]
            elif value_tree.data == "int_list":
                values = [int(tok.value) for tok in value_tree.children if isinstance(tok, Token)]
            elif value_tree.data == "float_list":
                values = [float(tok.value) for tok in value_tree.children if isinstance(tok, Token)]
            else:
                continue
            self.parameters[camel] = values
            self.parameters[snake] = values

    def _handle_assembly_definition(self, node: Tree):
        for entry in node.children:
            if not isinstance(entry, Tree) or entry.data != "assembly_command":
                continue
            control_clause = None
            payload = None
            for child in entry.children:
                if child is None:
                    continue
                if isinstance(child, Tree) and child.data == "control_clause":
                    control_clause = child
                elif isinstance(child, Tree) and child.data in {"legacy_action", "robot_command"}:
                    payload = child
            if payload is None:
                continue
            if payload.data == "robot_command":
                task = self._parse_robot_command(payload)
            else:
                control_type = "automated"
                if control_clause and control_clause.children:
                    control_type = control_clause.children[0].value
                task = self._parse_legacy_command(payload, control_type)
            if control_clause and control_clause.children and payload.data == "robot_command":
                task["control_type"] = control_clause.children[0].value
            self.workflow.append(task)

    def _parse_legacy_command(self, action_node: Tree, control_type: str) -> Dict[str, Any]:
        action = self._to_camel_case(action_node.data)
        named_params = self._named_args_to_dict(action_node)
        if named_params:
            params_obj: Any = named_params
        else:
            params_obj = [
                self._literal_from_node(child)
                for child in getattr(action_node, "children", [])
                if not getattr(child, "data", None) == "named_arg"
            ]
        return {
            "control_type": control_type,
            "action": action,
            "params": params_obj,
        }

    # ---------------------------------------
    # Helpers
    # ---------------------------------------
    def _parse_robot_command(self, node: Tree) -> Dict[str, Any]:
        robot_token = node.children[0]
        action_node = node.children[1]
        list_node = node.children[2] if len(node.children) > 2 else None
        robot = robot_token.value
        action_raw = action_node.children[0].value if isinstance(action_node, Tree) and action_node.children else action_node.value
        action = self._to_camel_case(action_raw)
        params: Dict[str, Any] = {}
        positional_values: List[Any] = []
        from_loc: Optional[str] = None
        to_loc: Optional[str] = None
        confirm = False
        items = getattr(list_node, "children", []) if isinstance(list_node, Tree) else []
        for item in items:
            inner = item
            if isinstance(item, Tree) and item.data == "robot_command_item" and item.children:
                inner = item.children[0]
            if not isinstance(inner, Tree):
                continue
            if inner.data == "robot_param":
                key = inner.children[0].value
                value_node = inner.children[1] if len(inner.children) > 1 else None
                value = self._literal_from_node(value_node)
                params[key] = value
            elif inner.data == "robot_positional_param":
                value_node = inner.children[0] if inner.children else None
                value = self._literal_from_node(value_node)
                positional_values.append(value)
            elif inner.data == "from_clause" and inner.children:
                from_loc = inner.children[0].value
            elif inner.data == "to_clause" and inner.children:
                to_loc = inner.children[0].value
            elif inner.data == "confirm_clause":
                confirm = True

        if positional_values:
            ordered_keys = ACTION_PARAM_ORDER.get(action, [])
            if not ordered_keys:
                raise DSLValidationError(
                    [f"Action '{action}' does not accept positional parameters. Provide explicit keys like 'param value'."]
                )
            key_index = 0
            for value in positional_values:
                # Advance to the next key that does not already have a value
                key = None
                while key_index < len(ordered_keys):
                    candidate = ordered_keys[key_index]
                    key_index += 1
                    if candidate not in params:
                        key = candidate
                        break
                if key is None:
                    raise DSLValidationError(
                        [
                            f"Action '{action}' accepts at most {len(ordered_keys)} positional parameter(s); "
                            f"no slot available for extra value '{value}'."
                        ]
                    )
                params[key] = value

        task = {
            "robot": robot,
            "action": action,
            "params": params,
            "from": from_loc,
            "to": to_loc,
            "confirm": confirm,
            "control_type": "manual" if confirm else "automated",
        }
        return task

    def _parse_unit_tree(self, node: Tree) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        for child in node.children:
            if not isinstance(child, Tree) or child.data != "unit_field":
                continue
            key = child.children[0].value
            value = self._parse_unit_value(child.children[1])
            result[key] = value
        return result

    def _parse_unit_value(self, node: Any):
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

    def _parse_screw_block(self, node: Tree) -> Dict[str, Any]:
        data: Dict[str, Any] = {}
        for field in node.children:
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

    def _literal_from_node(self, node: Any):
        if isinstance(node, Token):
            if node.type == "INT":
                return int(node.value)
            if node.type == "NUMBER":
                return float(node.value)
            if node.type == "STRING":
                return node.value.strip('"')
            return node.value
        if isinstance(node, Tree) and getattr(node, "children", None):
            return self._literal_from_node(node.children[0])
        return node

    def _named_args_to_dict(self, action_node: Tree) -> Dict[str, Any]:
        params: Dict[str, Any] = {}
        for child in getattr(action_node, "children", []):
            if not isinstance(child, Tree) or child.data != "named_arg":
                continue
            if len(child.children) < 2:
                continue
            key = child.children[0].value
            if key in params:
                raise DSLValidationError(
                    [f"Duplicate parameter '{key}' provided for action '{self._to_camel_case(action_node.data)}'."]
                )
            value = self._literal_from_node(child.children[1])
            params[key] = value
        return params

    def _parse_tray_step_entry(self, entry: Any):
        if isinstance(entry, Token):
            return ("name", entry.value)
        if isinstance(entry, Tree):
            if entry.data == "tray_pose_entry" and entry.children:
                return self._parse_tray_step_entry(entry.children[0])
            if entry.data == "vector6":
                return ("vector", [float(tok.value) for tok in entry.children])
        raise DSLValidationError(["TrayStepPoses entries must be pose names or [x,y,z,rx,ry,rz] vectors."])

    def _finalize_tray_step_poses(self):
        resolved: List[List[float]] = []
        names: List[Optional[str]] = []
        for kind, payload in getattr(self, "_tray_step_entries", []):
            if kind == "vector":
                resolved.append(payload)
                names.append(None)
            else:
                pose = self.named_poses.get(payload)
                if pose is None:
                    raise DSLValidationError([
                        f"TrayStepPose '{payload}' is not defined in Poses or Parameters."
                    ])
                resolved.append(pose)
                names.append(payload)
        self.tray_step_poses = resolved
        self.tray_step_pose_names = names

    def _to_camel_case(self, name: str) -> str:
        if not name:
            return name
        if "_" in name:
            parts = name.split("_")
            return "".join(part.capitalize() for part in parts if part)
        return name[0].upper() + name[1:]

    def _rule_to_name(self, rule: str) -> str:
        parts = rule.split("_")
        return "".join(part.capitalize() for part in parts if part)

    def _to_snake_case(self, name: str) -> str:
        snake = ""
        for ch in name:
            if ch.isupper() and snake:
                snake += "_"
            snake += ch.lower()
        return snake
