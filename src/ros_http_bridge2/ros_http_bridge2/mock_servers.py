#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# ---- custom srv types ----
from custom_interfaces.srv import (
    InitParameters,
    AddTray, AddTray2,
    BringTray,
    PickTray, PositionTray, OperatorPositionTray, PlaceTray,
    RechargeSequence,
    InternalScrewingSequence, InternalScrewByNum,
    ExternalScrewingSequence,
)

def _fmt_arr(arr, maxlen=12):
    try:
        n = len(arr)
        head = list(arr[:maxlen])
        tail = [] if n <= maxlen else ["..."]
        return f"{head + tail} (len={n})"
    except Exception:
        return str(arr)

class MockServiceServer(Node):
    def __init__(self):
        super().__init__("mock_custom_srv_server")

        # ---- advertise all services ----
        self.create_service(InitParameters,           "/init_parameters",           self.cb_init_parameters)
        self.create_service(AddTray,                  "/add_tray",                  self.cb_add_tray)
        self.create_service(AddTray2,                 "/add_tray2",                 self.cb_add_tray2)
        self.create_service(BringTray,                "/bring_tray",                self.cb_bring_tray)
        self.create_service(PickTray,                 "/pick_tray",                 self.cb_index_only("PickTray"))
        self.create_service(PositionTray,             "/position_tray",             self.cb_index_only("PositionTray"))
        self.create_service(OperatorPositionTray,     "/operator_position_tray",    self.cb_index_only("OperatorPositionTray"))
        self.create_service(PlaceTray,                "/place_tray",                self.cb_index_only("PlaceTray"))
        self.create_service(RechargeSequence,         "/recharge_sequence",         self.cb_index_only("RechargeSequence"))
        self.create_service(InternalScrewingSequence, "/internal_screwing_sequence",self.cb_index_only("InternalScrewingSequence"))
        self.create_service(InternalScrewByNum,       "/internal_screw_by_num",     self.cb_internal_screw_by_num)
        self.create_service(ExternalScrewingSequence, "/external_screwing_sequence",self.cb_index_only("ExternalScrewingSequence"))

        self.get_logger().info("✅ Mock service server is up. Waiting for requests…")

    # ---------- Callbacks ----------
    def cb_init_parameters(self, req: InitParameters.Request, res: InitParameters.Response):
        self.get_logger().info("📥 /init_parameters called")

        # Log a concise summary (avoid spamming the console with long arrays)
        self.get_logger().info(
            "tray_step_poses=" + _fmt_arr(req.tray_step_poses)
        )
        self.get_logger().info(f"tray_final_pose={list(req.tray_final_pose)}")
        self.get_logger().info(f"tray_pose_operator={list(req.tray_pose_operator)}")
        self.get_logger().info(f"table_pose={list(req.table_pose)} tray_init_pose={list(req.tray_init_pose)}")
        self.get_logger().info(f"bottom_init_pose={list(req.bottom_init_pose)} bottom_pick_pose={list(req.bottom_pick_pose)}")
        self.get_logger().info(f"recharge_pose={list(req.recharge_pose)} rest_pose={list(req.rest_pose)}")
        self.get_logger().info(f"pick_pose={list(req.pick_pose)} pre_pick_pose={list(req.pre_pick_pose)} post_pick_pose={list(req.post_pick_pose)}")
        self.get_logger().info(f"end_pose={list(req.end_pose)} pre_end_pose={list(req.pre_end_pose)}")
        self.get_logger().info(f"tip_offset={list(req.tip_offset)} angles=({req.tray_angle1}, {req.tray_angle2})")

        self.get_logger().info("tray_heights=" + _fmt_arr(req.tray_heights))
        self.get_logger().info("origin_to_bottom=" + _fmt_arr(req.origin_to_bottom))
        self.get_logger().info("tray_init_offset=" + _fmt_arr(req.tray_init_offset))

        self.get_logger().info("tray_down_steps=" + _fmt_arr(req.tray_down_steps))
        self.get_logger().info("operator_steps=" + _fmt_arr(req.operator_steps))
        self.get_logger().info("rotation_steps=" + _fmt_arr(req.rotation_steps))
        self.get_logger().info("new_tray_steps=" + _fmt_arr(req.new_tray_steps))

        self.get_logger().info("tray_unit_names=" + _fmt_arr(req.tray_unit_names))
        self.get_logger().info("tray_unit_pose_indices=" + _fmt_arr(req.tray_unit_pose_indices))

        self.get_logger().info("screw_quantities=" + _fmt_arr(req.screw_quantities))
        self.get_logger().info("screw_types=" + _fmt_arr(req.screw_types))
        self.get_logger().info(f"new_dummy='{req.new_dummy}'")

        res.success = True
        return res

    def cb_add_tray(self, req: AddTray.Request, res: AddTray.Response):
        self.get_logger().info(f"📥 /add_tray index={req.index}")
        res.success = True
        return res

    def cb_add_tray2(self, req: AddTray2.Request, res: AddTray2.Response):
        self.get_logger().info(f"📥 /add_tray2 tray_name='{req.tray_name}' object_name='{req.object_name}'")
        res.success = True
        return res

    def cb_bring_tray(self, req: BringTray.Request, res: BringTray.Response):
        self.get_logger().info(f"📥 /bring_tray tray_name='{req.tray_name}' location='{req.location}'")
        res.success = True
        return res

    def cb_index_only(self, tag: str):
        # factory to handle all single-index srvs that only return bool success
        def _cb(req, res):
            idx = getattr(req, "index", None)
            self.get_logger().info(f"📥 /{tag} index={idx}")
            res.success = True if hasattr(res, "success") else True
            return res
        return _cb

    def cb_internal_screw_by_num(self, req: InternalScrewByNum.Request, res: InternalScrewByNum.Response):
        self.get_logger().info(f"📥 /internal_screw_by_num index={req.index} screw_num={req.screw_num}")
        res.success = True
        # If your srv has a 'message' field (it appears so), fill it:
        if hasattr(res, "message"):
            res.message = f"OK screw_num={req.screw_num} at index={req.index}"
        return res


def main():
    rclpy.init()
    node = MockServiceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
