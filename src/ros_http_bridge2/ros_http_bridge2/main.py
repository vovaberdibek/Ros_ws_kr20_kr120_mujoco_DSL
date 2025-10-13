#!/usr/bin/env python3
from lark import Lark
from .robot_manager import RobotManager
import importlib.resources as pkg_res

def main():
    # Read grammar stored inside the package
    with pkg_res.files(__package__).joinpath("combined_dsl.lark").open("r") as f:
        grammar = f.read()

    # Adjust the path of your workflow file as you like, or accept text via CLI later
    workflow_path = "/home/user/ros2_ws/src/ros_http_bridge2/ros_http_bridge2/workflows/workflow.workflow"

    with open(workflow_path, "r") as wf:
        workflow_text = wf.read()

    parser = Lark(grammar, start="start", parser="lalr")
    tree = parser.parse(workflow_text)

    manager = RobotManager(tree)
    manager.execute_workflow()

if __name__ == "__main__":
    main()
