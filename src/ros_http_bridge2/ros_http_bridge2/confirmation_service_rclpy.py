#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ConfirmWorkflow

class ConfirmationServer(Node):
    def __init__(self):
        super().__init__('confirmation_server')
        self.srv = self.create_service(ConfirmWorkflow, 'confirm_workflow', self.handle)

    def handle(self, request, response):
        task = request.task_description
        self.get_logger().info(f"Task awaiting confirmation: {task}")
        while True:
            user_input = input(f"Confirm task '{task}'? (y/n): ").strip().lower()
            if user_input in ('y', 'n'):
                break
            print("Invalid input. Please enter 'y' or 'n'.")
        response.success = (user_input == 'y')
        return response

def main():
    rclpy.init()
    node = ConfirmationServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
