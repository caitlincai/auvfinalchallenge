#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ArmingClient(Node):
    def __init__(self):
        super().__init__('arming_client')
        self.client = self.create_client(SetBool, '/arming')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /arming service...')

    def send_request(self, arm: bool):
        request = SetBool.Request()
        request.data = arm

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Response: success={response.success}, message="{response.message}"'
            )
            return response
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return None


def main(args=None):
    rclpy.init(args=args)

    # Default to True (arm), but allow overriding via command-line arg
    # e.g. `ros2 run <pkg> arming_client false` to disarm
    arm_value = True
    if len(sys.argv) > 1:
        arm_value = sys.argv[1].lower() in ('true', '1', 'yes')

    node = ArmingClient()
    node.send_request(arm_value)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()