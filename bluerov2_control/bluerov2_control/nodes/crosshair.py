import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float64


class crosshair_node(Node):
    def __init__(self):
        super().__init__('crosshair_node')

        self.declare_parameter("kp", 0.1)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("i_limit", 20.0)
        self.declare_parameter("max_yaw", 30.0)
        self.declare_parameter("stale_timeout", 0.3)   # go quiet fast; orchestrator owns the coast

        self.pixel_error = 0.0

        self.crosshair_xcenter = 320   # 640 px / 2 = center of feed
        self.target_xcenter = self.crosshair_xcenter   # harmless default: zero error
        self.target_valid = False

        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0

        self.last_pid_time = self.get_clock().now()
        self.last_detection_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        # Publishes a plain yaw command. The orchestrator is the sole owner of
        # /manual_control and translates this into msg.r (incl. the NEUTRAL trick).
        self.pub = self.create_publisher(Float64, "/crosshair_yaw", 10)

        self.timer = self.create_timer(0.05, self.publish_yaw)   # 20 Hz

        self.detector_sub = self.create_subscription(
            Int16,
            "/target_error",
            self.callback_target_center,
            10
        )

    def callback_target_center(self, msg):
        self.target_xcenter = msg.data
        self.compute_error()
        self.last_detection_time = self.get_clock().now()

        if not self.target_valid:
            self.integral = 0.0
            self.prev_error = self.pixel_error
            self.last_pid_time = self.get_clock().now()

        self.target_valid = True
        self.pid_control_loop()

    def compute_error(self):
        self.pixel_error = self.target_xcenter
        self.get_logger().info(f"pix error: {self.pixel_error}")

    def pid_control_loop(self):
        Kp = self.get_parameter("kp").value
        Ki = self.get_parameter("ki").value
        Kd = self.get_parameter("kd").value
        i_limit = self.get_parameter("i_limit").value
        max_yaw = self.get_parameter("max_yaw").value

        now = self.get_clock().now()
        dt = (now - self.last_pid_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_pid_time = now

        # P
        p_term = Kp * self.pixel_error

        # I — clamp the CONTRIBUTION, not the raw accumulator, so the limit
        # means the same thing regardless of how Ki is tuned.
        self.integral += self.pixel_error * dt
        i_term = Ki * self.integral
        if i_term > i_limit:
            i_term = i_limit
            self.integral = i_limit / Ki if Ki else 0.0
        elif i_term < -i_limit:
            i_term = -i_limit
            self.integral = -i_limit / Ki if Ki else 0.0

        # D — precomputed at sensor rate
        derivative = (self.pixel_error - self.prev_error) / dt
        d_term = Kd * derivative

        self.output = p_term + i_term + d_term
        self.output = max(-max_yaw, min(max_yaw, self.output))
        self.get_logger().info(f"Output: {self.output}")
        self.prev_error = self.pixel_error

    def publish_yaw(self):
        stale_timeout = self.get_parameter("stale_timeout").value
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > stale_timeout:
            self.target_valid = False

        # Silence is the signal: publishing nothing tells the orchestrator
        # "I can't see the target" without ambiguity. A published 0.0 would be
        # indistinguishable from a perfect center lock.
        if not self.target_valid:
            return

        msg = Float64()
        msg.data = float(self.output)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = crosshair_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()