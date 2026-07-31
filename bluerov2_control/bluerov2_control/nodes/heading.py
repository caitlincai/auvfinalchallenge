import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64, Int16


class Heading(Node):
    def __init__(self):
        super().__init__("control_heading")

        self.current_heading = 0.0

        # Declare default parameters
        self.declare_parameter("kp", 0.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("target_heading", 90)
        self.declare_parameter("integral_cap", 30.0)
        self.declare_parameter("rate_hz", 5.0)
        self.rate_hz = self.get_parameter("rate_hz").value
        self.min_period = 1.0 / self.rate_hz
        self.last_run_time = self.get_clock().now()

        # Read parameters
        self.Kp = self.get_parameter("kp").value
        self.Ki = self.get_parameter("ki").value
        self.Kd = self.get_parameter("kd").value
        self.target_heading = self.get_parameter("target_heading").value
        self.integral_cap = self.get_parameter("integral_cap").value

        self.prev_error = 0.0
        self.error_accumulator = 0.0

        self.integral_cap = 30.0

        # Subscription to current heading
        self.current_heading_sub = self.create_subscription(
            Int16,  # message type
            "/heading",  # topic name
            self.update_current_heading,  # callback
            10,
        )

        # Subscription to target heading
        self.target_heading_sub = self.create_subscription(
            Int16,  # message type
            "/target_heading",  # topic name
            self.update_target_heading,
            10,
        )

        # Publisher for PID output (rotation thrust)
        self.output_pub = self.create_publisher(Float64, "/rotation_thrust", 10)

        self.get_logger().info("initialized heading/PID node")
        self.prev_time = self.get_clock().now()

    def update_current_heading(self, msg: Int16):
        self.current_heading = msg.data

        now = self.get_clock().now()
        elapsed = (now - self.last_run_time).nanoseconds * 1e-9
        if elapsed < self.min_period:
            return  # too soon — skip this update, wait for the next one

        self.last_run_time = now
        self.calculate_r_thrust()

    def update_target_heading(self, msg):
        self.target_heading = msg.data

    def calculate_r_thrust(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if dt <= 0:
            return

        msg = Float64()

        # if (self.target_heading - self.current_heading) >= 180:
        #     error = (self.target_heading - 360) - self.current_heading
        # elif abs(self.target_heading - self.current_heading) >= 180:
        #     error = abs((self.target_heading - 360) - self.current_heading)
        # else:
        #     error = self.target_heading - self.current_heading

        error = (
            (self.target_heading - self.current_heading + 180) % 360
        ) - 180  # ensures error is between -180 and 180
        self.get_logger().info(f"ERROR: {error}")

        # proportional term
        proportional = self.Kp * error

        # integral term
        self.error_accumulator += error * dt
        self.error_accumulator = min(
            self.integral_cap, max(-1 * self.integral_cap, self.error_accumulator)
        )
        integral = self.Ki * self.error_accumulator

        # derivative term
        derivative = self.Kd * ((error - self.prev_error) / dt)

        # PID output
        output_value = (proportional + integral + derivative) 
        msg.data = float(
            min(30, max(-30, output_value))
        )  # makes sure output is between -100 and 100

        self.get_logger().info(f"Heading PID Output: {msg.data}")

        self.prev_error = error
        self.output_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Heading()

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
