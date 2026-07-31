import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64


class Depth(Node):
    def __init__(self):
        super().__init__("control_depth")

        self.current_depth = 0.0

        # Declare default parameters
        self.declare_parameter("kp", 0.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("target_depth", 0.5)
        self.declare_parameter("integral_cap", 30.0)

        # Read parameters
        self.Kp = self.get_parameter("kp").value
        self.Ki = self.get_parameter("ki").value
        self.Kd = self.get_parameter("kd").value
        self.target_depth = self.get_parameter("target_depth").value
        self.integral_cap = self.get_parameter("integral_cap").value

        self.error_accumulator = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # Subscription to pressure sensor
        self.current_depth_sub = self.create_subscription(
            FluidPressure,          # message type
            "/pressure",            # topic name
            self.update_current_depth,  # callback
            10
        )

        # Subscription to target depth
        self.target_depth_sub = self.create_subscription(
            Float64,                # message type
            "/target_depth",        # topic name
            self.update_target_depth,
            10
        )

        # Publishes PID output (vertical thrust)
        self.output_pub = self.create_publisher(
            Float64,
            "/vertical_thrust",
            10
        )
        
        self.log_current_depth_pub = self.create_publisher(
            Float64,
            "/depth",
            10
        )

        self.get_logger().info("initialized depth/PID node")

    def update_current_depth(self, msg: FluidPressure):
        pressure = (msg.fluid_pressure) - 101325

        # depth = p / (rho * g), rho ~ 1000 kg/m^3, g ~ 9.81 m/s^2
        self.current_depth = pressure / (9.81 * 1000.0)

        # self.get_logger().info(f"Current Depth: {self.current_depth} m")
        msg = Float64()
        msg.data = self.current_depth
        self.log_current_depth_pub.publish(msg)
        
        self.calculate_PID_depth()

    def update_target_depth(self, msg):
        self.target_depth = msg.data

    def calculate_PID_depth(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if dt <= 0:
            return
        msg = Float64()

        error = self.target_depth - self.current_depth


        # proportional term
        proportional = self.Kp * error

        # integral term
        self.error_accumulator += error * dt
        self.error_accumulator = min(self.integral_cap, max(-1 * self.integral_cap, self.error_accumulator))
        integral = self.Ki * self.error_accumulator

        # derivative term
        derivative = self.Kd * ((error - self.prev_error) / dt)

        # PID output
        output_value = (proportional + integral + derivative) * -1
        msg.data = float(min(100, max(-100, output_value))) #makes sure output is between -100 and 100

        # self.get_logger().info(f"PID Output: {msg.data}")
        
        self.prev_error = error
        self.output_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Depth()

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