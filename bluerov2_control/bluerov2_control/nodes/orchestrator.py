import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import PoseArray

NEUTRAL = 0.01          # truthy, but int(0.01 * 10) == 0 -> real zero at MAVLink
OUTPUT_LIMIT = 100.0    # hardware ceiling on any ManualControl axis


class Orchestrator(Node):
    """Sole owner of /manual_control. Arbitrates which controller drives which axis."""

    # --- states ---
    INIT = 0
    TURN_180 = 1
    SEARCH = 2
    TRACK = 3
    LUNGE = 4
    FIRE = 5

    def __init__(self):
        super().__init__("orchestrator")

        # --- tunable parameters ---
        self.declare_parameter("hold_depth", 1.0)        # m, announced before the round
        self.declare_parameter("lunge_distance", 2.5)    # m, start driving forward
        self.declare_parameter("fire_distance", 1.0)     # m, win condition range
        self.declare_parameter("coast_timeout", 1.5)     # s of blind driving before giving up
        self.declare_parameter("turn_timeout", 8.0)      # s cap on the opening 180
        self.declare_parameter("turn_tolerance", 15.0)   # deg; must exceed the ~10 deg sensor floor
        self.declare_parameter("search_yaw", 15.0)       # sweep rate while hunting
        self.declare_parameter("lunge_thrust", 30.0)     # forward thrust during the attack
        self.declare_parameter("fresh_window", 0.5)      # s; newer than this == "I see it"

        self.state = self.INIT
        self.state_entered = self.get_clock().now()

        # --- latest inputs from the control nodes ---
        self.crosshair_yaw = None       # Float64, from crosshair (vision lock)
        self.heading_yaw = None         # Float64, from heading PID
        self.vertical_thrust = 0.0      # Float64, from depth PID
        self.current_heading = None     # Int16, raw compass
        self.opponent_distance = None   # float, nearest AprilTag range

        self.last_crosshair_time = None
        self.target_heading = None
        self.coast_heading = None

        # --- outputs ---
        self.control_pub = self.create_publisher(ManualControl, "/manual_control", 10)
        self.target_depth_pub = self.create_publisher(Float64, "/target_depth", 10)
        self.target_heading_pub = self.create_publisher(Int16, "/target_heading", 10)

        # --- inputs ---
        self.create_subscription(Float64, "/crosshair_yaw", self.get_crosshair_yaw, 10)
        self.create_subscription(Float64, "/rotation_thrust", self.get_heading_yaw, 10)
        self.create_subscription(Float64, "/vertical_thrust", self.get_vertical_thrust, 10)
        self.create_subscription(Int16, "/heading", self.get_current_heading, 10)
        self.create_subscription(PoseArray, "/tag_poses", self.get_tag_poses, 10)

        # 20 Hz: fast enough to keep the ArduSub failsafe fed
        self.timer = self.create_timer(0.05, self.update_thrusters)

        self.get_logger().info("orchestrator up - state INIT")

    # ---------- input callbacks (store only, never act) ----------
    def get_crosshair_yaw(self, msg):
        self.crosshair_yaw = msg.data
        self.last_crosshair_time = self.get_clock().now()

    def get_heading_yaw(self, msg):
        self.heading_yaw = msg.data

    def get_vertical_thrust(self, msg):
        self.vertical_thrust = msg.data

    def get_current_heading(self, msg):
        self.current_heading = msg.data

    def get_tag_poses(self, msg):
        if not msg.poses:
            self.opponent_distance = None
            return
        self.opponent_distance = min(
            (p.position.x ** 2 + p.position.y ** 2 + p.position.z ** 2) ** 0.5
            for p in msg.poses
        )

    # ---------- helpers ----------
    def enter(self, new_state, name):
        self.state = new_state
        self.state_entered = self.get_clock().now()
        self.coast_heading = None
        self.get_logger().info(f"state -> {name}")

    def time_in_state(self):
        return (self.get_clock().now() - self.state_entered).nanoseconds / 1e9

    def target_age(self):
        """Seconds since crosshair last reported a lock. Large == we can't see it."""
        if self.last_crosshair_time is None:
            return 999.0
        return (self.get_clock().now() - self.last_crosshair_time).nanoseconds / 1e9

    def heading_error(self):
        if self.current_heading is None or self.target_heading is None:
            return None
        return ((self.target_heading - self.current_heading + 180) % 360) - 180

    def set_target_heading(self, heading):
        self.target_heading = int(heading) % 360
        msg = Int16()
        msg.data = self.target_heading
        self.target_heading_pub.publish(msg)

    def set_target_depth(self, depth):
        msg = Float64()
        msg.data = float(depth)
        self.target_depth_pub.publish(msg)

    def safe(self, value):
        """Clamp to hardware limit, and turn a true zero into NEUTRAL."""
        value = max(-OUTPUT_LIMIT, min(OUTPUT_LIMIT, float(value)))
        return value if value != 0.0 else NEUTRAL

    # ---------- the one place that writes /manual_control ----------
    def update_thrusters(self):
        # Republish the depth setpoint every tick. A single publish at startup
        # can land before the depth node has subscribed and be lost forever,
        # silently leaving depth-hold on its own default -> depth-band DQ risk.
        self.set_target_depth(self.get_parameter("hold_depth").value)

        x = 0.0
        y = 0.0
        z = self.vertical_thrust   # depth-hold underlay, always running
        r = 0.0

        age = self.target_age()
        seen = age < self.get_parameter("fresh_window").value
        coast_timeout = self.get_parameter("coast_timeout").value
        lunge_distance = self.get_parameter("lunge_distance").value
        fire_distance = self.get_parameter("fire_distance").value
        dist = self.opponent_distance

        if self.state == self.INIT:
            if self.current_heading is not None:
                self.set_target_heading(self.current_heading + 180)
                self.enter(self.TURN_180, "TURN_180")

        elif self.state == self.TURN_180:
            r = self.heading_yaw or 0.0
            err = self.heading_error()
            tol = self.get_parameter("turn_tolerance").value
            if (err is not None and abs(err) < tol) or \
               self.time_in_state() > self.get_parameter("turn_timeout").value:
                self.enter(self.SEARCH, "SEARCH")

        elif self.state == self.SEARCH:
            r = self.get_parameter("search_yaw").value
            if seen:
                self.enter(self.TRACK, "TRACK")

        elif self.state == self.TRACK:
            r = self.crosshair_yaw or 0.0
            if not seen:
                self.enter(self.SEARCH, "SEARCH")
            elif dist is not None and dist <= lunge_distance:
                self.enter(self.LUNGE, "LUNGE")

        elif self.state == self.LUNGE:
            x = self.get_parameter("lunge_thrust").value
            if seen:
                r = self.crosshair_yaw or 0.0
                self.coast_heading = None
            else:
                # coasting: latch the compass heading we had, drive straight on it
                if self.coast_heading is None and self.current_heading is not None:
                    self.coast_heading = self.current_heading
                    self.set_target_heading(self.coast_heading)
                    self.get_logger().warn(f"lost target - coasting {coast_timeout}s")
                r = self.heading_yaw or 0.0
                if age > coast_timeout:
                    self.enter(self.SEARCH, "SEARCH")
            if dist is not None and dist <= fire_distance:
                self.enter(self.FIRE, "FIRE")

        elif self.state == self.FIRE:
            # apriltag_fire triggers the lights on its own; this state just stops the lunge
            r = (self.crosshair_yaw or 0.0) if seen else 0.0
            if self.time_in_state() > 1.0:
                self.enter(self.TRACK, "TRACK")

        msg = ManualControl()
        msg.x = self.safe(x)
        msg.y = self.safe(y)
        msg.z = self.safe(z)
        msg.r = self.safe(r)
        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()

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