import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64MultiArray, Int16, String


# --- lights: RC channel 9 = Lights 1 (ArduSub RC input map) ---
LIGHTS_CHANNEL = 9
LIGHTS_OFF = 1100
LIGHTS_ON = 1900
NOCHANGE = 65535
PUBLISH_RATE = 20.0


class AprilTagFire(Node):
    """Flash a 200 ms 'shot' when a tag is within fire range."""

    def __init__(self):
        super().__init__("apriltag_fire")

        self.declare_parameter("fire_distance", 1.0)   # m (win condition: <= 1 m)
        self.declare_parameter("flash_duration", 0.2)  # s (one competition shot)
        self.declare_parameter("cooldown", 0.75)       # s between shots
        self.declare_parameter("max_shots", 5)         # ammo

        self.pub = self.create_publisher(OverrideRCIn, "/override_rc", 10)
        self.shots_fired_pub = self.create_publisher(Int16, "/shots_fired", 10)
        self.flash_event_pub = self.create_publisher(String, "/fire_events", 10)
        self.sub = self.create_subscription(
            Float64MultiArray, "/tag_distances", self.on_distances, 10
        )

        self.shots_fired = 0
        self.flash_until = None
        self.last_shot = None

        # Continuous 20 Hz stream so the RC override never times out.
        self.timer = self.create_timer(0.2, self.drive_lights)
        self.get_logger().info("initialized apriltag_fire node")

    def on_distances(self, msg):
        fire_distance = self.get_parameter("fire_distance").value

        # Distances are pre-computed by april_tags, one entry per detected tag.
        if not msg.data:
            return
        closest = min(msg.data)
        if closest <= fire_distance:
            self.get_logger().info(f"CLOSEST DISTANCE: {closest}")
            self.try_fire(closest)

    def try_fire(self, dist):
        now = self.get_clock().now()
        max_shots = self.get_parameter("max_shots").value
        cooldown = self.get_parameter("cooldown").value
        flash_duration = self.get_parameter("flash_duration").value

        if self.shots_fired >= max_shots:
            return  # out of ammo
        if self.flash_until is not None and now < self.flash_until:
            return  # already flashing
        if self.last_shot is not None and \
           (now - self.last_shot).nanoseconds / 1e9 < cooldown:
            return  # cooling down

        self.flash_until = now + Duration(seconds=flash_duration)
        self.last_shot = now
        self.shots_fired += 1
        
        event_msg = String()
        event_msg.data = (
            f"shot={self.shots_fired}/{max_shots} "
            f"distance_m={dist:.3f} "
            f"stamp_ns={now.nanoseconds}"
        )
        self.flash_event_pub.publish(event_msg)

        self.get_logger().info(f"FIRE shot {self.shots_fired}/{max_shots} at {dist:.2f} m")

        msg = Int16()
        msg.data = self.shots_fired
        self.shots_fired_pub.publish(msg)

    def _msg(self, pwm):
        m = OverrideRCIn()
        chans = [NOCHANGE] * 18
        chans[LIGHTS_CHANNEL - 1] = pwm
        m.channels = chans
        return m

    def drive_lights(self):
        now = self.get_clock().now()
        on = self.flash_until is not None and now < self.flash_until
        self.pub.publish(self._msg(LIGHTS_ON if on else LIGHTS_OFF))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFire()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()