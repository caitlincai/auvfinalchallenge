import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float64, Int16, Int64, Float64MultiArray
from geometry_msgs.msg import Pose, PoseArray
import math


class Controller(Node):
    def __init__(self):
        super().__init__("controller_node")

        # STATES
        self.START = 0 # oscillate & wait for detection
        self.ATTACK = 1 # move forward while keeping opponent in crosshair; if opponent is lost, search by spinning
        self.DEFENSE = 2 # stay still & keep opponent in crosshair; if opponent is lost, search by oscillating
        self.LUNGE = 3 # full speed towards close robot
        self.SOS = 6 # out of shots, spin forever at full speed
        self.current_status = self.START

        # SCANNING
        self.LEFT = 4
        self.RIGHT = 5
        self.heading_status = self.LEFT
        self.heading_reached_time = None
        self.heading_hold_time = 0.3

        # THRUSTS
        self.depth_node_thrust = 0.0
        self.heading_node_thrust = 0.0

        # HEADINGS
        self.current_heading = None
        self.left = None
        self.right = None

        # SPINNING
        self.spin_start_time = None
        self.spin_interval = 8.0
        self.spin_velocity = 23.0

        # LUNGING
        self.lunge_distance = 2.0

        # SOS
        self.sos_velocity = 100.0
        self.sos_interval = 8.0
        self.sos_start_time = None

        # OPP ROV
        self.opponent_distance = float("inf")
        self.box_count = 0.0
        self.crosshair_yaw = 0.0
        self.last_tag_time = None
        self.tag_timeout = 1.5

        # SHOTS
        self.shots_fired = 0

        # PUBS
        self.time_pub = self.create_publisher(Int64, "/time", 10)
        self.control_pub = self.create_publisher(ManualControl, "/manual_control", 10)
        self.target_depth_pub = self.create_publisher(Float64, "/target_depth", 10)
        self.target_heading_pub = self.create_publisher(Int16, "/target_heading", 10)

        # SUBS
        self.v_thrust_sub = self.create_subscription(Float64, "/vertical_thrust", self.get_vertical_thrust, 10)
        self.crosshair_sub = self.create_subscription(Float64, "/crosshair_yaw", self.get_crosshair_yaw, 10)
        self.distance_sub = self.create_subscription(Float64MultiArray, "/tag_distances", self.update_distance, 10)
        self.box_count_sub = self.create_subscription(Int16, "/box_count", self.get_box_count, 10)
        self.heading_node_sub = self.create_subscription(Float64, "/rotation_thrust", self.get_rotation_thrust, 10)
        self.current_heading_sub = self.create_subscription(Int16, "/heading", self.update_current_heading, 10)
        self.shots_fired_sub = self.create_subscription(Int16, "/shots_fired", self.update_shots_fired, 10)

        # TIME
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update_thrusters)
        self.elapsed_timer = self.create_timer(0.05, self.send_elapsed_time)

    # CROSSHAIR
    def get_box_count(self, msg):
        self.box_count = msg.data

    def get_crosshair_yaw(self, msg):
        self.crosshair_yaw = msg.data

    # TIME
    def get_elapsed_seconds(self):
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def send_elapsed_time(self):
        elapsed = self.get_elapsed_seconds()
        msg = Int64()
        msg.data = int(elapsed)
        self.time_pub.publish(msg)

    # HEADING
    def update_current_heading(self, msg):
        if self.current_heading is None:
            self.turn_target = (msg.data + 180) % 360
        self.current_heading = msg.data

#Oscillation
    # def update_current_heading(self, msg):
    #     self.current_heading = msg.data
    #     if self.left is None:
    #         self.left = (self.current_heading + 180 - 50) % 360
    #         self.right = (self.current_heading + 180 + 50) % 360

    def set_target_heading(self, target_heading):
        msg = Int16()
        msg.data = int(target_heading)
        self.target_heading_pub.publish(msg)

    # DEPTH
    def set_target_depth(self, target_depth: float):
        msg = Float64()
        msg.data = float(target_depth)
        self.target_depth_pub.publish(msg)

    # THRUSTS
    def get_vertical_thrust(self, msg: Float64):
        self.depth_node_thrust = msg.data

    def get_rotation_thrust(self, msg: Float64):
        self.heading_node_thrust = msg.data

    # APRIL TAGS
    def update_distance(self, msg):
        self.last_tag_time = self.get_clock().now()
        closest = float("inf")
        for dist in msg.data:
            if dist < closest:
                closest = dist
        self.opponent_distance = closest

    def update_shots_fired(self, msg: Int16):
        self.shots_fired = msg.data

    # CONTROL LOOP!!!
    def update_thrusters(self):
        current_elapsed = self.get_elapsed_seconds()
        msg = ManualControl()
        msg.z = float(self.depth_node_thrust)
        oppDetected = (self.box_count != 0)

        if self.current_heading is None:
            return

        if self.last_tag_time is None or \
           (self.get_clock().now() - self.last_tag_time).nanoseconds / 1e9 > self.tag_timeout:
            self.opponent_distance = float("inf")

        if self.shots_fired >= 5 and self.current_status != self.SOS:
            self.current_status = self.SOS
            self.get_logger().info(f"STATUS SET: SOS")
        elif (self.opponent_distance < self.lunge_distance) and self.current_status != self.LUNGE:
            self.current_status = self.LUNGE
            self.get_logger().info(f"STATUS SET: LUNGE")

        if self.current_status == self.START:
            if oppDetected:
                self.get_logger().info(f"DETECTED ROBOT!!!!!!")
                self.get_logger().info(f"STATUS SET: DEFENSE")
                self.current_status = self.DEFENSE
                return

            if self.current_heading is None:
                # No compass reading yet - nothing safe to command, and
                # update_current_heading hasn't latched turn_target yet either.
                return

            # Opening 180: drive toward turn_target (set once, on the first
            # heading reading, in update_current_heading) using the heading
            # PID's output. Falls through to the publish at the bottom of
            # this function so /manual_control keeps streaming at 20 Hz
            # even while nothing is detected yet - avoids the ArduSub
            # failsafe from going silent while facing the wall.
            self.set_target_heading(self.turn_target)
            msg.r = self.heading_node_thrust
            msg.x = 0.0001
            msg.y = 0.0001

        elif self.current_status == self.DEFENSE:
            if not oppDetected:
                self.current_status = self.START
                self.get_logger().info(f"STATUS SET: START")
                return

            if current_elapsed >= 200:
                self.current_status = self.ATTACK
                self.get_logger().info(f"STATUS SET: ATTACK")
                return

            msg.r = float(self.crosshair_yaw)
            msg.x = 0.0001
            msg.y = 0.0001

        elif self.current_status == self.ATTACK:
            if oppDetected:
                self.spin_start_time = None
                msg.r = float(self.crosshair_yaw)
                msg.x = 30.0
                msg.y = 0.0001
            else:
                now = self.get_clock().now().nanoseconds / 1e9
                if self.spin_start_time is None:
                    self.spin_start_time = now
                elif (now - self.spin_start_time) >= self.spin_interval:
                    self.spin_velocity *= -1
                    self.spin_start_time = now

                msg.r = self.spin_velocity
                msg.x = 0.0001
                msg.y = 0.0001

        elif self.current_status == self.LUNGE:
            if self.opponent_distance >= self.lunge_distance:
                if current_elapsed >= 200:
                    self.current_status = self.ATTACK
                    self.get_logger().info(f"STATUS SET: ATTACK")
                    return
                else:
                    self.current_status = self.DEFENSE
                    self.get_logger().info(f"STATUS SET: DEFENSE")
                    return

            if oppDetected:
                msg.r = float(self.crosshair_yaw)
            else:
                msg.r = 0.0001

            msg.x = 100.0
            msg.y = 0.0001

        elif self.current_status == self.SOS:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.sos_start_time is None:
                self.sos_start_time = now
            elif (now - self.sos_start_time) >= self.sos_interval:
                self.sos_velocity *= -1
                self.sos_start_time = now

            msg.r = self.sos_velocity
            msg.x = 0.0001
            msg.y = 0.0001

        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()

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