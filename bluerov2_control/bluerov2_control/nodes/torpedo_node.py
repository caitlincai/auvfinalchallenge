import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray, Int16
import numpy as np
import math


class Torpedo(Node):
    def __init__(self):
        super().__init__("torpedo_node")

        self.LIGHT_CHANNEL = 9  
        self.PWM_OFF = 1100
        self.PWM_ON = 1900
        self.NO_CHANGE = 65535  # OverrideRCIn sentinel: leave this channel untouched

        # PUBSUB
        self.rc_pub = self.create_publisher(OverrideRCIn, "/override_rc", 10)
        self.translation_sub = self.create_subscription(
            PoseArray, "/tag_poses", self.update_distance, 10
        )
        self.current_heading_sub = self.create_subscription(
            Int16, "/heading", self.get_heading, 10
        )
        # self.rotation_sub = self.create_subscription(Float32MultiArray, '/rotation', self.update_rotation, 10)

        # TORPEDO STUFF
        self.count = 20 # TODO
        self.fire_distance = 1.0
        self.heading_tolerance = 5.0
        self.current_heading = None

        # STATUS READINGS
        self.in_range = False
        self.aimed = False
        self.ready_to_shoot = False
        

        # TIMING
        self.last_tag_time = None
        self.tag_timeout = 5.0

        self.shoot_time = 0.0
        self.light_duration = 3.0
        self.cooldown = 2.0 # TODO

        self.light_on = False
        self.timer = self.create_timer(0.05, self.shoot)

    def get_heading(self, msg):
        self.current_heading = msg.data

    def update_distance(self, msg):
        distance = 999999
        closest_tag = None

        for tag in msg.poses:
            tag_distance = tag.position.z
            if tag_distance < distance:  # closest april tag
                distance = tag_distance
                closest_tag = tag

        self.in_range = distance <= self.fire_distance
        self.get_logger().info(f"DISTANCE: {distance}")

        # if closest_tag is not None:
        #     bearing_deg = math.degrees(math.atan2(closest_tag.position.x, closest_tag.position.z))
        #     self.aimed = (abs(bearing_deg - self.current_heading) <= self.heading_tolerance)
        # else:
        #     self.aimed = False

    def publish_light(self, pwm):
        msg = OverrideRCIn()
        msg.channels = [self.NO_CHANGE] * 18
        msg.channels[self.LIGHT_CHANNEL - 1] = pwm
        self.last_tag_time = self.get_clock().now().nanoseconds / 1e9
        self.rc_pub.publish(msg)

    def shoot(self):
        self.ready_to_shoot = self.in_range
        # and self.aimed
        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_tag_time is None or (now - self.last_tag_time) > self.tag_timeout:
            self.in_range = False

        # turns off lights
        if self.light_on:
            if (now - self.shoot_time) >= self.light_duration:
                self.publish_light(self.PWM_OFF)
                self.get_logger().info("** LIGHTS OFF **")

                self.light_on = False
                self.shoot_time = now  # start cooldown timer
            return

        if self.count <= 0: # not enough torpedos
            self.timer.cancel()
            return

        if (now - self.shoot_time) < self.cooldown:  # cooldown running
            return

        if not self.ready_to_shoot: # not in range
            return

        self.publish_light(self.PWM_ON)
        self.get_logger().info(f"** FIRE ({self.count} remaining) **")

        self.count -= 1
        self.light_on = True
        self.shoot_time = now


def main(args=None):
    rclpy.init(args=args)
    node = Torpedo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()