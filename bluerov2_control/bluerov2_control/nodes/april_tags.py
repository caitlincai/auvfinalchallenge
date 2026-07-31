import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseArray, Pose
from dt_apriltags import Detector
import cv2
from cv_bridge import CvBridge
import numpy as np


class AprilTags(Node):
    def __init__(self):
        super().__init__("control_april_tags")

        self.bridge = CvBridge()

        self.latest_img = None

        self.K = np.array(
            [[273.25, 0.000000000000000000e+00, 307.89],
            [0.000000000000000000e+00, 261.76, 153.84],
            [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]]
            )

        self.D = np.array(
            [[1.818389051822951602e-02, -1.445940107627855138e-02, -3.952855470630491520e-04,
            -1.943097662521165594e-05, 5.055176384358054699e-03]]
            )

        # Initialize detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25
        )

        # Subscription to camera feed
        self.camera_sub = self.create_subscription(
            Image,
            "/camera",
            self.camera_callback,
            1
        )

        self.pose_pub = self.create_publisher(
            PoseArray,
            "/tag_poses",
            10
        )

        # One distance (meters) per detected tag, index-aligned with /tag_poses.
        self.dist_pub = self.create_publisher(
            Float64MultiArray,
            "/tag_distances",
            10
        )

        self.timer = self.create_timer(0.2, self.detect_tag)

    def camera_callback(self, msg):
        self.latest_img = msg

    def detect_tag(self):
        if self.latest_img is None:
            return
        
        pose_array_msg = PoseArray()
        dist_msg = Float64MultiArray()

        if self.latest_img is None:
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_img, desired_encoding="bgr8")
        # Get optimal new camera matrix
        h, w = img.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1, (w, h))

        # Undistort the image
        undistorted = cv2.undistort(img, self.K, self.D, None, new_K)

        # Convert to grayscale
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        # Camera intrinsics for pose estimation
        fx = new_K[0, 0]
        fy = new_K[1, 1]
        cx = new_K[0, 2]
        cy = new_K[1, 2]

        # Run detection
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=0.05  # in meters
        )

        for tag in tags:
            if tag.decision_margin < 20:   # prevents misfires when less than 30% certain
                self.get_logger().info("deleted tag, not certain enough")
                continue
                
            tag_pose = Pose()

            t = tag.pose_t.flatten()

            tag_pose.position.x = float(t[0])
            tag_pose.position.y = float(t[1])
            tag_pose.position.z = float(t[2])

            # Straight-line distance to the tag (same norm apriltag_fire uses).
            distance = float((t[0] ** 2 + t[2] ** 2) ** 0.5)
            dist_msg.data.append(distance)

            self.get_logger().info(f"tag {tag.tag_id}: {distance:.2f} m")

            pose_array_msg.poses.append(tag_pose)

        self.get_logger().info(f'Published {len(pose_array_msg.poses)} AprilTag poses.')

        self.pose_pub.publish(pose_array_msg)
        self.dist_pub.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTags()

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