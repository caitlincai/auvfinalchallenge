import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ROVdetector(Node):
    def __init__(self):
        super().__init__('ROVdetector')

        self.bridge = CvBridge()

        self.declare_parameter("model_path", "/home/canon/models/best_ncnn_model")
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("conf", 0.25)

        self.declare_parameter("max_jump", 150.0)
        self.declare_parameter("frame_skip", 3)
        self.locked_cx = None
        self.missed_frames = 0
        self.frame_count = 0

        model_path = self.get_parameter("model_path").value
        self.model = YOLO(model_path)
        self.get_logger().info(f"loaded model: {model_path}")

        self.image_sub = self.create_subscription(
            Image,
            "/camera",
            self.detect,
            1
        )

        self.target_error_pub = self.create_publisher(Int16, "/target_error", 10)
        self.box_count_pub = self.create_publisher(Int16, "/box_count", 10)

        self.logged_width = False

        self.get_logger().info(f"RUNNING FILE: {__file__}")

    def detect(self, msg):
        self.frame_count += 1
        frame_skip = self.get_parameter("frame_skip").value
        if self.frame_count % frame_skip != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.resize(frame, (640, 480))

        crosshair_x = frame.shape[1] / 2.00

        if not self.logged_width:
            self.get_logger().info(f"frame width: {frame.shape[1]}, crosshair_x: {crosshair_x}")
            self.logged_width = True

        imgsz = self.get_parameter("imgsz").value
        conf = self.get_parameter("conf").value

        results = self.model.predict(source=frame, imgsz=imgsz, conf=conf, verbose=True)
        self.boxes = results[0].boxes
        box_count_msg = Int16()
        box_count_msg.data = len(self.boxes)
        self.box_count_pub.publish(box_count_msg)

        # No detection this frame -> publish nothing.
        # The crosshair's stale_timeout flips target_valid False and
        # falls back to NEUTRAL. Silence IS the signal.
        # No detection this frame -> publish nothing.
        # The crosshair's stale_timeout flips target_valid False and
        # falls back to NEUTRAL. Silence IS the signal.
        if len(self.boxes) == 0:
            
            self.get_logger().info("no detection")
            return

        if len(self.boxes) > 1:
            self.get_logger().info(
                "boxes: " + ", ".join(
                    f"[cls={int(b.cls[0])} cx={float((b.xyxy[0][0]+b.xyxy[0][2])/2):.0f} "
                    f"w={float(b.xyxy[0][2]-b.xyxy[0][0]):.0f} conf={float(b.conf[0]):.2f}]"
                    for b in self.boxes
                )
            )

        # Multiple boxes -> take the WIDEST. Validated across 51 frames of
        # real footage: rejects sub-region classes (front/Side) that box part
        # of the same hull, and biases toward the nearest vehicle when more
        # than one ROV is in frame.
        max_jump = self.get_parameter("max_jump").value

        if self.locked_cx is None:
            # No lock yet -> acquire with widest box.
            best = max(self.boxes, key=lambda b: float(b.xyxy[0][2] - b.xyxy[0][0]))
        else:
            # Locked -> only consider boxes near where the target was.
            near = [b for b in self.boxes
                    if abs(float((b.xyxy[0][0] + b.xyxy[0][2]) / 2) - self.locked_cx) < max_jump]
            if not near:
                self.missed_frames += 1
                self.get_logger().info(f"no box near lock ({self.missed_frames})")
                if self.missed_frames > 5:
                    self.locked_cx = None
                    self.get_logger().info("lock released")
                return
            best = max(near, key=lambda b: float(b.xyxy[0][2] - b.xyxy[0][0]))

        self.missed_frames = 0
        x1, y1, x2, y2 = best.xyxy[0].cpu().numpy()
        cx = (x1 + x2) / 2.0
        self.locked_cx = cx

        # Positive error = target right of center = positive yaw = turn right.
        error = cx - crosshair_x

        out = Int16()
        out.data = int(error)
        self.target_error_pub.publish(out)
        self.get_logger().info(f"cx: {cx:.0f}  error: {out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ROVdetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()