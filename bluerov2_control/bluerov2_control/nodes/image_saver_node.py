import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.frame_id = 0
        self.output_dir = 'calib_images'
        os.makedirs(self.output_dir, exist_ok=True)
        self.save_image_flag = False
        self.timer = self.create_timer(0.05, self.enable_image_saving)
        self.get_logger().info('ImageSaver node started. Saving an image every 3 seconds.')

    def enable_image_saving(self):
        self.save_image_flag = True

    def listener_callback(self, msg):
        if not self.save_image_flag:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return
        filename = os.path.join(self.output_dir, f'image_{self.frame_id:05d}.png')
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Saved: {filename}')
        self.frame_id += 1
        self.save_image_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()