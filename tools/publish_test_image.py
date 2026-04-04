#!/usr/bin/env python3
"""
테스트용 이미지 퍼블리셔.
로컬 이미지 파일을 읽어서 /camera/image_raw 토픽으로 반복 발행.

사용법:
    python tools/publish_test_image.py --image ~/Pictures/Screenshots/cafe_ocean.png
    python tools/publish_test_image.py --image ~/Pictures/Screenshots/cafe_ocean.png --hz 0.2
"""
import argparse
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class TestImagePublisher(Node):
    def __init__(self, image_path: str, topic: str, hz: float):
        super().__init__("test_image_publisher")

        image_bgr = cv2.imread(image_path)
        if image_bgr is None:
            raise FileNotFoundError(f"Cannot read image: {image_path}")

        self.bridge = CvBridge()
        self.msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")

        self.pub = self.create_publisher(Image, topic, 10)
        self.timer = self.create_timer(1.0 / hz, self._publish)

        self.get_logger().info(f"Publishing '{image_path}' → '{topic}' at {hz} Hz")

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="Path to image file")
    parser.add_argument("--topic", default="/camera/image_raw", help="Topic name")
    parser.add_argument("--hz", type=float, default=0.5, help="Publish rate (default: 0.5 Hz = 2초마다)")
    args = parser.parse_args()

    rclpy.init()
    node = TestImagePublisher(args.image, args.topic, args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
