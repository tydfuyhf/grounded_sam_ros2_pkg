"""
qwen_stub_node.py

Stub bridge between grounded_sam_node and projector_node.
Replaces the real Qwen VLM while it is not yet integrated.

Subscribes:
  /grounded_sam/detections_json  std_msgs/String  — cache
  /grounded_sam/mask_image       sensor_msgs/Image — trigger

On each mask_image:
  1. Adds "category" field to each detection using label text lookup.
  2. Publishes /qwen/labeled_detections  (detections BEFORE mask so projector
     has the latest detections when mask triggers projection).
  3. Publishes /qwen/mask_image  (pass-through, triggers projector).

LABEL_TO_CATEGORY is hardcoded for the Gazebo demo scene (one cup, one table).
Replace with real Qwen inference to generalise.

# WARNING: labeled_detections and mask_image are two separate messages with a
# slight timing gap. projector_node caches detections and triggers on mask —
# safe for CPU inference (30-40 s per frame). If moving to GPU (sub-second
# inference), add ApproximateTimeSynchronizer in projector_node instead.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

LABEL_TO_CATEGORY: dict[str, str] = {
    "cup":   "TARGET",
    "table": "WORKSPACE",
}


class QwenStubNode(Node):

    def __init__(self) -> None:
        super().__init__("qwen_stub_node")

        self._latest_detections: list[dict] | None = None

        # QoS depth=10 matches grounded_sam_node's VOLATILE publishers
        self.create_subscription(
            String, "/grounded_sam/detections_json", self._json_cb, 10)
        self.create_subscription(
            Image,  "/grounded_sam/mask_image",       self._mask_cb, 10)

        self._pub_detections = self.create_publisher(
            String, "/qwen/labeled_detections", 10)
        self._pub_mask = self.create_publisher(
            Image,  "/qwen/mask_image", 10)

        self.get_logger().info("QwenStubNode ready")

    def _json_cb(self, msg: String) -> None:
        try:
            self._latest_detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"detections_json parse error: {e}")

    def _mask_cb(self, mask_msg: Image) -> None:
        if self._latest_detections is None:
            self.get_logger().warn("Waiting for detections_json...")
            return

        labeled = []
        for det in self._latest_detections:
            category = LABEL_TO_CATEGORY.get(
                det.get("label", "").lower().strip(), "OBSTACLE")
            labeled.append({**det, "category": category})

        # publish detections BEFORE mask so projector has latest when triggered
        self._pub_detections.publish(String(data=json.dumps(labeled)))
        self._pub_mask.publish(mask_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QwenStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
