"""
ROS2 node for GroundedSAMPipeline.

Subscribes:
  <image_topic>  (sensor_msgs/Image, default: /camera/image_raw)
    — raw camera image

Publishes:
  /grounded_sam/annotated_image  (sensor_msgs/Image)
    — BGR image with bbox + mask overlay (for visualization / rviz)

  /grounded_sam/mask_image  (sensor_msgs/Image, 8UC1)
    — label map: pixel value = detection index (1-based), 0 = background
    — consumed by mask_projection_pkg for 2D→3D projection

  /grounded_sam/detections_json  (std_msgs/String)
    — JSON array: [{label, confidence, bbox_xyxy}, ...]
    — used together with mask_image to know which index = which object

Parameters:
  model_config  (string) : absolute path to model_paths.yaml  [required]
  prompt        (string) : object noun phrase, e.g. "bottle, cup"  [default: "object"]
  image_topic   (string) : topic to subscribe to  [default: /camera/image_raw]
"""

import json
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

LATCHED_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

from .pipeline import GroundedSAMPipeline
from .postprocess import format_detections, format_masks
from .prompt_adapter import PromptAdapter
from .visualizer import draw_bboxes, draw_masks, save_result

OUTPUT_DIR = Path.home() / "gsam_ws" / "output"


class GroundedSAMNode(Node):
    def __init__(self):
        super().__init__("grounded_sam_node")

        self.declare_parameter("model_config", "")
        self.declare_parameter("prompt", "object")
        self.declare_parameter("image_topic", "/camera/image_raw")

        model_config = self.get_parameter("model_config").value
        self.prompt_raw = self.get_parameter("prompt").value
        image_topic = self.get_parameter("image_topic").value

        if not model_config:
            raise ValueError("Parameter 'model_config' must be set to the path of model_paths.yaml")

        self.bridge = CvBridge()
        self.adapter = PromptAdapter()

        self.get_logger().info("Loading models...")
        self.pipeline = GroundedSAMPipeline(model_config)

        # subscriber
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile=LATCHED_QOS,
        )

        # publishers
        self.pub_annotated = self.create_publisher(Image, "/grounded_sam/annotated_image", 10)
        self.pub_mask = self.create_publisher(Image, "/grounded_sam/mask_image", 10)
        self.pub_json = self.create_publisher(String, "/grounded_sam/detections_json", 10)

        self.get_logger().info(
            f"Ready — subscribed to '{image_topic}', prompt='{self.prompt_raw}'"
        )

    def _image_callback(self, msg: Image) -> None:
        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        prompt = self.adapter.adapt(self.prompt_raw)

        result = self.pipeline.run(image=image_bgr, prompt=prompt)

        det_list = format_detections(result["detections"], result["phrases"])
        self.get_logger().info(f"Detected {len(det_list)} object(s)")

        # --- annotated image ---
        vis = draw_bboxes(image_bgr, det_list)
        if result["masks"] is not None:
            mask_list = format_masks(result["masks"], result["mask_scores"])
            vis = draw_masks(vis, mask_list)
            label_map = self._build_label_map(image_bgr.shape[:2], mask_list)
        else:
            mask_list = []
            label_map = np.zeros(image_bgr.shape[:2], dtype=np.uint8)

        # publish annotated image (BGR → bgr8)
        self.pub_annotated.publish(
            self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        )

        # publish mask label map (8UC1, pixel = detection index 1-based)
        self.pub_mask.publish(
            self.bridge.cv2_to_imgmsg(label_map, encoding="mono8")
        )

        # publish detections JSON
        msg_json = String()
        msg_json.data = json.dumps(det_list)
        self.pub_json.publish(msg_json)

        # 파일 저장: result_gcbp.jpg 형식 (각 명사 첫 글자)
        initials = "".join(p.strip()[0] for p in self.prompt_raw.split(",") if p.strip())
        filename = f"result_{initials}.jpg"
        save_result(vis, OUTPUT_DIR / filename)
        self.get_logger().info(f"Saved → {OUTPUT_DIR / filename}")

    @staticmethod
    def _build_label_map(shape_hw, mask_list) -> np.ndarray:
        """
        Build a uint8 label map where pixel value = 1-based detection index.
        Later detections overwrite earlier ones where masks overlap.
        """
        label_map = np.zeros(shape_hw, dtype=np.uint8)
        for i, m in enumerate(mask_list, start=1):
            label_map[m["mask"]] = i
        return label_map


def main(args=None):
    rclpy.init(args=args)
    node = GroundedSAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
