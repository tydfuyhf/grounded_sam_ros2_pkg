"""
projector_node.py

ROS 2 node — wiring only.  No math, no message packing here.
All logic lives in the modules below:

  back_projection.depth_to_points()   depth image → 3D points
  label_mapper.apply_labels()         mask pixel → semantic category
  cloud_builder.build_pointcloud2()   CategoryPoints → PointCloud2 msg

Subscriptions
-------------
  /rgbd_camera/depth_image       sensor_msgs/Image       (32FC1, meters)
  /rgbd_camera/camera_info       sensor_msgs/CameraInfo
  /grounded_sam/mask_image       sensor_msgs/Image       (mono8, 1-based index)
  /grounded_sam/detections_json  std_msgs/String         (JSON, no header)

Publications
------------
  /labeled_points                sensor_msgs/PointCloud2 (XYZRGB + category field)
  /projection_result             std_msgs/String         (JSON centroid summary)

Parameters
----------
  min_depth  float  0.05  Discard depth readings below this value (m)
  max_depth  float  15.0  Discard depth readings above this value (m)

Design note — why no ApproximateTimeSynchronizer
-------------------------------------------------
SAM runs on CPU and takes 30–40 s per frame.  By the time mask_image is
published, the depth queue has moved far ahead and timestamp matching fails.
Instead, depth/camera_info/detections are cached as "latest", and projection
fires the moment a new mask_image arrives.  This is correct because:
  - depth changes slowly (static tabletop scene)
  - mask timestamp already matches the depth that was captured (rgb and
    depth share the same Gazebo sim-time stamp from the same sensor)
"""
from __future__ import annotations

import json
from typing import Dict, List, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String


from .back_projection import depth_to_points
from .cloud_builder import build_pointcloud2
from .label_mapper import CategoryPoints, apply_labels, CATEGORY_TARGET, CATEGORY_WORKSPACE


class MaskProjectorNode(Node):

    def __init__(self) -> None:
        super().__init__('mask_projector_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('min_depth', 0.05)
        self.declare_parameter('max_depth', 15.0)

        self._min_depth = self.get_parameter('min_depth').value
        self._max_depth = self.get_parameter('max_depth').value

        # ── cache — updated by individual subscribers ─────────────────────────
        self._bridge             = CvBridge()
        self._latest_depth:      Optional[Image]      = None
        self._latest_info:       Optional[CameraInfo] = None
        self._latest_detections: Optional[List[Dict]] = None

        # ── subscribers ───────────────────────────────────────────────────────
        # depth and camera_info: just cache the latest frame
        self.create_subscription(Image,      '/rgbd_camera/depth_image',
                                 self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info',
                                 self._info_cb, 10)
        # detections_json: cache (no header, published together with mask)
        self.create_subscription(String,     '/grounded_sam/detections_json',
                                 self._json_cb, 10)
        # mask_image: TRIGGER — runs projection when a new mask arrives
        self.create_subscription(Image,      '/grounded_sam/mask_image',
                                 self._mask_cb, 10)

        # ── publishers ───────────────────────────────────────────────────────
        self._pub_cloud  = self.create_publisher(PointCloud2, '/labeled_points',    10)
        self._pub_result = self.create_publisher(String,      '/projection_result', 10)

        self.get_logger().info(
            f'MaskProjectorNode ready — '
            f'depth=[{self._min_depth}, {self._max_depth}]m  '
            f'trigger=mask_image'
        )

    # ── cache callbacks ───────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image) -> None:
        self._latest_depth = msg

    def _info_cb(self, msg: CameraInfo) -> None:
        self._latest_info = msg

    def _json_cb(self, msg: String) -> None:
        try:
            self._latest_detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'detections_json parse error: {e}')

    # ── trigger callback ──────────────────────────────────────────────────────

    def _mask_cb(self, mask_msg: Image) -> None:
        if self._latest_depth is None or self._latest_info is None:
            self.get_logger().warn('Waiting for depth/camera_info...')
            return
        if self._latest_detections is None:
            self.get_logger().warn('Waiting for detections_json...')
            return

        # ── decode ────────────────────────────────────────────────────────────
        depth = self._bridge.imgmsg_to_cv2(self._latest_depth,
                                           desired_encoding='32FC1')
        mask  = self._bridge.imgmsg_to_cv2(mask_msg,
                                           desired_encoding='mono8')
        K     = np.array(self._latest_info.k, dtype=np.float64).reshape(3, 3)

        # ── back-project ──────────────────────────────────────────────────────
        points, pixel_coords = depth_to_points(
            depth, K,
            min_depth=self._min_depth,
            max_depth=self._max_depth,
        )
        if len(points) == 0:
            self.get_logger().warn('No valid depth points in this frame')
            return

        # ── label ─────────────────────────────────────────────────────────────
        category_points = apply_labels(
            points, pixel_coords, mask, self._latest_detections)

        self.get_logger().info(
            'Projected: ' +
            ', '.join(f'{cp.label}={len(cp.points)}pts' for cp in category_points)
        )

        # ── publish ───────────────────────────────────────────────────────────
        self._pub_cloud.publish(
            build_pointcloud2(self._latest_depth.header, category_points))
        self._pub_result.publish(
            String(data=_build_result_json(category_points)))


# ── helpers (module-level, easy to move/extend) ───────────────────────────────

def _build_result_json(category_points: List[CategoryPoints]) -> str:
    """
    Build a JSON summary with centroid + point count per category.

    Output shape (extensible toward target_coordinate protocol):
    {
      "target":    {"label": "cup",   "centroid": [x, y, z], "point_count": N},
      "workspace": {"label": "table", "centroid": [x, y, z], "point_count": N}
    }
    """
    category_to_key = {
        CATEGORY_TARGET:    'target',
        CATEGORY_WORKSPACE: 'workspace',
    }
    out: Dict = {}
    for cp in category_points:
        key      = category_to_key.get(cp.category, f'category_{cp.category}')
        centroid = cp.points.mean(axis=0).tolist()
        out[key] = {
            'label':       cp.label,
            'centroid':    [round(v, 4) for v in centroid],
            'point_count': len(cp.points),
        }
    return json.dumps(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MaskProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
