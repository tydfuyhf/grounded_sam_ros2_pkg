"""
static_projector.py

Offline PointCloud viewer — no Gazebo, no gsam node needed.
Loads pre-saved files and publishes a latched /labeled_points for RViz.

Required files
--------------
  --depth       float32 .npy (H, W), meters
  --camera-info .yaml with keys: fx, fy, cx, cy
  --mask        uint8 .png  pixel = 1-based detection index, 0 = background
  --detections  .json       [{label, confidence, bbox_xyxy, ...}, ...]

Parameters (ros-args)
---------------------
  depth_path       str   ~/test_projection/data/depth.npy
  camera_info_path str   ~/test_projection/data/camera_info.yaml
  mask_path        str   ~/gsam_ws/output/ct/mask_image.png
  detections_path  str   ~/gsam_ws/output/ct/detections.json
  frame_id         str   rgbd_camera/link/rgbd_camera
  min_depth        float 0.05
  max_depth        float 15.0
  target_mask_val  int   1   ← mask pixel value that maps to TARGET
  workspace_mask_val int 2   ← mask pixel value that maps to WORKSPACE

Usage
-----
  ros2 run mask_projection_pkg static_projector

  # Override target/workspace if GroundingDINO flipped the order:
  ros2 run mask_projection_pkg static_projector \
    --ros-args -p target_mask_val:=2 -p workspace_mask_val:=1
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from .back_projection import depth_to_points
from .cloud_builder import build_pointcloud2
from .label_mapper import (
    apply_labels,
    CATEGORY_TARGET,
    CATEGORY_WORKSPACE,
    MASK_VALUE_TO_CATEGORY,
    CATEGORY_COLOR,
)

_HOME = Path.home()
_LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class StaticProjectorNode(Node):

    def __init__(self) -> None:
        super().__init__('static_projector_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('depth_path',
            str(_HOME / 'test_projection' / 'data' / 'depth.npy'))
        self.declare_parameter('camera_info_path',
            str(_HOME / 'test_projection' / 'data' / 'camera_info.yaml'))
        self.declare_parameter('mask_path',
            str(_HOME / 'gsam_ws' / 'output' / 'ct' / 'mask_image.png'))
        self.declare_parameter('detections_path',
            str(_HOME / 'gsam_ws' / 'output' / 'ct' / 'detections.json'))
        self.declare_parameter('frame_id',          'rgbd_camera/link/rgbd_camera')
        self.declare_parameter('min_depth',          0.05)
        self.declare_parameter('max_depth',          15.0)
        # Which mask pixel value → TARGET / WORKSPACE
        # Override with -p target_mask_val:=2 if GroundingDINO flipped the order
        self.declare_parameter('target_mask_val',    1)
        self.declare_parameter('workspace_mask_val', 2)

        depth_path    = Path(self.get_parameter('depth_path').value)
        ci_path       = Path(self.get_parameter('camera_info_path').value)
        mask_path     = Path(self.get_parameter('mask_path').value)
        det_path      = Path(self.get_parameter('detections_path').value)
        frame_id      = self.get_parameter('frame_id').value
        min_depth     = self.get_parameter('min_depth').value
        max_depth     = self.get_parameter('max_depth').value
        tgt_val       = self.get_parameter('target_mask_val').value
        ws_val        = self.get_parameter('workspace_mask_val').value

        # ── runtime mask-value override ──────────────────────────────────────
        # Temporarily remap MASK_VALUE_TO_CATEGORY without touching the module
        mask_override = {tgt_val: CATEGORY_TARGET, ws_val: CATEGORY_WORKSPACE}

        # ── load files ───────────────────────────────────────────────────────
        depth      = np.load(str(depth_path))
        mask       = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        detections = json.loads(det_path.read_text())

        with open(ci_path) as f:
            ci = yaml.safe_load(f)
        K = np.array([
            [ci['fx'],      0,  ci['cx']],
            [    0,     ci['fy'], ci['cy']],
            [    0,         0,       1  ],
        ], dtype=np.float64)

        self.get_logger().info(f'depth  : {depth_path}  shape={depth.shape}')
        self.get_logger().info(f'mask   : {mask_path}   values={np.unique(mask).tolist()}')
        self.get_logger().info(f'K      : fx={ci["fx"]:.1f} fy={ci["fy"]:.1f} '
                               f'cx={ci["cx"]:.1f} cy={ci["cy"]:.1f}')
        self.get_logger().info(
            f'mapping: mask_pixel {tgt_val}→TARGET  {ws_val}→WORKSPACE')

        for i, d in enumerate(detections):
            mv = i + 1
            cat_name = ('TARGET' if mv == tgt_val
                        else 'WORKSPACE' if mv == ws_val
                        else 'ignored')
            color = CATEGORY_COLOR.get(
                mask_override.get(mv, 0), (128, 128, 128))
            self.get_logger().info(
                f'  detection[{i}] mask_pixel={mv}  label="{d["label"]}"'
                f'  → {cat_name}  color=RGB{color}')

        # ── back-project ─────────────────────────────────────────────────────
        points, pixel_coords = depth_to_points(depth, K, min_depth, max_depth)
        self.get_logger().info(f'valid depth points: {len(points)}')

        # apply_labels uses MASK_VALUE_TO_CATEGORY from the module;
        # patch it temporarily for this run
        original = dict(MASK_VALUE_TO_CATEGORY)
        MASK_VALUE_TO_CATEGORY.clear()
        MASK_VALUE_TO_CATEGORY.update(mask_override)

        category_points = apply_labels(points, pixel_coords, mask, detections)

        MASK_VALUE_TO_CATEGORY.clear()
        MASK_VALUE_TO_CATEGORY.update(original)

        self.get_logger().info(
            'Projected: ' +
            ', '.join(f'{cp.label}={len(cp.points)}pts' for cp in category_points)
        )

        # ── build message ────────────────────────────────────────────────────
        header          = Header()
        header.frame_id = frame_id
        self._cloud_msg = build_pointcloud2(header, category_points)

        # ── publish latched (RViz gets it on connect) ─────────────────────────
        self._pub   = self.create_publisher(PointCloud2, '/labeled_points', _LATCHED)
        self._timer = self.create_timer(1.0, self._publish)
        self._publish()

    def _publish(self) -> None:
        self._cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._cloud_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
