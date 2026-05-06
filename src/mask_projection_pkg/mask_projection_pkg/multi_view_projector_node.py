"""Two-camera world-frame PointCloud2 builder."""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from scipy.spatial import KDTree
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String

from .back_projection import depth_to_points
from .cloud_builder import build_pointcloud2
from .label_mapper import (
    CATEGORY_COLOR,
    CATEGORY_FREE,
    CATEGORY_TARGET,
    CATEGORY_UNKNOWN,
    CATEGORY_WORKSPACE,
    CategoryPoints,
    apply_labels,
)

_OUTPUT_DIR = Path.home() / "gsam_ws" / "output"


def _load_extrinsics(
    path: str,
    logger,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load camera extrinsics from YAML.  Returns (R_top, t_top, R_ee, t_ee).
    Falls back to identity / zeros on any error so the node still starts.
    """
    try:
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        R_top = np.array(cfg['top_camera']['R'], dtype=np.float64)
        t_top = np.array(cfg['top_camera']['t'], dtype=np.float64)
        R_ee  = np.array(cfg['ee_camera']['R'],  dtype=np.float64)
        t_ee  = np.array(cfg['ee_camera']['t'],  dtype=np.float64)
        assert R_top.shape == (3, 3) and t_top.shape == (3,)
        assert R_ee.shape  == (3, 3) and t_ee.shape  == (3,)
        logger.info(f'Loaded camera extrinsics from {path}')
        return R_top, t_top, R_ee, t_ee
    except Exception as exc:
        logger.warn(
            f'Failed to load extrinsics from "{path}": {exc}. '
            'Using identity transforms — point cloud will be in camera frame.'
        )
        return np.eye(3), np.zeros(3), np.eye(3), np.zeros(3)


class MultiViewProjectorNode(Node):

    def __init__(self) -> None:
        super().__init__('multi_view_projector_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('top_depth_topic',        '/top_camera/depth_image')
        self.declare_parameter('top_camera_info_topic',  '/top_camera/camera_info')
        self.declare_parameter('ee_depth_topic',         '/ee_camera/depth_image')
        self.declare_parameter('ee_camera_info_topic',   '/ee_camera/camera_info')
        self.declare_parameter('mask_topic',             '/grounded_sam/mask_image')
        self.declare_parameter('detections_topic',       '/grounded_sam/detections_json')
        self.declare_parameter('output_cloud_topic',     '/world_map')
        self.declare_parameter('output_result_topic',    '/world_map_result')
        self.declare_parameter('initials',               '')
        self.declare_parameter('min_depth', 0.05)
        self.declare_parameter('max_depth', 15.0)
        self.declare_parameter('ee_seg_filter_radius', 0.015)  # metres — XY footprint radius

        _default_extrinsics = os.path.join(
            get_package_share_directory('mask_projection_pkg'),
            'config', 'camera_extrinsics.yaml',
        )
        self.declare_parameter('extrinsics_config', _default_extrinsics)

        top_depth_topic        = self.get_parameter('top_depth_topic').value
        top_camera_info_topic  = self.get_parameter('top_camera_info_topic').value
        ee_depth_topic         = self.get_parameter('ee_depth_topic').value
        ee_camera_info_topic   = self.get_parameter('ee_camera_info_topic').value
        mask_topic             = self.get_parameter('mask_topic').value
        detections_topic       = self.get_parameter('detections_topic').value
        output_cloud_topic     = self.get_parameter('output_cloud_topic').value
        output_result_topic    = self.get_parameter('output_result_topic').value
        self._min_depth        = self.get_parameter('min_depth').value
        self._max_depth        = self.get_parameter('max_depth').value
        self._ee_seg_filter_radius = self.get_parameter('ee_seg_filter_radius').value
        self._initials         = self.get_parameter('initials').value
        extrinsics_path        = self.get_parameter('extrinsics_config').value
        # launch file may pass '' (empty string) → fall back to package default
        if not extrinsics_path:
            extrinsics_path = _default_extrinsics

        # ── camera extrinsics (loaded from YAML) ──────────────────────────────
        (self._R_TOP, self._t_TOP,
         self._R_EE,  self._t_EE) = _load_extrinsics(extrinsics_path, self.get_logger())

        # ── cache ─────────────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # Top camera (optional — node warns and falls back to EE-only if absent)
        self._top_depth:  Optional[Image]      = None
        self._top_info:   Optional[CameraInfo] = None

        # EE camera
        self._ee_depth:   Optional[Image]      = None
        self._ee_info:    Optional[CameraInfo] = None

        # GSAM outputs
        self._latest_detections: Optional[List[Dict]] = None

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Image,      top_depth_topic,       self._top_depth_cb,  10)
        self.create_subscription(CameraInfo, top_camera_info_topic, self._top_info_cb,   10)
        self.create_subscription(Image,      ee_depth_topic,        self._ee_depth_cb,   10)
        self.create_subscription(CameraInfo, ee_camera_info_topic,  self._ee_info_cb,    10)
        self.create_subscription(String,     detections_topic,      self._json_cb,       10)
        self.create_subscription(Image,      mask_topic,            self._mask_cb,       10)

        # ── publishers ────────────────────────────────────────────────────────
        self._pub_cloud  = self.create_publisher(PointCloud2, output_cloud_topic,  10)
        self._pub_result = self.create_publisher(String,      output_result_topic, 10)

        self.get_logger().info(
            f'MultiViewProjectorNode ready — '
            f'depth=[{self._min_depth}, {self._max_depth}]m  '
            f'trigger={mask_topic}  '
            f'extrinsics={extrinsics_path}'
        )

    # ── cache callbacks ───────────────────────────────────────────────────────

    def _top_depth_cb(self, msg: Image) -> None:
        self._top_depth = msg

    def _top_info_cb(self, msg: CameraInfo) -> None:
        self._top_info = msg

    def _ee_depth_cb(self, msg: Image) -> None:
        self._ee_depth = msg

    def _ee_info_cb(self, msg: CameraInfo) -> None:
        self._ee_info = msg

    def _json_cb(self, msg: String) -> None:
        try:
            self._latest_detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'detections_json parse error: {e}')

    # ── trigger callback ──────────────────────────────────────────────────────

    def _mask_cb(self, mask_msg: Image) -> None:
        # EE camera is required — top camera is optional (degrades gracefully)
        if self._ee_depth is None or self._ee_info is None:
            self.get_logger().warn('Waiting for EE camera depth/camera_info...')
            return
        if self._latest_detections is None:
            self.get_logger().warn('Waiting for detections_json...')
            return

        all_category_points: List[CategoryPoints] = []

        # ── EE camera view first (seg footprint needed before top-view filtering) ──
        ee_pts = self._project_labeled(
            self._ee_depth, self._ee_info, mask_msg,
            self._latest_detections, self._R_EE, self._t_EE,
        )
        if ee_pts:
            all_category_points.extend(ee_pts)

        # ── Top camera view (depth only → UNKNOWN, EE seg footprint 제외) ────────
        if self._top_depth is not None and self._top_info is not None:
            ee_seg_pts = _collect_seg_points(ee_pts)
            top_pts = self._project_unknown(self._top_depth, self._top_info,
                                            self._R_TOP, self._t_TOP,
                                            ee_seg_pts=ee_seg_pts,
                                            ee_seg_filter_radius=self._ee_seg_filter_radius)
            if top_pts is not None:
                all_category_points.append(top_pts)
        else:
            self.get_logger().warn(
                'Top camera not available — publishing EE view only. '
                'Check top_depth_topic / top_camera_info_topic parameters.'
            )

        if not all_category_points:
            self.get_logger().warn('No valid points from either camera.')
            return

        self.get_logger().info(
            'World map: ' +
            ', '.join(f'{cp.label}={len(cp.points)}pts' for cp in all_category_points)
        )

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'world'

        # ── publish ───────────────────────────────────────────────────────────
        self._pub_cloud.publish(build_pointcloud2(header, all_category_points))
        self._pub_result.publish(String(data=_build_result_json(all_category_points)))

        # ── save PLY to disk ──────────────────────────────────────────────────
        stamp  = self._ee_depth.header.stamp.sec
        prefix = f"{self._initials}_" if self._initials else ""
        _save_ply_labeled(
            _OUTPUT_DIR / f"world_map_{prefix}{stamp}.ply",
            all_category_points,
        )

    # ── projection helpers ────────────────────────────────────────────────────

    def _project_unknown(
        self,
        depth_msg:             Image,
        info_msg:              CameraInfo,
        R:                     np.ndarray,
        t:                     np.ndarray,
        ee_seg_pts:            Optional[np.ndarray] = None,
        ee_seg_filter_radius:  float = 0.015,
    ) -> Optional[CategoryPoints]:
        """Back-project top-view depth → UNKNOWN, removing points within
        ee_seg_filter_radius (XY plane) of any EE-segmented point."""
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        K     = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)

        pts_cam, _ = depth_to_points(depth, K,
                                     min_depth=self._min_depth,
                                     max_depth=self._max_depth)
        if len(pts_cam) == 0:
            return None

        pts_world = (R @ pts_cam.T).T + t

        if ee_seg_pts is not None and len(ee_seg_pts) > 0:
            tree  = KDTree(ee_seg_pts[:, :2])
            dists, _ = tree.query(pts_world[:, :2], workers=-1)
            pts_world = pts_world[dists > ee_seg_filter_radius]

        if len(pts_world) == 0:
            return None

        return _make_unknown_points(pts_world)

    def _project_labeled(
        self,
        depth_msg:  Image,
        info_msg:   CameraInfo,
        mask_msg:   Image,
        detections: List[Dict],
        R: np.ndarray,
        t: np.ndarray,
    ) -> List[CategoryPoints]:
        """Back-project EE depth, transform to world, apply GSAM labels."""
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        mask  = self._bridge.imgmsg_to_cv2(mask_msg,  desired_encoding='mono8')
        K     = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)

        # NOTE: pixel_coords are at depth image resolution.
        # apply_labels indexes mask_image (built at RGB resolution by GSAM) using
        # those coords.  If depth and RGB resolutions differ, add a resize step here.
        # Standard RGBD sensors produce equal resolutions → no resize needed.
        pts_cam, pixel_coords = depth_to_points(depth, K,
                                                min_depth=self._min_depth,
                                                max_depth=self._max_depth)
        if len(pts_cam) == 0:
            return []

        pts_world = (R @ pts_cam.T).T + t
        return apply_labels(pts_world, pixel_coords, mask, detections)


# ── module-level helpers ──────────────────────────────────────────────────────

def _collect_seg_points(category_points: List[CategoryPoints]) -> Optional[np.ndarray]:
    """EE CategoryPoints에서 FREE 제외한 seg 포인트 XY만 모아 (N,2) 반환."""
    seg = [cp.points for cp in category_points if cp.category != CATEGORY_FREE]
    if not seg:
        return None
    return np.concatenate(seg, axis=0)


def _make_unknown_points(points: np.ndarray) -> CategoryPoints:
    """Wrap an (N, 3) world-frame array as UNKNOWN CategoryPoints (top-view geometry)."""
    n     = len(points)
    color = CATEGORY_COLOR[CATEGORY_UNKNOWN]
    return CategoryPoints(
        label      = 'unknown',
        category   = CATEGORY_UNKNOWN,
        points     = points.astype(np.float32),
        colors     = np.tile(np.array(color, dtype=np.uint8), (n, 1)),
        categories = np.full(n, CATEGORY_UNKNOWN, dtype=np.uint8),
    )



def _build_result_json(category_points: List[CategoryPoints]) -> str:
    """
    JSON summary per category: label, centroid, bbox_3d_world, point_count.

    {
      "target":    {"label": "cup",   "centroid": [x,y,z],
                    "bbox_3d_world": {"min": [x,y,z], "max": [x,y,z]},
                    "point_count": N},
      "workspace": { ... },
      ...
    }
    """
    _CATEGORY_KEY = {
        CATEGORY_TARGET:    'target',
        CATEGORY_WORKSPACE: 'workspace',
    }
    out: Dict = {}
    for cp in category_points:
        key      = _CATEGORY_KEY.get(cp.category, cp.label)
        centroid = cp.points.mean(axis=0).tolist()
        pts_min  = cp.points.min(axis=0).tolist()
        pts_max  = cp.points.max(axis=0).tolist()
        out[key] = {
            'label':         cp.label,
            'centroid':      [round(v, 4) for v in centroid],
            'bbox_3d_world': {
                'min': [round(v, 4) for v in pts_min],
                'max': [round(v, 4) for v in pts_max],
            },
            'point_count':   len(cp.points),
        }
    return json.dumps(out)


def _save_ply_labeled(path: Path, category_points: List[CategoryPoints]) -> None:
    """Save world-frame labeled points (XYZ + RGB + category) as PLY."""
    all_pts  = np.concatenate([cp.points     for cp in category_points], axis=0)
    all_col  = np.concatenate([cp.colors     for cp in category_points], axis=0)
    all_cats = np.concatenate([cp.categories for cp in category_points], axis=0)
    N = len(all_pts)
    path.parent.mkdir(parents=True, exist_ok=True)
    dt = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('red', np.uint8), ('green', np.uint8), ('blue', np.uint8),
        ('category', np.uint8),
    ])
    arr             = np.zeros(N, dtype=dt)
    arr['x']        = all_pts[:, 0]
    arr['y']        = all_pts[:, 1]
    arr['z']        = all_pts[:, 2]
    arr['red']      = all_col[:, 0]
    arr['green']    = all_col[:, 1]
    arr['blue']     = all_col[:, 2]
    arr['category'] = all_cats
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {N}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "property uchar category\n"
        "end_header\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(arr.tobytes())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiViewProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
