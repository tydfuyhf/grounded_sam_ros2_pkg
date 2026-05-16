"""
vgn_grasp_node.py

Subscribes:
  /world_map        (PointCloud2)  — cached; all-category labeled cloud
  /world_map_result (String JSON)  — trigger; contains target centroid + point_count

Processing (on each /world_map_result):
  1. Parse target centroid, point_count  → skip if < min_point_count
  2. AABB ROI crop from cached /world_map (30 cm cube centred on centroid)
  3. KDTree unsigned SDF → 40×40×40 float32 grid
  4. VGN inference → NMS → grasp candidates
  5. Semantic filter: keep only grasps near TARGET (category==1) points
  6. TF world → panda_link0
  7. Publish /grasp_candidates (JSON String)
  8. Publish /grasp_markers (MarkerArray) — RViz visualization

RViz visualization:
  /grasp_markers — arrow per candidate (approach direction), color = quality
    green (quality=1.0) → red (quality=min_quality)
    arrow shaft = approach vector (gripper -Z), head = grasp position
  Add to RViz2: MarkerArray, topic=/grasp_markers, Fixed Frame=world

External dependency: external/vgn  (ethz-asl/vgn git submodule)
  git submodule add https://github.com/ethz-asl/vgn external/vgn
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

# ── VGN library path (external/vgn submodule) ────────────────────────────────
_WS = Path(os.environ.get('GSAM_WS', str(Path.home() / 'gsam_ws')))
_VGN_SRC = _WS / 'external' / 'vgn' / 'src'
if _VGN_SRC.exists() and str(_VGN_SRC) not in sys.path:
    sys.path.insert(0, str(_VGN_SRC))

try:
    import torch
    from scipy import ndimage as _ndimage
    # vgn.detection imports vgn.vis which imports rospy (ROS 1) — unusable in ROS 2.
    # vgn.networks and vgn.utils.transform have no rospy dependency → import directly.
    # predict / process / select are copied verbatim from ethz-asl/vgn detection.py.
    from vgn.networks import load_network
    from vgn.utils.transform import Transform as _VgnTransform, Rotation as _VgnRotation

    class _Grasp:
        def __init__(self, pose, width):
            self.pose = pose
            self.width = width

    def _vgn_predict(tsdf_vol, net, device):
        assert tsdf_vol.shape == (1, 40, 40, 40)
        x = torch.from_numpy(tsdf_vol).unsqueeze(0).to(device)
        with torch.no_grad():
            qual_vol, rot_vol, width_vol = net(x)
        return (qual_vol.cpu().squeeze().numpy(),
                rot_vol.cpu().squeeze().numpy(),
                width_vol.cpu().squeeze().numpy())

    def _vgn_process(tsdf_vol, qual_vol, rot_vol, width_vol,
                     gaussian_filter_sigma=1.0, min_width=1.33, max_width=9.33):
        tsdf_vol  = tsdf_vol.squeeze()
        qual_vol  = _ndimage.gaussian_filter(qual_vol, sigma=gaussian_filter_sigma,
                                             mode='nearest')
        outside   = tsdf_vol > 0.5
        inside    = np.logical_and(1e-3 < tsdf_vol, tsdf_vol < 0.5)
        valid     = _ndimage.morphology.binary_dilation(
                        outside, iterations=2, mask=np.logical_not(inside))
        qual_vol[valid == False] = 0.0  # noqa: E712
        qual_vol[np.logical_or(width_vol < min_width, width_vol > max_width)] = 0.0
        return qual_vol, rot_vol, width_vol

    def _vgn_select(qual_vol, rot_vol, width_vol, threshold=0.90, max_filter_size=4):
        qual_vol = qual_vol.copy()
        qual_vol[qual_vol < threshold] = 0.0
        max_vol  = _ndimage.maximum_filter(qual_vol, size=max_filter_size)
        qual_vol = np.where(qual_vol == max_vol, qual_vol, 0.0)
        grasps, scores = [], []
        for idx in np.argwhere(np.where(qual_vol, 1.0, 0.0)):
            i, j, k = idx
            score = qual_vol[i, j, k]
            ori   = _VgnRotation.from_quat(rot_vol[:, i, j, k])
            pos   = np.array([i, j, k], dtype=np.float64)
            grasps.append(_Grasp(_VgnTransform(ori, pos), width_vol[i, j, k]))
            scores.append(score)
        return grasps, scores

    def from_voxel_coordinates(grasp, voxel_size):
        grasp.pose.translation = grasp.pose.translation * voxel_size
        grasp.width = grasp.width * voxel_size
        return grasp

    _VGN_OK = True
except ImportError as _e:
    _VGN_OK = False
    _VGN_IMPORT_ERROR = str(_e)

# ── PointCloud2 dtype (matches cloud_builder.py, 20 bytes/point) ─────────────
_PC2_DTYPE = np.dtype([
    ('x',        np.float32),
    ('y',        np.float32),
    ('z',        np.float32),
    ('rgb',      np.float32),
    ('category', np.uint8),
    ('_pad',     np.uint8, (3,)),
])
_POINT_STEP = 20

_CAT_TARGET = np.uint8(1)


def _decode_world_map(msg: PointCloud2) -> Tuple[np.ndarray, np.ndarray]:
    """Decode /world_map PointCloud2 → (points (N,3) float32, categories (N,) uint8)."""
    if msg.width == 0:
        return np.empty((0, 3), np.float32), np.empty((0,), np.uint8)
    arr = np.frombuffer(msg.data, dtype=_PC2_DTYPE)
    pts = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)
    cats = arr['category'].copy()
    return pts, cats


def _build_tsdf_grid(roi_pts: np.ndarray, roi_size_m: float, reso: int) -> np.ndarray:
    """
    KDTree unsigned SDF approximation.
    roi_pts: (N,3) points already in roi frame (origin at [0,0,0]).
    Returns float32 grid (1, reso, reso, reso), values normalized to [0, 1].
    Normalization: dist_normalized = clip(dist, 0, trunc) / trunc
    VGN process() uses 0.5 as outside-surface threshold, so [0,1] range is required.
    """
    voxel_size = roi_size_m / reso
    trunc = 4.0 * voxel_size

    # Voxel centres in roi frame
    idx = np.arange(reso, dtype=np.float32)
    gi, gj, gk = np.meshgrid(idx, idx, idx, indexing='ij')
    centers = (np.stack([gi, gj, gk], axis=-1) + 0.5) * voxel_size  # (reso,reso,reso,3)
    centers_flat = centers.reshape(-1, 3)  # (reso^3, 3)

    if len(roi_pts) == 0:
        return np.ones((1, reso, reso, reso), dtype=np.float32)

    tree = KDTree(roi_pts)
    dists, _ = tree.query(centers_flat, workers=-1)  # (reso^3,)
    dists = (np.clip(dists, 0.0, trunc) / trunc).astype(np.float32)
    return dists.reshape(1, reso, reso, reso)


class VgnGraspNode(Node):

    def __init__(self) -> None:
        super().__init__('vgn_grasp_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('roi_size_m',             0.30)
        self.declare_parameter('tsdf_resolution',        40)
        self.declare_parameter('vgn_model_path',         'models/vgn_conv.pth')
        self.declare_parameter('min_quality',            0.5)
        self.declare_parameter('max_candidates',         5)
        self.declare_parameter('min_point_count',        50)
        self.declare_parameter('semantic_filter_radius', 0.05)
        self.declare_parameter('world_map_topic',        '/world_map')
        self.declare_parameter('world_map_result_topic', '/world_map_result')
        self.declare_parameter('grasp_candidates_topic', '/grasp_candidates')
        self.declare_parameter('world_frame',            'world')
        self.declare_parameter('robot_frame',            'panda_link0')

        self._roi_size_m   = self.get_parameter('roi_size_m').value
        self._reso         = self.get_parameter('tsdf_resolution').value
        self._min_quality  = self.get_parameter('min_quality').value
        self._max_k        = self.get_parameter('max_candidates').value
        self._min_pts      = self.get_parameter('min_point_count').value
        self._sem_radius   = self.get_parameter('semantic_filter_radius').value
        self._world_frame  = self.get_parameter('world_frame').value
        self._robot_frame  = self.get_parameter('robot_frame').value

        model_path_param = self.get_parameter('vgn_model_path').value
        model_path = Path(model_path_param)
        if not model_path.is_absolute():
            model_path = _WS / model_path

        # ── VGN network ───────────────────────────────────────────────────────
        if not _VGN_OK:
            self.get_logger().error(
                f'VGN import failed: {_VGN_IMPORT_ERROR}\n'
                f'Run: git submodule add https://github.com/ethz-asl/vgn external/vgn\n'
                f'Then: source launch_env.bash'
            )
            raise RuntimeError('VGN library not available')

        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'VGN device: {self._device}')

        if not model_path.exists():
            self.get_logger().error(f'VGN model not found: {model_path}')
            raise FileNotFoundError(str(model_path))

        self._net = load_network(model_path, self._device)
        self._net.eval()
        self.get_logger().info(f'VGN model loaded: {model_path}')

        # ── TF2 ───────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Cache ─────────────────────────────────────────────────────────────
        self._world_map_pts:  Optional[np.ndarray] = None  # (N,3)
        self._world_map_cats: Optional[np.ndarray] = None  # (N,)
        self._pending_result: Optional[str] = None          # /world_map_result raw JSON

        # ── Subscriptions ─────────────────────────────────────────────────────
        world_map_topic  = self.get_parameter('world_map_topic').value
        result_topic     = self.get_parameter('world_map_result_topic').value

        self.create_subscription(PointCloud2, world_map_topic,
                                 self._world_map_cb, 10)
        self.create_subscription(String, result_topic,
                                 self._result_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        grasp_topic = self.get_parameter('grasp_candidates_topic').value
        self._grasp_pub  = self.create_publisher(String,      grasp_topic,     10)
        self._marker_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)

        self.get_logger().info(
            f'vgn_grasp_node ready  '
            f'roi={self._roi_size_m}m  reso={self._reso}  '
            f'min_quality={self._min_quality}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _world_map_cb(self, msg: PointCloud2) -> None:
        pts, cats = _decode_world_map(msg)
        self._world_map_pts  = pts
        self._world_map_cats = cats
        if self._pending_result is not None:
            self.get_logger().info('/world_map 도착 — 대기 중인 world_map_result 처리')
            pending = String()
            pending.data = self._pending_result
            self._pending_result = None
            self._result_cb(pending)

    def _result_cb(self, msg: String) -> None:
        if self._world_map_pts is None:
            self._pending_result = msg.data
            self.get_logger().warn('/world_map 미도착 — world_map_result 캐시 후 대기')
            return

        # ── 1. Parse trigger ──────────────────────────────────────────────────
        try:
            result = json.loads(msg.data)
            target_info = result.get('target', {})
            centroid     = np.array(target_info['centroid'],    dtype=np.float32)
            point_count  = int(target_info['point_count'])
        except (KeyError, json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f'world_map_result parse error: {e}')
            return

        if point_count < self._min_pts:
            self.get_logger().info(
                f'TARGET point_count={point_count} < min={self._min_pts} — skip'
            )
            return

        pts  = self._world_map_pts
        cats = self._world_map_cats
        t0   = time.monotonic()

        # ── 2. AABB ROI crop ──────────────────────────────────────────────────
        half      = self._roi_size_m * 0.5
        roi_min   = centroid - half
        roi_max   = centroid + half

        in_roi = np.all((pts >= roi_min) & (pts <= roi_max), axis=1)
        roi_pts  = pts[in_roi]
        roi_cats = cats[in_roi]

        if len(roi_pts) == 0:
            self.get_logger().warn('ROI crop returned 0 points — skip')
            return

        # ── 3. Transform to ROI frame ─────────────────────────────────────────
        roi_pts_local = (roi_pts - roi_min).astype(np.float32)  # origin at [0,0,0]

        # ── 4. Build TSDF grid ────────────────────────────────────────────────
        grid = _build_tsdf_grid(roi_pts_local, self._roi_size_m, self._reso)

        # ── 5. VGN inference ──────────────────────────────────────────────────
        qual_vol, rot_vol, width_vol = _vgn_predict(grid, self._net, self._device)
        qual_vol, rot_vol, width_vol = _vgn_process(grid, qual_vol, rot_vol, width_vol)

        # ── 6. NMS + voxel → roi frame 변환 ──────────────────────────────────
        voxel_size = self._roi_size_m / self._reso
        grasps_voxel, scores = _vgn_select(
            qual_vol, rot_vol, width_vol,
            threshold=self._min_quality,
        )

        if not grasps_voxel:
            self.get_logger().info('No grasp candidates after NMS')
            return

        # voxel index → roi frame meters
        grasps = [from_voxel_coordinates(g, voxel_size) for g in grasps_voxel]

        # ── 7. Semantic filter ────────────────────────────────────────────────
        target_mask = (cats == _CAT_TARGET)
        target_pts  = pts[target_mask]

        if len(target_pts) > 0:
            target_tree = KDTree(target_pts)
            kept_grasps: list = []
            kept_scores: list = []
            for grasp, score in zip(grasps, scores):
                p_world = roi_min + grasp.pose.translation.astype(np.float32)
                dist, _ = target_tree.query(p_world.reshape(1, 3))
                if dist[0] <= self._sem_radius:
                    kept_grasps.append(grasp)
                    kept_scores.append(float(score))
        else:
            self.get_logger().warn('No TARGET points in /world_map — skipping semantic filter')
            kept_grasps = list(grasps)
            kept_scores = [float(s) for s in scores]

        if not kept_grasps:
            self.get_logger().info('All candidates rejected by semantic filter')
            return

        # ── 8. Sort, Top-K ────────────────────────────────────────────────────
        order = np.argsort(kept_scores)[::-1]
        top_grasps = [kept_grasps[i] for i in order[:self._max_k]]
        top_scores = [kept_scores[i] for i in order[:self._max_k]]

        # ── 9. TF world → panda_link0 ─────────────────────────────────────────
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._robot_frame, self._world_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1),
            )
            tf_ok = True
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed ({e}) — publishing in world frame')
            tf_ok = False

        output_frame = self._robot_frame if tf_ok else self._world_frame

        candidates = []
        for grasp, score in zip(top_grasps, top_scores):
            p_world = roi_min + grasp.pose.translation.astype(np.float32)
            quat    = grasp.pose.rotation.as_quat()  # [qx, qy, qz, qw]

            if tf_ok:
                p_world, quat = _apply_tf(tf_stamped, p_world, quat)

            candidates.append({
                'position':   p_world.tolist(),
                'quaternion': quat.tolist(),
                'width':      float(grasp.width),
                'quality':    score,
                'frame':      output_frame,
            })

        payload = {
            'candidates':      candidates,
            'target_centroid': centroid.tolist(),
            'stamp':           self.get_clock().now().nanoseconds * 1e-9,
        }

        out_msg = String()
        out_msg.data = json.dumps(payload)
        self._grasp_pub.publish(out_msg)

        self._publish_markers(candidates, output_frame)

        elapsed = time.monotonic() - t0
        self.get_logger().info(
            f'Published {len(candidates)} grasp(s)  '
            f'best_quality={top_scores[0]:.3f}  elapsed={elapsed:.2f}s'
        )

    def _publish_markers(self, candidates: list, frame: str) -> None:
        """
        Publish MarkerArray for RViz2 visualization.

        Per candidate (1-based id to avoid DELETEALL id=0 conflict):
          - ARROW  (id=i*2+1) : approach direction, color = quality (green→red)
          - SPHERE (id=i*2+2) : grasp position, same color, size = gripper width
        DELETEALL is sent in a separate prior message to avoid same-array id conflict.
        """
        from scipy.spatial.transform import Rotation as R

        now = self.get_clock().now().to_msg()

        # DELETEALL in its own message — avoids id=0 conflict with first Arrow
        clear_ma = MarkerArray()
        clear_m = Marker()
        clear_m.header.frame_id = frame
        clear_m.header.stamp    = now
        clear_m.ns              = 'vgn_grasps'
        clear_m.action          = Marker.DELETEALL
        clear_ma.markers        = [clear_m]
        self._marker_pub.publish(clear_ma)

        markers: list[Marker] = []
        for i, c in enumerate(candidates):
            q = float(c['quality'])
            # top candidate: bright cyan-white, lower: pure blue — all fully opaque
            color      = ColorRGBA(r=0.0, g=0.6 * q, b=1.0, a=1.0)
            color_ring = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # white ring = grasp width

            pos  = c['position']
            quat = c['quaternion']  # [qx, qy, qz, qw]
            rot  = R.from_quat(quat)

            approach  = rot.apply([0.0, 0.0, -1.0])
            arrow_len = 0.20  # 20 cm — clearly visible in world frame

            # ── ARROW (approach direction) ─────────────────────────────────────
            arrow = Marker()
            arrow.header.frame_id = frame
            arrow.header.stamp    = now
            arrow.ns              = 'vgn_grasps'
            arrow.id              = i * 3 + 1
            arrow.type            = Marker.ARROW
            arrow.action          = Marker.ADD
            tail = Point(x=pos[0] - approach[0] * arrow_len,
                         y=pos[1] - approach[1] * arrow_len,
                         z=pos[2] - approach[2] * arrow_len)
            head = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            arrow.points   = [tail, head]
            arrow.scale    = Vector3(x=0.02, y=0.04, z=0.0)  # shaft 2 cm, head 4 cm
            arrow.color    = color
            # lifetime=0: 다음 DELETEALL 전까지 유지 (추론 주기보다 짧으면 깜빡임)
            markers.append(arrow)

            # ── SPHERE at grasp centre ─────────────────────────────────────────
            sphere = Marker()
            sphere.header.frame_id = frame
            sphere.header.stamp    = now
            sphere.ns              = 'vgn_grasps'
            sphere.id              = i * 3 + 2
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position   = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            sphere.scale           = Vector3(x=0.03, y=0.03, z=0.03)  # 3 cm fixed sphere
            sphere.color           = color
            # lifetime=0: 다음 DELETEALL 전까지 유지
            markers.append(sphere)

            # ── CYLINDER showing gripper width ────────────────────────────────
            width = float(c.get('width', 0.05))
            ring = Marker()
            ring.header.frame_id = frame
            ring.header.stamp    = now
            ring.ns              = 'vgn_grasps'
            ring.id              = i * 3 + 3
            ring.type            = Marker.CYLINDER
            ring.action          = Marker.ADD
            ring.pose.position   = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            # cylinder axis = gripper open direction (VGN: X axis)
            gripper_x = rot.apply([1.0, 0.0, 0.0])
            # orient cylinder along gripper X: compute quaternion from Z→gripper_x
            from scipy.spatial.transform import Rotation as _R2
            z_axis = np.array([0.0, 0.0, 1.0])
            cross  = np.cross(z_axis, gripper_x)
            cross_norm = np.linalg.norm(cross)
            if cross_norm > 1e-6:
                axis  = cross / cross_norm
                angle = float(np.arccos(np.clip(np.dot(z_axis, gripper_x), -1, 1)))
                q_ring = _R2.from_rotvec(axis * angle).as_quat()
            else:
                q_ring = np.array([0.0, 0.0, 0.0, 1.0])
            ring.pose.orientation.x = float(q_ring[0])
            ring.pose.orientation.y = float(q_ring[1])
            ring.pose.orientation.z = float(q_ring[2])
            ring.pose.orientation.w = float(q_ring[3])
            ring.scale   = Vector3(x=width, y=width, z=0.005)  # flat disk = gripper span
            ring.color   = color_ring
            # lifetime=0: 다음 DELETEALL 전까지 유지
            markers.append(ring)

        ma = MarkerArray()
        ma.markers = markers
        self._marker_pub.publish(ma)


def _apply_tf(
    tf_stamped,
    p_world: np.ndarray,
    quat_world: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Apply TF world→robot to a position and orientation."""
    from scipy.spatial.transform import Rotation as R

    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    tf_rot = R.from_quat([r.x, r.y, r.z, r.w])
    tf_trans = np.array([t.x, t.y, t.z], dtype=np.float64)

    p_robot = tf_rot.apply(p_world.astype(np.float64)) + tf_trans
    q_world_rot = R.from_quat(quat_world)
    q_robot     = tf_rot * q_world_rot

    return p_robot.astype(np.float32), q_robot.as_quat().astype(np.float32)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VgnGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
