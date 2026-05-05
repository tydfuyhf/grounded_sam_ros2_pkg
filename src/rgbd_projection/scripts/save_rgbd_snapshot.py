#!/usr/bin/env python3
"""
Gazebo RGB-D 스냅샷 저장 노드.

RGB, depth, camera_info를 동기화해서 파일로 저장한다.
저장 후 자동 종료 (one-shot).

저장 경로: ~/test_projection/data/
    rgb.png           — BGR uint8 이미지
    depth.npy         — float32 (H, W), 단위: meters
    camera_info.yaml  — fx, fy, cx, cy, width, height

Subscriptions:
    /rgbd_camera/image        (sensor_msgs/Image, bgr8)
    /rgbd_camera/depth_image  (sensor_msgs/Image, 32FC1, meters)
    /rgbd_camera/camera_info  (sensor_msgs/CameraInfo)

Usage:
    source /opt/ros/jazzy/setup.bash
    source ~/test_projection/install/setup.bash
    python3 save_rgbd_snapshot.py [--output ~/test_projection/data]
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

DEFAULT_OUTPUT = str(Path.home() / "test_projection" / "data")


class RGBDSnapshotNode(Node):
    def __init__(self, output_dir: str):
        super().__init__("rgbd_snapshot_node")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self._saved = False

        # 동기화 구독: rgb + depth + camera_info
        self._sub_rgb = Subscriber(self, Image, "/rgbd_camera/image")
        self._sub_depth = Subscriber(self, Image, "/rgbd_camera/depth_image")
        self._sub_info = Subscriber(self, CameraInfo, "/rgbd_camera/camera_info")

        self._sync = ApproximateTimeSynchronizer(
            [self._sub_rgb, self._sub_depth, self._sub_info],
            queue_size=10,
            slop=0.05,
        )
        self._sync.registerCallback(self._callback)
        self.get_logger().info(
            f"대기 중... 첫 동기화 프레임 수신 시 저장 후 종료\n  출력 폴더: {self.output_dir}"
        )

    def _callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        if self._saved:
            return
        self._saved = True

        # --- rgb.png ---
        rgb_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        cv2.imwrite(str(self.output_dir / "rgb.png"), rgb_bgr)
        self.get_logger().info(f"[저장] rgb.png  {rgb_bgr.shape[1]}x{rgb_bgr.shape[0]}")

        # --- depth.npy  (float32, meters) ---
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth = np.array(depth, dtype=np.float32)
        np.save(str(self.output_dir / "depth.npy"), depth)
        valid = depth[np.isfinite(depth) & (depth > 0)]
        self.get_logger().info(
            f"[저장] depth.npy  min={valid.min():.3f}m  max={valid.max():.3f}m"
            if len(valid) else "[저장] depth.npy  (유효 픽셀 없음)"
        )

        # --- camera_info.yaml ---
        K = info_msg.k  # row-major 3x3
        camera_info = {
            "width":  info_msg.width,
            "height": info_msg.height,
            "fx": float(K[0]),
            "fy": float(K[4]),
            "cx": float(K[2]),
            "cy": float(K[5]),
            "distortion_model": info_msg.distortion_model,
            "D": list(info_msg.d),
        }
        with open(self.output_dir / "camera_info.yaml", "w") as f:
            yaml.dump(camera_info, f, default_flow_style=False)
        self.get_logger().info(
            f"[저장] camera_info.yaml  fx={camera_info['fx']:.2f}  fy={camera_info['fy']:.2f}"
            f"  cx={camera_info['cx']:.2f}  cy={camera_info['cy']:.2f}"
        )

        self.get_logger().info("저장 완료. 노드 종료.")
        # 이벤트 루프 밖에서 종료하기 위해 타이머로 처리
        self.create_timer(0.1, self._shutdown)

    def _shutdown(self):
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="저장 폴더 경로")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RGBDSnapshotNode(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
