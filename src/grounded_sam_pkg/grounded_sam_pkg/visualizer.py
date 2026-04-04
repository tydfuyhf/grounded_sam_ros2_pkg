from pathlib import Path
from typing import List, Dict, Any

import cv2
import numpy as np


def draw_bboxes(image_bgr: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
    """Draw bounding boxes and labels onto a copy of the image."""
    img = image_bgr.copy()
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det["bbox_xyxy"]]
        label = det.get("label", "")
        conf = det.get("confidence") or 0.0
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            img,
            f"{label} {conf:.2f}",
            (x1, max(y1 - 6, 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    return img


def draw_masks(image_bgr: np.ndarray, masks: List[Dict[str, Any]], alpha: float = 0.4) -> np.ndarray:
    """Overlay semi-transparent colored masks onto a copy of the image."""
    img = image_bgr.copy()
    overlay = img.copy()
    for i, m in enumerate(masks):
        # deterministic per-instance color
        color = np.array([
            (i * 67 + 100) % 256,
            (i * 113 + 50) % 256,
            (i * 151 + 200) % 256,
        ], dtype=np.uint8)
        mask = m["mask"].astype(bool)
        overlay[mask] = color
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    return img


def save_result(image_bgr: np.ndarray, output_path: str) -> None:
    """Save image to disk, creating parent directories if needed."""
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), image_bgr)
