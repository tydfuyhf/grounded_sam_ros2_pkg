from typing import List, Dict, Any

import torch


def format_detections(detections, phrases: List[str]) -> List[Dict[str, Any]]:
    """
    Convert sv.Detections + phrase labels into a plain list of dicts.

    Returns:
        List of dicts, each with:
            bbox_xyxy   : [x1, y1, x2, y2] in absolute pixel coords
            confidence  : float
            label       : str (matched phrase from GroundingDINO)
    """
    results = []
    confidences = (
        detections.confidence
        if detections.confidence is not None
        else [None] * len(detections.xyxy)
    )
    for i, box in enumerate(detections.xyxy):
        results.append({
            "bbox_xyxy": box.tolist(),
            "confidence": float(confidences[i]) if confidences[i] is not None else None,
            "label": phrases[i] if i < len(phrases) else "",
        })
    return results


def format_masks(masks: torch.Tensor, scores: torch.Tensor) -> List[Dict[str, Any]]:
    """
    Convert SAM output tensors into a plain list of dicts.

    Args:
        masks : bool tensor (N, 1, H, W)
        scores: float tensor (N, 1)
    Returns:
        List of dicts, each with:
            mask  : bool numpy array (H, W)
            score : float
    """
    results = []
    for i in range(masks.shape[0]):
        results.append({
            "mask": masks[i, 0].cpu().numpy().astype(bool),
            "score": float(scores[i, 0]) if scores is not None else None,
        })
    return results
