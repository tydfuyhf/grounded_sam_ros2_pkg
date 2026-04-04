#!/usr/bin/env python3
"""
Standalone inference script — no ROS2 required.

Usage:
    python scripts/run_inference.py --image /path/to/image.jpg --prompt "bottle"
    python scripts/run_inference.py --image /path/to/image.jpg --prompt "bottle, cup"
    python scripts/run_inference.py --image /path/to/image.jpg --prompt "bottle" --output /tmp/result.jpg

To change weights, edit config/model_paths.yaml (or pass --config).
"""
import argparse
import sys
from pathlib import Path

# Allow running directly without installing the package
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from grounded_sam_pkg.pipeline import GroundedSAMPipeline
from grounded_sam_pkg.prompt_adapter import PromptAdapter
from grounded_sam_pkg.postprocess import format_detections, format_masks
from grounded_sam_pkg.visualizer import draw_bboxes, draw_masks, save_result

DEFAULT_CONFIG = str(Path(__file__).resolve().parents[1] / "config" / "model_paths.yaml")
# workspace root = gsam_ws/  (script is at gsam_ws/src/grounded_sam_pkg/scripts/)
_WS_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = str(_WS_ROOT / "output" / "result.jpg")


def main():
    parser = argparse.ArgumentParser(description="Grounded SAM standalone inference")
    parser.add_argument("--image", required=True, help="Path to input image")
    parser.add_argument("--prompt", required=True, help="Object prompt, e.g. 'bottle' or 'bottle, cup'")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="Path to model_paths.yaml")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Path to save visualized result")
    args = parser.parse_args()

    adapter = PromptAdapter()
    adapted_prompt = adapter.adapt(args.prompt)
    print(f"[prompt]   '{args.prompt}'  →  '{adapted_prompt}'")

    print("[pipeline] Loading models (this takes a moment on CPU)...")
    pipeline = GroundedSAMPipeline(args.config)

    print(f"[pipeline] Running inference on: {args.image}")
    result = pipeline.run(image=args.image, prompt=adapted_prompt)

    det_list = format_detections(result["detections"], result["phrases"])

    print(f"\n[result]   {len(det_list)} detection(s):")
    for d in det_list:
        print(f"  label={d['label']!r}  conf={d['confidence']:.3f}  bbox={d['bbox_xyxy']}")

    if result["masks"] is not None:
        mask_list = format_masks(result["masks"], result["mask_scores"])
        vis = draw_bboxes(result["image_bgr"], det_list)
        vis = draw_masks(vis, mask_list)
        save_result(vis, args.output)
        print(f"\n[saved]    {args.output}")
    else:
        print("\n[result]   No detections — nothing to save.")


if __name__ == "__main__":
    main()
