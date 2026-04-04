# grounded_sam_ros2_pkg

ROS2 package that integrates [GroundingDINO](https://github.com/IDEA-Research/GroundingDINO) and [Segment Anything (SAM)](https://github.com/facebookresearch/segment-anything) for text-prompted instance segmentation on image topics.

## Requirements

- ROS2 Jazzy
- Python 3.12
- PyTorch, transformers, supervision (install via venv)

## Setup

### 1. Clone with submodules

```bash
git clone --recurse-submodules https://github.com/tydfuyhf/grounded_sam_ros2_pkg.git
cd grounded_sam_ros2_pkg
```

### 2. Download model weights

Place the following files under `models/`:

| Model | File | Download |
|---|---|---|
| GroundingDINO SwinT | `groundingdino_swint_ogc.pth` | [link](https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth) |
| SAM ViT-H | `sam_vit_h_4b8939.pth` | [link](https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth) |

```
models/
├── groundingdino_swint_ogc.pth
└── sam_vit_h_4b8939.pth
```

### 3. Build

```bash
source launch_env.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

### Real camera

```bash
source launch_env.bash
ros2 launch grounded_sam_pkg grounded_sam.launch.py \
  prompt:="bottle, cup" \
  image_topic:=/camera/image_raw
```

| Argument | Default | Description |
|---|---|---|
| `prompt` | `"object"` | Objects to detect (comma-separated) |
| `image_topic` | `/camera/image_raw` | Input image topic |
| `model_config` | `config/model_paths.yaml` | Path to model config |

### Test with static image

```bash
source launch_env.bash
ros2 launch grounded_sam_pkg test_inference.launch.py prompt:="glass, chair, person"
```

Publishes a test image from `test_image_pub.py` and runs inference on it. To change the test image, edit `IMAGE_PATH` at the top of [src/grounded_sam_pkg/grounded_sam_pkg/test_image_pub.py](src/grounded_sam_pkg/grounded_sam_pkg/test_image_pub.py).

## Changing model weights

Edit [`src/grounded_sam_pkg/config/model_paths.yaml`](src/grounded_sam_pkg/config/model_paths.yaml):

```yaml
grounding_dino:
  config_file: "/path/to/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
  checkpoint: "/path/to/models/groundingdino_swint_ogc.pth"
  box_threshold: 0.35
  text_threshold: 0.25
  device: "cpu"   # or "cuda"

sam:
  model_type: "vit_h"   # vit_h | vit_l | vit_b
  checkpoint: "/path/to/models/sam_vit_h_4b8939.pth"
  device: "cpu"   # or "cuda"
```

### SAM model variants

| `model_type` | Checkpoint file | Size |
|---|---|---|
| `vit_h` | `sam_vit_h_4b8939.pth` | 2.4 GB |
| `vit_l` | `sam_vit_l_0b3195.pth` | 1.2 GB |
| `vit_b` | `sam_vit_b_01ec64.pth` | 375 MB |

SAM checkpoints: https://github.com/facebookresearch/segment-anything#model-checkpoints
