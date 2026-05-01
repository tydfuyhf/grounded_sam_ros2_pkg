# Pipeline Interface Design

## Architecture (final, not current)

GSAM → Qwen → Projector → MoveIt2. Strictly linear. No branching between nodes.

```
camera/image → grounded_sam_node → /grounded_sam/bundle (GSAMBundle.msg)
                                              ↓
                               + /qwen/instruction (std_msgs/String)
                                              ↓
                                         qwen_node → /qwen/result (QwenResult.msg)
                                                               ↓
                                camera/depth_image ────────────┤
                                camera/camera_info ────────────┘
                                                               ↓
                                                        projector_node
                                                               ↓
                                                    /labeled_points (PointCloud2)
                                                 /projection_result (std_msgs/String)
```

depth_image and camera_info go directly to projector — they are only used for back-projection math, no reason to pass through Qwen.

---

## Custom Messages (not yet implemented)

**GSAMBundle.msg**
```
sensor_msgs/Image    original_image     # raw RGB
sensor_msgs/Image    annotated_image    # bbox + idx numbers drawn on image
sensor_msgs/Image    mask_image         # mono8, pixel = detection idx+1, 0=background
string               detections_json
```

**QwenResult.msg**
```
sensor_msgs/Image    mask_image         # pass-through from GSAMBundle
string               labeled_detections
```

---

## Key JSON Contracts

**detections_json** (in GSAMBundle)
- idx: 0-based, mask pixel value = idx+1
- bbox_xyxy: absolute pixel coords [x1,y1,x2,y2]
```json
[{"idx":0,"label":"cup","confidence":0.91,"bbox_xyxy":[10,20,80,90]},
 {"idx":1,"label":"cup","confidence":0.87,"bbox_xyxy":[200,30,270,100]},
 {"idx":2,"label":"book","confidence":0.88,"bbox_xyxy":[5,15,100,110]},
 {"idx":3,"label":"table","confidence":0.95,"bbox_xyxy":[0,0,640,480]}]
```

**labeled_detections** (in QwenResult)
- Qwen adds "category" field only. Everything else passes through.
- Exactly one TARGET, one WORKSPACE per frame. All others → OBSTACLE.
```json
[{"idx":0,"label":"cup","category":"TARGET","confidence":0.91,"bbox_xyxy":[10,20,80,90]},
 {"idx":1,"label":"cup","category":"OBSTACLE","confidence":0.87,"bbox_xyxy":[200,30,270,100]},
 {"idx":2,"label":"book","category":"OBSTACLE","confidence":0.88,"bbox_xyxy":[5,15,100,110]},
 {"idx":3,"label":"table","category":"WORKSPACE","confidence":0.95,"bbox_xyxy":[0,0,640,480]}]
```

**projection_result** (std_msgs/String)
```json
{"target":   {"label":"cup",  "centroid":[x,y,z],"point_count":1500},
 "workspace":{"label":"table","centroid":[x,y,z],"point_count":8000},
 "obstacles":[{"label":"cup", "centroid":[x,y,z],"point_count":400},
              {"label":"book","centroid":[x,y,z],"point_count":300}]}
```

---

## Projector Category Mapping (final)

Reads category string directly from labeled_detections. No hardcoded MASK_VALUE_TO_CATEGORY.

```python
# mask pixel mv → labeled_detections[mv-1]["category"] → category ID
{"TARGET":1, "WORKSPACE":2, "OBSTACLE":3}
# mv=0 → FREE (grey) automatically
# colors: TARGET=green(0,200,80), WORKSPACE=yellow(255,220,0), OBSTACLE=red(220,40,40), FREE=grey(80,80,80)
```

---

## Qwen Reasoning

Input: annotated_image (bbox+idx numbers) + detections_json (bbox coords) + instruction text
Task: identify which idx is TARGET and which is WORKSPACE from the instruction semantics
Example: "move the cup on the book to the other side of the table"
  → cup bbox overlapping book bbox → that cup is TARGET
  → table → WORKSPACE
  → everything else → OBSTACLE

---

## Current State vs Final

| Component | Current | Final |
|---|---|---|
| grounded_sam_node | publishes individual topics | publishes GSAMBundle.msg |
| projector_node | hardcoded {1:TARGET,2:WORKSPACE} fallback | reads category from QwenResult |
| qwen_node | not implemented | stub → full Qwen API |
| GSAMBundle.msg | not defined | needs package msg definition |
| QwenResult.msg | not defined | needs package msg definition |

---

## Pending Implementation (next session work)

Custom msg is deferred. For now, stub publishes two separate std topics that mirror QwenResult fields.
projector_node.py is READ-ONLY (Gazebo demo reference). Connect via launch param override only.

### 1. postprocess.py — add idx field
File: src/grounded_sam_pkg/grounded_sam_pkg/postprocess.py
Function: format_detections(), line ~23
Change: add "idx": i to each result dict
Why: makes mask pixel → detection mapping explicit (mask pixel = idx+1)

### 2. qwen_stub_node.py — new file
File: src/grounded_sam_pkg/grounded_sam_pkg/qwen_stub_node.py
Role: bridge between GSAM and projector while real Qwen is not implemented
Subscribe:
  /grounded_sam/mask_image      → cache, pass-through to /qwen/mask_image
  /grounded_sam/detections_json → add "category" field, publish to /qwen/labeled_detections
Category logic: label.lower().strip() lookup in LABEL_TO_CATEGORY dict
  LABEL_TO_CATEGORY = {"cup": "TARGET", "table": "WORKSPACE"}  ← Gazebo demo hardcoded
  anything else → "OBSTACLE"
NOTE: label matching uses .lower().strip() to handle GSAM output variance (e.g. "Cup", " cup ")
NOTE: mask and labeled_detections are published as two separate messages with slight timing gap.
      projector caches latest detections and triggers on mask — safe for CPU inference (30-40s).
      If GPU inference is used later, add ApproximateTimeSynchronizer.
Publish:
  /qwen/mask_image         sensor_msgs/Image   (pass-through)
  /qwen/labeled_detections std_msgs/String     (JSON with "category" added)

### 3. label_mapper.py — backwards-compatible category lookup
File: src/mask_projection_pkg/mask_projection_pkg/label_mapper.py
Add above MASK_VALUE_TO_CATEGORY:
  _CATEGORY_STR_TO_ID = {"TARGET": 1, "WORKSPACE": 2, "OBSTACLE": 3}
Change apply_labels() line ~106:
  OLD: category_id = MASK_VALUE_TO_CATEGORY.get(mv, CATEGORY_OBSTACLE)
  NEW: if det has "category" field → use _CATEGORY_STR_TO_ID[det["category"]]
       else → fallback MASK_VALUE_TO_CATEGORY.get(mv, CATEGORY_OBSTACLE)
Why backwards compatible: projector_node.py (read-only) still passes old detections without
  "category" field → fallback keeps it working. stub/Qwen path uses "category" field.

### 4. setup.py — register executable
File: src/grounded_sam_pkg/setup.py
Add to console_scripts:
  'qwen_stub_node = grounded_sam_pkg.qwen_stub_node:main'

### 5. launch — connect stub to projector (no code change to projector_node.py)
projector_node.py exposes mask_topic and detections_topic as ROS2 params.
Override at launch time:
  ros2 launch mask_projection_pkg mask_projector.launch.py \
    mask_topic:=/qwen/mask_image \
    detections_topic:=/qwen/labeled_detections

### Category/FREE design (confirmed intent)
mask pixel = 0           → FREE (background GSAM did not detect) → grey
mask pixel = 1,2,3,...   → GSAM-detected objects
  "category":"TARGET"    → green
  "category":"WORKSPACE" → yellow
  "category":"OBSTACLE"  → red  (everything not TARGET or WORKSPACE)
No changes needed to this logic — MASK_VALUE_TO_CATEGORY fallback already handles OBSTACLE correctly.
