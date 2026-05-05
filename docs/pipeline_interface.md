# Pipeline Interface

## 전체 파이프라인 (현재 구현 기준)

```
Gazebo / Isaac Sim
  ee_camera  (front-view)  ← GSAM RGB 입력
  top_camera (overhead)    ← depth only
        │
        ▼
  grounded_sam_node
    GroundingDINO → bounding box
    SAM (ViT-B)   → segmentation mask
        ├─▶ /grounded_sam/mask_image       (mono8, pixel = 1-based idx)
        ├─▶ /grounded_sam/detections_json  (JSON array)
        └─▶ /grounded_sam/annotated_image
        │
        ▼
  qwen_stub_node  (← 실제 Qwen VLM으로 교체 예정)
    label 텍스트 기반으로 category 할당
    cup → TARGET / table → WORKSPACE / 그 외 → OBSTACLE
        ├─▶ /qwen/mask_image           (pass-through)
        └─▶ /qwen/labeled_detections   (category 필드 추가된 JSON)
        │
        ▼
  multi_view_projector_node
    EE depth  + GSAM 마스크  →  TARGET / WORKSPACE / OBSTACLE / FREE
      └─ TARGET 3D bbox 계산 (world frame, ±5 cm margin)
    Top depth               →  UNKNOWN (보라), TARGET bbox 내부 제거
    두 뷰 world frame 변환 후 병합
        ├─▶ /world_map        (PointCloud2, frame_id="world")
        └─▶ /world_map_result (JSON: centroid + bbox_3d_world per category)
        │
        ▼
      RViz2 / MoveIt2 (octomap_server)
    회색→FREE  초록→TARGET  노랑→WORKSPACE  빨강→OBSTACLE  보라→UNKNOWN
```

---

## JSON 계약

### detections_json  (`/grounded_sam/detections_json`)

mask pixel 값 = 배열의 1-based 인덱스 (`idx=0` → pixel=1).

```json
[
  {"idx": 0, "label": "cup",   "confidence": 0.91, "bbox_xyxy": [10, 20, 80, 90]},
  {"idx": 1, "label": "table", "confidence": 0.95, "bbox_xyxy": [0, 0, 640, 480]},
  {"idx": 2, "label": "object","confidence": 0.82, "bbox_xyxy": [150, 60, 220, 130]}
]
```

### labeled_detections  (`/qwen/labeled_detections`)

`detections_json` 에 `category` 필드만 추가. 나머지는 pass-through.

```json
[
  {"idx": 0, "label": "cup",   "category": "TARGET",    "confidence": 0.91, "bbox_xyxy": [10, 20, 80, 90]},
  {"idx": 1, "label": "table", "category": "WORKSPACE", "confidence": 0.95, "bbox_xyxy": [0, 0, 640, 480]},
  {"idx": 2, "label": "object","category": "OBSTACLE",  "confidence": 0.82, "bbox_xyxy": [150, 60, 220, 130]}
]
```

`category` 값: `"TARGET"` / `"WORKSPACE"` / `"OBSTACLE"`

### world_map_result  (`/world_map_result`)

카테고리별 centroid, 3D bounding box, 포인트 수. 모두 world frame 좌표.

```json
{
  "target": {
    "label": "cup",
    "centroid": [x, y, z],
    "bbox_3d_world": {"min": [x, y, z], "max": [x, y, z]},
    "point_count": 1500
  },
  "workspace": {
    "label": "table",
    "centroid": [x, y, z],
    "bbox_3d_world": {"min": [x, y, z], "max": [x, y, z]},
    "point_count": 8000
  },
  "object": {
    "label": "object",
    "centroid": [x, y, z],
    "bbox_3d_world": {"min": [x, y, z], "max": [x, y, z]},
    "point_count": 400
  },
  "free": {
    "label": "free",
    "centroid": [x, y, z],
    "bbox_3d_world": {"min": [x, y, z], "max": [x, y, z]},
    "point_count": 20000
  },
  "unknown": {
    "label": "unknown",
    "centroid": [x, y, z],
    "bbox_3d_world": {"min": [x, y, z], "max": [x, y, z]},
    "point_count": 50000
  }
}
```

---

## 카테고리 매핑

| category 문자열 | category ID | 색상 (RGB) | 설명 |
|---|---|---|---|
| — | 0 FREE | (80, 80, 80) 회색 | EE 뷰 비탐지 배경 픽셀 |
| TARGET | 1 | (0, 200, 80) 초록 | 잡을 물체 |
| WORKSPACE | 2 | (255, 220, 0) 노랑 | 작업 테이블 |
| OBSTACLE | 3 | (220, 40, 40) 빨강 | 장애물 |
| — | 4 UNKNOWN | (150, 80, 200) 보라 | Top 뷰 기하 (분류 미적용) |

---

## 토픽 전체 목록

| 토픽 | 타입 | 발행 노드 |
|---|---|---|
| `/ee_camera/image` | `sensor_msgs/Image` | Gazebo / Isaac Sim |
| `/ee_camera/depth_image` | `sensor_msgs/Image` | Gazebo / Isaac Sim |
| `/ee_camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo / Isaac Sim |
| `/top_camera/depth_image` | `sensor_msgs/Image` | Gazebo / Isaac Sim |
| `/top_camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo / Isaac Sim |
| `/grounded_sam/mask_image` | `sensor_msgs/Image` | grounded_sam_node |
| `/grounded_sam/detections_json` | `std_msgs/String` | grounded_sam_node |
| `/grounded_sam/annotated_image` | `sensor_msgs/Image` | grounded_sam_node |
| `/qwen/mask_image` | `sensor_msgs/Image` | qwen_stub_node |
| `/qwen/labeled_detections` | `std_msgs/String` | qwen_stub_node |
| `/world_map` | `sensor_msgs/PointCloud2` | multi_view_projector_node |
| `/world_map_result` | `std_msgs/String` | multi_view_projector_node |

---

## 현재 구현 상태

| 컴포넌트 | 상태 | 비고 |
|---|---|---|
| `grounded_sam_node` | 완료 | GroundingDINO + SAM ViT-B |
| `qwen_stub_node` | 완료 | label 기반 category 할당 임시 구현 |
| `multi_view_projector_node` | 완료 | EE+Top 융합, UNKNOWN, TARGET bbox 필터링 |
| `camera_extrinsics.yaml` | 완료 | Gazebo 기준값 설정됨, Isaac Sim 전환 시 YAML 교체 |
| Qwen VLM 연동 | 미구현 | qwen_stub_node 교체 예정 |
| MoveIt2 연동 | 미구현 | `/world_map_result` TARGET centroid → goal pose |

---

## Isaac Sim 전환 체크리스트

1. `camera_extrinsics.yaml` 복사 후 USD stage World Transform 값으로 채우기
2. `multi_view_projector.launch.py` 실행 시 `extrinsics_config` + 토픽 오버라이드
3. `grounded_sam.launch.py` 실행 시 `image_topic:=/isaac/ee/image` 오버라이드
4. 카테고리 뒤바뀜 확인 → 문제 시 `qwen_stub_node` 의 `LABEL_TO_CATEGORY` 수정
5. GPU 추론 가능하면 `ApproximateTimeSynchronizer` 도입 검토
