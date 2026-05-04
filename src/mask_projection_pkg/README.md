# mask_projection_pkg

2D 세그멘테이션 마스크 + Depth 이미지 → **라벨링된 3D PointCloud2** 변환 패키지.

---

## 노드 구성

| 노드 | 역할 | 용도 |
|---|---|---|
| `mask_projector_node` | 단일 카메라 → `/labeled_points` | Gazebo 단일 카메라 데모 (수정 금지) |
| `multi_view_projector_node` | Top + EE 두 카메라 → `/world_map` | Isaac Sim 연동 대상 노드 |

---

## 모듈 구성

```
mask_projection_pkg/
├── back_projection.py            depth 이미지 → 카메라 좌표계 3D 포인트 (수학 로직)
├── label_mapper.py               마스크 픽셀값 → 시맨틱 카테고리 + 색상
├── cloud_builder.py              CategoryPoints → PointCloud2 메시지 패킹
├── projector_node.py             단일 카메라 노드 (Gazebo 데모, 수정 금지)
└── multi_view_projector_node.py  두 카메라 월드 맵 노드
config/
└── camera_extrinsics.yaml        카메라 외부 파라미터 (R/t 행렬)
launch/
├── mask_projector.launch.py         단일 카메라 launch (수정 금지)
└── multi_view_projector.launch.py   두 카메라 launch
```

---

## 시맨틱 카테고리

| ID | 이름 | 색상 (RGB) | 출처 | 의미 |
|---|---|---|---|---|
| 0 | FREE | (80, 80, 80) 회색 | EE 뷰 GSAM 비탐지 픽셀 | 배경 |
| 1 | TARGET | (0, 200, 80) 초록 | EE 뷰 GSAM | 잡을 물체 |
| 2 | WORKSPACE | (255, 220, 0) 노랑 | EE 뷰 GSAM | 작업 테이블 |
| 3 | OBSTACLE | (220, 40, 40) 빨강 | EE 뷰 GSAM | 장애물 |
| 4 | UNKNOWN | (150, 80, 200) 보라 | Top 뷰 전체 | 분류 안 된 씬 기하 |

RViz2 에서 `Color Transformer → RGB8` 로 설정하면 카테고리별 색상이 표시됩니다.

---

## PointCloud2 메시지 포맷

`cloud_builder.py` 가 생성하는 포인트 레이아웃 (point_step = 20 bytes):

| offset | 필드 | 타입 | 설명 |
|---|---|---|---|
| 0 | x | float32 | X 좌표 (m) |
| 4 | y | float32 | Y 좌표 (m) |
| 8 | z | float32 | Z 좌표 (m) |
| 12 | rgb | float32 | PCL packed RGB `(r<<16\|g<<8\|b)` |
| 16 | category | uint8 | 카테고리 ID |

---

## 노드 — multi_view_projector_node

Isaac Sim 연동 대상 노드입니다.

### 동작 방식

```
EE 카메라  depth + GSAM 마스크  →  TARGET / WORKSPACE / OBSTACLE / FREE
  └─ TARGET 3D bbox (world) 계산 (±5 cm margin)
Top 카메라 depth only           →  UNKNOWN (보라), TARGET bbox 내부 제거
두 뷰 world frame 변환 후 병합  →  /world_map  (frame_id="world")
```

- **EE 먼저 처리** → TARGET bbox 계산 → Top 포인트에서 TARGET 영역 필터링  
  (octomap에서 TARGET이 UNKNOWN으로 이중 등록되는 것 방지)
- Top 카메라가 없으면 경고 후 EE 뷰만으로 계속 동작

### 구독 토픽

| 토픽 | 타입 | 역할 |
|---|---|---|
| `<top_depth_topic>` | `sensor_msgs/Image` | Top 카메라 Depth (32FC1, m) |
| `<top_camera_info_topic>` | `sensor_msgs/CameraInfo` | Top 카메라 내부 파라미터 |
| `<ee_depth_topic>` | `sensor_msgs/Image` | EE 카메라 Depth (32FC1, m) |
| `<ee_camera_info_topic>` | `sensor_msgs/CameraInfo` | EE 카메라 내부 파라미터 |
| `<detections_topic>` | `std_msgs/String` | GSAM/Qwen 감지 결과 JSON — 캐시 |
| `<mask_topic>` | `sensor_msgs/Image` | GSAM/Qwen 마스크 — **트리거** |

### 발행 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/world_map` | `sensor_msgs/PointCloud2` | 월드 좌표계 라벨링 포인트클라우드 |
| `/world_map_result` | `std_msgs/String` | 카테고리별 요약 JSON |

### /world_map_result JSON 포맷

```json
{
  "target":    {"label": "cup",   "centroid": [x, y, z],
                "bbox_3d_world": {"min": [x,y,z], "max": [x,y,z]},
                "point_count": 1500},
  "workspace": {"label": "table", "centroid": [x, y, z],
                "bbox_3d_world": {"min": [x,y,z], "max": [x,y,z]},
                "point_count": 8000},
  "unknown":   {"label": "unknown", "centroid": [x, y, z],
                "bbox_3d_world": {"min": [x,y,z], "max": [x,y,z]},
                "point_count": 50000}
}
```

`centroid` 와 `bbox_3d_world` 모두 world frame 좌표입니다.

### ROS 2 파라미터 전체 목록

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `top_depth_topic` | `/top_camera/depth_image` | Top 카메라 depth |
| `top_camera_info_topic` | `/top_camera/camera_info` | Top 카메라 info |
| `ee_depth_topic` | `/ee_camera/depth_image` | EE 카메라 depth |
| `ee_camera_info_topic` | `/ee_camera/camera_info` | EE 카메라 info |
| `mask_topic` | `/grounded_sam/mask_image` | 마스크 트리거 토픽 |
| `detections_topic` | `/grounded_sam/detections_json` | 감지 결과 토픽 |
| `output_cloud_topic` | `/world_map` | 출력 PointCloud2 |
| `output_result_topic` | `/world_map_result` | 출력 결과 JSON |
| `min_depth` | `0.05` | 유효 depth 최솟값 (m) |
| `max_depth` | `15.0` | 유효 depth 최댓값 (m) |
| `initials` | `''` | PLY 파일명 접두사 |
| `extrinsics_config` | (설치된 YAML 경로) | 카메라 외부 파라미터 YAML |

### 실행 (Qwen stub 사용 시)

```bash
ros2 launch mask_projection_pkg multi_view_projector.launch.py \
  mask_topic:=/qwen/mask_image \
  detections_topic:=/qwen/labeled_detections
```

---

## 카메라 외부 파라미터 (config/camera_extrinsics.yaml)

카메라 좌표계 → 월드 좌표계 변환 행렬을 YAML로 관리합니다.

**변환 규칙:** `p_world = R @ p_cam + t`

- `p_cam` : `back_projection.py` 출력 (ROS camera optical frame: +Z forward, +X right, +Y down)
- `R` (3×3): camera optical frame → world frame 회전
- `t` (3,) : camera optical center의 world frame 좌표 (m)

```yaml
ee_camera:
  t: [0.5, 0.0, 2.5]
  R:
    - [ 0.0,  0.435, -0.900]
    - [ 1.0,  0.0,    0.0  ]
    - [ 0.0, -0.900, -0.435]

top_camera:
  t: [-3.0, 2.0, 2.5]
  R:
    - [ 0.0, -1.0,  0.0]
    - [-1.0,  0.0,  0.0]
    - [ 0.0,  0.0, -1.0]
```

### Isaac Sim 전환 방법

1. USD stage에서 각 카메라 prim 선택 → Property 패널 → World Transform 4×4 행렬 읽기
2. 좌상단 3×3 블록 = `R`, 우상단 3×1 열 = `t` (단위: meters)
3. 새 YAML 작성 후 launch 인자로 경로 전달:

```bash
ros2 launch mask_projection_pkg multi_view_projector.launch.py \
  extrinsics_config:=/path/to/camera_extrinsics_isaac.yaml \
  top_depth_topic:=/isaac/top/depth_image \
  top_camera_info_topic:=/isaac/top/camera_info \
  ee_depth_topic:=/isaac/ee/depth_image \
  ee_camera_info_topic:=/isaac/ee/camera_info \
  mask_topic:=/qwen/mask_image \
  detections_topic:=/qwen/labeled_detections
```

---

## 카테고리 할당 방식

### 현재 (qwen_stub 경로)

`qwen_stub_node` 가 `detections_json` 의 `label` 필드를 보고 `category` 필드를 추가합니다.  
`apply_labels()` 에서 `category` 필드가 있으면 해당 값을 사용하고, 없으면 mask pixel 순서 기반 fallback을 사용합니다.

### Qwen VLM 연동 시

`qwen_stub_node` 를 실제 Qwen API 호출로 교체합니다.  
발행 토픽 이름과 `labeled_detections` JSON 포맷을 유지하면 이 패키지 수정 없이 연동됩니다.

---

## 출력 파일

추론 실행 시 `~/gsam_ws/output/` 에 자동 저장됩니다.

| 파일 | 설명 |
|---|---|
| `world_map_{initials}_{stamp}.ply` | 월드 좌표계 라벨링 포인트클라우드 (XYZ + RGB + category) |

PLY 파일은 MeshLab, Open3D, CloudCompare 등으로 열 수 있습니다.

---

## 설계 메모

**왜 ApproximateTimeSynchronizer를 쓰지 않나?**  
GSAM CPU 추론이 프레임당 30~40초 소요됩니다. mask_image 도착 시점에 depth 큐가 한참 앞으로 나가 있어 타임스탬프 매칭이 실패합니다. 대신 depth/camera_info를 최신값으로 캐시하고 mask_image 수신을 트리거로 사용합니다. GPU 추론으로 전환하면 `ApproximateTimeSynchronizer` 교체를 권장합니다.

**FREE vs UNKNOWN 구분 이유**  
EE 뷰의 비탐지 픽셀(FREE)과 Top 뷰 전체(UNKNOWN)를 구분합니다. octomap_server에 넣을 때는 OBSTACLE + UNKNOWN만 사용하고 FREE는 제외하는 것을 권장합니다 (FREE를 포함하면 배경 픽셀이 occupied voxel로 마킹됨).
