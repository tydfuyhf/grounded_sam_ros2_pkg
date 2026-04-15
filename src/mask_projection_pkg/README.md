# mask_projection_pkg

2D 세그멘테이션 마스크 + Depth 이미지 → **라벨링된 3D PointCloud2** 변환 패키지.

두 가지 노드를 제공합니다.

| 노드 | 설명 | 카메라 수 |
|---|---|---|
| `mask_projector_node` | 단일 카메라, Gazebo 데모용 (수정 금지) | 1 |
| `multi_view_projector_node` | 두 카메라 (Top + EE) → 월드 좌표 PointCloud2 | 2 |

---

## 모듈 구성

```
mask_projection_pkg/
├── back_projection.py          depth 이미지 → 카메라 좌표계 3D 포인트 (수학 로직만)
├── label_mapper.py             마스크 픽셀값 → 시맨틱 카테고리 + 색상
├── cloud_builder.py            CategoryPoints → PointCloud2 메시지 패킹
├── projector_node.py           단일 카메라 ROS 2 노드 (데모, 수정 금지)
└── multi_view_projector_node.py  두 카메라 월드 맵 ROS 2 노드
launch/
├── mask_projector.launch.py         단일 카메라 launch
└── multi_view_projector.launch.py   두 카메라 launch
```

---

## 시맨틱 카테고리

마스크 픽셀값 → 카테고리 매핑은 `label_mapper.py` 에 정의됩니다.

| 카테고리 ID | 이름 | 색상 (RGB) | 의미 |
|---|---|---|---|
| 0 | FREE | (80, 80, 80) 회색 | 배경 / 빈 공간 |
| 1 | TARGET | (0, 200, 80) 초록 | 잡을 물체 (prompt 첫 번째) |
| 2 | WORKSPACE | (255, 220, 0) 노랑 | 작업 테이블 (prompt 두 번째) |
| 3 | OBSTACLE | (220, 40, 40) 빨강 | 그 외 감지된 물체 |

마스크 픽셀값 = `detections_json` 의 1-based 인덱스.  
pixel=1 → TARGET, pixel=2 → WORKSPACE, pixel=3+ → OBSTACLE.

> **Qwen 연동 예정:** Qwen VLM이 어떤 마스크 ID가 실제 TARGET/WORKSPACE인지 결정하면  
> `label_mapper.py` 의 `MASK_VALUE_TO_CATEGORY` 매핑만 교체하면 됩니다.  
> 다운스트림(cloud_builder, projector 노드)은 변경 불필요.

---

## PointCloud2 메시지 포맷

`cloud_builder.py` 가 생성하는 포인트 레이아웃 (20 bytes/point):

| offset | 필드 | 타입 | 설명 |
|---|---|---|---|
| 0 | x | float32 | X 좌표 (m) |
| 4 | y | float32 | Y 좌표 (m) |
| 8 | z | float32 | Z 좌표 (m) |
| 12 | rgb | float32 | PCL 방식 packed RGB `(r<<16\|g<<8\|b)` |
| 16 | category | uint8 | 카테고리 ID (0=FREE, 1=TARGET, 2=WS, 3=OBS) |

RViz2 에서 `Color Transformer → RGB8` 로 설정하면 카테고리별 색상이 표시됩니다.  
`category` 필드를 기준으로 필터링하면 MoveIt2 충돌 맵 또는 goal pose 추출에 활용 가능합니다.

---

## 노드 1 — `mask_projector_node` (단일 카메라, Gazebo 데모)

> **이 노드와 launch 파일은 수정하지 마세요.** Gazebo 데모 기준 레퍼런스입니다.

### 구독 토픽

| 토픽 | 타입 | 역할 |
|---|---|---|
| `/rgbd_camera/depth_image` | `sensor_msgs/Image` | Depth (32FC1, m) |
| `/rgbd_camera/camera_info` | `sensor_msgs/CameraInfo` | 카메라 내부 파라미터 |
| `/grounded_sam/mask_image` | `sensor_msgs/Image` | 세그멘테이션 마스크 → **트리거** |
| `/grounded_sam/detections_json` | `std_msgs/String` | GSAM 감지 결과 JSON |

### 발행 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/labeled_points` | `sensor_msgs/PointCloud2` | 라벨링된 포인트클라우드 |
| `/projection_result` | `std_msgs/String` | 카테고리별 centroid JSON |

### 실행

```bash
ros2 launch mask_projection_pkg mask_projector.launch.py \
  initials:=tc
```

---

## 노드 2 — `multi_view_projector_node` (두 카메라, 월드 맵)

Top 카메라 + EE 카메라 두 뷰를 **월드 좌표계**로 변환 후 병합합니다.

- **Top 카메라:** depth만 사용 → 전 포인트 FREE (씬 형상 제공)
- **EE 카메라:** depth + GSAM 마스크 → TARGET / WORKSPACE / OBSTACLE / FREE (시맨틱 제공)

GSAM은 EE 카메라에서만 실행합니다 (CPU 추론 속도 한계).

### 구독 토픽

| 토픽 | 타입 | 역할 |
|---|---|---|
| `/top_camera/depth_image` | `sensor_msgs/Image` | Top 카메라 Depth |
| `/top_camera/camera_info` | `sensor_msgs/CameraInfo` | Top 카메라 내부 파라미터 |
| `/ee_camera/depth_image` | `sensor_msgs/Image` | EE 카메라 Depth |
| `/ee_camera/camera_info` | `sensor_msgs/CameraInfo` | EE 카메라 내부 파라미터 |
| `/grounded_sam/mask_image` | `sensor_msgs/Image` | GSAM 마스크 → **트리거** |
| `/grounded_sam/detections_json` | `std_msgs/String` | GSAM 감지 결과 JSON |

Top 카메라가 없으면 경고 후 EE 뷰만으로 계속 동작합니다.

### 발행 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/world_map` | `sensor_msgs/PointCloud2` | 월드 좌표계 라벨링 포인트클라우드 |
| `/world_map_result` | `std_msgs/String` | 카테고리별 centroid JSON (월드 좌표) |

### 실행

```bash
# GSAM — EE 카메라 RGB 스트림으로 실행 (image_topic 오버라이드 필수)
ros2 launch grounded_sam_pkg grounded_sam.launch.py \
  image_topic:=/ee_camera/image \
  prompt:="cup, table, object"

# Multi-view projector
ros2 launch mask_projection_pkg multi_view_projector.launch.py \
  initials:=cto
```

---

## 카메라 → 월드 좌표 변환 설정 (TODO)

`multi_view_projector_node.py` 상단의 변환 행렬을 실제 값으로 교체해야 합니다.

**변환 규칙:**
```
p_world = R @ p_cam + t
```

| 변수 | 의미 | 현재 값 |
|---|---|---|
| `_R_TOP` | Top 카메라 좌표계 → 월드 좌표계 회전 (3×3) | `np.eye(3)` (placeholder) |
| `_t_TOP` | Top 카메라 원점의 월드 좌표 (m) | `np.zeros(3)` (placeholder) |
| `_R_EE` | EE 카메라 좌표계 → 월드 좌표계 회전 (home pose 기준, 3×3) | `np.eye(3)` (placeholder) |
| `_t_EE` | EE 카메라 원점의 월드 좌표 (home pose 기준, m) | `np.zeros(3)` (placeholder) |

### Isaac Sim에서 측정하는 방법

1. Isaac Sim USD stage를 엽니다.
2. 카메라 prim(예: `/World/top_camera`)을 선택합니다.
3. Property 패널 → **World Transform** 4×4 행렬을 확인합니다.
4. 좌상단 3×3 블록 = `R` (카메라 X/Y/Z 축의 월드 좌표 표현)
5. 우상단 3×1 열 = `t` (카메라 원점의 월드 좌표, meters)
6. `multi_view_projector_node.py` 의 `_R_TOP/_t_TOP` 에 붙여넣습니다.
7. EE 카메라도 동일하게 반복 (home joint configuration 상태에서).

**예시 (실제 값으로 교체하세요):**
```python
# top camera: 월드 원점 정중앙 위 1.5 m, 아래를 바라보는 경우
_R_TOP = np.array([
    [ 1.0,  0.0,  0.0],
    [ 0.0,  0.0,  1.0],
    [ 0.0, -1.0,  0.0],
])
_t_TOP = np.array([0.0, 0.0, 1.5])
```

---

## Isaac Sim 씬 구성 및 실행 (현재 타겟 씬)

### 씬 설명

테이블 위에 4개 물체:

| 물체 | 색상 | 카테고리 | 이유 |
|---|---|---|---|
| glass cup (유리컵) | 초록 — TARGET | 잡을 대상 | 이번 작업의 grasp 목표물 |
| table (테이블) | 노랑 — WORKSPACE | 작업 공간 | 조작이 이루어지는 표면 |
| red ball (빨간 공) | 빨강 — OBSTACLE | 장애물 | 충돌 회피 대상 |
| blue cube (파란 큐브) | 빨강 — OBSTACLE | 장애물 | 충돌 회피 대상 |
| book (책) | 빨강 — OBSTACLE | 장애물 | 충돌 회피 대상 |

### 실행 명령

```bash
# 터미널 1 — GSAM (EE 카메라 RGB 스트림, 유리컵을 prompt 첫 번째로)
ros2 launch grounded_sam_pkg grounded_sam.launch.py \
  image_topic:=/ee_camera/image \
  prompt:="glass cup, table, red ball, blue cube, book"

# 터미널 2 — Multi-view projector
ros2 launch mask_projection_pkg multi_view_projector.launch.py \
  initials:=gtrcb
```

**prompt 순서가 카테고리를 결정합니다:**
- 1번 `glass cup` → TARGET (초록)
- 2번 `table` → WORKSPACE (노랑)
- 3번 이후 `red ball`, `blue cube`, `book` → OBSTACLE (빨강)

### RViz2에서 확인하는 방법

1. `PointCloud2` 디스플레이 추가 → Topic: `/world_map`
2. `Color Transformer` → `RGB8` 선택
3. 씬 결과 예시:

```
초록 포인트  →  유리컵 위치 (TARGET)
노랑 포인트  →  테이블 표면 (WORKSPACE)
빨강 포인트  →  공 / 큐브 / 책 (OBSTACLE)
회색 포인트  →  배경 / 빈 공간 (FREE)
```

projection이 제대로 이루어졌다면 각 물체의 3D 형상이 해당 색상으로 표시됩니다.

---

## ⚠️ 카테고리 할당 방식: 환경 통합 시 반드시 수정 필요

### 현재 방식 — prompt 순서 기반 (임시)

현재 카테고리 할당은 **GSAM prompt의 단어 순서**에만 의존합니다.

```python
# label_mapper.py
MASK_VALUE_TO_CATEGORY = {
    1: CATEGORY_TARGET,     # prompt 첫 번째 단어가 감지된 마스크
    2: CATEGORY_WORKSPACE,  # prompt 두 번째 단어가 감지된 마스크
    # 3+ → OBSTACLE
}
```

즉, `prompt:="glass cup, table, ..."` 로 실행했을 때 GSAM이 유리컵을 첫 번째로 감지했다고 가정하는 방식입니다. GSAM 감지 결과 순서가 항상 prompt 순서와 일치한다는 보장이 없어 틀릴 수 있습니다.

### Isaac Sim 환경 통합 시 수정할 부분 — JSON 기반 매핑으로 교체

환경 통합 시점에 `detections_json` 의 `label` 필드를 기준으로 카테고리를 직접 매핑하는 방식으로 교체해야 합니다.

**수정 위치: `label_mapper.py` 또는 `multi_view_projector_node.py` 의 `_mask_cb`**

```python
# 환경 통합 시 적용할 JSON 기반 매핑 예시
# detections_json 예시:
# [
#   {"label": "red ball",   "confidence": 0.91, "bbox_xyxy": [...]},
#   {"label": "glass cup",  "confidence": 0.87, "bbox_xyxy": [...]},
#   {"label": "table",      "confidence": 0.95, "bbox_xyxy": [...]},
#   {"label": "blue cube",  "confidence": 0.82, "bbox_xyxy": [...]}
# ]
#
# label → category 딕셔너리를 detections_json에서 동적으로 생성:
LABEL_TO_CATEGORY = {
    "glass cup": CATEGORY_TARGET,
    "table":     CATEGORY_WORKSPACE,
    # 그 외 모든 label → CATEGORY_OBSTACLE (apply_labels의 fallback 활용)
}
```

`apply_labels()` 에 이 딕셔너리를 넘기도록 인터페이스를 확장하거나,  
`_mask_cb` 안에서 `detections_json` 을 받아 `MASK_VALUE_TO_CATEGORY` 를 동적으로 재구성하면 됩니다.  
`cloud_builder.py` 와 다운스트림 노드는 변경 불필요합니다.

> 이 작업은 Isaac Sim 환경 통합 시점에 진행합니다. 현재는 prompt 순서 기반으로 먼저 동작 검증을 합니다.

---

## Isaac Sim 토픽 오버라이드

모든 토픽 이름은 ROS 2 파라미터로 노출되어 있습니다.  
launch 파일에서 오버라이드만 하면 코드 변경 없이 Isaac Sim에 연결됩니다.

```bash
ros2 launch mask_projection_pkg multi_view_projector.launch.py \
  top_depth_topic:=/isaac/top/depth_image \
  top_camera_info_topic:=/isaac/top/camera_info \
  ee_depth_topic:=/isaac/ee/depth_image \
  ee_camera_info_topic:=/isaac/ee/camera_info
```

### 파라미터 전체 목록

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `top_depth_topic` | `/top_camera/depth_image` | Top 카메라 depth 토픽 |
| `top_camera_info_topic` | `/top_camera/camera_info` | Top 카메라 info 토픽 |
| `ee_depth_topic` | `/ee_camera/depth_image` | EE 카메라 depth 토픽 |
| `ee_camera_info_topic` | `/ee_camera/camera_info` | EE 카메라 info 토픽 |
| `mask_topic` | `/grounded_sam/mask_image` | GSAM 마스크 (트리거) |
| `detections_topic` | `/grounded_sam/detections_json` | GSAM 감지 결과 |
| `output_cloud_topic` | `/world_map` | 출력 PointCloud2 |
| `output_result_topic` | `/world_map_result` | 출력 결과 JSON |
| `min_depth` | `0.05` | 유효 depth 최솟값 (m) |
| `max_depth` | `15.0` | 유효 depth 최댓값 (m) |
| `initials` | `''` | 저장 PLY 파일명 접두사 |

---

## 출력 파일

추론 실행 시 `~/gsam_ws/output/` 에 자동 저장됩니다.

| 파일 | 노드 | 설명 |
|---|---|---|
| `cloud_original_{initials}_{stamp}.ply` | `mask_projector_node` | 원본 XYZ 포인트클라우드 |
| `cloud_labeled_{initials}_{stamp}.ply` | `mask_projector_node` | 라벨링된 포인트클라우드 |
| `world_map_{initials}_{stamp}.ply` | `multi_view_projector_node` | 월드 좌표계 라벨링 포인트클라우드 |

PLY 파일은 MeshLab, Open3D, CloudCompare 등으로 열 수 있습니다.

---

## 설계 메모

**왜 ApproximateTimeSynchronizer를 쓰지 않나?**

GSAM이 CPU에서 프레임당 30~40초 소요됩니다. mask_image가 도착할 시점에는 depth 큐가 한참 앞으로 나가 있어 타임스탬프 매칭이 실패합니다. 대신 depth/camera_info를 "최신값"으로 캐시하고 mask_image 도착을 트리거로 삼습니다. GPU 추론(Isaac Sim 환경)으로 전환하면 `ApproximateTimeSynchronizer` 로 교체하는 것이 좋습니다.

**왜 Top 카메라에는 GSAM을 실행하지 않나?**

CPU 추론 속도 한계로 두 뷰를 동시에 처리하기 어렵습니다. Top 카메라는 씬 형상(FREE 포인트)을 제공하고, EE 카메라가 시맨틱 라벨을 담당합니다. 추후 GPU 환경에서 Top 뷰 GSAM이 필요하다면 별도 브랜치/스크립트로 추가합니다.

**QoS**

모든 구독자는 VOLATILE QoS(depth=10)를 사용합니다. Gazebo bridge 및 Isaac Sim bridge 모두 VOLATILE로 발행하기 때문에 TRANSIENT_LOCAL로 구독하면 `incompatible QoS` 경고와 함께 메시지를 수신하지 못합니다.

---

## 향후 계획

- **Qwen VLM 연동:** GSAM 감지 결과 → Qwen이 TARGET/WORKSPACE 결정 → `label_mapper.py` 의 카테고리 매핑 교체. 다운스트림 코드 변경 없음.
- **MoveIt2 연동:** `/world_map_result` 의 TARGET centroid → goal pose, OBSTACLE 포인트 → 충돌 맵.
- **Isaac Sim 연결:** launch 파일 토픽 파라미터 오버라이드만으로 전환 가능 (동료 TODO).
