# Refactoring Plan — gsam_ws

## 범위: mask_projection_pkg, grounded_sam_pkg

---

## HIGH-1: 중복 코드 제거 — `ply_utils.py` 공통 모듈 추출

### 현황
`projector_node.py`와 `multi_view_projector_node.py` 양쪽에 동일 구현 존재:

| 함수 | projector_node.py | multi_view_projector_node.py |
|---|---|---|
| `_save_ply_xyz()` | lines 202–214 | 없음 |
| `_save_ply_labeled()` | lines 217–247 | lines 395–425 (동일) |
| `_build_result_json()` | lines 250–273 (bbox 없음) | lines 361–392 (bbox_3d_world 포함) |

### 목표
`mask_projection_pkg/ply_utils.py` 신규 생성:
- `save_ply_xyz(path, points)`
- `save_ply_labeled(path, category_points)`
- `build_result_json(category_points)` — multi_view 버전 (bbox_3d_world 포함)을 표준으로

### 진행상황
- [x] `ply_utils.py` 생성
- [x] `projector_node.py` import 교체 + 중복 함수 3개 제거
- [x] `multi_view_projector_node.py` import 교체 + 중복 함수 2개 제거
- [x] 빌드 확인 (`colcon build` 1.57s 성공)

---

## MEDIUM-2: `projection_engine.py` 추출 — multi_view_projector_node.py 분리

### 현황
`multi_view_projector_node.py` 437줄 — ROS 배선 + 수학/필터 로직 혼재

### 추출 대상 (→ `projection_engine.py`)

| 함수 | 현재 위치 | 분류 |
|---|---|---|
| `_load_extrinsics()` | lines 34–57 | YAML 파싱, ROS 무관 |
| `_project_labeled()` | lines 255–278 | 수학 로직 |
| `_project_unknown()` | lines 281–303 | 수학 + 필터 |
| `_collect_seg_points()` | lines 308–345 | 마스크 처리 |
| `_filter_free_by_unknown()` | lines 348–393 | 필터 로직 |
| `_make_unknown_points()` | lines 340–357 | 포인트 생성 |

### 목표
- `projection_engine.py`: ROS 의존성 없는 순수 로직
- `multi_view_projector_node.py`: ~250줄, ROS 배선만

### 진행상황
- [x] `projection_engine.py` 생성 (157줄, ROS 의존성 없는 순수 numpy 로직)
  - `load_extrinsics` / `project_labeled` / `project_unknown`
  - `collect_seg_points` / `filter_free_by_unknown` / `_make_unknown_points`
- [x] `multi_view_projector_node.py` 슬림화: 437줄 → 280줄 (ROS 배선 + thin wrapper만)
- [x] 빌드 확인 (`colcon build` 1.54s 성공)

---

## LOW-3 (보류): grounded_sam_pkg ros_node.py 정리

`_image_callback()` 한 메서드에서 디코딩/추론/정렬/시각화/publish/파일저장 혼재.
현재 125줄, 동작 안정적 → 나중에 필요 시 진행.

---

## 완료 내역

- **HIGH-1** `ply_utils.py` 신규 생성 — `save_ply_xyz`, `save_ply_labeled`, `build_result_json` 공통화
- **HIGH-1** `projector_node.py` / `multi_view_projector_node.py` 중복 함수 5개 제거, import 교체
- **MEDIUM-2** `projection_engine.py` 신규 생성 — 6개 함수 ROS-free 분리
- **MEDIUM-2** `multi_view_projector_node.py` 437줄 → 280줄 슬림화
