# VGN Integration Plan (Franka Panda + Grasp Detection)

**VGN GitHub:** https://github.com/ethz-asl/vgn  
**G-SAM GitHub:** https://github.com/tydfuyhf/grounded_sam_ros2_perception

---

## Background — Why VGN

Contact-GraspNet failed on RTX 3090 (CUDA 12.2) due to CUDA kernel hang.
TF 2.2 was built for CUDA 10.1, incompatible with Ampere (sm_86).
Switched to VGN (PyTorch-based). GIGA (successor) also considered but requires 100GB+ retraining data — VGN adopted.

---

## Existing Pipeline (already implemented)

```
src/
  grounded_sam_pkg/    GSAM inference + qwen_stub node
  mask_projection_pkg/ Projection node — labeled PointCloud2
  rgbd_projection/     Gazebo sim + RViz (demo only, do not modify)
external/
  GroundingDINO/
  segment-anything/
```

### Full data flow (existing)

```
Gazebo / Isaac Sim
  /ee_camera/{image, depth_image, camera_info}
  /top_camera/{depth_image, camera_info}
        ↓
grounded_sam_node  (EE RGB only)
  trigger: /ee_camera/image
  → /grounded_sam/mask_image        (mono8, pixel = 1-based idx)
  → /grounded_sam/detections_json   (JSON array)
  → /grounded_sam/annotated_image
        ↓
qwen_stub_node  (temporary — replace with real Qwen VLM later)
  cache:   /grounded_sam/detections_json
  trigger: /grounded_sam/mask_image
  → /qwen/mask_image          (pass-through)
  → /qwen/labeled_detections  (JSON + "category" field)
        ↓
multi_view_projector_node
  cache:   /ee_camera/depth_image, /ee_camera/camera_info
           /top_camera/depth_image, /top_camera/camera_info
           /qwen/labeled_detections
  trigger: /qwen/mask_image
  → /world_map        (labeled PointCloud2, frame="world")
  → /world_map_result (JSON: centroid, bbox_3d_world, point_count per category)
```

### /world_map PointCloud2 format (cloud_builder.py, 20 bytes/point)

| field    | type    | description |
|----------|---------|-------------|
| x, y, z  | float32 | world frame XYZ (metres) |
| rgb      | float32 | packed uint32: r<<16\|g<<8\|b |
| category | uint8   | 0=FREE 1=TARGET 2=WORKSPACE 3=OBSTACLE 4=UNKNOWN |

Category notes:
- FREE(0): EE view background pixels
- TARGET(1): object to grasp (cup)
- WORKSPACE(2): table surface
- OBSTACLE(3): other detected objects
- UNKNOWN(4): entire top camera view (GSAM not run on top camera)
- Target object's top surface appears as UNKNOWN — expected and acceptable for VGN

### /world_map_result JSON format (ply_utils.py:build_result_json)

```json
{
  "target":    {"label": "cup",   "centroid": [x,y,z], "bbox_3d_world": {"min":[x,y,z], "max":[x,y,z]}, "point_count": 1500},
  "workspace": {"label": "table", "centroid": [x,y,z], "bbox_3d_world": {"min":[x,y,z], "max":[x,y,z]}, "point_count": 8000},
  ...
}
```

VGN node uses: `result["target"]["centroid"]` (roi position) and `result["target"]["point_count"]` (skip check).

### camera_extrinsics.yaml — no changes needed for VGN

Convention: `p_world = R @ p_cam + t`  
**No K matrix addition. No load_extrinsics() signature change.**

---

## VGN Integration — Full Data Flow

```
grounded_sam_node → qwen_stub_node → multi_view_projector_node
                                              ↓
                                       /world_map  ──────────────────┐
                                       /world_map_result (trigger)   │
                                              │                      │
                                              ▼                      ▼
                                       vgn_grasp_node ←─────────────┘
                                              │     (/world_map: TSDF + semantic filter)
                                     /grasp_candidates
                                              ↓
                                          MoveIt2
                                  (/world_map: OBSTACLE+UNKNOWN → OctoMap)
```

rqt_graph 형태:

```
grounded_sam_node ──/grounded_sam/mask_image──► qwen_stub_node
                  ──/grounded_sam/detections_json──►

qwen_stub_node ──/qwen/mask_image──► multi_view_projector_node
               ──/qwen/labeled_detections──►

multi_view_projector_node ──/world_map──► RViz2
                          ──/world_map──► vgn_grasp_node
                          ──/world_map_result──► vgn_grasp_node (trigger)

vgn_grasp_node ──/grasp_candidates──► MoveIt2
```

### vgn_grasp_node subscriptions

| Topic | Type | Role |
|---|---|---|
| `/world_map` | PointCloud2 | ROI crop → TSDF + semantic filtering source |
| `/world_map_result` | String (JSON) | trigger — target centroid + point_count |

**depth topic 구독 없음. camera_info 구독 없음.**  
**K matrix 불필요. extrinsics.yaml 변경 불필요.**

---

## VGN Node Processing Steps

```
1. /world_map_result received (trigger)
   → parse target.centroid, target.point_count
   → skip if point_count < min_point_count

2. Define ROI
   roi_origin = centroid - [roi_size_m/2, roi_size_m/2, roi_size_m/2]
   roi frame  = coordinate frame with origin at roi_origin
   (30cm cube centred on target centroid)
   NOTE: "roi frame" ≠ WORKSPACE category (table label) — completely separate concept

3. ROI crop from cached /world_map
   Decode PointCloud2 → points (N,3), categories (N,) in world frame
   in_roi = all(points >= roi_origin) & all(points <= roi_origin + roi_size_m)
   roi_pts = points[in_roi]   ← ALL categories included (TARGET+WORKSPACE+OBSTACLE+UNKNOWN)
   (target top surface is UNKNOWN but geometry is correct — acceptable for VGN)

4. Build TSDF from cropped point cloud
   pts_roi = roi_pts - roi_origin   (transform to roi frame, origin at [0,0,0])
   voxel_size = roi_size_m / tsdf_resolution   (= 0.30/40 = 7.5mm)

   Build KDTree on pts_roi
   For each of 40×40×40 voxel centers:
     sdf_val = KDTree.query(voxel_center)   (distance to nearest surface point)
     truncate to [-4*voxel_size, +4*voxel_size]
   grid shape: (1, 40, 40, 40) float32

   Sign note: rigid body assumption — interior is solid.
   Unsigned SDF (all positive distances) is an approximation vs VGN training data
   (which uses ray-cast signed TSDF). Empirical impact on grasp quality TBD.

   TSDF 정규화: dist_normalized = clip(dist, 0, trunc) / trunc  → [0, 1] 범위
   VGN process()가 0.5 기준으로 outside/inside 판단하므로 정규화 필수.
   trunc = 4 * voxel_size (= 30mm)

5. VGN inference  ← 실제 API 확인 완료 (ethz-asl/vgn detection.py)
   qual_vol, rot_vol, width_vol = predict(grid, net, device)
     grid: (1, 40, 40, 40) float32, normalized to [0,1]
   qual_vol, rot_vol, width_vol = process(grid, qual_vol, rot_vol, width_vol)
     - Gaussian smoothing (sigma=1.0)
     - outside_voxels (tsdf > 0.5) mask → qual=0
     - width filter: reject voxels where width < 1.33 or > 9.33 voxels
   grasps_voxel, scores = select(qual_vol, rot_vol, width_vol, threshold=min_quality)
     - 결과 grasp.pose.translation은 voxel 인덱스 단위 (meters 아님)

6. Voxel → ROI frame 변환
   voxel_size = roi_size_m / tsdf_resolution
   grasps = [from_voxel_coordinates(g, voxel_size) for g in grasps_voxel]
     - grasp.pose.translation *= voxel_size  → meters in ROI frame
     - grasp.width *= voxel_size             → meters

   모델 파일명 규칙: load_network()가 path.stem.split("_")[1]로 네트워크명 결정
     예: vgn_conv.pth → "conv" → ConvNet
     "vgn_packed.pt" 등 존재하지 않는 이름은 KeyError 발생
   → 다운로드 파일명 확인 필수. 기본 파라미터: models/vgn_conv.pth

7. Semantic filtering — core justification for labeled PointCloud2 pipeline
   Extract TARGET points from cached /world_map (category == 1)
   Build KDTree on TARGET points (world frame)
   For each grasp candidate:
     p_world = roi_origin + grasp.pose.translation  (roi frame → world frame)
     dist, _ = KDTree.query(p_world)
     if dist > semantic_filter_radius: discard   ← landed on table/obstacle, not target
   GSAM labeling → TARGET identification → grasp semantic validation

8. Coordinate transform
   TF2: world → panda_link0
   NOTE: Isaac Sim team version has panda_link0 = world (static tf all-zero)
         Confirm with teammate — if identity, skip TF lookup entirely

9. Sort by quality descending → Top-K → publish /grasp_candidates
```

### /grasp_candidates output JSON

```json
{
  "candidates": [
    {
      "position":   [x, y, z],
      "quaternion": [qx, qy, qz, qw],
      "width":      0.052,
      "quality":    0.87,
      "frame":      "panda_link0"
    }
  ],
  "target_centroid": [x, y, z],
  "stamp": 1234567890.123
}
```

---

## New Package: vgn_grasp_pkg

Separate from mask_projection_pkg (different role, separate PyTorch deps, independent rsync).

```
src/vgn_grasp_pkg/
├── vgn_grasp_pkg/
│   ├── __init__.py
│   └── vgn_grasp_node.py
├── config/
│   └── vgn_params.yaml
├── launch/
│   └── vgn_grasp.launch.py
├── setup.py
└── package.xml
```

VGN library: `external/vgn/` as git submodule (ethz-asl/vgn).

---

## Parameters (vgn_params.yaml)

| Parameter | Default | Description |
|---|---|---|
| `roi_size_m` | `0.30` | ROI cube side length (m) — must match VGN model training size |
| `tsdf_resolution` | `40` | TSDF voxel grid side count (fixed, matches VGN input) |
| `vgn_model_path` | `models/vgn_packed.pt` | VGN weight file path |
| `min_quality` | `0.5` | grasp quality lower bound for NMS |
| `max_candidates` | `5` | max Top-K candidates to publish |
| `min_point_count` | `50` | skip if TARGET point_count below this |
| `semantic_filter_radius` | `0.05` | max distance (m) from TARGET surface to keep a grasp |
| `extrinsics_config` | `config/camera_extrinsics.yaml` | R, t only (no K) |

`voxel_size = roi_size_m / tsdf_resolution` — always computed, never a parameter.

---

## Environment: Reusing gsam_ws_venv

| Package | Current | VGN needs | Action |
|---|---|---|---|
| numpy | 1.26.4 | yes | reuse |
| scipy | 1.17.1 | yes | reuse |
| open3d | 0.19.0 | yes | reuse |
| scikit-learn | 1.8.0 | yes | reuse |
| torch | 2.11.0+cpu | yes | **reinstall CUDA build** |
| pytorch-ignite | — | yes | install |

```bash
source ~/gsam_ws/gsam_ws_venv/bin/activate
pip uninstall torch torchvision -y
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install pytorch-ignite tqdm
```

---

## Retraining (if 30cm model insufficient)

| ROI size | voxel_size | Retraining | Notes |
|---|---|---|---|
| 30cm | 7.5mm | not needed | default model, start here |
| 40cm | 10mm | needed | Seraph sbatch |
| 50cm | 12.5mm | needed | Seraph sbatch |

Weight swap + `roi_size_m` param change only — no pipeline code change.

---

## Team Coordination

- `/world_map`, `/world_map_result`: published by existing `multi_view_projector_node` — teammate needs no changes.
- VGN node subscribes only to `/world_map` and `/world_map_result` — no new Isaac Sim topics required.
- `/grasp_candidates`: consumed by MoveIt2 — coordinate JSON format with MoveIt2 teammate.
- **TF2 world→panda_link0**: confirm with teammate — likely identity (panda_link0 = world).
- **MoveIt2 OctoMap**: use OBSTACLE(3) + UNKNOWN(4) from `/world_map` only.

---

## Implementation Checklist

- [x] `git submodule add https://github.com/ethz-asl/vgn external/vgn`
- [x] Create `src/vgn_grasp_pkg/` package structure (setup.py, package.xml, launch)
- [x] Implement `vgn_grasp_node.py`:
  - [x] Cache `/world_map`, trigger on `/world_map_result`
  - [x] Parse target centroid + point_count, skip if insufficient
  - [x] ROI crop from /world_map (all categories)
  - [x] KDTree-based distance field → TSDF grid (40×40×40)
  - [x] VGN inference → NMS
  - [x] Semantic filtering: TARGET KDTree on /world_map
  - [x] world → panda_link0 transform (TF2 lookup, fallback to world frame)
  - [x] Publish /grasp_candidates (JSON) + /grasp_markers (MarkerArray, 파란색 화살표)
- [x] Install pytorch-ignite
- [x] full_pipeline.launch.py — 2-터미널 데모 (rgbd_sim + full_pipeline)
- [x] Race condition 수정: qwen_stub pending mask cache, VGN pending result cache
- [x] process_once 데모 모드 (GSAM 1회 탐지 후 구독 해제)
- [x] Empirically verify unsigned TSDF impact — **grasp가 물체 내부 관통 확인**
- [x] **[DONE]** Unsigned TSDF → depth image 기반 signed TSDF 적분으로 교체
- [ ] Reinstall torch CUDA build (현재 CPU only)
- [ ] Test with Isaac Sim (glass cup — transparent material depth quality)
- [ ] MoveIt2 Top-K fallback integration

---

## 알려진 문제 (2026-05-20 기준)

### TSDF 부호 문제 — grasp 관통 → **해결 완료**

**현상**: VGN grasp 후보가 물체 표면이 아닌 내부에 찍힘 (화살촉이 컵을 관통).

**원인**: `_build_tsdf_grid`가 KDTree unsigned SDF → VGN inside/outside 판단 불가.

**해결**: `_build_tsdf_raycasting` 으로 교체 완료 (2026-05-20).  
depth image ray-casting signed TSDF: `tsdf = clip(sdf/trunc, -1,1)*0.5+0.5`  
→ 1.0=outside, 0.5=surface, 0.0=inside. VGN `tsdf>0.5` outside 판단 정상 작동.

---

## 완료된 작업 — depth image 기반 signed TSDF 교체 (2026-05-20)

### 변경 범위

**수정 파일 2개만:**
- `src/vgn_grasp_pkg/vgn_grasp_pkg/vgn_grasp_node.py`
- `src/vgn_grasp_pkg/launch/vgn_grasp.launch.py`

나머지 패키지(grounded_sam_pkg, mask_projection_pkg, rgbd_projection) 무수정. 팀과 합의한 발행 토픽 인터페이스(/world_map, /world_map_result, /grasp_candidates 등) 변경 없음.

### 현재 vgn_grasp_node 구독

```
/world_map        (PointCloud2) — 캐시: TSDF 소스 + semantic filter용 TARGET 포인트
/world_map_result (String JSON) — 트리거: target centroid + bbox_3d + point_count
```

### 변경 후 vgn_grasp_node 구독

```
/ee_camera/depth_image   (Image)      — 캐시: EE depth, TSDF 적분용
/ee_camera/camera_info   (CameraInfo) — 캐시: EE 내부 파라미터 K
/top_camera/depth_image  (Image)      — 캐시: Top depth, TSDF 적분용 (optional)
/top_camera/camera_info  (CameraInfo) — 캐시: Top 내부 파라미터 K (optional)
/world_map_result        (String JSON) — 트리거 (기존과 동일)

/world_map 구독 제거 — RViz2 디버그 전용으로만 사용, vgn_grasp_node는 더 이상 구독 안 함
```

depth/camera_info 토픽들은 multi_view_projector_node가 이미 구독하는 것과 동일한 토픽. 두 노드가 같은 토픽을 각자 독립적으로 구독함.

### TSDF 적분 방식 (ray-casting)

```
트리거 수신 시:
  world_map_result.target.centroid → roi_min = centroid - 0.15m, roi_max = centroid + 0.15m
  40×40×40 voxel grid 생성 (각 voxel = 7.5mm, world frame 기준 roi 내부)
  voxel centers (64,000개) → (64000, 3) numpy 배열

[EE 카메라]
  extrinsics YAML R_ee, t_ee 사용:
    p_cam = R_ee.T @ (p_world - t_ee)   # world → camera frame
    u = fx*(px/pz) + cx                  # image 투영 (K from camera_info)
    v = fy*(py/pz) + cy
  이미지 범위 밖이면 skip
  d_obs = ee_depth[v, u]                 # 관측 depth
  sdf = d_obs - p_cam[2]                 # signed distance (양수=free, 음수=occupied)

[Top 카메라] (동일 방식, R_top, t_top 사용)

두 카메라 weighted average → clip(-trunc, trunc) / trunc → (1, 40, 40, 40) float32
```

2D depth image를 crop하지 않음. 64,000개의 voxel 위치를 depth image에 역투영해서 픽셀값을 읽는 것. ROI 밖 voxel은 계산 대상 아님.

### semantic filter 변경

```
현재: /world_map에서 TARGET(category==1) 포인트 추출 → KDTree → 거리 ≤ 0.05m
변경: /world_map_result.target.bbox_3d_world 기준 → grasp 위치가 bbox 내에 있는가

이유:
  - /world_map 구독 제거로 KDTree 소스가 없어짐
  - world_map_result에 bbox_3d_world가 이미 있음
  - vgn_architecture_note.md: "Use target bbox_3d for all filtering checks"
```

### launch 파일 추가 파라미터

```python
# vgn_grasp.launch.py에 추가
ee_depth_topic          default='/ee_camera/depth_image'
ee_camera_info_topic    default='/ee_camera/camera_info'
top_depth_topic         default='/top_camera/depth_image'
top_camera_info_topic   default='/top_camera/camera_info'
extrinsics_config       default=''  # 빈 문자열 시 mask_projection_pkg 기본값 사용
use_top_depth           default='true'
```

full_pipeline.launch.py 수정 불필요 (Gazebo 기본값이 자동으로 맞음).

### _pending_result 로직 변경

현재: `_world_map_pts is None`이면 result 대기
변경: `_ee_depth is None or _ee_info is None`이면 result 대기 (depth 캐시 기준)

---

## 알려진 한계 — semantic filter 정밀도

### 문제: 타겟 물체의 일부분만 라벨링됨

GSAM은 EE 카메라 RGB에서만 실행됨. Top 카메라는 depth only (GSAM 미실행).

결과:
```
TARGET 포인트 = EE 카메라가 본 컵의 앞/옆면만
컵 위쪽 표면 = Top 카메라에서 UNKNOWN으로만 분류됨

→ world_map_result.target.centroid = 컵 전체 중심이 아닌 EE에서 본 부분의 무게중심
→ 컵 앞쪽으로 약간 치우침
→ target.bbox_3d_world 도 일부만 커버
```

### TSDF에는 영향 없음

TSDF는 depth geometry만 사용. 라벨 무관. EE depth + Top depth가 합쳐져 컵 전체 geometry를 표현함.

### semantic filter가 문제될 경우

현상: 타겟(컵)에 가까운 grasp가 bbox 필터에서 걸러짐, 또는 인접 물체의 grasp가 통과됨.
원인: bbox_3d가 실제 컵보다 작거나 치우쳐 있음.

**해결 방향 (추후 필요 시 vgn_grasp_node 내부에서만 수정):**

DBSCAN을 /world_map의 TARGET 포인트에 적용:
```
/world_map에서 TARGET(category==1) 포인트 추출
DBSCAN 클러스터링 (eps=0.02, min_samples=10 등)
가장 큰 클러스터 = 타겟 물체의 보이는 면
→ 클러스터 hull 또는 확장된 bbox를 semantic filter 기준으로 사용
```

RANSAC은 불필요 — 테이블은 이미 WORKSPACE(category==2)로 라벨링되어 있음.

투명 물체(유리컵) depth hole 문제: Gazebo 시뮬에서는 depth hole이 적으나, 실제 환경에서는 DBSCAN 클러스터가 단편화될 수 있음.

이 수정은 vgn_grasp_node.py 내부에서만 이루어지며 외부 인터페이스 변경 없음.
