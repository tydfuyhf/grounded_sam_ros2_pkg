# vgn_grasp_pkg

`/world_map` (라벨링된 PointCloud2) → **VGN grasp 후보 검출** → `/grasp_candidates` (JSON)

---

## 노드

| 노드 | 역할 |
|---|---|
| `vgn_grasp_node` | /world_map 크롭 → KDTree TSDF → VGN inference → 시맨틱 필터 → grasp 발행 |

---

## 전제 조건

### 1. VGN 서브모듈 추가

```bash
cd ~/gsam_ws
git submodule add https://github.com/ethz-asl/vgn external/vgn
```

### 2. pytorch-ignite 설치

GPU 환경 (RTX 3090 등 CUDA 12.1):

```bash
source ~/gsam_ws/gsam_ws_venv/bin/activate
pip uninstall torch torchvision -y
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install pytorch-ignite tqdm
```

노트북 / CPU only:

```bash
source ~/gsam_ws/gsam_ws_venv/bin/activate
pip install pytorch-ignite tqdm
# torch는 기존 CPU 빌드 그대로 사용 가능
```

### 3. VGN 모델 가중치

ethz-asl/vgn GitHub → README의 Google Drive 링크에서 `data.zip` 다운로드:

```
data.zip 압축 해제 → data/models/vgn_conv.pth
cp data/models/vgn_conv.pth ~/gsam_ws/models/vgn_conv.pth
```

> **파일명 규칙**: `vgn_<network>.pth` 형식 필수.  
> `load_network()`가 파일명 두 번째 필드(`vgn_conv.pth` → `conv`)를 파싱해 네트워크 종류를 결정함.  
> 파일명이 다르면 `KeyError` 발생.

---

## 빌드

```bash
source ~/gsam_ws/launch_env.bash
colcon build --packages-select vgn_grasp_pkg
source install/setup.bash
```

---

## 실행

```bash
ros2 launch vgn_grasp_pkg vgn_grasp.launch.py
```

정상 기동 시 출력:

```
[vgn_grasp_node]: VGN device: cpu
[vgn_grasp_node]: VGN model loaded: .../models/vgn_conv.pth
[vgn_grasp_node]: vgn_grasp_node ready  roi=0.3m  reso=40  min_quality=0.5
```

파라미터 오버라이드 예시:

```bash
ros2 launch vgn_grasp_pkg vgn_grasp.launch.py \
  vgn_model_path:=/abs/path/to/vgn_conv.pth \
  min_quality:=0.4 \
  max_candidates:=3 \
  semantic_filter_radius:=0.08
```

---

## 파라미터 (`config/vgn_params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `roi_size_m` | `0.30` | ROI 큐브 한 변 길이 (m) — VGN 학습 크기와 일치해야 함 |
| `tsdf_resolution` | `40` | TSDF 복셀 그리드 크기 (40×40×40 고정) |
| `vgn_model_path` | `models/vgn_conv.pth` | 가중치 경로 (절대 or `$GSAM_WS` 기준 상대) |
| `min_quality` | `0.5` | NMS grasp quality 하한값 |
| `max_candidates` | `5` | 발행할 Top-K 후보 수 |
| `min_point_count` | `50` | TARGET 포인트 수 미달 시 skip |
| `semantic_filter_radius` | `0.05` | TARGET 표면까지 최대 거리 (m) — 초과 시 grasp 제거 |
| `world_frame` | `world` | 입력 포인트클라우드 프레임 |
| `robot_frame` | `panda_link0` | 출력 grasp 프레임 (TF world→robot_frame) |

`voxel_size = roi_size_m / tsdf_resolution` — 항상 계산, 파라미터 아님 (= 7.5 mm)

---

## 구독 / 발행 토픽

| 토픽 | 타입 | 역할 |
|---|---|---|
| `/world_map` | `sensor_msgs/PointCloud2` | 라벨링된 포인트클라우드 — **캐시** |
| `/world_map_result` | `std_msgs/String` (JSON) | 카테고리별 centroid + point_count — **트리거** |
| `/grasp_candidates` | `std_msgs/String` (JSON) | Top-K grasp 후보 출력 |
| `/grasp_markers` | `visualization_msgs/MarkerArray` | RViz2 시각화 |

---

## 출력 JSON `/grasp_candidates`

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

TF 조회 실패 시 `frame`은 `"world"`로 폴백.

---

## RViz2 시각화 (`/grasp_markers`)

```
RViz2 → Add → MarkerArray → Topic: /grasp_markers
Fixed Frame: world
```

| 마커 | 의미 |
|---|---|
| 화살표 | grasp approach 방향 (그리퍼 -Z축), 색깔 = quality |
| 납작한 타원 | grasp 위치, 지름 = gripper width |
| 초록 | quality 높음 / 빨강 | quality 낮음 |

마커 lifetime 5초 — 새 결과 수신 시 자동 갱신.

---

## 처리 흐름

```
/world_map_result (트리거)
  → target.centroid, target.point_count 파싱
  → point_count < min_point_count 이면 skip

캐시된 /world_map
  → AABB ROI crop (centroid ± roi_size_m/2, 모든 카테고리 포함)
  → roi frame 변환 (origin = roi_min)
  → KDTree unsigned SDF → normalize [0,1] → 40×40×40 float32 grid

VGN inference (ethz-asl/vgn — predict/process/select 직접 사용)
  → predict(grid, net, device) → qual_vol, rot_vol, width_vol
  → process(...)  Gaussian smoothing + outside-surface mask + width filter
  → select(..., threshold=min_quality)  NMS → grasps (voxel 좌표)
  → from_voxel_coordinates(grasp, voxel_size)  → roi frame meters 변환

시맨틱 필터링
  → /world_map에서 TARGET(category==1) 포인트 KDTree
  → grasp position까지 거리 > semantic_filter_radius → discard

TF world → panda_link0
  → TF 조회 실패 시 world frame 그대로 발행

quality 내림차순 정렬 → Top-K → /grasp_candidates, /grasp_markers 발행
```

---

## VGN 라이브러리 호환성 메모

`ethz-asl/vgn`은 ROS 1(catkin) 기반으로 `vgn.detection`이 `vgn.vis`를 import하고, `vgn.vis`는 `rospy`를 사용함.  
`vgn.networks`와 `vgn.utils.transform`은 rospy 의존성이 없음.

이 패키지는 `predict` / `process` / `select` 함수를 `vgn.detection` 대신 노드 내부에 직접 구현해 rospy 의존성을 우회함. VGN 네트워크 가중치 로딩(`load_network`)과 좌표 변환(`Transform`, `Rotation`)만 VGN 라이브러리에서 import.

---

## 팀 레포 rsync 방법

```bash
cd ~/capstone_ws/robot_capstone
git checkout main && git pull
git checkout -b sanghyun_MMDD

rsync -a --exclude="__pycache__" --exclude="*.pyc" \
  ~/gsam_ws/src/vgn_grasp_pkg  ros_pkgs/src/

git add ros_pkgs/src/vgn_grasp_pkg
git commit -m "Add vgn_grasp_pkg"
git push origin sanghyun_MMDD
```

기존 패키지(`grounded_sam_pkg`, `mask_projection_pkg`)는 rsync 금지 — [TEAM_DIFF.md](../../../TEAM_DIFF.md) 참고.
