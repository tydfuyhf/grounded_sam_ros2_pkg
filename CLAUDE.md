# CLAUDE.md

이 파일은 Claude Code가 이 프로젝트의 맥락을 파악하기 위한 가이드입니다.

---

## 프로젝트 목표

ROS 2 기반 로봇 조작 파이프라인:
**Grounded SAM → 3D Projection → (예정) Qwen VLM → MoveIt2**

Isaac Sim에서 동료가 연동 작업을 진행 중입니다. 이 저장소는 GSAM + Projection 파트를 담당합니다.

---

## 패키지 구성

```
src/
  grounded_sam_pkg/      GSAM 추론 노드 (GroundingDINO + SAM)
  mask_projection_pkg/   Projection 노드 — 주요 작업 영역
  rgbd_projection/       Gazebo 시뮬 + RViz (데모용, 수정 금지)
external/
  GroundingDINO/         서브모듈
  segment-anything/      서브모듈
```

---

## 현재 씬 (Isaac Sim 기준)

테이블 위 물체 4개:

| 물체 | 카테고리 | 색상 |
|---|---|---|
| 유리컵 (glass cup) | TARGET | 초록 (0, 200, 80) |
| 테이블 (table) | WORKSPACE | 노랑 (255, 220, 0) |
| 빨간 공 / 파란 큐브 / 책 | OBSTACLE | 빨강 (220, 40, 40) |
| 배경 | FREE | 회색 (80, 80, 80) |

카테고리 색상은 `label_mapper.py` 에 정의되어 있으며 변경하지 않습니다.

---

## 절대 수정 금지

- `src/mask_projection_pkg/mask_projection_pkg/projector_node.py`
- `src/mask_projection_pkg/launch/mask_projector.launch.py`

Gazebo 데모 레퍼런스입니다. 새 기능은 항상 별도 파일로 추가합니다.

---

## 카테고리 할당 방식 — 중요

### 현재 (임시): prompt 순서 기반

`label_mapper.py` 의 `MASK_VALUE_TO_CATEGORY` 가 mask pixel 1 → TARGET, 2 → WORKSPACE 로 고정.
`prompt:="glass cup, table, red ball, blue cube, book"` 처럼 순서를 맞춰서 실행해야 함.

### Isaac Sim 통합 시 수정 필요: detections_json label 기반으로 교체

GSAM이 항상 prompt 순서대로 감지하지 않을 수 있음.  
`detections_json` 의 `label` 필드 → 카테고리 딕셔너리 매핑 방식으로 교체해야 함.  
수정 위치: `label_mapper.py` 또는 `multi_view_projector_node.py` 의 `_mask_cb`.  
`cloud_builder.py` 와 다운스트림은 변경 불필요.

---

## multi_view_projector_node 동작 방식

```
Top 카메라 depth  →  전 포인트 FREE  (씬 형상)
EE 카메라 depth + GSAM 마스크  →  TARGET / WORKSPACE / OBSTACLE / FREE
두 뷰 월드 변환 후 병합  →  /world_map  (frame_id="world")
```

변환 행렬 `_R_TOP/_t_TOP/_R_EE/_t_EE` 는 현재 identity placeholder.  
Isaac Sim USD stage의 카메라 World Transform 값으로 채워야 함 (동료 TODO).

---

## 동료 (Isaac Sim 연동) 가 할 일

1. `src/mask_projection_pkg/mask_projection_pkg/multi_view_projector_node.py` 상단 R/t 채우기
2. launch 토픽 오버라이드 (코드 수정 없음):
   ```bash
   ros2 launch mask_projection_pkg multi_view_projector.launch.py \
     top_depth_topic:=/isaac/top/depth_image \
     top_camera_info_topic:=/isaac/top/camera_info \
     ee_depth_topic:=/isaac/ee/depth_image \
     ee_camera_info_topic:=/isaac/ee/camera_info
   ```
3. GSAM은 EE 카메라 RGB로 실행:
   ```bash
   ros2 launch grounded_sam_pkg grounded_sam.launch.py \
     image_topic:=/ee_camera/image \
     prompt:="glass cup, table, red ball, blue cube, book"
   ```

---

## 환경 설정

```bash
# 매 터미널마다 필수
source launch_env.bash   # venv + ROS + PYTHONPATH 통합
source install/setup.bash
```

`launch_env.bash` 없이 실행하면 `ModuleNotFoundError: torch / groundingdino` 발생.

---

## 향후 계획

- **Qwen VLM 연동**: Qwen이 어떤 마스크가 TARGET/WORKSPACE인지 판별 → `label_mapper.py` 매핑 교체
- **MoveIt2 연동**: `/world_map_result` TARGET centroid → goal pose, OBSTACLE → 충돌 맵
- **Top 카메라 GSAM**: GPU 환경에서 필요 시 별도 브랜치로 추가

---

## Git 레포 구조 — 중요

작업 공간과 Git 레포가 분리되어 있습니다. 혼동하지 마세요.

| 역할 | 경로 | 리모트 |
|---|---|---|
| 개인 작업 공간 | `/home/parksanghyun/gsam_ws` | `https://github.com/tydfuyhf/grounded_sam_ros2_pkg` |
| 팀 레포 (클론) | `/home/parksanghyun/tmp/robot_capstone` (또는 별도 클론 위치) | `https://github.com/ChanwonJung/robot_capstone` |

### 작업 흐름

- **gsam_ws** 에서 개발 → 개인 레포(`origin`)에 push
- 팀 레포에 반영할 때는 **robot_capstone을 별도로 클론**해서 패키지를 복사 추가 후 PR
- gsam_ws에서 직접 팀 레포로 push하지 않음 (히스토리가 달라 conflict 위험)

### 팀 레포에 패키지 추가하는 절차

```bash
# 1. 팀 레포 클론 (처음 한 번만)
git clone https://github.com/ChanwonJung/robot_capstone.git ~/tmp/robot_capstone
cd ~/tmp/robot_capstone

# 2. main 기반 새 브랜치 생성
git checkout -b sanghyun_gsam

# 3. gsam_ws에서 패키지 복사
cp -r /home/parksanghyun/gsam_ws/src/grounded_sam_pkg  src/
cp -r /home/parksanghyun/gsam_ws/src/mask_projection_pkg  src/

# 4. 커밋 후 push
git add src/grounded_sam_pkg src/mask_projection_pkg
git commit -m "Add grounded_sam_pkg and mask_projection_pkg"
git push origin sanghyun_gsam
```

### Git 규칙

- 커밋 메시지에 `Co-Authored-By: Claude` 남기지 않음
- gsam_ws 내 `capstone` 리모트는 이전 작업 잔재 — 팀 레포 작업은 별도 클론에서 진행
