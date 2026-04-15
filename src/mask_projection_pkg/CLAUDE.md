# CLAUDE.md — mask_projection_pkg

이 파일은 Claude Code가 이 패키지의 맥락을 파악하기 위한 가이드입니다.

---

## 이 패키지가 하는 일

GSAM 세그멘테이션 마스크 + Depth 이미지 → **라벨링된 3D PointCloud2** 변환.

두 노드 제공:
- `projector_node` — 단일 카메라, Gazebo 데모 레퍼런스 (수정 금지)
- `multi_view_projector_node` — Top + EE 두 카메라 → 월드 좌표계 `/world_map`

---

## 절대 수정 금지

- `projector_node.py`
- `launch/mask_projector.launch.py`

Gazebo 데모 레퍼런스입니다. 새 기능은 항상 별도 파일로 추가합니다.

---

## 현재 씬 (Isaac Sim)

테이블 위 물체 4개:

| 물체 | 카테고리 | 색상 |
|---|---|---|
| 유리컵 (glass cup) | TARGET | 초록 (0, 200, 80) |
| 테이블 (table) | WORKSPACE | 노랑 (255, 220, 0) |
| 빨간 공 / 파란 큐브 / 책 | OBSTACLE | 빨강 (220, 40, 40) |
| 배경 | FREE | 회색 (80, 80, 80) |

카테고리 색상은 `label_mapper.py` 에 정의되어 있으며 변경하지 않습니다.

---

## 카테고리 할당 — 현재 방식과 한계

### 현재 (임시): prompt 순서 기반

`MASK_VALUE_TO_CATEGORY` 가 mask pixel 1 → TARGET, 2 → WORKSPACE 로 고정.  
`prompt:="glass cup, table, red ball, blue cube, book"` 처럼 순서를 맞춰서 실행해야 함.

### Isaac Sim 통합 시 수정 필요

GSAM이 항상 prompt 순서대로 감지하지 않을 수 있음.  
`detections_json` 의 `label` 필드 기반 딕셔너리 매핑으로 교체해야 함.  
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

1. `multi_view_projector_node.py` 상단 R/t 행렬 채우기
2. launch 토픽 오버라이드:
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

## 향후 계획

- **Qwen VLM 연동**: Qwen이 어떤 마스크가 TARGET/WORKSPACE인지 판별 → `label_mapper.py` 매핑 교체
- **MoveIt2 연동**: `/world_map_result` TARGET centroid → goal pose, OBSTACLE → 충돌 맵

---

## Git 규칙

- 커밋 메시지에 `Co-Authored-By: Claude` 남기지 않음
- 연동 전 브랜치: `sanghyun_4_15` → `https://github.com/ChanwonJung/robot_capstone.git`
