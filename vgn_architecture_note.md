# VGN / G-SAM / Projection Architecture Handoff

> **How to use this document**: Read the full document first to understand the current state and decisions made. Then discuss with Claude to decide on implementation details before writing any code. Do not start coding immediately — use this as a basis for conversation.

---

## Context Summary
- Robot: Franka Emika Panda (7-DOF, parallel-jaw gripper, 80mm max opening)
- Cameras: EE camera (wrist-mounted, RGB-D) + Top camera (overhead, depth only)
- Stack: ROS2 Jazzy / Isaac Sim / Grounded-SAM / VGN / MoveIt2
- Language: Python 3.12
- Repo: https://github.com/tydfuyhf/grounded_sam_ros2_perception

---

## Goal
Do not rewrite the whole repository. Keep the existing pipeline: Grounded-SAM -> Qwen/role assignment -> Projection node -> /world_map, /world_map_result -> vgn_grasp_node. The main change is inside VGN input generation. Currently /world_map is used as the core source for VGN TSDF generation. Change /world_map into RViz2/debug visualization only, and build the actual VGN input TSDF from raw depth images.

---

## Current Problem
The current VGN node crops a 30cm ROI from the fused labeled point cloud /world_map around the target, then builds a 40x40x40 grid using KDTree nearest distance. This is not a true TSDF. It is closer to UDF(Unsigned Distance Field), because it only measures distance to observed point cloud surface points. It does not encode free space, observed surface, unknown region, occlusion, or inside/outside sign. Because of this, VGN can output grasp poses that penetrate the object surface or appear below the object/table.

---

## Core Decision
Keep /world_map, but do not use it as the core computation source for VGN. Use /world_map only as a semantic colored point cloud for RViz2 projection/debug checking. Use raw EE depth + raw Top depth to build a depth-based local TSDF for VGN. Use /world_map_result or object_instances only to determine target ROI and to semantically filter VGN candidates. Use geometry-only fused cloud / octomap for MoveIt collision checking.

---

## Why Grounded-SAM Still Exists
Grounded-SAM is still required because depth, point cloud, TSDF, Octomap, and VGN only understand geometry. They do not know which object is the target, which object is an obstacle, which surface is workspace/table, or which cup is meant by an instruction like "pick up the cup on the book". Grounded-SAM provides 2D masks for semantic objects. Qwen or rule-based role assignment selects target/obstacle/workspace roles. Projection converts selected masks into 3D object information. VGN then generates grasp candidates from local geometry, and semantic filtering checks whether candidates actually grasp the target.

---

## Why Projection Node Still Exists
Grounded-SAM output is 2D: mask image, 2D bbox, text label, confidence. Robot planning needs 3D: target centroid in world frame, target bbox_3d, target surface points, VGN ROI center, target/obstacle/workspace role, and MoveIt target handling. Therefore the projection node remains necessary. Its role changes from "generate dense labeled point cloud as VGN input" to "generate object_instances or /world_map_result for semantic reasoning, ROI selection, candidate filtering, and MoveIt target handling". It should still publish /world_map for RViz2 debug visualization.

---

## /world_map Role
/world_map must remain for RViz2 visual checking. It should show semantic colored projection results: target=green, workspace/table=yellow, obstacle=red, unknown=gray, raw geometry=white if needed. Its purpose is to verify that G-SAM masks are correctly projected into 3D, target/obstacle/workspace colors are correct, EE view and Top view alignment is correct, and object bbox/centroid results are plausible. /world_map should not be used as actual VGN TSDF input and should not be the actual MoveIt collision source.

---

## Dense Labeled Point Cloud Judgment
Dense labeled point cloud is useful for RViz2 debugging but weak as a core data structure. Problems: every point needs a label, two-view fusion creates label conflicts, semantic update rules become complicated, MoveIt Octomap does not need labels, VGN input should not be built from labeled point cloud, and KDTree fake TSDF cannot represent free-space/unknown/inside/outside. Keep dense labeled point cloud as debug visualization only. Use object_instances for semantic computation. Use raw depth or geometry-only cloud for geometry computation.

---

## Recommended Architecture
Semantic flow: EE RGB image -> Grounded-SAM -> 2D masks/labels -> Qwen or rule-based role assignment -> Projection node -> object_instances or /world_map_result -> VGN ROI decision, semantic candidate filtering, MoveIt target handling, and /world_map debug cloud. Geometry flow: EE depth + Top depth -> geometry-only fusion -> /world_cloud_raw -> /world_octomap -> MoveIt planning scene. VGN flow: object_instances or /world_map_result gives target centroid/bbox -> define 30cm ROI -> raw EE depth + raw Top depth are integrated in ROI frame -> depth-based 40x40x40 local TSDF -> VGN Top-K candidates -> semantic filtering -> MoveIt IK/path check -> final grasp pose.

---

## VGN Input Change
Remove or deprecate the old path: /world_map crop -> KDTree nearest distance -> fake TSDF. Replace it with: target centroid or bbox center -> define 30cm cubic ROI -> integrate EE depth + Top depth into ROI frame -> generate 40x40x40 local TSDF -> run VGN inference. Keep roi_size=0.30m, resolution=40, voxel_size=0.0075m unless there is a strong reason to change. Do not build target-only TSDF. The local TSDF must include target object, table/support surface, nearby obstacles, and local scene geometry inside the ROI. Target-only crop removes table/support context and can make VGN produce unrealistic grasps below the object.

---

## Scan Pose and EE Camera Assumption
Current setup: EE camera is physically mounted on Franka wrist but scan is performed at a fixed pre-defined pose. camera_extrinsics.yaml stores the fixed camera-to-EE transform. EE-to-world transform is hardcoded as a fixed value corresponding to the scan pose. This is valid as long as depth is always captured at the same scan pose before VGN inference.

**Problem**: This assumption breaks if scan pose needs to vary later (occlusion handling, dynamic scenes, mobile manipulation). The fixed hardcoded transform will silently produce wrong world-frame TSDF without any error.

**Solution**: Introduce a `use_fixed_scan_pose` parameter (default: true). When true, use hardcoded EE-to-world. When false, read EE-to-world from /tf at capture time using `panda_EE` -> `world` transform. This keeps current behavior unchanged but opens the interface for future extension without a full rewrite.

> **Discuss with Claude**: Whether to add this parameter now in Phase 1 or defer. Current simulation-only setup makes it low priority, but the interface cost is small.

---

## Top Depth Coverage Check
**Problem**: The architecture assumes EE depth + Top depth fusion always improves TSDF quality. However, Top camera is overhead and covers the table plane well but may have limited coverage of object sides and EE-view geometry within the 30cm ROI. If Top depth contributes little to the ROI, fusing it adds noise without benefit.

**Solution**: In Phase 1, verify Top depth ROI coverage by checking what percentage of the 40x40x40 voxel grid is filled by Top depth alone vs EE depth alone. If Top depth fills less than 20% of the ROI voxels that EE depth does not already cover, consider making Top depth fusion optional via `use_top_depth` parameter (default: true).

> **Discuss with Claude**: How to measure voxel coverage in Phase 1 verification. Whether 20% threshold is appropriate given your specific camera placement.

---

## object_instances Minimal Design
Start with JSON String if custom ROS msg is too much. Minimum fields: object_id, class_name, role(target/obstacle/workspace/unknown), confidence, centroid_world, bbox_3d_world, source_view. Do not include surface_points in early phases.

**Problem**: surface_points with count=256 downsampled points is too sparse for reliable finger closing region check. Using sparse surface points for proximity filtering produces frequent misses and false rejections.

**Solution**: Use target bbox_3d for proximity and containment checks instead of surface points. bbox_3d is already available from /world_map_result and is sufficient for target proximity check, below-bbox rejection, and finger closing region overlap check. Add surface_points only in Phase 3 if bbox_3d proves insufficient for fine-grained filtering.

Short term, /world_map_result centroid/bbox can be reused. object_instances can be Phase 3 refactor.

---

## VGN Candidate Filtering
VGN does not know semantic target. After VGN Top-K output, filter candidates using hard filters only in Phase 2. Do not use soft score in early phases.

**Problem**: The composite soft score originally proposed (final_score = VGN_quality + target_surface_score + gripper_target_overlap_score - obstacle_collision_penalty - ...) has too many terms with different scales. Weight tuning becomes the bottleneck and makes debugging difficult. A bad weight combination can silently reject all valid candidates or accept invalid ones.

**Solution**: Use hard boolean filters in Phase 2. Apply in order:
1. target proximity check: reject if grasp center is farther than 2 voxels = 0.015m from target bbox surface
2. below-bbox rejection: reject if grasp center z is below target bbox_3d min z
3. table penetration check: reject if grasp center or finger tips are below table plane z
4. obstacle collision check: reject if gripper swept volume overlaps obstacle/workspace points

Pass/fail only, no scoring. After hard filters, rank surviving candidates by VGN quality score only. Add soft scoring only in a later phase once hard filters are validated.

MoveIt IK feasibility check and MoveIt path collision check using octomap are Phase 4 additions.

> **Discuss with Claude**: Exact threshold values for each filter given your scene geometry. Whether Top-K should be 5 or 10. Whether rejection_reason logging is needed from Phase 2 or can be deferred.

---

## MoveIt Octomap Warning
If target remains inside MoveIt Octomap, MoveIt treats the target as an obstacle. Then the robot must grasp the cup but MoveIt tries to avoid collision with the cup. During grasp, handle target specially: either remove target region from octomap, or register target as a separate collision object, then allow collision with gripper during grasp, and after successful grasp attach the object. Target and obstacle must be separated in semantic layer and handled differently in MoveIt planning scene.

**Problem**: If depth stream continues during grasp execution, octomap updates continuously and the removed target region gets re-inserted from new depth frames. This causes MoveIt to suddenly treat the target as an obstacle mid-execution.

**Solution**: At grasp execution start, freeze octomap updates by stopping /world_cloud_raw publication or pausing the octomap server. Resume after grasp completes or fails. Alternatively, mask out the target bbox_3d region from depth integration so target points never enter the geometry flow. Add a `octomap_frozen` state flag to vgn_grasp_node or a separate grasp execution node.

> **Discuss with Claude**: Which freeze strategy fits better with your execution node design. Whether to freeze the full octomap or only mask the target bbox region.

---

## Recommended Topics
Debug visualization: /world_map as semantic colored PointCloud2 for RViz2 projection checking; optionally rename to /debug_labeled_cloud later. Actual geometry: /world_cloud_raw as geometry-only fused PointCloud2 with no labels; /world_octomap as MoveIt collision map with no labels. Semantic computation: /object_instances for object-level semantic registry. VGN debug/result: /debug_vgn_roi for 30cm ROI cube marker; /debug_vgn_local_tsdf optional; /grasp_candidates for Top-K; /grasp_markers for raw candidates; /debug_filtered_grasp_candidates for accepted/rejected candidates with rejection_reason annotation.

---

## RViz2 Debug Checklist
Keep /world_map visible to verify projection colors and mask-to-3D alignment. Add or keep /world_cloud_raw to verify actual geometry fusion. Show /world_octomap to verify MoveIt collision map. Show /debug_object_bbox and /debug_object_centroid to verify object_instances or /world_map_result. Show /debug_vgn_roi to verify 30cm ROI is centered on target and includes local context. Show /grasp_markers for raw VGN Top-K candidates. Show /debug_filtered_grasp_candidates for filtered candidates. Suggested colors: raw candidates=blue, accepted=green, rejected=red, final best=purple or thick green. Log or mark rejection reasons: not near target, table penetration, obstacle collision, IK failed.

---

## Expected Code Change Size
Not a full rewrite. grounded_sam_pkg: almost no change. qwen_stub_node: almost no change. multi_view_projector_node.py: small change; keep /world_map debug, optionally add /object_instances, around 10~25% if refactored. projection_engine.py: mostly keep; optionally clean centroid/bbox calculation; remove surface_points from early phases. cloud_builder.py: keep, but treat as debug cloud builder. vgn_grasp_node.py: biggest change; remove/deprecate _build_tsdf_grid and add raw depth two-view TSDF generation, add use_fixed_scan_pose parameter, around 60~70% change in this file. launch/yaml: add depth topics, camera_info topics, extrinsics config, use_depth_tsdf parameter, use_fixed_scan_pose parameter, use_top_depth parameter, around 20% change. Total repository change roughly 25~35%.

---

## Implementation Phases

**Phase 1**: Minimal fix. Only replace VGN input generation. Add raw EE depth and raw Top depth subscription to vgn_grasp_node.py. Use /world_map_result target centroid/bbox to define 30cm ROI. Integrate EE depth + Top depth into ROI frame to create 40x40x40 TSDF. Add use_fixed_scan_pose parameter (default: true) and use_top_depth parameter (default: true). Verify Top depth ROI coverage. Connect this TSDF to existing VGN inference. Keep existing /grasp_candidates and /grasp_markers. Keep /world_map unchanged as RViz2 debug output. Do not add object_instances yet if not necessary.

**Phase 2**: Strengthen semantic filtering with hard boolean filters only. Use /world_map_result target bbox_3d for all proximity and containment checks. Do not use surface_points. Apply filters in order: target proximity (threshold 0.015m from bbox surface), below-bbox rejection, table penetration rejection, obstacle/workspace collision rejection. Rank surviving candidates by VGN quality only. Add rejection_reason logs and /debug_filtered_grasp_candidates marker.

**Phase 3**: Add object_instances. Add /object_instances topic. Store object_id, class_name, role, bbox_3d, centroid. Add surface_points only if bbox_3d filtering proves insufficient. Use object_instances for VGN ROI decision, semantic candidate filtering, and MoveIt target collision handling. Continue publishing /world_map as debug colored cloud.

**Phase 4**: MoveIt integration. Create /world_cloud_raw geometry-only cloud and build /world_octomap. Feed octomap into MoveIt planning scene. Handle target object using allowed collision or attach object. Add octomap_frozen state flag: freeze octomap updates at grasp execution start, resume after completion or failure. Run IK/path feasibility check for VGN Top-K candidates.

---

## Open Questions (Discuss Before Implementing)
- use_fixed_scan_pose: add in Phase 1 or defer?
- Top depth coverage threshold: is 20% the right cutoff for your camera placement?
- Top-K value: 5 or 10 candidates?
- Hard filter thresholds: 0.015m proximity, table plane z — confirm against your scene dimensions
- octomap freeze strategy: full freeze vs target bbox masking
- object_instances: JSON string or custom ROS msg?

---

## Codex Instruction
Inspect the repository. Do not remove Grounded-SAM, the projection node, or /world_map visualization. Keep /world_map as RViz2 debug semantic colored point cloud only. Do not use /world_map as VGN TSDF input. Replace KDTree fake TSDF in vgn_grasp_node.py with depth-based local TSDF from raw EE depth and raw Top depth using camera intrinsics/extrinsics and ROI frame. Add use_fixed_scan_pose parameter (default: true); when false read EE-to-world from /tf. Add use_top_depth parameter (default: true). Use /world_map_result or object_instances only for target ROI and semantic filtering. Use target bbox_3d for all filtering checks; do not rely on surface_points in early phases. Keep ROI size 0.30m and resolution 40. Keep dense labeled cloud only for debug. Use geometry-only fused cloud/octomap for MoveIt collision. After VGN Top-K generation, apply hard boolean filters in order: target proximity, below-bbox, table penetration, obstacle collision. Rank by VGN quality only; no soft scoring in early phases. Handle target separately in MoveIt using allowed collision or attach object. Freeze octomap updates during grasp execution.
