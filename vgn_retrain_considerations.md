# VGN Retraining Considerations

repo: https://github.com/ethz-asl/vgn | local: external/vgn/

## pipeline context
stack: ROS2 Jazzy / Isaac Sim / Grounded-SAM → Projection → VGN / Franka Panda. EE camera (wrist RGB-D, fixed scan pose) + Top camera (overhead depth-only). current VGN node subscribes /world_map (labeled PointCloud2) and /world_map_result (JSON trigger), crops 30cm ROI around target centroid, builds KDTree unsigned SDF → 40x40x40 grid → VGN inference → semantic filter (TARGET KDTree) → /grasp_candidates. known bug: grasp penetrates object interior because KDTree gives unsigned distance only, VGN process() uses tsdf>0.5 as outside which fails without sign → fix needed: replace with Open3D ScalableTSDFVolume ray-cast signed TSDF from raw EE+Top depth images. this is higher priority than retraining.

## data generation pipeline (generate_data.py + construct_dataset.py)
simulator: PyBullet (NOT Isaac Sim). data gen uses PyBullet, deployment uses Isaac Sim depth → TSDF format is identical so cross-sim is fine. three stages: (1) generate_data.py: spawn random scene in PyBullet, render N depth images, integrate into 120^3 TSDF via Open3D, sample 120 grasp candidates per scene from full-scene point cloud, execute each grasp in sim, label SUCCESS/FAILURE. (2) construct_dataset.py: downsample 120^3 → 40^3, convert grasp coords to voxel indices. (3) train_vgn.py: multi-task loss (quality BCE + rotation quat loss + width MSE), only rot+width loss backpropped on successes.

## hard constraints (cannot change without full retrain from scratch)
workspace cubic size = 6 * finger_depth = 6 * 0.05 = 0.30m enforced by assert in construct_dataset.py. network input hardcoded to (1,40,40,40) in perception.py get_grid() and networks.py Decoder interpolate(x,40). voxel_size = 0.30/40 = 7.5mm. truncation = 4*voxel_size = 30mm. changing any of these requires modifying perception.py + networks.py + full retrain.

## object shape dependency
none. network input is pure TSDF geometry, no labels. any URDF placed in data/urdfs/<object_set>/ is auto-discovered and randomly selected per scene. object-specificity comes only from what URDFs are in the set. at inference, semantic filtering (TARGET KDTree from /world_map) handles which grasps to keep.

## scene types
pile: objects dropped into box with full 3D random rotation (roll/pitch/pitch all random), Poisson(lambda=4)+1 count, scale uniform(0.8,1.0), box removed after settling. packed: objects placed upright (yaw-only random, roll/pitch=0), xy uniform(0.08,0.22), scale uniform(0.7,0.9), collision check → restore state if overlap, max 12 attempts. packed is more appropriate for tabletop manipulation (our scene). pile produces side/bottom grasps from randomly rotated objects.

## grasp candidate sampling — NOT per-object, biased by surface area
sample_grasp_point selects uniform random index from entire scene point cloud filtered by normal[2]>-0.1 (upward normals only). consequence: objects with larger surface area and more upward-facing normals are sampled more. in our scene: book (flat 12x20cm surface) and table dominate; glass cup (cylinder, only top + partial side visible) gets fewer samples; ball (sphere, only upper hemisphere passes normal filter) gets fewer. 120 grasps per scene are NOT uniformly distributed across objects. if cup is occluded by other objects in pile scene, it may appear in few/no points and get almost no samples. solutions: (a) use packed scene so all objects visible, (b) reduce OBJECT_COUNT_LAMBDA so fewer objects per scene and cup gets more relative coverage, (c) create separate single-object or cup-dominated scenes and merge datasets, (d) modify generate_data.py to do per-object stratified sampling using PyBullet body IDs and per-body AABB crop.

## Isaac Sim object sizes (from capstone_ws/robot_capstone/sim/setup_initial_scene.py)
glass cup: cylinder collider radius 7.6cm diameter 15.2cm height 18cm scale [0.02,0.02,0.02]. red ball: sphere radius 9.65cm diameter 19.3cm scale [0.05,0.05,0.05]. blue cube: 10x10x10cm scale [0.05,0.05,0.05]. book: 11.9x3.0x19.6cm scale [0.08,0.08,0.08]. cup diameter 15.2cm in 30cm workspace = 20 voxels at 7.5mm, adequate but not fine. if workspace shrunk to 20cm, voxel = 5mm, cup = 30 voxels.

## camera angle mismatch — critical for retraining
VGN generate_data.py render_images: theta=uniform(0, pi/4) azimuth from vertical, phi=uniform(0,2pi), r=uniform(1.6,2.4)*size. this covers 0-45 degrees from vertical (top-down to moderate tilt). Isaac Sim EE camera rotation matrix R col2 (look direction) = [0.910, 0.026, -0.414] → camera is ~65-70 degrees from vertical (nearly horizontal, slight downward tilt). Isaac Sim Top camera: position [0.84, 0.0, 2.0] looking straight down → ~5-15 degrees from vertical. EE camera angle is completely outside VGN training distribution (0-45 deg). top camera is within range. retraining must include EE-like viewpoints. suggested fix for render_images: split camera sampling 50/50 between ee-like (theta uniform(60deg,75deg), phi near 180deg +-30deg) and top-like (theta uniform(0,20deg), phi uniform(0,2pi)).

## workspace size vs resolution tradeoff
option A: keep 30cm/40^3 (7.5mm), no code change needed for workspace, but 80^3 would require networks.py Decoder + perception.py get_grid change + full retrain, memory 8x, speed 8x. option B: shrink workspace to 20cm/40^3 → 5mm voxel, same speed, only change needed is remove/adjust the assert np.isclose(size, 6*finger_depth) in construct_dataset.py, full retrain required, ROI tighter (less table context). option C: keep pretrained model, fix signed TSDF first, see if penetration is resolved before deciding on retrain. recommended order: fix signed TSDF → evaluate → decide retrain scope.

## recommended URDF set for retraining
place all target object URDFs in data/urdfs/our_objects/. YCB dataset URDFs usable directly (025_mug, 065_cups, etc.). glass cup needs URDF with cylinder collision geometry matching real dimensions. include cup, mug, cylinder-like objects to cover TARGET shape space. include box/cube/ball as obstacles. table plane already in setup/plane.urdf.

## mpirun parallelization
generate_data.py uses mpi4py. data gen is CPU-only (PyBullet headless). run on GPU server CPU cores: mpirun -np <N> python scripts/generate_data.py data/raw/our_packed --scene packed --object-set our_objects --num-grasps 50000. training needs GPU: python scripts/train_vgn.py --dataset data/datasets/our_packed --augment --epochs 30 --batch-size 32.

## open questions before starting data gen
(1) fix signed TSDF first and test — penetration may be resolved without retraining. (2) confirm final workspace size: 30cm keeps pretrained weights usable for transfer; 20cm needs full retrain but better resolution. (3) confirm object set: which objects will be in actual deployment scenes. (4) per-object stratified sampling: worth implementing if cup coverage is insufficient in packed scenes with 4 objects. (5) camera angle distribution in render_images: must be adjusted to match EE+Top actual angles before generating training data. (6) packed scene xy range (currently 0.08-0.22) may be too small for workspace — verify against actual table setup dimensions.
