# COLMAP Telemetry Fork — Implementation Plan

> **Audience:** a coding agent working **only inside this repository**
> (`C:\splat\pipeline\colmap`, upstream `https://github.com/colmap/colmap.git`,
> checkout `9d49dcd4`, post-4.1.0 main).
> **Branch:** create `telemetry` from the current HEAD. All work lands there as
> the ordered, separable commits F0…F6 described in §5.
>
> **Mission:** extend COLMAP so that noisy consumer-grade telemetry — GPS
> position, compass heading, gravity direction, full attitude — can be injected
> as *per-image pose priors* that (a) seed and constrain reconstruction, and
> (b) make the output **metric-scale and north-aligned** without any
> post-processing. Every feature is independently flag-gated, defaults OFF, and
> changes nothing when disabled. Each feature is shaped like an upstreamable PR:
> generic, documented, tested — nothing vendor-specific (no GoPro/GPMF/DJI
> parsing in C++; telemetry decoding happens outside and arrives via the CSV
> contract in §2).
>
> All file:line anchors below were verified against this checkout on
> 2026-07-11. Re-verify before editing; treat drift as a signal to re-read the
> surrounding code, not to guess.

---

## 0. Ground rules

1. **Flag independence.** Every feature has its own option(s), default off.
   With all flags off, behavior and output must be bit-for-bit identical to the
   base commit (existing tests are the guard). Features must work in any
   combination; §6 lists the combos to test.
2. **Ceres only for prior math.** The pose-prior BA factory hard-fails on the
   Caspar backend (`src/colmap/estimators/bundle_adjustment.cc:386-394` —
   `LOG(FATAL_THROW) << "Caspar BA backend does not support pose priors"`).
   Every new prior-consuming path must check `options.backend ==
   BundleAdjustmentBackend::CERES` early and fail with an actionable message
   ("set --Mapper.ba_backend ceres"). Do **not** attempt Caspar support — it is
   a SymForce codegen project (see `estimators/caspar/caspar_model_adapter.h:904`:
   only `kSimpleRadial`/`kPinhole` have adapters), explicitly out of scope.
3. **Rig-aware.** Since 4.x, poses live on `Frame::RigFromWorld()`; an image
   may be a non-reference sensor in its frame. Priors are keyed to images
   (`corr_data_id`). Where the existing code restricts prior handling to
   ref-in-frame images (`rotation_averaging_impl.cc:56`) or splits ref/non-ref
   (`bundle_adjustment_ceres.cc:1009-1025`), mirror that structure.
4. **Determinism.** Every stochastic addition must respect `random_seed`
   plumbing (see `RotationEstimatorOptions::random_seed`,
   `alignment_ransac_options.random_seed` at `sfm/incremental_mapper.cc:1166`).
5. **Tests are the spec.** Each feature section names the existing test file to
   mirror. A feature is done only when its new tests pass **and** the full
   existing suite passes.

---

## 1. The data: what the priors actually are

### 1.1 Reference dataset profile (what to optimize for)

The canonical producer is a drone/action-cam pipeline that runs an
aerospace-style ESKF over consumer sensors and interpolates the smoothed track
to every video frame timestamp. Properties the design must exploit:

- **Sequential video**: images are consecutive frames; every image has a prior;
  the true camera is guaranteed to lie on the smoothed track between its
  neighbors. Density is the point: per-frame absolute priors already encode the
  "can't be anywhere else" constraint — no special inter-frame machinery is
  required for v1 (F6 exists as an optional experiment).
- **Position**: consumer GNSS (u-blox M10 class), ESKF-smoothed.
  Typical σ_East/σ_North ≈ 1.5–4 m, σ_Up ≈ 3–8 m. Smooth, locally consistent,
  possibly globally biased by a few meters (common-mode; harmless to shape,
  absorbed by alignment).
- **Gravity**: accelerometer-derived down vector in the camera frame,
  σ ≈ 1–2°. Available for essentially every frame.
- **Heading/attitude**: magnetometer + GPS-course fused yaw, σ ≈ 2–5°;
  combined with gravity this yields a full attitude quaternion per frame with
  strongly **anisotropic** confidence (tilt good, yaw worse).
- **Camera**: single `OPENCV_FISHEYE` camera, intrinsics roughly known,
  refined during SfM. 2D-3D registration is the fragile step on fisheye video —
  which is why F4 (prior-seeded registration) matters.

The same CSV contract must also be trivially producible from what "the average
COLMAP user" has: EXIF GPS, DJI SRT logs, PX4/ArduPilot logs, ARKit/ARCore
poses. That constraint drove every schema decision below.

### 1.2 Coordinate and convention cheat-sheet (memorize; all bugs live here)

| Fact | Source |
|---|---|
| `Rigid3d` stores `[qx, qy, qz, qw, tx, ty, tz]`; maps a→b as `x_b = R x_a + t` | `src/colmap/geometry/rigid3.h:44-57` |
| Poses are `cam_from_world` (a=world, b=camera). Camera center `C = -R^T t` (`Image::ProjectionCenter()` → `TgtOriginInSrc()`, `rigid3.h:110`) | |
| Camera frame: +X right, +Y **down**, +Z forward (viewing direction) | COLMAP docs, `sensor/models.h` |
| `PosePrior::position` = sensor center in world; WGS84 = (lat°, lon°, alt m) or CARTESIAN | `src/colmap/geometry/pose_prior.h:61-66` |
| `PosePrior::gravity` = **unit down vector in the sensor/camera frame**. Level, upright camera ⇒ gravity ≈ (0, 1, 0) | `pose_prior.h:68-69` |
| Position prior residual = `prior_position − C` (`AbsolutePosePositionPriorCostFunctor`: `residual = prior + R⁻¹ t`) | `estimators/cost_functions/pose_prior.h:75-93` |
| WGS84→ENU happens once at DB-cache load, **only** when `convert_pose_priors_to_enu` is set; origin = **first** WGS84 prior; ENU = x-East, y-North, z-Up | `scene/database_cache.cc:467-503` |
| Position **covariance is never converted** — it is always interpreted in local metric (ENU) axes, even when the position is WGS84 | `database_cache.cc` (absence of conversion), `bundle_adjustment_ceres.cc:999-1007` |
| Rotation-averaging gravity world: `gravity_dir = +Y` (world Y points **down**) | `estimators/rotation_averaging.h:44-45` |
| `GravityAlignedRotation(g)` returns R with `col(1) = g` (maps world-Y-down to the camera-frame gravity); null-space basis via Householder QR — arbitrary but deterministic | `geometry/pose.cc:278-297` |
| 1-DOF yaw of a gravity frame: `R_cam_from_raWorld = GravityAlignedRotation(g) · R_y(θ)`; extract θ via `YAxisAngleFromRotation` | `geometry/pose.cc:299-305`, derivation in §4.F2 |
| `Reconstruction::Normalize()` applies **translation + scale only** (`Sim3d(scale, Identity, -scale·centroid)`) — rotations are never touched by normalization | `scene/reconstruction.cc:698-727` |

**Fixed change of basis between ENU and the RA gravity world** (needed by F2):

```
R_ra_from_enu = [ 1  0  0 ]      // East  -> x_ra
                [ 0  0 -1 ]      // Up    -> -y_ra  (gravity down = +y_ra)
                [ 0  1  0 ]      // North -> z_ra
det = +1 (right-handed). R_enu_from_ra = R_ra_from_enuᵀ.
```

### 1.3 New prior fields (added by F0)

`PosePrior` gains a full **rotation prior** instead of a scalar heading — the
producer already has fused attitude, average users (drones, phones) have
quaternions, and a heading-only user composes gravity+heading into a
quaternion with one helper:

- `rotation` — `Eigen::Quaterniond`, the prior **`cam_from_world`** rotation,
  where *world* is the same frame as `position` after ENU conversion (i.e.
  local ENU axes). NaN-coeffs = absent.
- `rotation_covariance` — `Eigen::Matrix3d`, rad². Convention: covariance of
  the small-angle error δ defined by `R_est = R_prior · Exp(δ̂)` with **δ
  expressed in world (ENU) axes** (right-perturbation of a cam_from_world
  matrix). Consequences:
  - δ = (δ_E, δ_N, δ_U); **yaw variance = cov(2,2)** (Up axis), tilt = the E/N
    block. Compass-limited attitude ⇒ anisotropic diagonal like
    `diag(σ_tilt², σ_tilt², σ_yaw²)`.
  - Sensor-frame error (what `AbsolutePosePriorCostFunctor`-style residuals
    produce, see §4.F5): ε = R_prior_cam_from_world · δ, so
    `cov_sensor = R_prior · cov_world · R_priorᵀ`.

Heading-only helper (part of F0, lives in `geometry/pose_prior.{h,cc}`):

```cpp
// Compose a cam_from_enu rotation prior from a camera-frame gravity (down)
// vector and a compass heading of the camera's viewing direction (radians,
// clockwise from North, i.e. standard compass bearing of +Z_cam projected
// onto the horizontal plane).
Eigen::Quaterniond RotationPriorFromGravityAndHeading(
    const Eigen::Vector3d& gravity_in_cam, double heading_rad);
```

Implement by: tilt from gravity (align cam +Y to ENU −Z via the minimal
rotation), then yaw about ENU Up so the horizontal projection of the camera +Z
axis has bearing `heading_rad`. Unit-test against hand-computed cases
(level camera facing North / East / Down-tilted 30° facing West).

### 1.4 The CSV contract (consumed by F1 `pose_prior_importer`)

One row per image. Header row required (column order fixed). Empty cell or
`nan` = field absent; a group (covariance, quaternion) must be all-present or
all-absent.

```
image_name,coord_system,px,py,pz,cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz,gx,gy,gz,qw,qx,qy,qz,rcov_xx,rcov_xy,rcov_xz,rcov_yy,rcov_yz,rcov_zz
```

| Group | Meaning | Validation on import |
|---|---|---|
| `image_name` | must match `images.name` in the database exactly | reject unknown names (warn + count), reject duplicate rows |
| `coord_system` | `WGS84` or `CARTESIAN` (case-insensitive) | all rows must agree — `DatabaseCache::ConvertPosePriorsToENU` THROWs on mixed systems (`database_cache.cc:481-482`) |
| `px,py,pz` | WGS84: lat°, lon°, alt m (ellipsoidal). CARTESIAN: meters, local ENU | lat ∈ [−90,90], lon ∈ [−180,180] |
| `cov_*` | position covariance, **m², ENU axes** (upper triangle, symmetric fill) | symmetric PSD (eigenvalues ≥ 0); reject otherwise |
| `gx,gy,gz` | gravity **down** vector, **camera frame** | normalize; warn if ‖g‖ deviates >1% from 1 |
| `qw,qx,qy,qz` | `cam_from_world(ENU)` quaternion, Hamilton, **w first in CSV** | normalize; warn if ‖q‖ deviates >0.1% |
| `rcov_*` | rotation covariance, **rad², world(ENU)-axes δ** per §1.3 | symmetric PSD |

Storage notes for the importer:
- The DB blob for `rotation` stores `Eigen::Vector4d` in **Eigen coeffs order
  (x, y, z, w)** — matching `Rigid3d::params` and `Quaterniond::coeffs()`.
  CSV is w-first for humans; the importer converts. Document in both places;
  this is the classic quaternion-order bug.
- Rotation priors are expressed in ENU **axes**, which are position-independent
  at drone scale (meridian convergence over a few km is negligible) — so they
  are valid whether or not positions get ENU-converted later.
- The importer **upserts**: the feature extractor may already have written
  EXIF-derived priors for the same images
  (`controllers/feature_extraction.cc:305-323` and `:645`,
  `controllers/image_reader.cc:302-319`). Match on existing
  `(corr_data_id, corr_sensor_id, corr_sensor_type)` and `UpdatePosePrior`,
  else `WritePosePrior`. `--clear_existing` wipes the table first.
- One prior per image, enforced: `ExtractFrameToPosePrior` THROWs on duplicates
  (`rotation_averaging_impl.cc:58-59`), and so does the unique index
  `pose_prior_data_assignment` (`database_sqlite.cc:2002-2003`).

---

## 2. Feature matrix

| ID | Feature | New flag(s) (defaults) | Depends on |
|----|---------|------------------------|------------|
| **F0** | `PosePrior.rotation` (+cov): struct, DB schema + migration, pycolmap | none — pure data-model extension | — |
| **F1** | `colmap pose_prior_importer` CLI (CSV → `pose_priors` table) | new command | F0 |
| **F2a** | Absolute yaw anchors in rotation averaging (compass/north lock) | `GlobalMapper.ra_use_rotation_priors=0` (+ same on `rotation_averager`) | F0; effective with `ra_use_gravity=1` |
| **F2b** | Initialize rotation averaging from prior rotations (skip MST init) | `GlobalMapper.ra_init_from_priors=0` | F0 |
| **F3** | Position priors + metric scale in `global_mapper` (upstream's own TODOs) | `GlobalMapper.use_prior_position=0`, `GlobalMapper.use_robust_loss_on_prior_position=0`, `GlobalMapper.prior_position_loss_scale=7.815`, `GlobalMapper.prior_position_fallback_stddev=1` | — |
| **F4** | Prior-gated + prior-seeded incremental registration | `Mapper.use_prior_pose_for_registration=0`, `Mapper.reg_prior_max_position_error=0` (≤0 = gate off) | pairs naturally with `Mapper.use_prior_position`; rotation seed needs F0 |
| **F5** | Rotation-prior residuals in pose-prior BA | `Mapper.use_prior_rotation=0` (and `GlobalMapper.use_prior_rotation=0` via F3), `prior_rotation_fallback_stddev_deg=5` | F0; runs inside the existing `PosePriorBundleAdjuster` |
| **F6** | *(optional, experimental, last)* sequential relative-pose smoothness | `Mapper.use_sequential_pose_smoothness=0` | F0 |

Metric-scale output ("feature 5" of the mission) is not a separate switch: it
falls out of F3 (global) and the existing incremental
`--Mapper.use_prior_position` path, both of which skip re-normalization and
align to priors, leaving the model in metric ENU. `model_aligner
--alignment_type enu --estimate_scale 1` then becomes a *validation* step whose
recovered scale must be ≈ 1.0.

---

## 3. Verified hook inventory (cross-check before every edit)

### 3.1 Data model & persistence

- `PosePrior` struct: `src/colmap/geometry/pose_prior.h:43-77` (fields:
  `pose_prior_id`, `corr_data_id`, `position`, `position_covariance`,
  `coordinate_system`, `gravity`; helpers `HasPosition/HasPositionCov/
  HasGravity`; `kNaN` sentinel pattern — copy it for the new fields).
- SQLite `CREATE TABLE pose_priors`: `scene/database_sqlite.cc:1991-2006`.
  **Trap:** CREATE order is `(…, position, position_covariance, gravity,
  coordinate_system)` but the SELECT statements return `coordinate_system` at
  index 6 and `gravity` at index 7 (`ReadPosePriorRow`,
  `database_sqlite.cc:401-418`). Column indices are defined by each SQL
  statement's explicit column list, **not** by table order. Grep every
  occurrence of `pose_priors` in `database_sqlite.cc` (read/write/update/
  exists) and extend each column list consistently; append new columns at the
  end of each list and read them at indices 8 (`rotation`) and 9
  (`rotation_covariance`).
- Migration hooks: `PreMigrateTables()` (`database_sqlite.cc:2068-2076`,
  legacy `image_id` rename) and `PostMigrateTables()`
  (`database_sqlite.cc:2078-2096`) — the `maybe_add_two_view_geometries_blob_
  column` lambda is the exact pattern to copy: `ALTER TABLE pose_priors ADD
  COLUMN rotation BLOB;` guarded by `ExistsColumn`. NULL blob → NaN field on
  read (verify `ReadStaticMatrixBlob` NULL behavior in a round-trip test with
  an old-schema DB).
- DB abstract API: `scene/database.h:171` (`ReadAllPosePriors`), `:216`
  (`WritePosePrior`); `UpdatePosePrior` impl `database_sqlite.cc:1491`.
- Cache: `DatabaseCache::PosePriors()` (`scene/database_cache.h:114`),
  `AddPosePrior` (`database_cache.cc:453`), `ConvertPosePriorsToENU`
  (`database_cache.cc:467-503`), `convert_pose_priors_to_enu` option
  (`database_cache.h:68`).
- **Trap:** the incremental pipeline sets `convert_pose_priors_to_enu =
  use_prior_position` (`controllers/incremental_pipeline.cc:72-73`); the
  global pipeline builds its cache at `controllers/global_pipeline.cc:78-83`
  and does **not** — F3 must add it there.
- pycolmap binding: `src/pycolmap/geometry/pose_prior.cc:16-44` — add
  `def_readwrite("rotation", …)`, `def_readwrite("rotation_covariance", …)`,
  `has_rotation`, `has_rotation_cov`; `MakeDataclass(PyPosePrior)` picks up the
  rest. Quaternion binding: follow how `Rigid3d.rotation` is bound elsewhere in
  pycolmap (grep `Eigen::Quaterniond` under `src/pycolmap/`).
- Tests-as-spec: `scene/database_test.cc:294` (pose-prior round trip),
  `geometry/pose_prior_test.cc` (EXIF gravity).

### 3.2 Cost functions (all in `estimators/cost_functions/pose_prior.h`)

- `AbsolutePosePriorCostFunctor` (6-DOF, residual in sensor frame; rotation
  part at lines 52-58 is the template for F5's rotation-only functor).
  **Currently dead code** — referenced only by its test. Same for
  `RelativePosePriorCostFunctor` (lines 139-171; 6-DOF relative, derivation in
  its comment) — F6's ready-made residual.
- `AbsolutePosePositionPriorCostFunctor` (3-DOF, lines 75-93) and
  `AbsoluteRigPosePositionPriorCostFunctor` (lines 96-126) — the live ones.
- `CovarianceWeightedCostFunctor<T>::Create(cov, …)` wrapper
  (`cost_functions/utils.h`) — whitens residuals with cov⁻¹ᐟ²; use it for every
  new residual. Test spec: `cost_functions/pose_prior_test.cc`.

### 3.3 Pose-prior BA (incremental path, works today — F3/F5 build on it)

`PosePriorBundleAdjuster`, `estimators/bundle_adjustment_ceres.cc:890-1091`.
Control flow (order matters, do not disturb):

1. ctor filters priors: needs `HasPosition()`, `SensorType::CAMERA`, image in
   config (lines 903-913).
2. `AlignReconstruction()` (1028-1080): Sim3 RANSAC of camera centers onto
   prior positions via `AlignReconstructionToPosePriors`
   (`estimators/alignment.cc:240-279`; needs ≥3 registered prior'd images).
   `max_error` auto-derived as `sqrt(kChiSquare95ThreeDof · median(trace(cov)/3))`
   (lines 1030-1052). On success the reconstruction is *transformed into the
   metric prior world* (rotation included).
3. `Normalize(fixed_scale=true)` → `normalized_from_metric_` (translation
   + scale only, rotation Identity — verified `reconstruction.cc:723`);
   priors are transformed per-residual instead (positions at 994-998,
   covariance conjugated by the scaled rotation at 999-1007).
4. Residuals: ref-in-frame images → 3-DOF functor on `rig_from_world`;
   non-ref → rig functor on `(cam_from_rig, rig_from_world)` (1009-1025).
5. `Solve()` transforms the reconstruction back to metric
   (`Transform(Inverse(normalized_from_metric_))`, line 960) — **output is
   metric ENU**.

Consumers: `sfm/incremental_mapper.cc:1144-1173` (requires >2 images in BA
config before switching to prior BA; Cauchy loss opt-in;
`prior_position_loss_scale` default 7.815 = χ²₉₅(3)); skip-normalize keyed on
`use_prior_position` at `incremental_mapper.cc:1256-1260`. Options struct:
`estimators/bundle_adjustment.h:252-261` (`prior_position_fallback_stddev`,
`alignment_ransac_options`). Factory dispatch:
`estimators/bundle_adjustment.cc:373-399`. Test spec:
`estimators/bundle_adjustment_ceres_test.cc:1443` (prior outliers),
`estimators/alignment_test.cc:103`, `controllers/incremental_pipeline_test.cc:499+`
(end-to-end with priors — the model for F3's pipeline test).

### 3.4 Rotation averaging (F2 ground)

- Options: `estimators/rotation_averaging.h:25-96` (`use_gravity`,
  `use_stratified`, `skip_initialization`, `max_rotation_error_deg`,
  `gravity_dir = UnitY`, `ridge_regularization` — its comment names *long
  sequential video chains* as the conditioning hazard).
- Problem build: `rotation_averaging_impl.cc:79-101`
  (`ExtractFrameToPosePrior` at 40-65: CAMERA-type, ref-in-frame only, THROW on
  duplicate).
- Gravity pair math: `BuildPairConstraints` — conjugation
  `R_ga(g2)ᵀ · R_c2_from_c1 · R_ga(g1)` (lines 281-291); both-gravity pairs
  become `GravityAligned1DOF{aa[1], aa[0]²+aa[2]²}` (302-303) — the second
  member is the off-axis energy used as a consistency weight/diagnostic.
- 1-DOF residual with wrap-to-[−π,π] + boundary jitter:
  `ComputeGravityAligned1DOFResidual` (17-38). Reuse its wrapping for anchors.
- Stratified two-phase solve: `MaybeSolveGravityAlignedSubset`
  (`rotation_averaging.cc:313`); entry `RunRotationAveraging`
  (`rotation_averaging.h:601` decl).
- Global-mapper call site: two-pass RA (all frames → filter → registered only)
  `sfm/global_mapper.cc:102-137`.
- CLI patterns to copy: `controllers/option_manager.cc:841-852`
  (`GlobalMapper.ra_use_gravity`, `ra_use_stratified`, enum options);
  standalone `rotation_averager` command `exe/sfm.cc:659+`.
- Test spec: `estimators/rotation_averaging_test.cc:81+`.

### 3.5 Global mapper (F3 ground)

- 5-stage `Solve()`: `sfm/global_mapper.cc:483`. Stage internals:
  `RotationAveraging` (102), `EstablishTracks` (139),
  `GlobalPositioning` (~280), `IterativeBundleAdjustment` (~342),
  `IterativeRetriangulateAndRefine` (~418).
- The three `Normalize()` TODO sites (verbatim: *"TODO: Skip normalization when
  position priors are used (similar to incremental mapper's
  !use_prior_position condition)"*): `global_mapper.cc:334-337`, `371-374`,
  `480-483`.
- Local BA helper to extend: `RunBundleAdjustment` (`global_mapper.cc:19-41`) —
  currently always `CreateDefaultBundleAdjuster` with
  `TWO_CAMS_FROM_WORLD` gauge.
- Options struct: `sfm/global_mapper.h:16+` (`GlobalMapperOptions`; note
  existing `BundleAdjustment()` composer at `global_mapper.cc:65-76`).
- Pipeline/controller: `controllers/global_pipeline.cc:78-91`
  (`DatabaseCache::Options` — add ENU flag here; `GlobalPipelineOptions`).

### 3.6 Incremental registration (F4 ground)

- `RegisterInitialImagePair` (`sfm/incremental_mapper.cc:150-187`): sets
  identity + `cam2_from_cam1` — gauge is arbitrary at init; do not try to seed
  world frame here (alignment happens in prior BA).
- `RegisterNextImage` (`incremental_mapper.cc:189-440ish`): 2D-3D
  correspondence harvest (244-303), `EstimateAbsolutePose` RANSAC (387-399),
  then refinement (`RefineAbsolutePose`, follows). Hook points for F4:
  after-RANSAC gate and on-failure prior-seeded fallback.
- Estimators: `estimators/pose.h` (`AbsolutePoseEstimationOptions`,
  `EstimateAbsolutePose`, `RefineAbsolutePose`) — read before implementing the
  fallback; the fallback re-uses `RefineAbsolutePose` on prior-pose inliers.

### 3.7 Alignment & scale

- `AlignReconstructionToPosePriors`: `estimators/alignment.cc:240-279`
  (RANSAC Sim3 if `max_error > 0`, else exact `EstimateSim3d`).
- `model_aligner`: `exe/model.cc:257` (`RunModelAligner`), ENU conversion
  `:103`; `AlignToENUPlane` `estimators/coordinate_frame.cc:355`.
- `GPSTransform` (WGS84↔ENU/ECEF): `geometry/gps.{h,cc}`.

### 3.8 Command / option registration

- Command table: `exe/colmap.cc:109-148` (`pose_prior_mapper` at 142).
- `RunPosePriorMapper`: `exe/sfm.cc:463-535` — option surface to mirror
  (`overwrite_priors_covariance`, `prior_position_std_x/y/z`,
  `use_robust_loss_on_prior_position`, `prior_position_loss_scale`);
  `UpdateDatabasePosePriorsCovariance` helper at `exe/sfm.cc:66-80`.
- Existing DB utility commands live in `exe/database.cc` — put
  `RunPosePriorImporter` there.

---

## 4. Feature specifications

### F0 — Rotation prior in the data model *(foundation; no behavior change)*

**Touch:** `geometry/pose_prior.{h,cc}`, `scene/database_sqlite.cc`,
`scene/database.h` (doc comment), `src/pycolmap/geometry/pose_prior.cc`,
tests.

1. Extend the struct (mirror the NaN-sentinel style exactly):

```cpp
// The prior rotation of the sensor as cam_from_world, where world is the
// same coordinate frame as `position` (local ENU axes when GPS priors are
// converted). Stored/serialized in Eigen coeffs order (x, y, z, w).
Eigen::Quaterniond rotation = Eigen::Quaterniond(kNaN, kNaN, kNaN, kNaN);
// Covariance (rad^2) of the right-perturbation error delta, expressed in
// world (ENU) axes: R_est = R_prior * Exp(delta^). cov(2,2) is yaw (Up).
Eigen::Matrix3d rotation_covariance = Eigen::Matrix3d::Constant(kNaN);

inline bool HasRotation() const { return rotation.coeffs().allFinite(); }
inline bool HasRotationCov() const { return rotation_covariance.allFinite(); }
```

   Update `operator==`, `operator<<` (`pose_prior.cc`), and add
   `RotationPriorFromGravityAndHeading` (§1.3).
2. DB: add `rotation BLOB` (Vector4d, coeffs order) and
   `rotation_covariance BLOB` (Matrix3d) columns — CREATE TABLE, every SQL
   column list (§3.1 trap), `ReadPosePriorRow` indices 8/9, write/update
   binds (NULL when `!HasRotation()`), `PostMigrateTables` add-column guards.
3. pycolmap bindings + dataclass.
4. **Tests:** extend `database_test.cc:294` round trip (absent + present);
   new migration test: build a DB with the old `CreatePosePriorTable` SQL,
   reopen, assert columns exist and legacy rows read with
   `HasRotation()==false`; `pose_prior_test.cc` cases for the heading helper.
   **Done when:** full suite green; a stock-4.1 database opens cleanly and
   round-trips.

### F1 — `colmap pose_prior_importer`

**Touch:** `exe/database.cc` (new `RunPosePriorImporter`), `exe/colmap.cc`
command table, docs.

Options: `--database_path` (required), `--import_path` CSV (required),
`--clear_existing` (default 0), `--dry_run` (parse+validate only, print
summary). Behavior per §1.4: build name→`Image` map from `ReadAllImages()`;
construct `corr_data_id` exactly as the feature-extraction writer does
(`controllers/feature_extraction.cc:305-323` — copy the `data_t`/sensor
construction, don't invent it); upsert inside a single
`DatabaseTransaction`. Print: rows parsed / matched / position priors /
gravity / rotation / skipped-unknown / rejected-invalid.

**Tests:** new `exe`-level or `scene`-level test writing a temp CSV
(WGS84 + CARTESIAN variants, missing groups, bad covariance, duplicate row,
unknown image) → import → `ReadAllPosePriors` assertions. **Done when:** a CSV
produced per §1.4 lands byte-correct in the DB and `pose_prior_mapper` runs
against it unchanged.

### F2a — Absolute yaw anchors in rotation averaging

**Goal:** compass heading fixes the global yaw gauge (north alignment) and
suppresses yaw drift on sequential chains. Grounded in the 1-DOF/circular-
regression formulation COLMAP's gravity RA already implements (Pan et al.,
ECCV 2024) — an absolute anchor is one extra row per prior'd frame in the same
1-DOF linear system.

**Touch:** `estimators/rotation_averaging.{h,cc}`,
`estimators/rotation_averaging_impl.{h,cc}`, `controllers/option_manager.cc`,
`exe/sfm.cc` (`rotation_averager`).

Mechanics (v1 constraints: anchors only for frames that have gravity **and**
rotation prior, `use_gravity=1` — matches the reference dataset where both are
always present):

1. Per anchored frame i compute once:
   `R_cam_from_ra = prior.rotation.toRotationMatrix() * R_enu_from_ra` (§1.2),
   `θ_prior_i = YAxisAngleFromRotation(GravityAlignedRotation(g_i)ᵀ · R_cam_from_ra)`.
   *Same `GravityAlignedRotation` call as the pair constraints — the Householder
   basis is deterministic and must match (`pose.cc:286`).* The product is only
   approximately a pure Y-rotation (gravity noise); `YAxisAngleFromRotation`'s
   aa[1] extraction is the right approximation, and the off-axis magnitude
   `aa[0]²+aa[2]²` should be logged like line 303 does.
2. Add one residual row per anchored frame to both L1 and IRLS phases:
   `w_i · wrap(θ_i − θ_prior_i)` with `w_i = 1/σ_yaw,i`,
   `σ_yaw,i = sqrt(rotation_covariance(2,2))` (fallback flag
   `ra_rotation_prior_default_yaw_std_deg`, default 10). Reuse the
   `std::remainder` wrapping of `ComputeGravityAligned1DOFResidual`.
3. Keep the existing gauge-fixing rows (`num_gauge_fixing_residuals_`) —
   anchors make yaw observable, but the gauge row is harmless and removing it
   changes conditioning; revisit only with evidence.
4. Flags: `RotationEstimatorOptions::use_rotation_priors=false`; plumb
   `GlobalMapper.ra_use_rotation_priors` (copy the `ra_use_gravity` pattern at
   `option_manager.cc:842-843`) and the same on the standalone
   `rotation_averager`.

**Tests:** clone the gravity test in `rotation_averaging_test.cc:81+`:
synthetic rotations + gravity + noisy yaw priors (σ=3°) → assert absolute
convergence to the anchored (ENU) frame, not merely up-to-gauge; a
no-priors run with the flag on must be identical to flag-off. **Done when:**
anchored synthetic yaw RMSE < prior σ and all stock RA tests pass.

### F2b — Initialize rotation averaging from priors

For a full-coverage ESKF track, MST initialization is both unnecessary and a
failure mode on weak graphs. When `ra_init_from_priors=1`: for every frame
with `HasRotation()`, set the initial rotation estimate to
`R_cam_from_ra` (converted as in F2a); frames without priors keep MST/identity
init; when **all** active frames have priors, set
`options.skip_initialization=true` (`rotation_averaging.h:69`). Implementation
site: where `RotationEstimator::EstimateRotations` seeds initial values
(`rotation_averaging.cc:279+` — read the init path first).
**Test:** clean synthetic with exact priors converges in ≤2 IRLS iterations;
corrupted-MST scenario (few matches) succeeds only with the flag.

### F3 — Position priors + metric scale in `global_mapper`

Upstream's own TODOs. **Touch:** `sfm/global_mapper.{h,cc}`,
`controllers/global_pipeline.cc`, `controllers/option_manager.cc`.

1. `GlobalMapperOptions`: add `bool use_prior_position = false;`,
   `bool use_robust_loss_on_prior_position = false;`,
   `double prior_position_loss_scale = 7.815;`,
   `double prior_position_fallback_stddev = 1.0;` (+ expose via
   `GlobalMapper.*` options; register beside the RA flags).
2. `controllers/global_pipeline.cc:78-83`: set
   `database_cache_options.convert_pose_priors_to_enu =
   options_.mapper.use_prior_position;` (mirror
   `incremental_pipeline.cc:72-73`).
3. Extend `RunBundleAdjustment` (`global_mapper.cc:19-41`): new parameters
   (priors + prior options + flag). When enabled and ≥3 registered images have
   position priors: `THROW_CHECK` Ceres backend (rule §0.2), build
   `PosePriorBundleAdjustmentOptions` exactly like
   `incremental_mapper.cc:1158-1172`, call `CreatePosePriorBundleAdjuster`,
   and do **not** call `ba_config.FixGauge` (the prior BA manages gauge
   itself — see §3.3 step 1-2 and `bundle_adjustment_ceres.cc:917-925`).
   Otherwise fall through to the current default path.
4. Guard the three `Normalize()` calls (`global_mapper.cc:337, 374, 483`) with
   `if (!options.use_prior_position)`.
5. Do **not** pre-align before BA — `PosePriorBundleAdjuster` aligns
   internally per solve (§3.3). Global positioning (stage 3) keeps its own
   init; seeding GP positions from priors is a possible v2
   (`gp_init_positions_from_priors`), not in scope now.

**Tests:** mirror `incremental_pipeline_test.cc:499+` for the global pipeline:
synthetic dataset (use the existing synthetic-scene helpers those tests use)
with WGS84 priors + known-metric ground truth → run global pipeline with the
flag → assert (a) camera centers within χ²-consistent distance of priors,
(b) pairwise-distance ratio vs ground truth ≈ 1.0 (metric scale preserved),
(c) flag-off run bit-identical to base. **Done when:** those pass and a
subsequent `model_aligner --alignment_type enu --estimate_scale 1` on the
output reports scale ∈ [0.99, 1.01] on the synthetic.

### F4 — Prior-gated and prior-seeded incremental registration

The highest-leverage feature for fisheye sequential video: 2D-3D RANSAC is the
flaky step; the prior both vetoes bad poses and rescues failed ones.

**Touch:** `sfm/incremental_mapper.{h,cc}`, `controllers/option_manager.cc`
(`Mapper.*`), `exe/sfm.cc` (`pose_prior_mapper` exposes both flags).

Options on `IncrementalMapper::Options`:
`bool use_prior_pose_for_registration = false;`
`double reg_prior_max_position_error = 0;  // meters; <=0 disables the gate`.

**Precondition — frame consistency.** The live reconstruction is only in the
metric prior (ENU) world after the first successful pose-prior global BA
(§3.3 step 5). Neither gate nor seed may fire before that. Implement an
explicit `bool reconstruction_aligned_to_priors_` on `IncrementalMapper`, set
true after each successful prior BA in `AdjustGlobalBundle`
(`incremental_mapper.cc:1201` return path), reset on `BeginReconstruction`/
model discard. Both mechanisms below require
`options.use_prior_position && reconstruction_aligned_to_priors_ &&` prior for
this image `HasPosition()`. This gating deserves its own unit test.

1. **Gate:** in `RegisterNextImage` after successful estimate+refine, compute
   `‖ProjectionCenter(cam_from_world) − prior.position‖`. If
   `reg_prior_max_position_error > 0` and the distance exceeds
   `max(reg_prior_max_position_error, k·sqrt(trace(cov)/3))` (k = 3, so tight
   thresholds never undercut honest GPS noise), reject the registration
   (VLOG the distance; return false — the image gets retried later like other
   failures).
2. **Seed / fallback:** if `EstimateAbsolutePose` fails (or the gate rejected
   a RANSAC pose) and the prior has rotation + position: build
   `cam_from_world_prior = Rigid3d(prior.rotation, −(prior.rotation * prior.position))`
   *(t = −R·C — derive, don't guess: C = −Rᵀt ⇒ t = −R·C)*. Count inliers of
   this pose over the harvested `tri_points2D/3D` using the camera model's
   reprojection with threshold `options.abs_pose_max_error`. If
   ≥ `abs_pose_min_num_inliers`, run `RefineAbsolutePose` on those inliers and
   accept under the same inlier checks the RANSAC path applies. VLOG which
   path registered the image.
3. Leave `RegisterInitialImagePair` untouched (arbitrary gauge at init is
   fine; alignment happens in the prior BA). Optional v2: prior-informed
   initial-pair *selection* in `IncrementalMapperImpl::FindInitialImagePair`.

**Tests:** synthetic incremental pipeline where (a) one image's matches are
corrupted so RANSAC yields a wrong pose → gate rejects (with gate off it
registers wrongly — assert the difference); (b) one image's matches are
thinned below RANSAC viability but the prior is good → fallback registers it;
(c) flags off ⇒ identical trajectory to base. **Done when:** those pass plus a
no-priors-in-DB run with flags on is a clean no-op.

### F5 — Rotation-prior residuals in pose-prior BA

**Touch:** `estimators/cost_functions/pose_prior.h`,
`estimators/bundle_adjustment_ceres.cc` (`PosePriorBundleAdjuster`),
`estimators/bundle_adjustment.{h,cc}` (options + Check), option registration,
`exe/sfm.cc`.

1. New functor (rotation part of `AbsolutePosePriorCostFunctor`, lines 52-58,
   with the translation rows deleted):

```cpp
// 3-DoF error on the absolute sensor rotation, computed in the sensor frame:
// residual = Log(R_sensor_from_world_est * R_world_from_sensor_prior).
struct AbsoluteRotationPriorCostFunctor
    : public AutoDiffCostFunctor<AbsoluteRotationPriorCostFunctor, 3, 7> { … };
```

2. In `PosePriorBundleAdjuster::AddImagePosePriorToProblem`: when
   `prior_options_.use_prior_rotation && pose_prior.HasRotation()`, add the
   rotation residual on `rig_from_world` — **v1 restricts to
   `image.IsRefInFrame()`** (for non-ref images the prior rotation would need
   `sensor_from_rig` composition; follow-up). Covariance:
   `cov_sensor = R_prior · cov_world · R_priorᵀ` (§1.3), fallback
   `deg2rad(prior_rotation_fallback_stddev_deg)²·I`; wrap in
   `CovarianceWeightedCostFunctor`, robust loss shared with the position term.
   **Frame validity:** the ctor's `AlignReconstruction()` has already rotated
   the reconstruction into the prior/ENU world, and `Normalize` applies
   identity rotation (`reconstruction.cc:723`) — so the prior rotation is used
   **untransformed**. Assert this understanding in a test, not a comment.
3. Also filter: the ctor's prior filter (lines 904-913) currently drops
   priors without position — relax to keep priors that have rotation when
   `use_prior_rotation` is set (a rotation-only prior is now meaningful), but
   `AlignReconstruction` must keep using only position priors.
4. Options: `PosePriorBundleAdjustmentOptions::use_prior_rotation=false`,
   `prior_rotation_fallback_stddev_deg=5`, `prior_rotation_loss_*` (reuse the
   position loss enum plumbing); update `Check()`
   (`bundle_adjustment.cc:368-371`). Expose on `pose_prior_mapper`
   (`exe/sfm.cc:463+`), `Mapper.use_prior_rotation`, and via F3 on
   `GlobalMapper.*`.

**Tests:** `pose_prior_test.cc`-style functor test (autodiff vs numeric;
identity ⇒ zero residual; known 10° yaw error ⇒ 10° residual about Up mapped
to sensor axes); `bundle_adjustment_ceres_test.cc:1443`-style: perturb
orientations, rotation priors pull them back; outlier rotation priors with
Cauchy loss don't wreck the solution; position-only behavior unchanged when
flag off. **Done when:** with the reference-profile noise (σ_yaw 3°, σ_tilt
1.5°) synthetic BA recovers orientations better than position-only BA, and
flag-off runs are bit-identical.

### F6 — Sequential relative-pose smoothness *(optional; build last; default off)*

Dense absolute priors already encode the track; add this only if F3+F4+F5
evaluation shows residual jitter between neighbors. Residual exists and is
tested: `RelativePosePriorCostFunctor` (`cost_functions/pose_prior.h:139-171`).
Design sketch: for consecutive frames (by `Frame` ordering / image name sort)
both having full priors, add the functor with
`i_from_j_prior = prior_i · Inverse(prior_j)` (as `Rigid3d`, translation in
metric ENU) weighted by a conservative relative covariance
(sum of absolute covariances; a proper relative-cov CSV column is v2).
Flag `Mapper.use_sequential_pose_smoothness`. Scale caveat: the functor's
translation residual assumes identical scale — only valid inside the
pose-prior BA where the reconstruction is metric-aligned. Gate accordingly.
**Do not start this without a failing evaluation case to justify it.**

---

## 5. Commit plan

| # | Commit | Contains | Merge gate |
|---|--------|----------|-----------|
| 1 | `F0 data model` | struct + DB + migration + pycolmap + tests | full suite green; old-DB migration test |
| 2 | `F1 importer` | command + tests + docs page | CSV fixtures round-trip |
| 3 | `F3 global priors` | flags, ENU wiring, prior BA in global mapper, skip-normalize | synthetic metric-scale test; TODO comments removed |
| 4 | `F2a RA anchors` (+`F2b` in same or next commit) | anchors, init-from-priors, flags | anchored-frame synthetic test |
| 5 | `F5 BA rotation` | functor + BA wiring + flags | functor + BA tests |
| 6 | `F4 registration` | gate + seed + alignment-state tracking | corruption/rescue tests |
| 7 | *(only with evidence)* `F6 smoothness` | | |

F3 before F2 deliberately: it is upstream's own TODO, self-contained, and
gives the earliest end-to-end win (metric global mapper) for real-data
validation of everything after it.

Each commit message: what/why, flags added, upstream-PR-sized. Update
`doc/` (there is a docs tree) with a `pose_priors.rst`-style page describing
the CSV format and flags once F1 lands.

---

## 6. Test & validation matrix

**Unit/integration (ctest)** — per feature, listed above. Cross-feature
combos that must run in the synthetic pipeline tests:

| Combo | Assert |
|---|---|
| all flags off | bit-identical to base (golden checksums where tests support it) |
| F3 only | metric ENU output, scale ≈ 1 |
| F3+F2a | metric + north-aligned (yaw of output vs ground truth < prior σ) |
| F3+F5 | orientation RMSE ≤ F3-only |
| F3+F4 | rescue/gate behavior; no regressions when priors are clean |
| F1 upsert after EXIF extraction | EXIF gravity preserved unless CSV overrides |

**Numerical hygiene:** every new residual gets an autodiff-vs-numeric Jacobian
check (pattern in `cost_functions/pose_prior_test.cc`); every new option in
`Check()`.

**CLI smoke (post-build):** `colmap help` lists `pose_prior_importer`;
`colmap pose_prior_importer --dry_run` on a fixture CSV; `colmap
global_mapper -h | findstr use_prior_position`; run `global_mapper` twice on a
small dataset with/without flags and diff `points3D.bin` — flag-off must match
base binary output.

---

## 7. Research grounding (why these designs)

- **Pan, Pollefeys, Baráth — “Gravity-Aligned Rotation Averaging with Circular
  Regression”, ECCV 2024** (arXiv:2410.12763). The basis of COLMAP's gravity
  RA (`rotation_averaging.h:13-15` cites it). Key takeaways used here: 1-DOF
  planar averaging with proper angle wrapping outperforms treating gravity as
  a soft auxiliary constraint; stratified solving handles partial gravity
  coverage. F2a's yaw anchors are absolute unary terms in exactly that 1-DOF
  system.
- **Pan et al. — “Global Structure-from-Motion Revisited” (GLOMAP), ECCV
  2024** (arXiv:2407.20219) — the `global_mapper` architecture. Its
  decoupled rotation→position design is why heading belongs in RA (F2) and
  GPS in positioning/BA (F3), not mixed.
- **MGSfM, ICCV 2025** (3dv-casia/MGSfM): global SfM specialized for
  sequential single/multi-camera captures — evidence that sequential-video
  global SfM benefits from strong per-frame constraints rather than bespoke
  temporal factors; supports the “dense absolute priors first, relative
  smoothness only if needed” ordering (F6 last).
- **“Making Rotation Averaging Fast and Robust with Anisotropic Coordinate
  Descent”, 2025** (arXiv:2506.01940): anisotropic uncertainty in rotation
  constraints is the right model — justification for the anisotropic
  `rotation_covariance` (tight tilt from gravity, loose yaw from compass)
  instead of a scalar orientation weight.
- **Robust GNSS fusion practice** (DCS/Cauchy kernels in GNSS-aided SLAM;
  COLMAP already ships Cauchy with scale 7.815 = χ²₉₅(3) at
  `incremental_mapper.h:151`): keep covariance-weighted residuals + Cauchy as
  the consumer-GPS default; per-frame covariance from the producer (ESKF/DOP)
  is strictly better than the global `--prior_position_std_*` override, which
  is why the importer makes per-frame covariance first-class and
  `overwrite_priors_covariance` stays a blunt fallback.

---

## 8. Building & validating on this machine (RTX 3070 Ti)

Builds are orchestrated by **`C:\splat\pipeline\build_gpu_colmap`** (lyehe's
build repo — vcpkg + CMake ExternalProject wrapper that produces COLMAP +
pycolmap with CUDA). It is *not* COLMAP; you never edit COLMAP code there. Its
`third_party/colmap` submodule (currently uninitialized, pinned to `fa8e3b3` =
v4.1.0 release) must be pointed at this fork.

**Machine facts (verified 2026-07-11):** RTX 3070 Ti = Ampere **SM 8.6**;
driver 596.36; CUDA Toolkit **12.8** installed at
`C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8` (`CUDA_PATH` set —
this is the sensible CUDA version: newest installed, first in the build
script's search list, fully supported by the driver); **no cuDSS** (optional
2-5× sparse-solver speedup; not required — skip); **Ninja not on PATH**
(required: `choco install ninja` or `winget install Ninja-build.Ninja`);
Visual Studio 2022 with C++ workload required — run everything from
**“Developer PowerShell for VS 2022”**.

### 8.1 One-time setup

```powershell
# From Developer PowerShell for VS 2022
cd C:\splat\pipeline\build_gpu_colmap
# Guard against system vcpkg interference (script warns too):
$env:VCPKG_ROOT = $null; $env:VCPKG_INSTALLATION_ROOT = $null
# Optional but recommended — persistent vcpkg binary cache:
$env:VCPKG_DEFAULT_BINARY_CACHE = "C:\vcpkg-cache"   # create the dir first

git submodule update --init third_party/vcpkg third_party/ceres-solver third_party/colmap

# Point the COLMAP submodule at the fork (local path remote is fine):
cd third_party\colmap
git remote add fork C:\splat\pipeline\colmap
git fetch fork
git checkout -B telemetry fork/telemetry
cd ..\..
```

**Build-time optimization (local edit, do not commit to the build repo):**
`build_gpu_colmap/CMakeLists.txt:62-66` **FORCE-caches**
`CMAKE_CUDA_ARCHITECTURES "75;80;86;89;90"` — a `-D` override will NOT stick.
For dev iteration, edit line 66 to `"86"` (this GPU only); restore before any
release build. This cuts CUDA compile time roughly 5×.

### 8.2 Build

```powershell
cd C:\splat\pipeline\build_gpu_colmap
.\scripts_windows\build_colmap.ps1 -Configuration Release
# -NoCuda for a quick CPU-only sanity build; -Clean to nuke build/
```

The script self-bootstraps vcpkg, builds Ceres then COLMAP (Ninja, ~25% of
cores), and copies CUDA 12.8 runtime DLLs into
`build\install\colmap\bin` (self-contained). First build: expect 1–2 h
(vcpkg); incremental rebuilds after COLMAP-only edits re-run just the COLMAP
ExternalProject and are fast. Binary: `build\install\colmap\bin\colmap.exe`.

Known issues (already handled by the script, don't fight them):
`-DGFLAGS_USE_TARGET_NAMESPACE=ON` is mandatory (gflags.lib link error
otherwise); Caspar stays ON (harmless — prior paths are Ceres-gated per §0.2).

### 8.3 Run the C++ tests

The orchestrator builds COLMAP with tests enabled (only the
colmap-for-pycolmap variant disables them). The COLMAP build dir is an
ExternalProject subdirectory — locate it once:

```powershell
Get-ChildItem C:\splat\pipeline\build_gpu_colmap\build -Recurse -Filter CTestTestfile.cmake |
  Select-Object -First 5 FullName
# then, from the colmap subbuild directory:
ctest -C Release -R "pose_prior|rotation_averaging|database|alignment|bundle_adjustment" --output-on-failure
```

Run the targeted set on every feature commit; run full `ctest` before
declaring a commit done.

### 8.4 pycolmap wheels (needed because F0 touches bindings)

```powershell
cd C:\splat\pipeline\build_gpu_colmap
git submodule update --init --recursive third_party/colmap-for-pycolmap
cd third_party\colmap-for-pycolmap
git remote add fork C:\splat\pipeline\colmap; git fetch fork; git checkout -B telemetry fork/telemetry
cd ..\..
.\scripts_windows\build_pycolmap_wheels.ps1     # builds for all installed Py 3.10+
python -c "import pycolmap; p = pycolmap.PosePrior(); print(p.has_rotation())"
```

### 8.5 Release-style validation

1. `colmap.exe help` — new command listed; `colmap.exe global_mapper -h` — new
   flags listed; flag-off run on a stock dataset matches the base build.
2. `python scripts/validate_caspar_sample.py --colmap build\install\colmap\bin\colmap.exe`
   (in the build repo) — confirms the CUDA/Caspar path wasn't disturbed.
3. End-user metric check on a real prior'd dataset: `pose_prior_importer` →
   `global_mapper --GlobalMapper.use_prior_position 1 --GlobalMapper.ra_use_gravity 1
   --GlobalMapper.ra_use_rotation_priors 1 --GlobalMapper.ba_backend ceres` →
   `model_aligner --alignment_type enu --estimate_scale 1` must report
   scale ≈ 1.0 and a small median position error consistent with the prior
   covariances.
