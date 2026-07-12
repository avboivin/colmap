.. _pose-priors:

Pose Priors
===========

COLMAP can constrain Structure-from-Motion with per-image pose priors
(position, gravity, and/or full attitude). Priors are stored in the
``pose_priors`` database table and consumed by the incremental and global
mappers when the corresponding flags are enabled (all default **off**).

CSV import
----------

Import priors with::

    colmap pose_prior_importer \
        --database_path /path/to/database.db \
        --import_path /path/to/priors.csv

Optional flags:

- ``--clear_existing 1`` — wipe ``pose_priors`` before import
- ``--dry_run 1`` — parse and validate only

CSV format (header required, column order fixed)::

    image_name,coord_system,px,py,pz,cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz,gx,gy,gz,qw,qx,qy,qz,rcov_xx,rcov_xy,rcov_xz,rcov_yy,rcov_yz,rcov_zz

| Group | Meaning |
|---|---|
| ``image_name`` | Must match ``images.name`` exactly |
| ``coord_system`` | ``WGS84`` or ``CARTESIAN`` (all rows must agree) |
| ``px,py,pz`` | WGS84: lat°, lon°, alt m; CARTESIAN: meters in local ENU |
| ``cov_*`` | Position covariance, m², ENU axes (upper triangle) |
| ``gx,gy,gz`` | Gravity **down** vector in the camera frame |
| ``qw,qx,qy,qz`` | ``cam_from_world`` quaternion (**w first** in CSV) |
| ``rcov_*`` | Rotation covariance, rad², world (ENU) right-perturbation |

Empty cell or ``nan`` means the field is absent. Covariance and quaternion
groups must be entirely present or entirely absent. The importer **upserts**
on the image's sensor data id (overrides EXIF-derived priors when present).

Quaternion note: CSV is w-first for humans; the database stores Eigen
coefficient order ``(x, y, z, w)``.

Mapper flags
------------

Incremental (``mapper`` / ``pose_prior_mapper``):

- ``Mapper.use_prior_position`` — metric position priors in BA
- ``Mapper.use_prior_rotation`` — rotation-prior residuals in pose-prior BA
- ``Mapper.use_prior_pose_for_registration`` — gate/seed registration with priors
- ``Mapper.reg_prior_max_position_error`` — gate threshold in meters (``<=0`` disables)

Global (``global_mapper``):

- ``GlobalMapper.use_prior_position`` — position priors + skip re-normalization
- ``GlobalMapper.use_prior_rotation`` — rotation priors in prior BA
- ``GlobalMapper.ra_use_rotation_priors`` — absolute yaw anchors in rotation averaging
- ``GlobalMapper.ra_init_from_priors`` — initialize RA from prior rotations

Pose-prior BA requires the Ceres backend (``--Mapper.ba_backend ceres`` /
``--GlobalMapper.ba_backend ceres``).
