# Initial Source Migration

## Source repository

- Repository: `RoboPro2027/torobo_sim`
- Commit: `94acc1a`
- Package: `src/torobo_autonomy`

## Migration scope

The initial migration changes package ownership and naming while preserving runtime behavior.

Preserved contracts:

- `mission_runner`
- `/navigate_to_pose`
- `/experiment/events`
- `/cmd_vel`
- `odom`
- `base_footprint`
- Existing Behavior Tree Mission definitions
- Existing Nav2 odometry sandbox

## Deferred changes

The following changes are intentionally deferred:

- ABU 2027 field integration
- 3D perception integration
- Elevation-aware navigation
- Full 3D navigation
- Legacy Catalog removal
- Torobo runtime package deletion
