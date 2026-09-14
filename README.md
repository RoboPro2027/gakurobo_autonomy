# gakurobo_autonomy

Autonomy, Behavior Tree, and navigation integration for Gakurobo 2027.

## Responsibilities

- Behavior Tree Mission Runner
- Navigation action adapters
- Structured experiment events
- Nav2 launch and configuration
- Future elevation-aware or 3D navigation integration

## Initial migration contract

The initial migration preserves:

- Executable: `mission_runner`
- Navigation action: `/navigate_to_pose`
- Event topic: `/experiment/events`
- Command topic: `/cmd_vel`
- Frames: `odom` and `base_footprint`
- Existing two-point Mission behavior

## Migration source

The initial implementation was migrated from:

- Repository: `RoboPro2027/torobo_sim`
- Source commit: `94acc1a`
- Source package: `src/torobo_autonomy`

The initial migration intentionally preserves the existing two-dimensional navigation behavior before ABU 2027 navigation is introduced.
