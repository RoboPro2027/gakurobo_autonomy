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

## Cooperative Mission Runner contract

<!-- EXECUTION_CONTRACT_V1_DOCS -->

`gakurobo_autonomy` executes generated runtime artifacts for one robot role. It is not the source of truth for Semantic Mission and does not compile or edit the cooperative graph.

TR and BR use independent Runner instances, XML files, blackboards, Observation caches, and deployment bindings. Runtime endpoints such as navigation Actions and object-operation ports must be provided by Runner parameters, generated manifests, or adapters rather than embedded in canonical Mission assets.

A Runner may consume only robot-local Observation and local mechanism state. It must not consume Field evaluation ground truth, the peer Runner state, or the peer blackboard. Cooperative synchronization is field-mediated.

The current runtime still exposes the migrated global `/navigate_to_pose` contract and only registers `LogMessage`, `Wait`, `NavigateToPose`, and `PublishEvent`. The v1 cooperative implementation must add robot-specific navigation binding, an Observation cache, `WaitForObservation`, `TransferObservedObject`, and release or placement lowering.

The normative contract is maintained in `gakurobo_sim_cooperative/src/gakurobo_mission/docs/EXECUTION_CONTRACT_V1.md`.
