# Spatio Temporal Voxel Layer

This package provides a Nav2-compatible voxel grid costmap plugin backed by
OpenVDB. The recent refactor introduces internal helpers to keep the top-level
layer focused on orchestration.

## Module layout

- `stvl_core` – ROS-neutral voxel grid logic living in
  `spatio_temporal_voxel_grid.*` and the `core/` headers.  It owns OpenVDB
  integration, occupancy elevation tracking, and the frustum models.
- `stvl_bridge` – ROS 2 bridge hosting the observation ingestion pipeline in
  `bridge/measurement_buffer.*` and the pruning heuristics in
  `internal/pruning_manager.*`.  Helper utilities under `bridge/` (for example
  `point_cloud_conversions.hpp`) concentrate ROS ↔ PCL conversions.
- `stvl_plugin` – the Nav2 costmap plugin in `spatio_temporal_voxel_layer.cpp`
  plus the `internal/observation_manager.*` orchestration layer.  It wires the
  ROS lifecycle, parameters, and publishers/subscribers on top of the bridge.
- `stvl_vdb2pc` – a small utility library in `vdb2pc.*` that converts archived
  OpenVDB volumes into PCL point clouds.
- Tests: `test/pruning_manager_test.cpp` exercises pruning decisions, while
  `test/measurement_buffer_test.cpp` validates the measurement buffering flow.

## Running the tests

After sourcing ROS Humble, build and test just this package with:

```bash
colcon build --packages-select spatio_temporal_voxel_layer
colcon test --packages-select spatio_temporal_voxel_layer
```

The bridge-level tests provide fast feedback when iterating on the pruning and
measurement buffering logic.
