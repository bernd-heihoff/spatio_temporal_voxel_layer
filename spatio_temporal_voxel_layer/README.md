# Spatio Temporal Voxel Layer

This package provides a Nav2-compatible voxel grid costmap plugin backed by
OpenVDB. The recent refactor introduces internal helpers to keep the top-level
layer focused on orchestration.

## Module layout

- `internal/pruning_manager.[hpp|cpp]` – encapsulates the logic that decides
  when to clip the voxel grid to a bounding box.  The class tracks the pruning
  configuration, robot motion thresholds, and the last computed bounding box so
  it can be unit-tested in isolation.
- `spatio_temporal_voxel_layer.cpp` – now delegates pruning decisions to the
  manager, which reduces the file size and simplifies future refactors for
  other subsystems (observation buffers, elevation updates, etc.).
- `test/pruning_manager_test.cpp` – gtests covering the pruning heuristics and
  bounding box generation.

## Running the tests

After sourcing ROS Humble, build and test just this package with:

```bash
colcon build --packages-select spatio_temporal_voxel_layer
colcon test --packages-select spatio_temporal_voxel_layer
```

The new unit tests provide fast feedback when iterating on the pruning logic.
