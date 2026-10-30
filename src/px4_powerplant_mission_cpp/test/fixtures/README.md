# Frozen pre-deletion oracle fixtures

These fixtures preserve the behaviour of the retired Python mapper/controller
for C++ regression tests. They were generated from the last pre-deletion
checkout (`9931daf7`, 2026-10-30) before the Python legacy files were removed.

| File | Covers | Used by |
|---|---|---|
| `parameter_defaults.json` | Python `_declare_parameters` defaults for the two retained nodes; includes the intentional C++ `require_image_for_photo=true` override | `test_ros_contract.py` |
| `mapper_case.json` | Fixed depth/lidar mapping case: expected observed points, counts, revision | `test_ros_contract.py` |
| `ray_cases.json` | Eight exact-ray voxel/oracle cases: cells, classification counts, 2D grid, revision | `test_reference_parity.py` |
| `route_cases.json` | Eight random 3D known-free routes and expected paths | `test_reference_parity.py` |
| `planner_cases.json` | Frontier extraction, 3D gap path, basic/surface/clearance/unreachable exploration cases | `test_reference_parity.py` |

Rules:

- Do not regenerate or edit these files to make a failing C++ test pass. Treat a
  diff as a behaviour change and review it explicitly.
- If the fixtures must be regenerated, check out the pre-deletion commit in a
  separate worktree, run the legacy Python oracle there, and record the source
  commit in the change.
- The fixtures are test data only; runtime code must not import them.
