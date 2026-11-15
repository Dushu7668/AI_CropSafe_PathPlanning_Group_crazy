# Hitesh — Member 1

## Scope

Design the potato-field grid environment and state the search problem formally.

## Tasks

- Define grid coordinates, traversable cells, crop rows, protected cells/buffers, known obstacles, start, and goal.
- Provide a sample field configuration shared by all algorithms.
- Specify initial state, legal actions, transition model, goal test, and path-cost model.
- Confirm movement rules with the team; the README currently proposes four-directional movement.
- Document the map representation and symbol legend for visualization.

## Acceptance evidence

- Other modules can load or construct the field scenario consistently.
- The formal problem definition matches implemented movement and cost rules.
- Start/goal validity and blocked-cell behavior are clear.

## Completion evidence

| Deliverable | File | Status |
|---|---|---|
| Grid coordinate system (12 × 18, 0-based, row-major) | `docs/FIELD_ENVIRONMENT.md` §1 | ✅ Done |
| Symbol legend (`.` `C` `P` `X` `?` `S` `G`) | `docs/FIELD_ENVIRONMENT.md` §2 | ✅ Done |
| Sample field config (all constants in one place) | `src/cropsafe_pathplanning/field_scenario.py` | ✅ Done |
| `build_field()` → `Field` for A\* / cost model | `src/cropsafe_pathplanning/field_scenario.py` | ✅ Done |
| `build_field_map()` → `FieldMap` for BFS/DFS/GBFS | `src/cropsafe_pathplanning/field_scenario.py` | ✅ Done |
| `field_as_matrix()` → 2-D list (loadable by any module) | `src/cropsafe_pathplanning/field_scenario.py` | ✅ Done |
| `render_field()` ASCII printout with legend | `src/cropsafe_pathplanning/field_scenario.py` | ✅ Done |
| Formal problem definition (AIMA style) | `docs/FIELD_ENVIRONMENT.md` §4 | ✅ Done |
| Initial state, goal test | `docs/FIELD_ENVIRONMENT.md` §4.2–4.3 | ✅ Done |
| Legal actions (4-directional confirmed) | `docs/FIELD_ENVIRONMENT.md` §4.4 | ✅ Done |
| Transition model | `docs/FIELD_ENVIRONMENT.md` §4.5 | ✅ Done |
| Path-cost model (uniform + risk-aware) | `docs/FIELD_ENVIRONMENT.md` §4.6 | ✅ Done |
| Start/goal validity and blocked-cell behaviour table | `docs/FIELD_ENVIRONMENT.md` §7 | ✅ Done |
| Movement-rule confirmation table (matches all src files) | `docs/FIELD_ENVIRONMENT.md` §5 | ✅ Done |
| Data-flow diagram (which modules load the scenario) | `docs/FIELD_ENVIRONMENT.md` §8 | ✅ Done |

### Key notes

- **Four-directional movement confirmed** for all algorithms (UP, DOWN, LEFT, RIGHT) — consistent with `types.CARDINAL_DIRECTIONS`, `model.Field.neighbors()`, and all search files.
- `field_scenario.py` is the **single source of truth** for the field scenario. It exposes both a `Field` and a `FieldMap` interface so every algorithm can load the same scenario without coupling to the other's dependency.
- The **Manhattan-distance heuristic remains admissible** under the risk-aware cost model because `risk_penalty ≥ 0` and the base step cost is 1 — so `h(n) = Manhattan(n, goal) ≤ true_cost(n, goal)` always holds.
- Start `(6, 1)` and goal `(6, 16)` are **validated at construction** (`Field.__post_init__`) and at search time (`_validate_endpoints` in `bfs_dfs.py`). Both are confirmed unblocked in the sample scenario.

## Contribution score

Pending peer review: ___ / 100. Record reviewer and evidence in `TEAM_TASKS.md` after completion.
