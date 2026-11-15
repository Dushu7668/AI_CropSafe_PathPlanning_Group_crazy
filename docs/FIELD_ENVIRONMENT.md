# Field Environment and Formal Problem Definition

> **Author:** Hitesh — Member 1  
> **Scope:** Potato-field grid environment design and formal search-problem statement  
> **Shared by:** BFS, DFS, Greedy Best-First Search, A\*, and the online-replanning module

---

## 1. Grid Coordinate System

The field is a **12 × 18 rectangular grid** (12 rows, 18 columns).

| Dimension | Value | Range |
|---|---|---|
| Rows (height) | 12 | 0 … 11 (top → bottom) |
| Columns (width) | 18 | 0 … 17 (left → right) |

Every cell is identified by an integer 2-tuple **`(row, col)`** with **0-based indexing**:

- **Row 0** is the top edge of the field.
- **Column 0** is the left edge of the field.
- The bottom-right corner is `(11, 17)`.

This convention matches Python's row-major matrix layout and is used consistently across `model.py`, `types.py`, `field_scenario.py`, and all algorithm implementations.

---

## 2. Cell Classification and Symbol Legend

Each cell belongs to exactly one of the following categories:

| Symbol | Category | Traversable? | Description |
|:---:|---|:---:|---|
| `.` | Free soil / path | ✅ Yes | Open traversable ground |
| `C` | Crop-row boundary | ✅ Yes | Outer edge of a crop band; traversable but adjacent to protected cells, so the step-cost model applies a risk surcharge to any step that arrives adjacent to protected cells |
| `P` | Protected crop cell | ❌ **Blocked** | Inner crop cells — strictly off-limits; entering this cell is illegal |
| `X` | Known static obstacle | ❌ **Blocked** | Physical obstacle known to the robot before planning begins |
| `?` | Hidden obstacle | ✅ Yes (in robot's initial map) | Unknown to the robot at planning time; discovered during online execution at `(10, 9)` |
| `S` | Start | ✅ Yes | Robot's initial position `(6, 1)` |
| `G` | Goal | ✅ Yes | Target destination `(6, 16)` |

> [!IMPORTANT]
> **Blocked cells** = `PROTECTED_CELLS ∪ KNOWN_OBSTACLES`.  
> The hidden obstacle `(10, 9)` is **not** in the robot's initial blocked set.  
> `Start` and `Goal` are always validated to be traversable before any search begins.

---

## 3. Sample Field Configuration (Shared Scenario)

All algorithms are benchmarked on this **fixed, deterministic** configuration.

### 3.1 Key Positions

| Role | Coordinate | Symbol |
|---|---|:---:|
| Robot start | `(6, 1)` | `S` |
| Goal | `(6, 16)` | `G` |
| Hidden obstacle (true map only) | `(10, 9)` | `?` |

**Manhattan distance from start to goal:** \|6−6\| + \|16−1\| = **15 steps** (ideal lower bound, ignoring obstacles).

### 3.2 Protected Crop Bands

Two horizontal bands protect potato-crop rows:

| Band | Protected rows | Protected columns | Cells blocked |
|---|---|---|---|
| Upper band | Row 3 | Cols 2 … 15 | 14 cells |
| Lower band | Row 8 | Cols 2 … 15 | 14 cells |

**Total protected cells:** 28 (all blocked).

Crop-row boundary cells `C` are the outer columns (col 1 and col 16) of rows 3 and 8 — traversable but risk-adjacent.

```python
PROTECTED_CELLS = {(r, c) for r in (3, 8) for c in range(2, 16)}
CROP_ROW_CELLS  = {(r, c) for r in (3, 8) for c in range(1, 17)}
```

### 3.3 Known Static Obstacles

Four cells are blocked from the start of planning (always visible to the robot):

| Obstacle | Position |
|---|---|
| Left pillar top | `(5, 5)` |
| Left pillar bottom | `(6, 5)` |
| Right pillar top | `(5, 12)` |
| Right pillar bottom | `(6, 12)` |

These form two vertical 2-cell pillars on the robot's direct east–west route at row 6, forcing a detour.

### 3.4 Field Map (ASCII Render)

```
      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17
    +-------------------------------------------------------+
  0 |  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
  1 |  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
  2 |  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
  3 |  .  C  P  P  P  P  P  P  P  P  P  P  P  P  P  P  C  . |  ← upper crop band
  4 |  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
  5 |  .  .  .  .  .  X  .  .  .  .  .  .  X  .  .  .  .  . |  ← pillar tops
  6 |  .  S  .  .  .  X  .  .  .  .  .  .  X  .  .  .  G  . |  ← start / goal / pillar bottoms
  7 |  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
  8 |  .  C  P  P  P  P  P  P  P  P  P  P  P  P  P  P  C  . |  ← lower crop band
  9 |  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
 10 |  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |  ← hidden obstacle at (10,9)
 11 |  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
    +-------------------------------------------------------+
```

> [!NOTE]
> The hidden obstacle at `(10, 9)` is not shown in the robot's known map (above).  
> The ground-truth visualisation (SVG) uses `?` / orange to mark it.

---

## 4. Formal Search Problem Definition

### 4.1 State Space

**State:** A single grid coordinate `(row, col)` representing the robot's current position.

- Total possible states: 12 × 18 = **216 cells**
- **Traversable states:** 216 − 28 (protected) − 4 (known obstacles) = **184 cells** (robot's known map)
- **True traversable states:** 184 − 1 (hidden obstacle) = **183 cells**

### 4.2 Initial State

```
s₀ = (6, 1)
```

The robot begins at row 6, column 1. This cell is validated to be in-bounds and unblocked.

### 4.3 Goal Test

```
goal_test(s) = True  iff  s == (6, 16)
```

The search succeeds when the robot reaches row 6, column 16. No partial-credit states exist.

### 4.4 Legal Actions

At any state `(r, c)`, the robot has **up to four** actions:

| Action | Δrow | Δcol | Destination |
|---|:---:|:---:|---|
| **UP** | −1 | 0 | `(r−1, c)` |
| **DOWN** | +1 | 0 | `(r+1, c)` |
| **LEFT** | 0 | −1 | `(r, c−1)` |
| **RIGHT** | 0 | +1 | `(r, c+1)` |

**Diagonal moves are not permitted.** An action is legal only if the resulting cell is:
1. Within bounds: `0 ≤ row < 12` and `0 ≤ col < 18`
2. Not in the blocked set: `destination ∉ (PROTECTED_CELLS ∪ KNOWN_OBSTACLES)`

> [!IMPORTANT]
> **Four-directional (cardinal) movement is confirmed** for all algorithms.  
> This was agreed in the README and implemented consistently in `types.CARDINAL_DIRECTIONS`, `model.Field.neighbors()`, `FieldMap.get_neighbors()`, and all search files.

### 4.5 Transition Model

```
result(s, a) = s′  where  s′ = s + Δ(a)
```

The transition is deterministic: applying action `a = (Δr, Δc)` to state `s = (r, c)` yields `s′ = (r + Δr, c + Δc)`, provided the action is legal. Illegal actions are simply not generated (the successor function yields only valid neighbors).

### 4.6 Path-Cost Model

#### Uniform cost (used by BFS and DFS)

Each legal move costs **1**. Total path cost = number of moves = `len(path) − 1`.

#### Risk-aware cost (used by Greedy Best-First and A\*)

```
step_cost(s, a, s′) = 1.0 + risk_surcharge(s′)

risk_surcharge(s′) = risk_penalty    if any 4-connected neighbor of s′ is in PROTECTED_CELLS
                   = 0.0             otherwise

risk_penalty = 2.0  (default, configurable)
```

This cost model is **consistent** (monotone) for A\*:
- Base cost ≥ 1 and risk surcharge ≥ 0, so `h(n) = Manhattan_distance(n, goal) ≤ step_cost(n, a, n′) + h(n′)`.
- Manhattan distance remains an admissible lower bound because every step costs at least 1 and no diagonal shortcuts exist.

#### Buffer / proximity effect

The risk surcharge discourages routes that skirt protected crop bands. A\* will prefer paths that stay away from rows 3 and 8 (or their adjacent rows 2, 4, 7, 9) even when such paths are geometrically longer.

---

## 5. Movement Rule Confirmation

| Property | Rule | Source |
|---|---|---|
| Directions | 4-directional cardinal (U/D/L/R) | `types.CARDINAL_DIRECTIONS`, `model.Field.neighbors()` |
| Bounds check | `0 ≤ row < height` and `0 ≤ col < width` | `model.Field.in_bounds()`, `FieldMap.is_within_bounds()` |
| Blocked check | `destination ∉ (protected ∪ obstacles)` | `model.Field.blocked`, `FieldMap.is_blocked()` |
| Deterministic order | UP → RIGHT → DOWN → LEFT | Ensures reproducible results across all algorithms |
| Start validity | `start ∉ blocked` checked at construction | `Field.__post_init__()`, `_validate_endpoints()` in `bfs_dfs.py` |
| Goal validity | `goal ∉ blocked` checked at construction | Same as above |

---

## 6. Loading the Scenario

### Via `field_scenario` module (recommended — works with all algorithms)

```python
from cropsafe_pathplanning.field_scenario import (
    build_field,         # → Field  (for A* / risk-cost model)
    build_field_map,     # → FieldMap (for BFS / DFS / GBFS)
    field_as_matrix,     # → list[list[str]] (2-D symbol grid)
    render_field,        # → str (ASCII printout)
    START, GOAL,
    FIELD_HEIGHT, FIELD_WIDTH,
    PROTECTED_CELLS, KNOWN_OBSTACLES, HIDDEN_OBSTACLE,
)

# Robot's known map (hidden obstacle absent)
field = build_field()
field_map = build_field_map()

# Ground-truth map (all obstacles visible)
true_field = build_field(include_hidden_obstacle=True)
```

### Via `model` module (legacy, used by demo and visualizer)

```python
from cropsafe_pathplanning.model import demo_field, demo_unknown_obstacle

field = demo_field()                   # robot's known map
hidden = demo_unknown_obstacle()       # (10, 9)
```

Both APIs return the **same underlying scenario**; `field_scenario` simply exposes every constant and both interface types in one place.

### From a 2-D matrix

```python
from cropsafe_pathplanning.field_scenario import field_as_matrix
from cropsafe_pathplanning.types import FieldMap

matrix = field_as_matrix()
field_map = FieldMap.from_matrix(matrix, blocked_symbols={"P", "X"})
```

---

## 7. Start / Goal Validity and Blocked-Cell Behaviour

| Situation | Behaviour |
|---|---|
| `start` or `goal` is out of bounds | `ValueError` raised at `Field` construction; `_validate_endpoints` returns failure `SearchResult` for `FieldMap`-based algorithms |
| `start` or `goal` is in `blocked` | Same as above |
| A cell is blocked mid-path | `neighbors()` / `get_neighbors()` never yields blocked cells — blocked cells cannot be entered |
| Blocked cell discovered at runtime | Robot detects it, calls `field.with_obstacles(new_set)` to build an updated known map, and replans with A\* from the current position |
| No path exists | Algorithm exhausts the frontier and returns `SearchResult(path=[], cost=inf, reached_goal=False)` |

---

## 8. Data-Flow Diagram

```
field_scenario.py
 ├─ build_field()       ──→  model.Field    ──→  A* (search.py), visualize.py
 ├─ build_field_map()   ──→  types.FieldMap ──→  BFS / DFS (bfs_dfs.py), GBFS (gbfs.py)
 └─ field_as_matrix()   ──→  list[list[str]] ──→  FieldMap.from_matrix()

model.demo_field()  ──→  model.Field  (same data, used by __main__ and replanning)
```

---

## 9. References

- Peter Hart, Nils Nilsson, Bertram Raphael, *A Formal Basis for the Heuristic Determination of Minimum Cost Paths* (1968) — formalises cost-optimal A\* and admissibility requirements.
- Russell & Norvig, *Artificial Intelligence: A Modern Approach* — AIMA search problem formulation (§3.1): state space, initial state, actions, transition model, goal test, path cost.
- Amit Patel, [Red Blob Games — Introduction to the A\* Algorithm](https://www.redblobgames.com/pathfinding/a-star/introduction.html) — Manhattan distance, heuristic consistency on grids.
