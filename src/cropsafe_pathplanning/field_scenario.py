"""Shared potato-field scenario for the CropSafe Path-Planning project.

This module is the **single source of truth** for the sample field that all
search algorithms (BFS, DFS, Greedy Best-First, A*) are benchmarked against.
Import :func:`build_field_map` to get a :class:`~cropsafe_pathplanning.types.FieldMap`
(the lightweight interface used by BFS/DFS/GBFS) or :func:`build_field` to get
the full :class:`~cropsafe_pathplanning.model.Field` object (which also carries
the step-cost/risk model used by A*).

Field layout  (12 rows × 18 columns, 0-based indexing)
------------------------------------------------------
::

    col →  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17
    row ↓
     0     .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
     1     .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
     2     .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
     3     .  C  P  P  P  P  P  P  P  P  P  P  P  P  P  P  C  .   ← crop/protected band
     4     .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
     5     .  .  .  .  .  X  .  .  .  .  .  .  X  .  .  .  .  .   ← known obstacles cols 5,12
     6     .  S  .  .  .  X  .  .  .  .  .  .  X  .  .  .  G  .   ← start (6,1) goal (6,16)
     7     .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
     8     .  C  P  P  P  P  P  P  P  P  P  P  P  P  P  P  C  .   ← crop/protected band
     9     .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
    10     .  .  .  .  .  .  .  .  .  ?  .  .  .  .  .  .  .  .   ← hidden obstacle (10,9)
    11     .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .

Symbol key
----------
  .  free traversable soil / path cell
  C  crop-row cell (traversable by algorithms that use FieldMap; treated as
     a risk-adjacent marker in Field.step_cost)
  P  protected crop cell — **blocked** (cannot be entered)
  X  known static obstacle — **blocked**
  ?  hidden/unknown obstacle — absent from the robot's initial known map;
     revealed at runtime during the online-replanning experiment
  S  robot start position  (6, 1)
  G  goal position         (6, 16)

Coordinate convention
---------------------
  All positions are ``(row, col)`` integer 2-tuples with **0-based** indexing.
  Row 0 is the top of the field; column 0 is the left edge.

Movement rules
--------------
  Four-directional cardinal movement only:
      UP    (row - 1, col)
      DOWN  (row + 1, col)
      LEFT  (row, col - 1)
      RIGHT (row, col + 1)
  Diagonal movement is **not** permitted.
  A move is legal iff the destination cell is in-bounds AND not blocked.

Cost model
----------
  Base step cost: 1.0 per move.
  Risk surcharge: +2.0 (configurable via ``risk_penalty``) when the
      destination cell is adjacent (4-connected) to any protected crop cell.
  Total step cost = 1.0 + (risk_penalty if adjacent_to_protected else 0.0)
  This keeps the Manhattan-distance heuristic admissible because
  risk_penalty ≥ 0 and the base move cost is 1.
"""

from __future__ import annotations

from cropsafe_pathplanning.model import Field, Position, demo_unknown_obstacle
from cropsafe_pathplanning.types import Coordinate, FieldMap

# ---------------------------------------------------------------------------
# Field dimensions
# ---------------------------------------------------------------------------

FIELD_HEIGHT: int = 12   # number of rows  (0 … 11)
FIELD_WIDTH: int = 18    # number of columns (0 … 17)

# ---------------------------------------------------------------------------
# Key positions
# ---------------------------------------------------------------------------

START: Coordinate = (6, 1)    # robot's initial position
GOAL: Coordinate = (6, 16)   # robot's target position

# ---------------------------------------------------------------------------
# Cell classification sets
# ---------------------------------------------------------------------------

#: Rows that contain crop plants — used for visualisation coloring.
#: Crop-row cells themselves are traversable but serve as the outer edge
#: of a protected band; the inner protected cells are blocked.
CROP_ROW_CELLS: frozenset[Coordinate] = frozenset(
    {(r, c) for r in (3, 8) for c in range(1, 17)}
)

#: Protected (inner) crop cells — **blocked**.
#: Columns 2–15 on rows 3 and 8 are strictly off-limits to protect the crop.
PROTECTED_CELLS: frozenset[Coordinate] = frozenset(
    {(r, c) for r in (3, 8) for c in range(2, 16)}
)

#: Statically known obstacles — **blocked** from the first planning step.
KNOWN_OBSTACLES: frozenset[Coordinate] = frozenset(
    {(5, 5), (6, 5), (5, 12), (6, 12)}
)

#: The single hidden obstacle revealed only during online execution.
#: It is NOT present in the robot's initial known map.
HIDDEN_OBSTACLE: Coordinate = demo_unknown_obstacle()   # (10, 9)

#: Risk surcharge applied when stepping adjacent to a protected cell.
RISK_PENALTY: float = 2.0


# ---------------------------------------------------------------------------
# Factory functions
# ---------------------------------------------------------------------------

def build_field(*, include_hidden_obstacle: bool = False) -> Field:
    """Return the shared :class:`~cropsafe_pathplanning.model.Field` scenario.

    Parameters
    ----------
    include_hidden_obstacle:
        When ``True`` the hidden obstacle at ``(10, 9)`` is added to the
        obstacle set, giving the *ground-truth* map used only for evaluation
        or visualisation.  When ``False`` (default) the robot's *known* map
        is returned — the hidden obstacle is absent.

    Returns
    -------
    Field
        The 12×18 potato-field scenario.
    """
    obstacles: set[Position] = set(KNOWN_OBSTACLES)
    if include_hidden_obstacle:
        obstacles.add(HIDDEN_OBSTACLE)
    return Field(
        height=FIELD_HEIGHT,
        width=FIELD_WIDTH,
        start=START,
        goal=GOAL,
        crop_rows=set(CROP_ROW_CELLS),
        protected=set(PROTECTED_CELLS),
        obstacles=obstacles,
        risk_penalty=RISK_PENALTY,
    )


def build_field_map(*, include_hidden_obstacle: bool = False) -> FieldMap:
    """Return a :class:`~cropsafe_pathplanning.types.FieldMap` for the shared scenario.

    The ``FieldMap`` merges protected cells and known obstacles into a single
    ``blocked_cells`` set so that BFS, DFS, and GBFS can consume it without
    depending on the richer ``Field`` model.

    Parameters
    ----------
    include_hidden_obstacle:
        Same semantics as in :func:`build_field`.

    Returns
    -------
    FieldMap
        Lightweight 12×18 grid with ``blocked_cells`` pre-populated.
    """
    blocked: set[Coordinate] = set(PROTECTED_CELLS) | set(KNOWN_OBSTACLES)
    if include_hidden_obstacle:
        blocked.add(HIDDEN_OBSTACLE)
    return FieldMap(
        height=FIELD_HEIGHT,
        width=FIELD_WIDTH,
        blocked_cells=blocked,
    )


def field_as_matrix(*, include_hidden_obstacle: bool = False) -> list[list[str]]:
    """Return the field as a 2-D list of single-character strings.

    The returned matrix can be passed directly to
    :meth:`~cropsafe_pathplanning.types.FieldMap.from_matrix` or printed for
    quick inspection.

    Symbol mapping (see module docstring for the full legend):

    * ``"."``  — free traversable cell
    * ``"C"``  — crop-row boundary cell (traversable, risk-adjacent)
    * ``"P"``  — protected crop cell (blocked)
    * ``"X"``  — known static obstacle (blocked)
    * ``"?"``  — hidden obstacle position (traversable in robot's known map)
    * ``"S"``  — robot start
    * ``"G"``  — robot goal
    """
    hidden: set[Coordinate] = set()
    if include_hidden_obstacle:
        hidden.add(HIDDEN_OBSTACLE)

    matrix: list[list[str]] = []
    for r in range(FIELD_HEIGHT):
        row: list[str] = []
        for c in range(FIELD_WIDTH):
            pos: Coordinate = (r, c)
            if pos == START:
                row.append("S")
            elif pos == GOAL:
                row.append("G")
            elif pos in PROTECTED_CELLS:
                row.append("P")
            elif pos in KNOWN_OBSTACLES:
                row.append("X")
            elif pos in hidden:
                row.append("?")
            elif pos in CROP_ROW_CELLS:
                row.append("C")
            else:
                row.append(".")
        matrix.append(row)
    return matrix


def render_field(*, include_hidden_obstacle: bool = False) -> str:
    """Render the field as a human-readable ASCII string.

    Returns
    -------
    str
        Multi-line string with column/row rulers and the symbol legend.
    """
    matrix = field_as_matrix(include_hidden_obstacle=include_hidden_obstacle)

    # Column header
    col_header = "      " + "".join(f"{c:3d}" for c in range(FIELD_WIDTH))
    separator = "    +" + "---" * FIELD_WIDTH + "-+"
    lines = [col_header, separator]

    for r, row in enumerate(matrix):
        row_str = " ".join(f" {ch}" for ch in row)
        lines.append(f" {r:2d} | {row_str} |")

    lines.append(separator)
    lines.append("")
    lines.append("Legend:  S=Start  G=Goal  .=Free  C=Crop-row  P=Protected(blocked)  X=Obstacle(blocked)  ?=Hidden obstacle")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Convenience re-exports for modules that need both interfaces
# ---------------------------------------------------------------------------

__all__ = [
    # Dimension constants
    "FIELD_HEIGHT",
    "FIELD_WIDTH",
    # Key positions
    "START",
    "GOAL",
    "HIDDEN_OBSTACLE",
    # Cell classification
    "CROP_ROW_CELLS",
    "PROTECTED_CELLS",
    "KNOWN_OBSTACLES",
    "RISK_PENALTY",
    # Factories
    "build_field",
    "build_field_map",
    "field_as_matrix",
    "render_field",
]
