# Team Tasks and Contribution Tracking

This division follows the supplied assignment's suggested responsibilities, grouped into six deliverables. Each member should be able to explain the overall approach as well as their own work.

| Member | Assignment scope | Main deliverables | Status | Work score |
|---|---|---|---|---|
| Hitesh (Member 1) | Field environment and problem formulation | Grid/map model, field symbols and scenario, initial state/actions/transition/goal/cost documentation | Complete: `field_scenario.py` + `docs/FIELD_ENVIRONMENT.md` delivered; formal problem definition written; scenario loadable by all algorithms | Pending peer review / 100 |
| Dushyant (Member 2) | BFS and DFS | Both search implementations, shared result format, correctness notes for path completeness/optimality | Complete: BFS & DFS implemented; 25/25 tests pass; docs written | 90 / 100 provisional |
| Ricky (Member 3) | Greedy Best-First Search and heuristic design | Greedy implementation, Manhattan heuristic, explanation of assumptions and limitations | Awaiting work in this repo | 0 / 100 repo-progress points |
| Aashutosh (Member 4) | A* and crop-safe cost | A* implementation, crop buffer/proximity penalty model, examples showing safer route preference | Awaiting work in this repo | 0 / 100 repo-progress points |
| Sharad (Member 5) | Unknown-obstacle detection and replanning | Hidden obstacle scenario, simulated detection, known-map update, A* replanning and replan count | Complete: hidden obstacle detected; one replan; goal reached | 90 / 100 provisional |
| Sharad (Member 6) | Testing, visualization, evaluation, and documentation | Integration runner, field/path visualization, algorithm comparison, output files, README and focused checks | Implementation complete; peer integration review pending | 85 / 100 provisional |

Sharad's completion evidence: `src/cropsafe_pathplanning/replanning.py`, `tests/test_replanning.py` (2 checks passing), `results/algorithm_comparison.csv`, `results/replanning_summary.csv`, `results/screenshots/`, and the root `README.md`. Final integration review depends on the other members' implementations being delivered.

Hitesh's completion evidence: `src/cropsafe_pathplanning/field_scenario.py` (single-source-of-truth for the shared 12×18 field scenario; exports `build_field()` → `Field`, `build_field_map()` → `FieldMap`, `field_as_matrix()`, `render_field()`, and all constants: `START`, `GOAL`, `FIELD_HEIGHT`, `FIELD_WIDTH`, `PROTECTED_CELLS`, `KNOWN_OBSTACLES`, `HIDDEN_OBSTACLE`, `RISK_PENALTY`), `docs/FIELD_ENVIRONMENT.md` (formal AIMA-style problem definition: state space, initial state, goal test, 4-directional legal actions, transition model, uniform and risk-aware path-cost models, movement-rule confirmation table, start/goal validity table, data-flow diagram, ASCII field map, full symbol legend). Reviewer: pending peer review.

Dushyant's completion evidence: `src/cropsafe_pathplanning/bfs_dfs.py` (BFS + DFS, shared `FieldMap`/`SearchResult` interface, no field mutation, `raise_on_invalid` flag, optional `cost_fn`), `tests/test_bfs_dfs.py` (25 tests, all passing — reachable, unreachable, optimality, obstacle avoidance, reproducibility, traceability, demo field), `docs/BFS_DFS.md` (algorithm theory, API reference, 4 reproducible examples). Reviewer: pending peer review.

## Shared interface and integration expectations

- Agree on coordinate format, map representation, and movement rules before parallel implementation.
- Each search should use a consistent result structure containing at least `path`, `cost`, `expanded_nodes`, and `reached_goal`; elapsed time can be measured by the evaluation runner.
- Report no-path cases clearly. Do not count a failed route as a successful zero-length path.
- Use the same start, goal, known obstacles, and cost settings when comparing algorithms. Record any settings that differ.
- Keep the true map and robot-known map distinct for the unknown-obstacle experiment.
- Review each other's work and ensure the complete program runs from the documented setup before submission. Sharad's integration runner currently uses the shared starter modules; peer review against the members' implementations is pending.

## Score rubric (suggested)

The table is a provisional repository-progress score based only on deliverables visible in this repo as of this update. The four 0 scores mean no deliverable for that scope is recorded here yet; they are not a judgment about work outside the repo. Sharad's scores reflect completed work with peer integration review still pending. The team should revise these provisional scores after reviewing each member's work and contribution evidence.

| Criterion | Weight |
|---|---:|
| Completeness against assigned scope and acceptance evidence | 40 |
| Correctness and integration with agreed interfaces | 30 |
| Explanation, documentation, and ability to present the work | 20 |
| Collaboration, review, and timely handoff | 10 |
| **Total** | **100** |

Record reviewer notes or evidence beside each score when work is assessed. The score should reflect completed and reviewed work, not task difficulty alone.

## Identity details to complete

The assignment asks for full names and roll numbers. Add each member's official full name and roll number here before submission; only first names were provided in this request.
