# Design: motion-planning-cpp

A C++17 library of motion planning algorithms for autonomous driving, built
from first principles with no framework dependencies, plus a lightweight 2D
playground to run and visualize them.

## Goals

- Implement the planning algorithms that production autonomous-driving stacks
  are built on: graph search (A*, Hybrid A*), sampling (RRT, RRT*), and
  trajectory post-processing (smoothing / time parameterization).
- Keep every algorithm readable and testable in isolation — each one lives
  behind a small interface and is covered by unit tests.
- Make results reproducible and visual: every demo scenario dumps a JSON
  artifact that a Python script renders to an image.

## Non-goals

- Not a production planner: no ROS integration, no real-time guarantees.
- No perception or control — the input is an occupancy map, the output is a
  path/trajectory.
- No heavy dependencies: the core library uses the C++ standard library only.

## Architecture

Three layers, each depending only on the layer below:

```
apps/       demo executables (scenario setup, JSON export)
  │
include/mpl, src/   the library: map + planners
  │
tools/      Python visualization (reads JSON, writes PNG/GIF)
```

### Map layer

`mpl::GridMap` — a 2D occupancy grid with resolution and world origin.
Row-major `uint8_t` storage; conversions between world coordinates
(`Point2d`, meters) and grid indices (`GridIndex`, cells). Out-of-bounds
queries via `isFree()` return `false` rather than throwing, so planners can
probe neighbors without pre-checking bounds; mutation of out-of-bounds cells
throws.

### Planner layer

Every planner exposes one entry point:

```cpp
PlanResult plan(const GridMap& map, GridIndex start, GridIndex goal) const;
```

`PlanResult` carries success, the path (start and goal inclusive), and
`PlanStats` (nodes expanded, runtime, path cost) so demos and tests can make
quantitative assertions. Planner-specific knobs live in a per-planner config
struct passed at construction (e.g. `AStarConfig::heuristic_weight`).

Planned planners, in implementation order:

1. **A\*** — 4/8-connected grid search, octile/Manhattan heuristic, optional
   weighted mode. Diagonal moves must not cut corners.
2. **Hybrid A\*** — kinematically feasible search with a bicycle model,
   Reeds-Shepp analytic expansions; targets parking scenarios.
3. **RRT / RRT\*** — sampling-based planning in continuous space with goal
   biasing and rewiring.
4. **Trajectory smoothing** — gradient-based path smoothing, then velocity
   profile generation under acceleration limits.

### Demo & visualization layer

`apps/plan_demo` builds canned scenarios (open field with random obstacles,
U-shaped trap, narrow gap), runs a planner, prints a stats table, and writes
one JSON file per scenario to `out/`. `tools/visualize.py` renders those JSON
files to PNG images used in the README. JSON is written by hand (no
third-party JSON library) to keep the core dependency-free.

## Error handling

- Constructor arguments that make an object unusable (non-positive map
  dimensions or resolution) throw `std::invalid_argument`.
- Planning failures (occupied/out-of-bounds start or goal, unreachable goal)
  are not exceptions — they return `PlanResult{success = false}` because they
  are expected runtime outcomes.

## Testing

- GoogleTest via `find_package` with a `FetchContent` fallback.
- Each module gets its own test file; tests assert both behavior (paths are
  valid: every cell free, consecutive cells adjacent) and quantitative
  properties (optimal cost on known maps, corner-cutting is impossible,
  weighted A* never beats optimal cost).
- CI (GitHub Actions, Ubuntu) configures, builds, and runs the full test
  suite on every push and pull request.

## Roadmap

- [x] M0: scaffolding — CMake, clang-format, CI, design doc
- [x] M1: `GridMap` + A* with full test coverage, demo scenarios, README
- [x] M2: Hybrid A* — vehicle footprint collision, Reeds-Shepp curves,
      distance-field heuristic, analytic expansion, parking demos
- [ ] M3: RRT / RRT* + comparison benchmarks against A*
- [ ] M4: trajectory smoothing + velocity profiles
- [ ] M5: benchmark suite across scenario library, results table in README
