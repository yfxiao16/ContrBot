# GridWorld Multi-Robot Contract Synthesis Demo

Contract-based framework for synthesizing verified robot controllers on discrete 2D grids using GR(1) reactive synthesis.

## Overview

This demo generates temporal logic contracts for multi-robot grid world scenarios and synthesizes finite-state machine (FSM) controllers using the CHASE/Slugs toolchain. It supports:

- **Variable number of robots** (1 to N)
- **Variable grid sizes** (NxN with configurable obstacles)
- **Variable target assignment** (dispatcher assigns target coordinates at runtime)
- **Variable-position obstacles** (stationary but runtime-configurable obstacle positions)
- **Distributed control** (per-robot contracts composed in parallel)
- **Centralized control** (request-grant architecture with Arbiter)

## Grid World Model

```
         x=0         x=1         x=2
       +----------+----------+----------+
  y=0  | (0,0)    | (1,0)    | (2,0)    |
       | R1 Depot |   Free   | R2 Depot |
       +----------+----------+----------+
  y=1  | (0,1)    | (1,1)    | (2,1)    |
       |   Free   | OBSTACLE |   Free   |
       +----------+----------+----------+
  y=2  | (0,2)    | (1,2)    | (2,2)    |
       |   Free   |   Free   |   Free   |
       +----------+----------+----------+
```

- **5 actions**: Stay (0), North (1), South (2), East (3), West (4)
- Moves into boundaries or obstacles are clamped (robot stays in place)
- Obstacle at (1,1) by default; configurable via `--obstacles`

## Quick Start

### Prerequisites

- `logics_tool` (CHASE binary)
- `slugs` (GR(1) reactive synthesis)
- Python 3
- Optional: `nuxmv` for refinement checking

### Generate contracts and synthesize

```bash
cd demos/GridWorld
export DYLD_LIBRARY_PATH=../../third_party/antlr4/lib

# 1. Generate contract files
python3 generate_gridworld.py --num-robots 2 --variable-targets --output-dir out/

# 2. Parse and generate structuredSlugs specs
../../chase/bin/logics_tool -i out/specs.ltl -o out/ -c out/synthesis.chase

# 3. Compile and synthesize each robot
for r in robot1 robot2; do
    python3 ../../slugs/tools/StructuredSlugsParser/compiler.py \
        out/${r}.structuredSlugs > out/${r}.slugsin
    ../../slugs/src/slugs --explicitStrategy --jsonOutput \
        out/${r}.slugsin > out/${r}.json
done

# 4. (Optional) Verify composition refinement
../../chase/bin/logics_tool -i out/specs.ltl -o out/ -c out/verification.chase -V
nuxmv out/refinement.smv
```

Or use the all-in-one demo script:

```bash
./run_demo.sh /tmp/gridworld_output
```

## Contract Generator Usage

```
python3 generate_gridworld.py [OPTIONS]
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--grid-size N` | 3 | Grid dimension (NxN) |
| `--num-robots N` | 1 | Number of robots |
| `--obstacles x,y;...` | `1,1` | Obstacle cells |
| `--depots x,y;...` | auto (corners) | Depot position per robot |
| `--control-mode` | `distributed` | `distributed` or `centralized` |
| `--variable-targets` | off | Use variable target assignment (target_x, target_y as inputs) |
| `--variable-obstacles N` | 0 | Number of variable-position obstacles |
| `--fixed-stations x,y;...` | auto (opposite corners) | Fixed station per robot |
| `--output-dir DIR` | `.` | Output directory |

### Examples

```bash
# Single robot, variable targets, 3x3 grid
python3 generate_gridworld.py --num-robots 1 --variable-targets --output-dir out/single/

# 2 robots, distributed, fixed stations
python3 generate_gridworld.py --num-robots 2 --fixed-stations "2,2;0,2" --output-dir out/dist_fixed/

# 2 robots, centralized arbiter, variable targets
python3 generate_gridworld.py --num-robots 2 --control-mode centralized \
    --variable-targets --output-dir out/central/

# 2 robots with variable-position obstacle
python3 generate_gridworld.py --num-robots 2 --variable-obstacles 1 \
    --variable-targets --output-dir out/with_blocks/
```

## Contract Structure

### Assume-Guarantee Paradigm

Each contract consists of **Assumptions** (what the environment must satisfy) and **Guarantees** (what the controller promises). The contract structure differs between the two control modes.

### Distributed Mode

Each robot has its own contract and observes all other robots' positions.

**Robot Assumptions**:
- Movement dynamics (action → position update)
- Task lifecycle (arrive, persist until done, get cleared)
- Target validity and persistence (not on obstacle, fixed during active task)
- Variable obstacle constraints (stationary, no overlap with static obstacles/depots)
- Fairness on other robots (they eventually return to depot)

**Robot Guarantees**:
- Obstacle avoidance: never enter obstacle cells
- Done semantics: `done <-> (at target & task active)`
- Directional collision avoidance: don't move toward a neighbor cell occupied by another robot (one constraint per direction)
- Liveness: complete tasks and return to depot infinitely often

```
Robot1 || Robot2 || ... || RobotN  refines  System
```

### Centralized Mode (Request-Grant)

Each robot decides its own action but must get approval from the Arbiter before moving. The Arbiter has NO dynamics knowledge — it only compares next positions.

**Robot Assumptions**:
- Conditional dynamics (`grant` → position updates to `next_x/next_y`, otherwise stays)
- Task lifecycle (same as distributed)
- Target validity and persistence (same as distributed)
- Variable obstacle constraints (same as distributed)

**Robot Guarantees**:
- Next-position computation: correctly compute `next_x, next_y` from action and obstacles
- Request semantics: `action ≠ 0 ↔ request`
- Obstacle avoidance: `next_x/next_y` never points to an obstacle
- Done semantics: `done <-> (at target & task active)`
- Liveness: complete tasks and return to depot infinitely often

**Arbiter Assumptions**:
- Initially no requests

**Arbiter Guarantees**:
- Collision freedom: simultaneously granted moves never place two robots on the same cell
- Request guard: only grant when a request is present
- Fairness: every request is eventually granted

```
Robot1 || Robot2 || ... || RobotN || Arbiter  refines  System
```

## Generated Files

| File | Description |
|------|-------------|
| `specs.ltl` | LTL contract specifications |
| `verification.chase` | CHASE commands for composition and refinement check |
| `synthesis.chase` | CHASE commands for per-robot synthesis |
| `*.structuredSlugs` | Generated by logics_tool (intermediate) |
| `*.slugsin` | Compiled Slugs input (intermediate) |
| `*.json` | Synthesized FSM controllers |
| `refinement.smv` | NuSMV model for refinement checking |

## Toolchain Pipeline

```
specs.ltl ──[logics_tool]──> *.structuredSlugs ──[compiler.py]──> *.slugsin ──[slugs]──> *.json (FSM)
    │
    └──[logics_tool -V]──> refinement.smv ──[nuxmv]──> refinement result
```
