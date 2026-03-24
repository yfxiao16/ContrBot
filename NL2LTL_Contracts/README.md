# NL2LTL Contracts

Task-level Assume-Guarantee contracts in CHASE `.ltl` format, derived from LTL
patterns in the [NL2LTL](../../Downloads/NL2LTL-main) dataset.

These contracts operate at the **task abstraction level** (as described in the
NL2RC paper), using propositions like `at_shelf`, `has_item`, `go_to_dock` --
NOT at the grid-coordinate level used in GridWorld.

## Abstraction Levels

| Layer | Example Props | Project |
|-------|--------------|---------|
| **Task-level** (this folder) | `at_shelf`, `pickup`, `has_item` | NL2LTL / NL2RC paper |
| **Grid-level** | `r1_x`, `r1_y`, `r1_action` | GridWorld |

## Contract Architecture

Each `.ltl` file contains **component contracts** and a **System contract**:

- **Component contracts** (Robot, PatrolRobot, Robot1/Robot2/Arbiter): synthesizable controllers. Environment physics (navigation liveness, task fairness, state persistence) go in component `Assumptions`. Robot behavioral commitments go in `Guarantees`.
- **System contract**: high-level specification for the composed system. `System.A` includes all environment assumptions that any component requires (including inter-component assumptions like grant fairness for CHASE compatibility). `System.G` specifies end-to-end requirements (task completion, resource safety, liveness).

> **Note**: Environment contracts were removed. Environment dynamics are placed directly in component `Assumptions` (for synthesis) and `System.A` (for verification). This follows the GridWorld pattern where environment state variables are System inputs and physical dynamics are System assumptions.

## Contracts

| File | Scenario | Components | Description |
|------|----------|------------|-------------|
| `agv_warehouse.ltl` | Warehouse AGV | `Robot`, `System` | Single robot: shelf pickup -> dock delivery |
| `agv_patrol.ltl` | Patrol & Respond | `PatrolRobot`, `System` | Robot patrols zones A/B/C and responds to alerts |
| `agv_multi_robot.ltl` | Multi-Robot AGV | `Robot1`, `Robot2`, `Arbiter`, `System` | Two robots with centralized task arbitration |

## LTL Pattern Origins (from NL2LTL datasets)

| Pattern | Meaning | Dataset Source |
|---------|---------|---------------|
| `G(p -> F q)` | Response | dataset_3000 id=3,9,16 |
| `G !p` | Invariance/Safety | dataset_3000 id=5,7 |
| `(!p) U q` | Precedence | dataset_3000 id=10,12 |
| `G(p -> X(X q))` | Bounded response | dataset_3000 id=6,11,18 |
| `G(p -> G p)` | Persistence | dataset_3000 id=2 |
| `G F p` | Recurrence | nested id=2,11 |
| `F G p` | Eventual persistence | nested id=6,7 |
| `G(p -> (!p U q))` | Until-response | dataset_3000 id=14,20 |
| `G !(p & q)` | Mutual exclusion | but_3000 id=4 |
| `G(p -> F G q)` | Stability response | but_long id=3,4 |

## CHASE Pipeline

Each contract has two CHASE command files:

### Synthesis (per-component controller generation)

| File | Synthesizes |
|------|-------------|
| `agv_warehouse_synthesis.chase` | `Robot` controller |
| `agv_patrol_synthesis.chase` | `PatrolRobot` controller |
| `agv_multi_robot_synthesis.chase` | `Robot1`, `Robot2`, `Arbiter` controllers |

Steps: `saturate` (converts LTL to GR(1)) -> `synthesize` (produces FSM via slugs)

### Verification (refinement check)

| File | Checks |
|------|--------|
| `agv_warehouse_verification.chase` | `Robot <= System` |
| `agv_patrol_verification.chase` | `PatrolRobot <= System` |
| `agv_multi_robot_verification.chase` | `Robot1 || Robot2 || Arbiter <= System` |

Steps: `saturate` all -> `compose` components (multi-robot only) -> `refinement` check (generates NuXMV `.smv`)

### Verification Results

| Case | Synthesis | Refinement (NuXMV) |
|------|-----------|-------------------|
| Warehouse | realizable | both specs TRUE |
| Patrol | realizable | both specs TRUE |
| Multi-robot | all 3 components realizable | spec 1 TRUE, spec 2 no counterexample in BMC k=30 |

### Usage

```bash
# Set up environment
export DYLD_LIBRARY_PATH="/path/to/third_party/antlr4/lib:$DYLD_LIBRARY_PATH"

# Synthesis: generate controller FSMs
logics_tool -i agv_warehouse.ltl -c agv_warehouse_synthesis.chase -o out_warehouse
# Compile and run slugs
python3 compiler.py out_warehouse/robot.structuredSlugs > out_warehouse/robot.slugsin
slugs --explicitStrategy --jsonOutput out_warehouse/robot.slugsin

# Verification: check refinement
logics_tool -i agv_warehouse.ltl -c agv_warehouse_verification.chase -o out_warehouse
nuxmv out_warehouse/refinement_warehouse.smv
```

## Dependencies

- [CHASE](https://github.com/) -- contract parsing, saturation, composition, refinement SMV generation
- [slugs](https://github.com/VerifiableRobotics/slugs) -- GR(1) reactive synthesis
- [NuXMV](https://nuxmv.fbk.eu) -- LTL model checking for refinement verification
- ANTLR4 runtime (libantlr4-runtime.4.5.4.dylib)
