# NL2LTL Contracts

Task-level Assume-Guarantee contracts in CHASE `.ltl` format, derived from LTL
patterns in the [NL2LTL](../../Downloads/NL2LTL-main) dataset.

These contracts operate at the **task abstraction level** (as described in the
NL2RC paper), using propositions like `at_Shelf`, `has_Item`, `go_to_Dock` —
NOT at the grid-coordinate level used in GridWorld.

## Abstraction Levels

| Layer | Example Props | Project |
|-------|--------------|---------|
| **Task-level** (this folder) | `at_Shelf_1`, `pickup_X`, `has_X` | NL2LTL / NL2RC paper |
| **Grid-level** | `r1_x`, `r1_y`, `r1_action` | GridWorld |

## Contracts

| File | Scenario | Description |
|------|----------|-------------|
| `agv_warehouse.ltl` | Warehouse AGV | Single robot: shelf pickup → dock delivery |
| `agv_multi_robot.ltl` | Multi-Robot AGV | Two robots with task arbitration |
| `agv_patrol.ltl` | Patrol & Respond | Robot patrols zones and responds to alerts |

## LTL Pattern Origins (from NL2LTL datasets)

| Pattern | Meaning | Dataset Source |
|---------|---------|---------------|
| `G(p → F q)` | Response | dataset_3000 id=3,9,16 |
| `G ¬p` | Invariance/Safety | dataset_3000 id=5,7 |
| `(¬p) U q` | Precedence | dataset_3000 id=10,12 |
| `G(p → X(X q))` | Bounded response | dataset_3000 id=6,11,18 |
| `G(p → G p)` | Persistence | dataset_3000 id=2 |
| `G F p` | Recurrence | nested id=2,11 |
| `F G p` | Eventual persistence | nested id=6,7 |
| `G(p → (¬p U q))` | Until-response | dataset_3000 id=14,20 |
| `G ¬(p ∧ q)` | Mutual exclusion | but_3000 id=4 |
| `G(p → F G q)` | Stability response | but_long id=3,4 |

## CHASE Pipeline

Each contract has two CHASE command files:

### Synthesis (per-component controller generation)

| File | Synthesizes |
|------|-------------|
| `agv_warehouse_synthesis.chase` | `Robot` controller |
| `agv_multi_robot_synthesis.chase` | `Robot1`, `Robot2`, `Arbiter` controllers |
| `agv_patrol_synthesis.chase` | `PatrolRobot` controller |

Steps: `saturate` (converts LTL to GR(1)) → `synthesize` (produces FSM via Slugs)

### Verification (composition then refinement check)

| File | Checks |
|------|--------|
| `agv_warehouse_verification.chase` | `Environment ∥ Robot ⊑ System` |
| `agv_multi_robot_verification.chase` | `Robot1 ∥ Robot2 ∥ Arbiter ⊑ System` |
| `agv_patrol_verification.chase` | `PatrolEnvironment ∥ PatrolRobot ⊑ System` |

Steps: `saturate` all → `compose` components → `refinement` check (generates NuSMV `.smv`)

### Usage

```bash
# Synthesis: generate controller FSMs
logics_tool specs.ltl < agv_warehouse_synthesis.chase

# Verification: check composition refines system contract
logics_tool specs.ltl < agv_warehouse_verification.chase
# Then verify with NuSMV:
nusmv refinement_warehouse.smv
```
