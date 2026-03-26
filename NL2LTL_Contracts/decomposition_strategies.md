# Contract Decomposition Strategies

Given a system requirement (e.g., "3 robots deliver 5 boxes"), how should we decompose it into assume-guarantee contracts?

## Decomposition Dimensions

### Static Allocation
Tasks are pre-assigned to robots at specification time. No dispatcher needed.
- Robot1 -> box 1, 3; Robot2 -> box 2
- Each robot has a fixed task sequence
- Simplest contracts (fewest variables), but inflexible

### Dynamic Allocation
A Dispatcher observes robot completion and assigns tasks reactively.
- Any robot can handle any box
- Dispatcher contract: `G(ri_done & !all_done -> F(ri_task))`
- More variables (Dispatcher state), but fault-tolerant and adaptive

### Hierarchical Decomposition
Multi-layer: high-level Planner decomposes goals -> mid-level Coordinators assign sub-goals -> low-level Robots execute.
- Suitable for complex missions with nested sub-tasks
- Each layer has its own assume-guarantee contract
- Scales better than flat composition

### Spatial Decomposition
Partition by physical regions. Robot1 handles all tasks in Zone A, Robot2 handles Zone B.
- Similar to static but organized by geography
- Natural for warehouse/factory floor layouts
- Reduces inter-robot coordination

### Functional Decomposition
Partition by capability. Robot1 does transport, Robot2 does inspection, Robot3 manages charging.
- For heterogeneous robot teams
- Each robot type has a distinct contract structure
- Contracts reflect capability differences

### Temporal Decomposition
Partition by phase. Phase 1: all robots pick up; Phase 2: all robots deliver.
- Pipeline/batch processing model
- Synchronization contracts between phases
- Reduces concurrent complexity

### Hybrid
Combinations of the above. E.g., spatial partitioning first, then dynamic allocation within each zone.

## Choosing a Strategy

The decomposition strategy depends on system constraints, not the task itself:

| Constraint | Preferred Strategy |
|-----------|-------------------|
| Tasks known ahead of time | Static |
| Tasks arrive online | Dynamic |
| Homogeneous robots | Dynamic (any robot works) |
| Heterogeneous robots | Functional |
| Clear zone boundaries | Spatial |
| Fault tolerance required | Dynamic (can reassign) |
| Minimize contract complexity | Static (no Dispatcher) |
| Nested sub-tasks | Hierarchical |
| Phased operations | Temporal |

## Agent-Assisted Decomposition

For an LLM agent to assist with decomposition, a two-step approach:

### Step 1: Extract Decomposition Meta-Information

From the natural language requirement, extract:

1. **Agent structure**: number and type of robots (homogeneous/heterogeneous?)
2. **Task structure**: known ahead of time? arriving online? recurring?
3. **Shared resources**: contention? ordering constraints?
4. **Spatial constraints**: zones? collision avoidance?
5. **Failure modes**: what happens if a robot fails?

### Step 2: Select from a Template Library

Rather than inventing decomposition from scratch, the agent selects from a library of verified contract architecture templates. Each template specifies:

- Applicable conditions (when to use this pattern)
- Contract structure (which components, their interfaces)
- Variable ownership (what is input/output for each component)
- Assume-guarantee relationships

This follows the library-based contract synthesis approach (Iannopollo et al., DATE 2014).

## Research Direction

The key research question:

> Given a single NL requirement, can we automatically generate multiple contract decompositions and compare which is better?

This requires a full pipeline:

```
NL requirement
    |
    v
Decomposition strategy selection (which pattern?)
    |
    v
Contract generation (generate_contracts.py)
    |
    v
Synthesis (slugs) + Verification (nuxmv IC3)
    |
    v
Evaluation (compare decompositions)
```

### Evaluation Metrics

| Metric | Meaning |
|--------|---------|
| Contract variable count | Affects synthesis feasibility |
| FSM state count | Affects deployment efficiency |
| Fault tolerance | Can the system recover from robot failure? |
| Scalability | How much changes when adding a robot? |
| Verification time | IC3/BMC runtime for refinement check |

The `generate_contracts.py` script handles the middle steps (contract generation, synthesis/verification pipeline). The open problems are:
- **Step 1**: NL -> decomposition strategy (requires domain knowledge + constraint extraction)
- **Last step**: evaluation metrics and automated comparison
