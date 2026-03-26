#!/usr/bin/env python3
"""
Generate parameterized multi-robot warehouse delivery contracts.

Two decomposition modes for the same system requirement:
  static  - Tasks pre-assigned to robots at spec time, no dispatcher
  dynamic - Dispatcher reactively assigns tasks to idle robots

Usage:
  python3 generate_contracts.py --robots 2 --boxes 3 --mode static
  python3 generate_contracts.py --robots 3 --boxes 5 --mode dynamic
  python3 generate_contracts.py --robots 2 --boxes 3 --mode both
"""

import argparse
import os


def box_assignment(n_robots, n_boxes):
    """Round-robin assignment of boxes to robots. Returns {robot_id: [box_indices]}."""
    assignment = {}
    for j in range(1, n_boxes + 1):
        r = ((j - 1) % n_robots) + 1
        assignment.setdefault(r, []).append(j)
    return assignment


# ---------------------------------------------------------------------------
# Static mode
# ---------------------------------------------------------------------------

def static_robot_contract(i, assigned_boxes):
    """Robot_i for static mode. Tracks delivery completions for assigned boxes."""
    p = f"r{i}"
    K = len(assigned_boxes)
    lines = []
    lines.append(f"CONTRACT Robot{i}:")
    lines.append(f"    # Robot {i}: delivers {K} assigned box(es): {assigned_boxes}")
    lines.append("")

    # Input variables
    lines.append(f"    input boolean variable {p}_task;")
    lines.append(f"    input boolean variable {p}_at_pickup;")
    lines.append(f"    input boolean variable {p}_has_box;")
    lines.append(f"    input boolean variable {p}_at_delivery;")
    lines.append(f"    input boolean variable {p}_delivered;")
    for j in range(1, K + 1):
        lines.append(f"    input boolean variable {p}_del_{j};")
    lines.append("")

    # Output variables
    lines.append(f"    output boolean variable {p}_go_pickup;")
    lines.append(f"    output boolean variable {p}_pick;")
    lines.append(f"    output boolean variable {p}_go_delivery;")
    lines.append(f"    output boolean variable {p}_drop;")
    lines.append(f"    output boolean variable {p}_done;")
    lines.append("")

    # done condition: last delivery flag
    done_cond = f"{p}_del_{K}"

    # Assumptions
    lines.append(f"    Assumptions:")
    lines.append(f"        # Initial state")
    for v in ["task", "at_pickup", "has_box", "at_delivery", "delivered"]:
        lines.append(f"        !{p}_{v};")
    for j in range(1, K + 1):
        lines.append(f"        !{p}_del_{j};")
    lines.append("")

    lines.append(f"        # Navigation liveness")
    lines.append(f"        G({p}_go_pickup -> F({p}_at_pickup));")
    lines.append(f"        G({p}_pick -> F({p}_has_box));")
    lines.append(f"        G({p}_go_delivery -> F({p}_at_delivery));")
    lines.append(f"        G({p}_drop -> F({p}_delivered));")
    lines.append("")

    lines.append(f"        # Task fairness: tasks come while deliveries remain")
    lines.append(f"        G F({p}_task | {done_cond});")
    lines.append(f"        G({p}_task & !{p}_delivered -> X({p}_task));")
    lines.append("")

    lines.append(f"        # Delivery tracking: sequential crediting")
    for j in range(1, K + 1):
        if j == 1:
            precond = f"{p}_done & !{p}_del_1"
        else:
            precond = f"{p}_done & {p}_del_{j-1} & !{p}_del_{j}"
        lines.append(f"        G({precond} -> X({p}_del_{j}));")
    lines.append("")

    lines.append(f"        # Delivery persistence: once credited, stays credited")
    for j in range(1, K + 1):
        lines.append(f"        G({p}_del_{j} -> X({p}_del_{j}));")
    lines.append("")

    # Guarantees
    lines.append(f"    Guarantees:")
    lines.append(f"        # Initial state")
    for v in ["go_pickup", "pick", "go_delivery", "drop", "done"]:
        lines.append(f"        !{p}_{v};")
    lines.append("")

    lines.append(f"        # Behavior chain: task -> pickup -> delivery")
    lines.append(f"        G({p}_task & !{p}_has_box & !{p}_at_pickup -> F({p}_go_pickup));")
    lines.append(f"        G({p}_at_pickup -> F({p}_pick));")
    lines.append(f"        G({p}_has_box -> F({p}_go_delivery));")
    lines.append(f"        G({p}_at_delivery & {p}_has_box -> F({p}_drop));")
    lines.append("")

    lines.append(f"        # Safety: actions require preconditions")
    lines.append(f"        G(!{p}_at_pickup -> !{p}_pick);")
    lines.append(f"        G(!{p}_at_delivery | !{p}_has_box -> !{p}_drop);")
    lines.append("")

    lines.append(f"        # Precedence: ordering constraints")
    lines.append(f"        (!{p}_go_delivery) U {p}_has_box;")
    lines.append(f"        (!{p}_drop) U {p}_at_delivery;")
    lines.append("")

    lines.append(f"        # Mutual exclusion of actions")
    lines.append(f"        G(!({p}_go_pickup & {p}_go_delivery));")
    lines.append(f"        G(!({p}_pick & {p}_drop));")
    lines.append("")

    lines.append(f"        # Done semantics")
    lines.append(f"        G({p}_delivered <-> {p}_done);")
    lines.append("")

    lines.append(f"        # Liveness: complete tasks or all assigned boxes done")
    lines.append(f"        G F({p}_done | {done_cond});")
    lines.append("")

    return "\n".join(lines)


def static_system_contract(n_robots, n_boxes, assignment):
    """System contract for static mode."""
    lines = []
    lines.append("CONTRACT System:")
    lines.append(f"    # Static allocation: {n_robots} robots, {n_boxes} boxes.")
    assign_desc = ", ".join(
        f"Robot{r}->[{','.join(str(b) for b in boxes)}]"
        for r, boxes in sorted(assignment.items())
    )
    lines.append(f"    # Assignment: {assign_desc}")
    lines.append("")

    # Input variables (environment state)
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        lines.append(f"    input boolean variable {p}_task;")
        lines.append(f"    input boolean variable {p}_at_pickup;")
        lines.append(f"    input boolean variable {p}_has_box;")
        lines.append(f"    input boolean variable {p}_at_delivery;")
        lines.append(f"    input boolean variable {p}_delivered;")
        K = len(assignment[i])
        for j in range(1, K + 1):
            lines.append(f"    input boolean variable {p}_del_{j};")
    lines.append("")

    # Output variables
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        lines.append(f"    output boolean variable {p}_go_pickup;")
        lines.append(f"    output boolean variable {p}_pick;")
        lines.append(f"    output boolean variable {p}_go_delivery;")
        lines.append(f"    output boolean variable {p}_drop;")
        lines.append(f"    output boolean variable {p}_done;")
    lines.append("")

    # all_done: conjunction of all robots' last delivery flags
    all_done_parts = [f"r{i}_del_{len(assignment[i])}" for i in range(1, n_robots + 1)]
    all_done = " & ".join(all_done_parts)

    # Assumptions
    lines.append("    Assumptions:")
    lines.append("        # Initial state")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        for v in ["task", "at_pickup", "has_box", "at_delivery", "delivered"]:
            lines.append(f"        !{p}_{v};")
        K = len(assignment[i])
        for j in range(1, K + 1):
            lines.append(f"        !{p}_del_{j};")
    lines.append("")

    lines.append("        # Navigation liveness")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        lines.append(f"        G({p}_go_pickup -> F({p}_at_pickup));")
        lines.append(f"        G({p}_pick -> F({p}_has_box));")
        lines.append(f"        G({p}_go_delivery -> F({p}_at_delivery));")
        lines.append(f"        G({p}_drop -> F({p}_delivered));")
    lines.append("")

    lines.append("        # Task fairness: tasks come while robot has boxes remaining")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        done_cond = f"{p}_del_{len(assignment[i])}"
        lines.append(f"        G F({p}_task | {done_cond});")
    lines.append("")

    lines.append("        # Task persistence")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        lines.append(f"        G({p}_task & !{p}_delivered -> X({p}_task));")
    lines.append("")

    lines.append("        # Delivery tracking")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        K = len(assignment[i])
        for j in range(1, K + 1):
            if j == 1:
                precond = f"{p}_done & !{p}_del_1"
            else:
                precond = f"{p}_done & {p}_del_{j-1} & !{p}_del_{j}"
            lines.append(f"        G({precond} -> X({p}_del_{j}));")
    lines.append("")

    lines.append("        # Delivery persistence")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        K = len(assignment[i])
        for j in range(1, K + 1):
            lines.append(f"        G({p}_del_{j} -> X({p}_del_{j}));")
    lines.append("")

    # Guarantees
    lines.append("    Guarantees:")
    lines.append("        # Initial controller state")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        for v in ["go_pickup", "pick", "go_delivery", "drop", "done"]:
            lines.append(f"        !{p}_{v};")
    lines.append("")

    lines.append("        # All boxes eventually delivered")
    lines.append(f"        F({all_done});")
    lines.append("")

    lines.append("        # Completion persistence: once all done, stays done")
    lines.append(f"        G(({all_done}) -> X({all_done}));")
    lines.append("")

    lines.append("        # Liveness per robot")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        done_cond = f"{p}_del_{len(assignment[i])}"
        lines.append(f"        G F({p}_done | {done_cond});")
    lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Dynamic mode
# ---------------------------------------------------------------------------

def dynamic_robot_contract(i, n_boxes):
    """Robot_i for dynamic mode. Generic worker, sees all_done signal."""
    p = f"r{i}"
    lines = []
    lines.append(f"CONTRACT Robot{i}:")
    lines.append(f"    # Robot {i}: generic pickup-deliver worker.")
    lines.append("")

    # Input variables
    lines.append(f"    input boolean variable {p}_task;")
    lines.append(f"    input boolean variable {p}_at_pickup;")
    lines.append(f"    input boolean variable {p}_has_box;")
    lines.append(f"    input boolean variable {p}_at_delivery;")
    lines.append(f"    input boolean variable {p}_delivered;")
    lines.append(f"    input boolean variable all_done;")
    lines.append("")

    # Output variables
    lines.append(f"    output boolean variable {p}_go_pickup;")
    lines.append(f"    output boolean variable {p}_pick;")
    lines.append(f"    output boolean variable {p}_go_delivery;")
    lines.append(f"    output boolean variable {p}_drop;")
    lines.append(f"    output boolean variable {p}_done;")
    lines.append("")

    # Assumptions
    lines.append(f"    Assumptions:")
    lines.append(f"        # Initial state")
    for v in ["task", "at_pickup", "has_box", "at_delivery", "delivered"]:
        lines.append(f"        !{p}_{v};")
    lines.append(f"        !all_done;")
    lines.append("")

    lines.append(f"        # Navigation liveness")
    lines.append(f"        G({p}_go_pickup -> F({p}_at_pickup));")
    lines.append(f"        G({p}_pick -> F({p}_has_box));")
    lines.append(f"        G({p}_go_delivery -> F({p}_at_delivery));")
    lines.append(f"        G({p}_drop -> F({p}_delivered));")
    lines.append("")

    lines.append(f"        # Task fairness: tasks come while boxes remain")
    lines.append(f"        G F({p}_task | all_done);")
    lines.append(f"        G({p}_task & !{p}_delivered -> X({p}_task));")
    lines.append("")

    lines.append(f"        # Completion persistence")
    lines.append(f"        G(all_done -> X(all_done));")
    lines.append("")

    # Guarantees
    lines.append(f"    Guarantees:")
    lines.append(f"        # Initial state")
    for v in ["go_pickup", "pick", "go_delivery", "drop", "done"]:
        lines.append(f"        !{p}_{v};")
    lines.append("")

    lines.append(f"        # Behavior chain: task -> pickup -> delivery")
    lines.append(f"        G({p}_task & !{p}_has_box & !{p}_at_pickup -> F({p}_go_pickup));")
    lines.append(f"        G({p}_at_pickup -> F({p}_pick));")
    lines.append(f"        G({p}_has_box -> F({p}_go_delivery));")
    lines.append(f"        G({p}_at_delivery & {p}_has_box -> F({p}_drop));")
    lines.append("")

    lines.append(f"        # Safety: actions require preconditions")
    lines.append(f"        G(!{p}_at_pickup -> !{p}_pick);")
    lines.append(f"        G(!{p}_at_delivery | !{p}_has_box -> !{p}_drop);")
    lines.append("")

    lines.append(f"        # Precedence: ordering constraints")
    lines.append(f"        (!{p}_go_delivery) U {p}_has_box;")
    lines.append(f"        (!{p}_drop) U {p}_at_delivery;")
    lines.append("")

    lines.append(f"        # Mutual exclusion of actions")
    lines.append(f"        G(!({p}_go_pickup & {p}_go_delivery));")
    lines.append(f"        G(!({p}_pick & {p}_drop));")
    lines.append("")

    lines.append(f"        # Done semantics")
    lines.append(f"        G({p}_delivered <-> {p}_done);")
    lines.append("")

    lines.append(f"        # Liveness: complete tasks or all boxes done")
    lines.append(f"        G F({p}_done | all_done);")
    lines.append("")

    return "\n".join(lines)


def dynamic_dispatcher_contract(n_robots):
    """Dispatcher for dynamic mode. Assigns tasks to idle robots."""
    lines = []
    lines.append("CONTRACT Dispatcher:")
    lines.append("    # Reactive task dispatcher: assigns tasks while boxes remain.")
    lines.append("")

    # Input: robot done signals + all_done
    for i in range(1, n_robots + 1):
        lines.append(f"    input boolean variable r{i}_done;")
    lines.append(f"    input boolean variable all_done;")
    lines.append("")

    # Output: task assignments
    for i in range(1, n_robots + 1):
        lines.append(f"    output boolean variable r{i}_task;")
    lines.append("")

    # Assumptions
    lines.append("    Assumptions:")
    for i in range(1, n_robots + 1):
        lines.append(f"        !r{i}_done;")
    lines.append(f"        !all_done;")
    lines.append("")
    lines.append(f"        # Completion persistence")
    lines.append(f"        G(all_done -> X(all_done));")
    lines.append("")

    # Guarantees
    lines.append("    Guarantees:")
    for i in range(1, n_robots + 1):
        lines.append(f"        !r{i}_task;")
    lines.append("")

    lines.append("        # Reactive assignment: completed robot gets new task while boxes remain")
    for i in range(1, n_robots + 1):
        lines.append(f"        G(r{i}_done & !all_done -> F(r{i}_task));")
    lines.append("")

    lines.append("        # Fairness: every robot gets work while boxes remain")
    for i in range(1, n_robots + 1):
        lines.append(f"        G F(r{i}_task | all_done);")
    lines.append("")

    return "\n".join(lines)


def dynamic_system_contract(n_robots, n_boxes):
    """System contract for dynamic mode."""
    lines = []
    lines.append("CONTRACT System:")
    lines.append(f"    # Dynamic allocation: {n_robots} robots, {n_boxes} boxes.")
    lines.append("")

    # Input variables
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        lines.append(f"    input boolean variable {p}_at_pickup;")
        lines.append(f"    input boolean variable {p}_has_box;")
        lines.append(f"    input boolean variable {p}_at_delivery;")
        lines.append(f"    input boolean variable {p}_delivered;")
    for j in range(1, n_boxes + 1):
        lines.append(f"    input boolean variable b{j}_done;")
    lines.append(f"    input boolean variable all_done;")
    lines.append("")

    # Output variables
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        lines.append(f"    output boolean variable {p}_task;")
        lines.append(f"    output boolean variable {p}_go_pickup;")
        lines.append(f"    output boolean variable {p}_pick;")
        lines.append(f"    output boolean variable {p}_go_delivery;")
        lines.append(f"    output boolean variable {p}_drop;")
        lines.append(f"    output boolean variable {p}_done;")
    lines.append("")

    # all boxes done conjunction
    all_boxes = " & ".join(f"b{j}_done" for j in range(1, n_boxes + 1))
    # any robot done disjunction
    any_done = " | ".join(f"r{i}_done" for i in range(1, n_robots + 1))

    # Assumptions
    lines.append("    Assumptions:")
    lines.append("        # Initial state")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        for v in ["at_pickup", "has_box", "at_delivery", "delivered"]:
            lines.append(f"        !{p}_{v};")
    for j in range(1, n_boxes + 1):
        lines.append(f"        !b{j}_done;")
    lines.append(f"        !all_done;")
    # Initial state for outputs that are also needed
    for i in range(1, n_robots + 1):
        lines.append(f"        !r{i}_task;")
        lines.append(f"        !r{i}_done;")
    lines.append("")

    lines.append("        # Navigation liveness")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        lines.append(f"        G({p}_go_pickup -> F({p}_at_pickup));")
        lines.append(f"        G({p}_pick -> F({p}_has_box));")
        lines.append(f"        G({p}_go_delivery -> F({p}_at_delivery));")
        lines.append(f"        G({p}_drop -> F({p}_delivered));")
    lines.append("")

    lines.append("        # Task fairness (needed for CHASE compose compatibility)")
    for i in range(1, n_robots + 1):
        lines.append(f"        G F(r{i}_task | all_done);")
    lines.append("")

    lines.append("        # Task persistence")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        lines.append(f"        G({p}_task & !{p}_delivered -> X({p}_task));")
    lines.append("")

    lines.append("        # Box persistence: once delivered, stays delivered")
    for j in range(1, n_boxes + 1):
        lines.append(f"        G(b{j}_done -> X(b{j}_done));")
    lines.append("")

    lines.append("        # Delivery crediting: sequential box completion")
    for j in range(1, n_boxes + 1):
        if j == 1:
            precond = f"({any_done}) & !b1_done"
        else:
            prev_done = " & ".join(f"b{k}_done" for k in range(1, j))
            precond = f"({any_done}) & {prev_done} & !b{j}_done"
        lines.append(f"        G({precond} -> X(b{j}_done));")
    lines.append("")

    lines.append("        # all_done semantics")
    lines.append(f"        G(all_done <-> ({all_boxes}));")
    lines.append("")

    # Guarantees
    lines.append("    Guarantees:")
    lines.append("        # Initial controller state")
    for i in range(1, n_robots + 1):
        p = f"r{i}"
        for v in ["task", "go_pickup", "pick", "go_delivery", "drop", "done"]:
            lines.append(f"        !{p}_{v};")
    lines.append("")

    lines.append("        # All boxes eventually delivered")
    lines.append(f"        F(all_done);")
    lines.append("")

    lines.append("        # Completion persistence")
    lines.append(f"        G(all_done -> X(all_done));")
    lines.append("")

    lines.append("        # Liveness per robot")
    for i in range(1, n_robots + 1):
        lines.append(f"        G F(r{i}_done | all_done);")
    lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# LTL file assembly
# ---------------------------------------------------------------------------

def generate_ltl(n_robots, n_boxes, mode):
    """Generate complete .ltl file content."""
    lines = []
    tag = "Static" if mode == "static" else "Dynamic"
    lines.append(f"NAME: AGV_{tag}_{n_robots}R_{n_boxes}B;")
    lines.append("")

    assignment = box_assignment(n_robots, n_boxes)

    if mode == "static":
        assign_desc = ", ".join(
            f"Robot{r} -> box {','.join(str(b) for b in boxes)}"
            for r, boxes in sorted(assignment.items())
        )
        lines.append(f"# Static decomposition: {n_robots} robots, {n_boxes} boxes.")
        lines.append(f"# Pre-assignment (round-robin): {assign_desc}")
        lines.append(f"# Each robot tracks delivery completions for its assigned boxes.")
        lines.append(f"# System terminates when all robots finish their assignments.")
    else:
        lines.append(f"# Dynamic decomposition: {n_robots} robots, {n_boxes} boxes.")
        lines.append(f"# Dispatcher assigns tasks reactively to idle robots.")
        lines.append(f"# System terminates when all {n_boxes} boxes are delivered.")
    lines.append("")

    if mode == "static":
        for i in range(1, n_robots + 1):
            lines.append(static_robot_contract(i, assignment[i]))
        lines.append(static_system_contract(n_robots, n_boxes, assignment))
    else:
        for i in range(1, n_robots + 1):
            lines.append(dynamic_robot_contract(i, n_boxes))
        lines.append(dynamic_dispatcher_contract(n_robots))
        lines.append(dynamic_system_contract(n_robots, n_boxes))

    return "\n".join(lines)


def generate_synthesis_chase(n_robots, mode):
    """Generate synthesis .chase file."""
    lines = []
    for i in range(1, n_robots + 1):
        lines.append(f"saturate Robot{i};")
    if mode == "dynamic":
        lines.append("saturate Dispatcher;")
    lines.append("")
    for i in range(1, n_robots + 1):
        lines.append(f"synthesize Robot{i} robot{i};")
    if mode == "dynamic":
        lines.append("synthesize Dispatcher dispatcher;")
    lines.append("")
    return "\n".join(lines)


def generate_verification_chase(n_robots, n_boxes, mode):
    """Generate verification .chase file."""
    tag = "static" if mode == "static" else "dynamic"
    lines = []

    for i in range(1, n_robots + 1):
        lines.append(f"saturate Robot{i};")
    if mode == "dynamic":
        lines.append("saturate Dispatcher;")
    lines.append("saturate System;")
    lines.append("")

    components = [f"Robot{i}" for i in range(1, n_robots + 1)]
    if mode == "dynamic":
        components.append("Dispatcher")

    if len(components) == 1:
        composed = components[0]
    else:
        composed = components[0]
        for k, comp in enumerate(components[1:], start=1):
            new_name = f"C{k}" if k < len(components) - 1 else "Composed"
            lines.append(f"compose {composed} {comp} {new_name};")
            composed = new_name
        lines.append("")

    lines.append(f"refinement {composed} System refinement_{tag}_{n_robots}r_{n_boxes}b.smv;")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Generate parameterized multi-robot warehouse contracts."
    )
    parser.add_argument("--robots", "-r", type=int, default=2,
                        help="Number of robots (default: 2)")
    parser.add_argument("--boxes", "-b", type=int, default=3,
                        help="Number of boxes (default: 3)")
    parser.add_argument("--mode", "-m", choices=["static", "dynamic", "both"],
                        default="both", help="Decomposition mode (default: both)")
    parser.add_argument("--output-dir", "-o", type=str, default=".",
                        help="Output directory (default: current dir)")
    args = parser.parse_args()

    modes = ["static", "dynamic"] if args.mode == "both" else [args.mode]
    os.makedirs(args.output_dir, exist_ok=True)

    for mode in modes:
        tag = f"{mode}_{args.robots}r_{args.boxes}b"
        base = f"agv_{tag}"

        ltl_path = os.path.join(args.output_dir, f"{base}.ltl")
        with open(ltl_path, "w") as f:
            f.write(generate_ltl(args.robots, args.boxes, mode))
        print(f"  {ltl_path}")

        syn_path = os.path.join(args.output_dir, f"{base}_synthesis.chase")
        with open(syn_path, "w") as f:
            f.write(generate_synthesis_chase(args.robots, mode))
        print(f"  {syn_path}")

        ver_path = os.path.join(args.output_dir, f"{base}_verification.chase")
        with open(ver_path, "w") as f:
            f.write(generate_verification_chase(args.robots, args.boxes, mode))
        print(f"  {ver_path}")

        print()


if __name__ == "__main__":
    main()
