#!/usr/bin/env python3
"""
GridWorld LTL Contract Generator

Generates .ltl contract files and .chase pipeline files for multi-robot
grid world scenarios with configurable parameters:

  --grid-size N          NxN grid (default: 3)
  --num-robots N         Number of robots (default: 1)
  --obstacles x,y;...    Obstacle cells (default: "1,1")
  --depots x,y;...       Depot positions per robot
  --control-mode         "distributed" or "centralized" (default: distributed)
  --variable-targets     Use variable target assignment (target_x, target_y as inputs)
  --fixed-stations x,y;  Fixed station per robot (default if not --variable-targets)
  --output-dir DIR       Output directory (default: current dir)

Examples:
  # Single robot, variable targets, 3x3 grid
  python3 generate_gridworld.py --num-robots 1 --variable-targets --output-dir out/

  # 2 robots, distributed, fixed stations
  python3 generate_gridworld.py --num-robots 2 --fixed-stations "2,2;0,2" --output-dir out/

  # 2 robots, centralized arbiter
  python3 generate_gridworld.py --num-robots 2 --control-mode centralized --variable-targets --output-dir out/
"""

import argparse
import os
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


# ============================================================
# Data Model
# ============================================================

@dataclass
class GridConfig:
    size: int  # NxN
    obstacles: List[Tuple[int, int]]

    def is_free(self, x: int, y: int) -> bool:
        return 0 <= x < self.size and 0 <= y < self.size and (x, y) not in self.obstacles


@dataclass
class RobotConfig:
    robot_id: int
    depot: Tuple[int, int]
    station: Optional[Tuple[int, int]]  # None if variable targets
    variable_targets: bool


@dataclass
class SystemConfig:
    grid: GridConfig
    robots: List[RobotConfig]
    control_mode: str  # "distributed" or "centralized"
    num_variable_obstacles: int = 0


# ============================================================
# Variable Naming
# ============================================================

class VarNamer:
    """Variable naming: single robot uses pos_x/pos_y, multi uses r{i}_x/r{i}_y."""

    def __init__(self, robot_id: int, num_robots: int):
        self.robot_id = robot_id
        self.num_robots = num_robots
        if num_robots == 1:
            self._pfx = ""
        else:
            self._pfx = f"r{robot_id}_"

    @property
    def x(self): return f"{self._pfx}x" if self.num_robots > 1 else "pos_x"

    @property
    def y(self): return f"{self._pfx}y" if self.num_robots > 1 else "pos_y"

    @property
    def action(self): return f"{self._pfx}action" if self.num_robots > 1 else "action"

    @property
    def task(self): return f"{self._pfx}task" if self.num_robots > 1 else "task"

    @property
    def done(self): return f"{self._pfx}done" if self.num_robots > 1 else "done"

    @property
    def target_x(self): return f"{self._pfx}target_x" if self.num_robots > 1 else "target_x"

    @property
    def target_y(self): return f"{self._pfx}target_y" if self.num_robots > 1 else "target_y"

    @property
    def request(self): return f"{self._pfx}request" if self.num_robots > 1 else "request"

    @property
    def grant(self): return f"{self._pfx}grant" if self.num_robots > 1 else "grant"

    @property
    def next_x(self): return f"{self._pfx}next_x" if self.num_robots > 1 else "next_x"

    @property
    def next_y(self): return f"{self._pfx}next_y" if self.num_robots > 1 else "next_y"

    @staticmethod
    def block_x(k: int) -> str:
        """Variable obstacle k's x variable (shared across all contracts)."""
        return f"block{k}_x"

    @staticmethod
    def block_y(k: int) -> str:
        """Variable obstacle k's y variable (shared across all contracts)."""
        return f"block{k}_y"


# ============================================================
# Grid Dynamics - Movement Assumption Generation
# ============================================================

class GridDynamics:
    """Generates movement assumption LTL formulas for a grid with obstacles."""

    def __init__(self, grid: GridConfig, num_variable_obstacles: int = 0):
        self.grid = grid
        self.N = grid.size
        self.num_variable_obstacles = num_variable_obstacles

    def _obstacle_blocks(self, direction: str) -> List[str]:
        """Return list of blocking conditions (on source cell) for each obstacle."""
        blocks = []
        for ox, oy in self.grid.obstacles:
            if direction == "E" and ox - 1 >= 0:
                blocks.append((ox - 1, oy))
            elif direction == "W" and ox + 1 < self.N:
                blocks.append((ox + 1, oy))
            elif direction == "N" and oy + 1 < self.N:
                blocks.append((ox, oy + 1))
            elif direction == "S" and oy - 1 >= 0:
                blocks.append((ox, oy - 1))
        return blocks

    def _block_condition(self, cells: List[Tuple[int, int]], namer: VarNamer) -> str:
        """Build an OR of (x = cx & y = cy) for blocking cells."""
        parts = []
        for cx, cy in cells:
            parts.append(f"({namer.x} = {cx} & {namer.y} = {cy})")
        if len(parts) == 1:
            return parts[0]
        return "(" + " | ".join(parts) + ")"

    def _dynamic_block_cond(self, direction: str, namer: VarNamer) -> str:
        """Build variable obstacle blocking condition for a direction.

        Returns a conjunction of !(next_cell == block_k) for each variable obstacle,
        or empty string if no variable obstacles.
        """
        if self.num_variable_obstacles == 0:
            return ""
        x, y = namer.x, namer.y
        parts = []
        for k in range(1, self.num_variable_obstacles + 1):
            bx = VarNamer.block_x(k)
            by = VarNamer.block_y(k)
            if direction == "E":
                parts.append(f"!({x} + 1 = {bx} & {y} = {by})")
            elif direction == "W":
                # next cell is (x-1, y); use bx + 1 = x to avoid subtraction
                parts.append(f"!({bx} + 1 = {x} & {y} = {by})")
            elif direction == "N":
                # next cell is (x, y-1); use by + 1 = y
                parts.append(f"!({x} = {bx} & {by} + 1 = {y})")
            elif direction == "S":
                parts.append(f"!({x} = {bx} & {y} + 1 = {by})")
        return " & ".join(parts)

    def _dynamic_block_or(self, direction: str, namer: VarNamer) -> str:
        """Build variable obstacle OR condition (for blocked branch).

        Returns a disjunction of (next_cell == block_k), or empty string.
        """
        if self.num_variable_obstacles == 0:
            return ""
        x, y = namer.x, namer.y
        parts = []
        for k in range(1, self.num_variable_obstacles + 1):
            bx = VarNamer.block_x(k)
            by = VarNamer.block_y(k)
            if direction == "E":
                parts.append(f"({x} + 1 = {bx} & {y} = {by})")
            elif direction == "W":
                parts.append(f"({bx} + 1 = {x} & {y} = {by})")
            elif direction == "N":
                parts.append(f"({x} = {bx} & {by} + 1 = {y})")
            elif direction == "S":
                parts.append(f"({x} = {bx} & {y} + 1 = {by})")
        if len(parts) == 1:
            return parts[0]
        return "(" + " | ".join(parts) + ")"

    def generate_movement_assumptions(self, namer: VarNamer) -> List[str]:
        """Generate all movement dynamics assumptions."""
        a = namer.action
        x, y = namer.x, namer.y
        N = self.N
        formulas = []

        # --- X-axis: stay/N/S don't change x ---
        formulas.append(f"G(({a} = 0 | {a} = 1 | {a} = 2) -> {x}' = {x});")

        # East (action=3)
        e_blocks = self._obstacle_blocks("E")
        dyn_e = self._dynamic_block_cond("E", namer)
        dyn_e_or = self._dynamic_block_or("E", namer)
        static_blk = self._block_condition(e_blocks, namer) if e_blocks else ""
        # Build combined success/blocked conditions
        success_extra = ""  # extra AND conditions for success
        blocked_extra = ""  # extra OR conditions for blocked
        if static_blk:
            success_extra += f" & !{static_blk}"
            blocked_extra = static_blk
        if dyn_e:
            success_extra += f" & {dyn_e}"
            if blocked_extra:
                blocked_extra = f"({blocked_extra} | {dyn_e_or})"
            else:
                blocked_extra = dyn_e_or

        if success_extra:
            formulas.append(f"G({a} = 3 & {x} < {N-1}{success_extra} -> {x}' = {x} + 1);")
            formulas.append(f"G({a} = 3 & ({x} = {N-1} | {blocked_extra}) -> {x}' = {x});")
        else:
            formulas.append(f"G({a} = 3 & {x} < {N-1} -> {x}' = {x} + 1);")
            formulas.append(f"G({a} = 3 & {x} = {N-1} -> {x}' = {x});")

        # West (action=4)
        w_blocks = self._obstacle_blocks("W")
        dyn_w = self._dynamic_block_cond("W", namer)
        dyn_w_or = self._dynamic_block_or("W", namer)
        static_blk = self._block_condition(w_blocks, namer) if w_blocks else ""
        success_extra = ""
        blocked_extra = ""
        if static_blk:
            success_extra += f" & !{static_blk}"
            blocked_extra = static_blk
        if dyn_w:
            success_extra += f" & {dyn_w}"
            if blocked_extra:
                blocked_extra = f"({blocked_extra} | {dyn_w_or})"
            else:
                blocked_extra = dyn_w_or

        if success_extra:
            formulas.append(f"G({a} = 4 & {x} > 0{success_extra} -> {x}' + 1 = {x});")
            formulas.append(f"G({a} = 4 & ({x} = 0 | {blocked_extra}) -> {x}' = {x});")
        else:
            formulas.append(f"G({a} = 4 & {x} > 0 -> {x}' + 1 = {x});")
            formulas.append(f"G({a} = 4 & {x} = 0 -> {x}' = {x});")

        formulas.append("")  # blank line separator

        # --- Y-axis: stay/E/W don't change y ---
        formulas.append(f"G(({a} = 0 | {a} = 3 | {a} = 4) -> {y}' = {y});")

        # North (action=1, y decreases)
        n_blocks = self._obstacle_blocks("N")
        dyn_n = self._dynamic_block_cond("N", namer)
        dyn_n_or = self._dynamic_block_or("N", namer)
        static_blk = self._block_condition(n_blocks, namer) if n_blocks else ""
        success_extra = ""
        blocked_extra = ""
        if static_blk:
            success_extra += f" & !{static_blk}"
            blocked_extra = static_blk
        if dyn_n:
            success_extra += f" & {dyn_n}"
            if blocked_extra:
                blocked_extra = f"({blocked_extra} | {dyn_n_or})"
            else:
                blocked_extra = dyn_n_or

        if success_extra:
            formulas.append(f"G({a} = 1 & {y} > 0{success_extra} -> {y}' + 1 = {y});")
            formulas.append(f"G({a} = 1 & ({y} = 0 | {blocked_extra}) -> {y}' = {y});")
        else:
            formulas.append(f"G({a} = 1 & {y} > 0 -> {y}' + 1 = {y});")
            formulas.append(f"G({a} = 1 & {y} = 0 -> {y}' = {y});")

        # South (action=2, y increases)
        s_blocks = self._obstacle_blocks("S")
        dyn_s = self._dynamic_block_cond("S", namer)
        dyn_s_or = self._dynamic_block_or("S", namer)
        static_blk = self._block_condition(s_blocks, namer) if s_blocks else ""
        success_extra = ""
        blocked_extra = ""
        if static_blk:
            success_extra += f" & !{static_blk}"
            blocked_extra = static_blk
        if dyn_s:
            success_extra += f" & {dyn_s}"
            if blocked_extra:
                blocked_extra = f"({blocked_extra} | {dyn_s_or})"
            else:
                blocked_extra = dyn_s_or

        if success_extra:
            formulas.append(f"G({a} = 2 & {y} < {N-1}{success_extra} -> {y}' = {y} + 1);")
            formulas.append(f"G({a} = 2 & ({y} = {N-1} | {blocked_extra}) -> {y}' = {y});")
        else:
            formulas.append(f"G({a} = 2 & {y} < {N-1} -> {y}' = {y} + 1);")
            formulas.append(f"G({a} = 2 & {y} = {N-1} -> {y}' = {y});")

        return formulas

    def collision_avoidance_guarantees(self, namer_i: VarNamer, namer_j: VarNamer) -> List[str]:
        """Generate directional collision avoidance guarantees for robot i w.r.t. robot j."""
        xi, yi, ai = namer_i.x, namer_i.y, namer_i.action
        xj, yj = namer_j.x, namer_j.y
        N = self.N
        formulas = []

        # North (action=1): target cell is (xi, yi-1), check if rj is there
        n_blocks = self._obstacle_blocks("N")
        n_blk_str = ""
        if n_blocks:
            n_blk_str = f" & !{self._block_condition(n_blocks, namer_i)}"
        formulas.append(
            f"G({yi} > 0{n_blk_str} & {xj} = {xi} & {yj} + 1 = {yi} -> {ai} != 1);"
        )

        # South (action=2): target cell is (xi, yi+1)
        s_blocks = self._obstacle_blocks("S")
        s_blk_str = ""
        if s_blocks:
            s_blk_str = f" & !{self._block_condition(s_blocks, namer_i)}"
        formulas.append(
            f"G({yi} < {N-1}{s_blk_str} & {xj} = {xi} & {yj} = {yi} + 1 -> {ai} != 2);"
        )

        # East (action=3): target cell is (xi+1, yi)
        e_blocks = self._obstacle_blocks("E")
        e_blk_str = ""
        if e_blocks:
            e_blk_str = f" & !{self._block_condition(e_blocks, namer_i)}"
        formulas.append(
            f"G({xi} < {N-1}{e_blk_str} & {yj} = {yi} & {xj} = {xi} + 1 -> {ai} != 3);"
        )

        # West (action=4): target cell is (xi-1, yi)
        w_blocks = self._obstacle_blocks("W")
        w_blk_str = ""
        if w_blocks:
            w_blk_str = f" & !{self._block_condition(w_blocks, namer_i)}"
        formulas.append(
            f"G({xi} > 0{w_blk_str} & {yj} = {yi} & {xj} + 1 = {xi} -> {ai} != 4);"
        )

        return formulas


# ============================================================
# Contract Builder
# ============================================================

class ContractBuilder:
    def __init__(self, config: SystemConfig):
        self.config = config
        self.grid = config.grid
        self.dynamics = GridDynamics(config.grid, config.num_variable_obstacles)
        self.N = config.grid.size
        self.num_robots = len(config.robots)
        self.num_variable_obstacles = config.num_variable_obstacles

    def _namer(self, robot_id: int) -> VarNamer:
        return VarNamer(robot_id, self.num_robots)

    def _other_robots(self, robot_id: int) -> List[int]:
        """Return IDs of all other robots."""
        return [r.robot_id for r in self.config.robots if r.robot_id != robot_id]

    def _indent(self, lines: List[str], level: int = 2) -> str:
        """Indent lines with spaces."""
        prefix = "    " * level
        result = []
        for line in lines:
            if line == "":
                result.append("")
            else:
                result.append(f"{prefix}{line}")
        return "\n".join(result)

    # ---------- Variable Declarations ----------

    def _var_declarations(self, robot_id: int, visible: List[int], is_system: bool = False) -> Tuple[List[str], List[str]]:
        """Return (input_lines, output_lines) for variable declarations."""
        n = self._namer(robot_id)
        inputs = []
        outputs = []
        max_val = self.N - 1

        if is_system:
            # System contract: all robot vars
            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                inputs.append(f"input integer (0:{max_val}) variable {rn.x};")
                inputs.append(f"input integer (0:{max_val}) variable {rn.y};")
                inputs.append(f"input boolean variable {rn.task};")
                if r.variable_targets:
                    inputs.append(f"input integer (0:{max_val}) variable {rn.target_x};")
                    inputs.append(f"input integer (0:{max_val}) variable {rn.target_y};")
                outputs.append(f"output integer (0:4) variable {rn.action};")
                outputs.append(f"output boolean variable {rn.done};")
            # Dynamic obstacle inputs
            for k in range(1, self.num_variable_obstacles + 1):
                inputs.append(f"input integer (0:{max_val}) variable {VarNamer.block_x(k)};")
                inputs.append(f"input integer (0:{max_val}) variable {VarNamer.block_y(k)};")
            return inputs, outputs

        # Per-robot contract
        inputs.append(f"input integer (0:{max_val}) variable {n.x};")
        inputs.append(f"input integer (0:{max_val}) variable {n.y};")
        inputs.append(f"input boolean variable {n.task};")

        robot = self.config.robots[robot_id - 1]
        if robot.variable_targets:
            inputs.append(f"input integer (0:{max_val}) variable {n.target_x};")
            inputs.append(f"input integer (0:{max_val}) variable {n.target_y};")

        # Other robots' positions
        for vid in visible:
            vn = self._namer(vid)
            inputs.append(f"input integer (0:{max_val}) variable {vn.x};")
            inputs.append(f"input integer (0:{max_val}) variable {vn.y};")

        # Dynamic obstacle inputs
        for k in range(1, self.num_variable_obstacles + 1):
            inputs.append(f"input integer (0:{max_val}) variable {VarNamer.block_x(k)};")
            inputs.append(f"input integer (0:{max_val}) variable {VarNamer.block_y(k)};")

        outputs.append(f"output integer (0:4) variable {n.action};")
        outputs.append(f"output boolean variable {n.done};")

        return inputs, outputs

    # ---------- Assumptions ----------

    def _assumptions(self, robot_id: int, visible: List[int], is_system: bool = False) -> List[str]:
        """Generate assumption formulas."""
        lines = []

        if is_system:
            # System contract: all robots' dynamics and task lifecycle
            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                dx, dy = r.depot
                lines.append(f"{rn.x} = {dx};")
                lines.append(f"{rn.y} = {dy};")
                lines.append(f"!{rn.task};")
            lines.append("")

            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                lines.extend(self.dynamics.generate_movement_assumptions(rn))
                lines.append("")

            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                lines.extend(self._task_lifecycle(rn, r))
                lines.append("")

            # Dynamic obstacle assumptions
            if self.num_variable_obstacles > 0:
                lines.extend(self._dynamic_obstacle_assumptions())
                lines.append("")

            return lines

        # Per-robot contract
        n = self._namer(robot_id)
        robot = self.config.robots[robot_id - 1]
        dx, dy = robot.depot

        # Initial conditions
        lines.append(f"{n.x} = {dx};")
        lines.append(f"{n.y} = {dy};")
        lines.append(f"!{n.task};")
        lines.append("")

        # Movement dynamics
        lines.extend(self.dynamics.generate_movement_assumptions(n))
        lines.append("")

        # Task lifecycle
        lines.extend(self._task_lifecycle(n, robot))
        lines.append("")

        # Dynamic obstacle assumptions
        if self.num_variable_obstacles > 0:
            lines.extend(self._dynamic_obstacle_assumptions())
            lines.append("")

        # Fairness on other robots
        for vid in visible:
            vr = self.config.robots[vid - 1]
            vn = self._namer(vid)
            vdx, vdy = vr.depot
            lines.append(f"G F({vn.x} = {vdx} & {vn.y} = {vdy});")
        if visible:
            lines.append("")

        return lines

    def _task_lifecycle(self, n: VarNamer, robot: RobotConfig) -> List[str]:
        """Generate task lifecycle assumptions."""
        lines = []
        lines.append(f"G({n.task} & !{n.done} -> X({n.task}));")
        lines.append("")
        lines.append(f"G F({n.task});")
        lines.append(f"G F(!{n.task});")
        lines.append(f"G F({n.task} & {n.done} -> X(!{n.task}));")

        if robot.variable_targets:
            lines.append("")
            # Target is not an obstacle (static)
            for ox, oy in self.grid.obstacles:
                lines.append(f"G({n.task} -> !({n.target_x} = {ox} & {n.target_y} = {oy}));")
            # Target is not a variable obstacle
            for k in range(1, self.num_variable_obstacles + 1):
                bx = VarNamer.block_x(k)
                by = VarNamer.block_y(k)
                lines.append(f"G({n.task} -> !({n.target_x} = {bx} & {n.target_y} = {by}));")
            # Target persistence during active task
            lines.append(f"G({n.task} & !{n.done} -> {n.target_x}' = {n.target_x});")
            lines.append(f"G({n.task} & !{n.done} -> {n.target_y}' = {n.target_y});")

        return lines

    def _dynamic_obstacle_assumptions(self) -> List[str]:
        """Generate assumptions for variable obstacles."""
        lines = []
        N = self.N
        for k in range(1, self.num_variable_obstacles + 1):
            bx = VarNamer.block_x(k)
            by = VarNamer.block_y(k)
            # Stationary
            lines.append(f"G({bx}' = {bx});")
            lines.append(f"G({by}' = {by});")
            # Not on static obstacles
            for ox, oy in self.grid.obstacles:
                lines.append(f"G(!({bx} = {ox} & {by} = {oy}));")
            # Not at robot depots
            for r in self.config.robots:
                dx, dy = r.depot
                lines.append(f"G(!({bx} = {dx} & {by} = {dy}));")
        # Pairwise distinct variable obstacles
        for i in range(1, self.num_variable_obstacles + 1):
            for j in range(i + 1, self.num_variable_obstacles + 1):
                lines.append(f"G({VarNamer.block_x(i)} != {VarNamer.block_x(j)} | {VarNamer.block_y(i)} != {VarNamer.block_y(j)});")
        return lines

    # ---------- Guarantees ----------

    def _guarantees(self, robot_id: int, visible: List[int], is_system: bool = False) -> List[str]:
        """Generate guarantee formulas."""
        lines = []

        if is_system:
            # System-level guarantees
            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                lines.append(f"{rn.action} = 0;")
            lines.append("")

            # Obstacle avoidance for each robot
            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                for ox, oy in self.grid.obstacles:
                    lines.append(f"G({rn.x} != {ox} | {rn.y} != {oy});")
                for k in range(1, self.num_variable_obstacles + 1):
                    lines.append(f"G({rn.x} != {VarNamer.block_x(k)} | {rn.y} != {VarNamer.block_y(k)});")

            # Done semantics for each robot
            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                lines.append("")
                lines.append(self._done_formula(rn, r))
            lines.append("")

            # Post-done
            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                lines.append(f"G({rn.done} -> X({rn.action} = 0));")
            lines.append("")

            # Pairwise collision avoidance (simple form)
            for i in range(len(self.config.robots)):
                for j in range(i + 1, len(self.config.robots)):
                    ni = self._namer(self.config.robots[i].robot_id)
                    nj = self._namer(self.config.robots[j].robot_id)
                    lines.append(f"G({ni.x} != {nj.x} | {ni.y} != {nj.y});")
            lines.append("")

            # Liveness
            for r in self.config.robots:
                rn = self._namer(r.robot_id)
                dx, dy = r.depot
                lines.append(f"G F({rn.done});")
                lines.append(f"G F({rn.x} = {dx} & {rn.y} = {dy});")

            return lines

        # Per-robot guarantees
        n = self._namer(robot_id)
        robot = self.config.robots[robot_id - 1]

        # Initial action
        lines.append(f"{n.action} = 0;")
        lines.append("")

        # Obstacle avoidance
        for ox, oy in self.grid.obstacles:
            lines.append(f"G({n.x} != {ox} | {n.y} != {oy});")
        for k in range(1, self.num_variable_obstacles + 1):
            lines.append(f"G({n.x} != {VarNamer.block_x(k)} | {n.y} != {VarNamer.block_y(k)});")
        lines.append("")

        # Done semantics
        lines.append(self._done_formula(n, robot))
        lines.append("")

        # Post-done
        lines.append(f"G({n.done} -> X({n.action} = 0));")
        lines.append("")

        # Collision avoidance with other robots
        for vid in visible:
            vn = self._namer(vid)
            lines.extend(self.dynamics.collision_avoidance_guarantees(n, vn))
        if visible:
            lines.append("")

        # Liveness
        dx, dy = robot.depot
        lines.append(f"G F({n.done});")
        lines.append(f"G F({n.x} = {dx} & {n.y} = {dy});")

        return lines

    def _done_formula(self, n: VarNamer, robot: RobotConfig) -> str:
        if robot.variable_targets:
            return f"G(({n.x} = {n.target_x} & {n.y} = {n.target_y} & {n.task}) <-> {n.done});"
        else:
            sx, sy = robot.station
            return f"G(({n.x} = {sx} & {n.y} = {sy} & {n.task}) <-> {n.done});"

    # ---------- Centralized Mode: Robot (with request) + Arbiter + System ----------

    def _centralized_robot_contract(self, robot_id: int) -> str:
        """Build a robot controller contract for centralized (request-grant) mode.

        Each robot independently decides its action, computes next position,
        and sends a request. Movement occurs only if the Arbiter grants.

          Input:  position, task, target, blocks, grant (from Arbiter)
          Output: action, done, request, next_x, next_y
          Assumes: dynamics (position changes only if granted), task lifecycle
          Guarantees: next position correctly computed, obstacle avoidance,
                      request semantics, done semantics, liveness
        """
        n = self._namer(robot_id)
        robot = self.config.robots[robot_id - 1]
        max_val = self.N - 1
        name = f"Robot{robot_id}" if self.num_robots > 1 else "Robot"

        # Inputs
        inputs = []
        inputs.append(f"input integer (0:{max_val}) variable {n.x};")
        inputs.append(f"input integer (0:{max_val}) variable {n.y};")
        inputs.append(f"input boolean variable {n.task};")
        if robot.variable_targets:
            inputs.append(f"input integer (0:{max_val}) variable {n.target_x};")
            inputs.append(f"input integer (0:{max_val}) variable {n.target_y};")
        inputs.append(f"input boolean variable {n.grant};")
        for k in range(1, self.num_variable_obstacles + 1):
            inputs.append(f"input integer (0:{max_val}) variable {VarNamer.block_x(k)};")
            inputs.append(f"input integer (0:{max_val}) variable {VarNamer.block_y(k)};")

        # Outputs
        outputs = []
        outputs.append(f"output integer (0:4) variable {n.action};")
        outputs.append(f"output boolean variable {n.done};")
        outputs.append(f"output boolean variable {n.request};")
        outputs.append(f"output integer (0:{max_val}) variable {n.next_x};")
        outputs.append(f"output integer (0:{max_val}) variable {n.next_y};")

        # Assumptions
        dx, dy = robot.depot
        assumptions = []
        assumptions.append(f"{n.x} = {dx};")
        assumptions.append(f"{n.y} = {dy};")
        assumptions.append(f"!{n.task};")
        assumptions.append(f"!{n.grant};")
        assumptions.append("")

        # Dynamics with grant: position changes only if granted
        assumptions.append(f"G({n.grant} -> ({n.x}' = {n.next_x} & {n.y}' = {n.next_y}));")
        assumptions.append(f"G(!{n.grant} -> ({n.x}' = {n.x} & {n.y}' = {n.y}));")
        assumptions.append("")

        # Task lifecycle
        assumptions.extend(self._task_lifecycle(n, robot))
        assumptions.append("")

        # Variable obstacle assumptions
        if self.num_variable_obstacles > 0:
            assumptions.extend(self._dynamic_obstacle_assumptions())
            assumptions.append("")

        # Guarantees
        guarantees = []
        guarantees.append(f"{n.action} = 0;")
        guarantees.append(f"!{n.request};")
        guarantees.append(f"{n.next_x} = {dx};")
        guarantees.append(f"{n.next_y} = {dy};")
        guarantees.append("")

        # next_x, next_y correctly computed from action + dynamics
        guarantees.extend(self._next_position_guarantees(n))
        guarantees.append("")

        # request = true when action != 0 (robot wants to move)
        guarantees.append(f"G({n.action} != 0 <-> {n.request});")
        guarantees.append("")

        # Obstacle avoidance (for next position)
        for ox, oy in self.grid.obstacles:
            guarantees.append(f"G({n.next_x} != {ox} | {n.next_y} != {oy});")
        for k in range(1, self.num_variable_obstacles + 1):
            guarantees.append(f"G({n.next_x} != {VarNamer.block_x(k)} | {n.next_y} != {VarNamer.block_y(k)});")
        guarantees.append("")

        # Done semantics
        guarantees.append(self._done_formula(n, robot))
        guarantees.append("")

        # Post-done
        guarantees.append(f"G({n.done} -> X({n.action} = 0));")
        guarantees.append("")

        # Liveness
        guarantees.append(f"G F({n.done});")
        guarantees.append(f"G F({n.x} = {dx} & {n.y} = {dy});")

        lines = [f"CONTRACT {name}:"]
        for line in inputs:
            lines.append(f"    {line}")
        lines.append("")
        for line in outputs:
            lines.append(f"    {line}")
        lines.append("")
        lines.append("    Assumptions:")
        lines.append(self._indent(assumptions))
        lines.append("")
        lines.append("    Guarantees:")
        lines.append(self._indent(guarantees))

        return "\n".join(lines)

    def _next_position_guarantees(self, n: VarNamer) -> List[str]:
        """Generate guarantees for next_x, next_y based on action and current position."""
        x, y, a = n.x, n.y, n.action
        nx, ny = n.next_x, n.next_y
        N = self.N
        formulas = []

        # Stay: next = current
        formulas.append(f"G({a} = 0 -> ({nx} = {x} & {ny} = {y}));")

        # North (action=1, y decreases)
        n_blocks = self.dynamics._obstacle_blocks("N")
        dyn_n = self.dynamics._dynamic_block_cond("N", n)
        if n_blocks or dyn_n:
            static = self.dynamics._block_condition(n_blocks, n) if n_blocks else ""
            blk_parts = []
            if static:
                blk_parts.append(static)
            if dyn_n:
                blk_parts.append(self.dynamics._dynamic_block_or("N", n))
            blk_or = " | ".join(blk_parts) if len(blk_parts) == 1 else "(" + " | ".join(blk_parts) + ")"
            success_extra = ""
            if static:
                success_extra += f" & !{static}"
            if dyn_n:
                success_extra += f" & {dyn_n}"
            formulas.append(f"G({a} = 1 & {y} > 0{success_extra} -> ({nx} = {x} & {ny} + 1 = {y}));")
            formulas.append(f"G({a} = 1 & ({y} = 0 | {blk_or}) -> ({nx} = {x} & {ny} = {y}));")
        else:
            formulas.append(f"G({a} = 1 & {y} > 0 -> ({nx} = {x} & {ny} + 1 = {y}));")
            formulas.append(f"G({a} = 1 & {y} = 0 -> ({nx} = {x} & {ny} = {y}));")

        # South (action=2, y increases)
        s_blocks = self.dynamics._obstacle_blocks("S")
        dyn_s = self.dynamics._dynamic_block_cond("S", n)
        if s_blocks or dyn_s:
            static = self.dynamics._block_condition(s_blocks, n) if s_blocks else ""
            blk_parts = []
            if static:
                blk_parts.append(static)
            if dyn_s:
                blk_parts.append(self.dynamics._dynamic_block_or("S", n))
            blk_or = " | ".join(blk_parts) if len(blk_parts) == 1 else "(" + " | ".join(blk_parts) + ")"
            success_extra = ""
            if static:
                success_extra += f" & !{static}"
            if dyn_s:
                success_extra += f" & {dyn_s}"
            formulas.append(f"G({a} = 2 & {y} < {N-1}{success_extra} -> ({nx} = {x} & {ny} = {y} + 1));")
            formulas.append(f"G({a} = 2 & ({y} = {N-1} | {blk_or}) -> ({nx} = {x} & {ny} = {y}));")
        else:
            formulas.append(f"G({a} = 2 & {y} < {N-1} -> ({nx} = {x} & {ny} = {y} + 1));")
            formulas.append(f"G({a} = 2 & {y} = {N-1} -> ({nx} = {x} & {ny} = {y}));")

        # East (action=3, x increases)
        e_blocks = self.dynamics._obstacle_blocks("E")
        dyn_e = self.dynamics._dynamic_block_cond("E", n)
        if e_blocks or dyn_e:
            static = self.dynamics._block_condition(e_blocks, n) if e_blocks else ""
            blk_parts = []
            if static:
                blk_parts.append(static)
            if dyn_e:
                blk_parts.append(self.dynamics._dynamic_block_or("E", n))
            blk_or = " | ".join(blk_parts) if len(blk_parts) == 1 else "(" + " | ".join(blk_parts) + ")"
            success_extra = ""
            if static:
                success_extra += f" & !{static}"
            if dyn_e:
                success_extra += f" & {dyn_e}"
            formulas.append(f"G({a} = 3 & {x} < {N-1}{success_extra} -> ({nx} = {x} + 1 & {ny} = {y}));")
            formulas.append(f"G({a} = 3 & ({x} = {N-1} | {blk_or}) -> ({nx} = {x} & {ny} = {y}));")
        else:
            formulas.append(f"G({a} = 3 & {x} < {N-1} -> ({nx} = {x} + 1 & {ny} = {y}));")
            formulas.append(f"G({a} = 3 & {x} = {N-1} -> ({nx} = {x} & {ny} = {y}));")

        # West (action=4, x decreases)
        w_blocks = self.dynamics._obstacle_blocks("W")
        dyn_w = self.dynamics._dynamic_block_cond("W", n)
        if w_blocks or dyn_w:
            static = self.dynamics._block_condition(w_blocks, n) if w_blocks else ""
            blk_parts = []
            if static:
                blk_parts.append(static)
            if dyn_w:
                blk_parts.append(self.dynamics._dynamic_block_or("W", n))
            blk_or = " | ".join(blk_parts) if len(blk_parts) == 1 else "(" + " | ".join(blk_parts) + ")"
            success_extra = ""
            if static:
                success_extra += f" & !{static}"
            if dyn_w:
                success_extra += f" & {dyn_w}"
            formulas.append(f"G({a} = 4 & {x} > 0{success_extra} -> ({nx} + 1 = {x} & {ny} = {y}));")
            formulas.append(f"G({a} = 4 & ({x} = 0 | {blk_or}) -> ({nx} = {x} & {ny} = {y}));")
        else:
            formulas.append(f"G({a} = 4 & {x} > 0 -> ({nx} + 1 = {x} & {ny} = {y}));")
            formulas.append(f"G({a} = 4 & {x} = 0 -> ({nx} = {x} & {ny} = {y}));")

        return formulas

    def _arbiter_contract(self) -> str:
        """Build the Arbiter contract for centralized (request-grant) mode.

        Arbiter receives next positions and requests from all robots,
        outputs grants. No dynamics knowledge needed.

          Input:  next_x, next_y, request (from each robot)
          Output: grant (for each robot)
          Assumes: (none beyond initial state)
          Guarantees: collision freedom, fairness
        """
        max_val = self.N - 1
        inputs = []
        outputs = []

        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            inputs.append(f"input integer (0:{max_val}) variable {rn.next_x};")
            inputs.append(f"input integer (0:{max_val}) variable {rn.next_y};")
            inputs.append(f"input boolean variable {rn.request};")
            outputs.append(f"output boolean variable {rn.grant};")

        assumptions = []
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            assumptions.append(f"!{rn.request};")

        guarantees = []
        # Initial: no grants
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            guarantees.append(f"!{rn.grant};")
        guarantees.append("")

        # Collision freedom: if both granted, next positions must differ
        for i in range(len(self.config.robots)):
            for j in range(i + 1, len(self.config.robots)):
                ni = self._namer(self.config.robots[i].robot_id)
                nj = self._namer(self.config.robots[j].robot_id)
                guarantees.append(
                    f"G({ni.grant} & {nj.grant} -> ({ni.next_x} != {nj.next_x} | {ni.next_y} != {nj.next_y}));"
                )
        guarantees.append("")

        # Only grant if requested
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            guarantees.append(f"G({rn.grant} -> {rn.request});")
        guarantees.append("")

        # Fairness: every request eventually granted
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            guarantees.append(f"G F({rn.request} -> {rn.grant});")

        lines = ["CONTRACT Arbiter:"]
        for line in inputs:
            lines.append(f"    {line}")
        lines.append("")
        for line in outputs:
            lines.append(f"    {line}")
        lines.append("")
        lines.append("    Assumptions:")
        lines.append(self._indent(assumptions))
        lines.append("")
        lines.append("    Guarantees:")
        lines.append(self._indent(guarantees))

        return "\n".join(lines)

    def _centralized_system_contract(self) -> str:
        """Build the system-level contract for centralized (request-grant) mode.

        System interface: external inputs (tasks, targets, blocks),
        all internal variables as outputs.
        """
        max_val = self.N - 1
        inputs = []
        outputs = []

        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            inputs.append(f"input boolean variable {rn.task};")
            if r.variable_targets:
                inputs.append(f"input integer (0:{max_val}) variable {rn.target_x};")
                inputs.append(f"input integer (0:{max_val}) variable {rn.target_y};")
            outputs.append(f"output integer (0:{max_val}) variable {rn.x};")
            outputs.append(f"output integer (0:{max_val}) variable {rn.y};")
            outputs.append(f"output integer (0:4) variable {rn.action};")
            outputs.append(f"output boolean variable {rn.done};")
            outputs.append(f"output boolean variable {rn.request};")
            outputs.append(f"output integer (0:{max_val}) variable {rn.next_x};")
            outputs.append(f"output integer (0:{max_val}) variable {rn.next_y};")
            outputs.append(f"output boolean variable {rn.grant};")

        for k in range(1, self.num_variable_obstacles + 1):
            inputs.append(f"input integer (0:{max_val}) variable {VarNamer.block_x(k)};")
            inputs.append(f"input integer (0:{max_val}) variable {VarNamer.block_y(k)};")

        # Assumptions: task lifecycle + target/block validity
        assumptions = []
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            assumptions.append(f"!{rn.task};")
        assumptions.append("")

        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            assumptions.extend(self._task_lifecycle(rn, r))
            assumptions.append("")

        if self.num_variable_obstacles > 0:
            assumptions.extend(self._dynamic_obstacle_assumptions())
            assumptions.append("")

        # Guarantees: safety + liveness
        guarantees = []
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            dx, dy = r.depot
            guarantees.append(f"{rn.x} = {dx};")
            guarantees.append(f"{rn.y} = {dy};")
            guarantees.append(f"{rn.action} = 0;")
        guarantees.append("")

        # Obstacle avoidance
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            for ox, oy in self.grid.obstacles:
                guarantees.append(f"G({rn.x} != {ox} | {rn.y} != {oy});")
            for k in range(1, self.num_variable_obstacles + 1):
                guarantees.append(f"G({rn.x} != {VarNamer.block_x(k)} | {rn.y} != {VarNamer.block_y(k)});")

        # Collision avoidance
        for i in range(len(self.config.robots)):
            for j in range(i + 1, len(self.config.robots)):
                ni = self._namer(self.config.robots[i].robot_id)
                nj = self._namer(self.config.robots[j].robot_id)
                guarantees.append(f"G({ni.x} != {nj.x} | {ni.y} != {nj.y});")
        guarantees.append("")

        # Done semantics
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            guarantees.append(self._done_formula(rn, r))
        guarantees.append("")

        # Liveness
        for r in self.config.robots:
            rn = self._namer(r.robot_id)
            dx, dy = r.depot
            guarantees.append(f"G F({rn.done});")
            guarantees.append(f"G F({rn.x} = {dx} & {rn.y} = {dy});")

        lines = ["CONTRACT System:"]
        for line in inputs:
            lines.append(f"    {line}")
        lines.append("")
        for line in outputs:
            lines.append(f"    {line}")
        lines.append("")
        lines.append("    Assumptions:")
        lines.append(self._indent(assumptions))
        lines.append("")
        lines.append("    Guarantees:")
        lines.append(self._indent(guarantees))

        return "\n".join(lines)

    # ---------- Build Full LTL File ----------

    def build_robot_contract(self, robot_id: int) -> str:
        """Build a single robot contract string."""
        visible = self._other_robots(robot_id)
        inputs_l, outputs_l = self._var_declarations(robot_id, visible)
        assumptions = self._assumptions(robot_id, visible)
        guarantees = self._guarantees(robot_id, visible)

        robot = self.config.robots[robot_id - 1]
        name = f"Robot{robot_id}" if self.num_robots > 1 else "Robot"

        lines = [f"CONTRACT {name}:"]
        for line in inputs_l:
            lines.append(f"    {line}")
        lines.append("")
        for line in outputs_l:
            lines.append(f"    {line}")
        lines.append("")
        lines.append("    Assumptions:")
        lines.append(self._indent(assumptions))
        lines.append("")
        lines.append("    Guarantees:")
        lines.append(self._indent(guarantees))

        return "\n".join(lines)

    def build_system_contract(self) -> str:
        """Build the system-level contract for composition checking."""
        inputs_l, outputs_l = self._var_declarations(1, [], is_system=True)
        assumptions = self._assumptions(1, [], is_system=True)
        guarantees = self._guarantees(1, [], is_system=True)

        lines = ["CONTRACT System:"]
        for line in inputs_l:
            lines.append(f"    {line}")
        lines.append("")
        for line in outputs_l:
            lines.append(f"    {line}")
        lines.append("")
        lines.append("    Assumptions:")
        lines.append(self._indent(assumptions))
        lines.append("")
        lines.append("    Guarantees:")
        lines.append(self._indent(guarantees))

        return "\n".join(lines)

    def build_specs_ltl(self) -> str:
        """Build the complete .ltl file."""
        mode = self.config.control_mode.capitalize()
        n = self.num_robots

        name = f"GridWorld_{mode}_{n}R"
        parts = [f"NAME: {name};", ""]

        if self.config.control_mode == "centralized":
            # Plant contracts for each robot
            for r in self.config.robots:
                parts.append(self._centralized_robot_contract(r.robot_id))
                parts.append("")
            # Arbiter (controller)
            parts.append(self._arbiter_contract())
            parts.append("")
            # System contract for refinement checking
            parts.append(self._centralized_system_contract())
        else:
            # Distributed: per-robot contracts + system contract
            for r in self.config.robots:
                parts.append(self.build_robot_contract(r.robot_id))
                parts.append("")
            if n > 1:
                parts.append(self.build_system_contract())

        return "\n".join(parts) + "\n"


# ============================================================
# CHASE File Generator
# ============================================================

class ChaseFileGenerator:
    def __init__(self, config: SystemConfig):
        self.config = config
        self.num_robots = len(config.robots)

    def verification_chase(self) -> str:
        """Generate verification.chase for composition refinement check."""
        if self.config.control_mode == "centralized":
            lines = ["saturate Arbiter;"]
            robot_names = []
            for r in self.config.robots:
                name = f"Robot{r.robot_id}" if self.num_robots > 1 else "Robot"
                lines.append(f"saturate {name};")
                robot_names.append(name)
            lines.append("saturate System;")
            compose_args = "Arbiter " + " ".join(robot_names)
            lines.append(f"compose {compose_args} Composed;")
            lines.append("refinement Composed System refinement.smv;")
            return "\n".join(lines) + "\n"

        lines = []
        for r in self.config.robots:
            name = f"Robot{r.robot_id}" if self.num_robots > 1 else "Robot"
            lines.append(f"saturate {name};")
        lines.append("saturate System;")

        # Compose all robots
        robot_names = []
        for r in self.config.robots:
            robot_names.append(f"Robot{r.robot_id}" if self.num_robots > 1 else "Robot")
        compose_args = " ".join(robot_names)
        lines.append(f"compose {compose_args} Composed;")
        lines.append("refinement Composed System refinement.smv;")

        return "\n".join(lines) + "\n"

    def synthesis_chase(self) -> str:
        """Generate synthesis.chase for per-robot synthesis."""
        if self.config.control_mode == "centralized":
            lines = []
            for r in self.config.robots:
                name = f"Robot{r.robot_id}" if self.num_robots > 1 else "Robot"
                lines.append(f"saturate {name};")
                lines.append(f"synthesize {name} robot{r.robot_id};")
            lines.append("saturate Arbiter;")
            lines.append("synthesize Arbiter arbiter;")
            return "\n".join(lines) + "\n"

        lines = []
        for r in self.config.robots:
            if self.num_robots > 1:
                name = f"Robot{r.robot_id}"
                out = f"robot{r.robot_id}"
            else:
                name = "Robot"
                out = "robot"
            lines.append(f"saturate {name};")
            lines.append(f"synthesize {name} {out};")

        return "\n".join(lines) + "\n"


# ============================================================
# Default Configurations
# ============================================================

def default_depots(num_robots: int, grid_size: int) -> List[Tuple[int, int]]:
    """Generate default depot positions: corners of the grid."""
    corners = [
        (0, 0),
        (grid_size - 1, 0),
        (0, grid_size - 1),
        (grid_size - 1, grid_size - 1),
    ]
    if num_robots <= len(corners):
        return corners[:num_robots]
    # For more robots, distribute along edges
    depots = list(corners)
    for i in range(len(corners), num_robots):
        depots.append((i % grid_size, 0))
    return depots[:num_robots]


def default_stations(num_robots: int, grid_size: int, depots: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """Generate default station positions: opposite corner from depot."""
    stations = []
    for dx, dy in depots:
        sx = grid_size - 1 - dx
        sy = grid_size - 1 - dy
        stations.append((sx, sy))
    return stations


def parse_coord_list(s: str) -> List[Tuple[int, int]]:
    """Parse 'x1,y1;x2,y2;...' into list of (x,y) tuples."""
    coords = []
    for pair in s.split(";"):
        pair = pair.strip()
        if pair:
            x, y = pair.split(",")
            coords.append((int(x.strip()), int(y.strip())))
    return coords


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="GridWorld LTL Contract Generator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--grid-size", type=int, default=3, help="Grid dimension N (NxN)")
    parser.add_argument("--num-robots", type=int, default=1, help="Number of robots")
    parser.add_argument("--obstacles", type=str, default="1,1", help="Obstacle cells: x1,y1;x2,y2;...")
    parser.add_argument("--depots", type=str, default=None, help="Depot positions per robot: x1,y1;x2,y2;...")
    parser.add_argument("--control-mode", choices=["distributed", "centralized"], default="distributed")
    parser.add_argument("--variable-obstacles", type=int, default=0, help="Number of variable obstacles (as input variables)")
    parser.add_argument("--variable-targets", action="store_true", help="Use variable target assignment")
    parser.add_argument("--fixed-stations", type=str, default=None, help="Fixed station per robot: x1,y1;x2,y2;...")
    parser.add_argument("--output-dir", type=str, default=".", help="Output directory")

    args = parser.parse_args()

    N = args.grid_size
    obstacles = parse_coord_list(args.obstacles) if args.obstacles else []

    # Depots
    if args.depots:
        depots = parse_coord_list(args.depots)
    else:
        depots = default_depots(args.num_robots, N)

    # Stations
    if args.variable_targets:
        stations = [None] * args.num_robots
    elif args.fixed_stations:
        stations = parse_coord_list(args.fixed_stations)
    else:
        stations = default_stations(args.num_robots, N, depots)

    # Validate
    assert len(depots) == args.num_robots, f"Need {args.num_robots} depots, got {len(depots)}"
    assert len(stations) == args.num_robots, f"Need {args.num_robots} stations, got {len(stations)}"

    grid = GridConfig(size=N, obstacles=obstacles)
    robots = []
    for i in range(args.num_robots):
        robots.append(RobotConfig(
            robot_id=i + 1,
            depot=depots[i],
            station=stations[i],
            variable_targets=args.variable_targets,
        ))

    config = SystemConfig(
        grid=grid,
        robots=robots,
        control_mode=args.control_mode,
        num_variable_obstacles=args.variable_obstacles,
    )

    # Generate
    builder = ContractBuilder(config)
    chase_gen = ChaseFileGenerator(config)

    os.makedirs(args.output_dir, exist_ok=True)

    specs = builder.build_specs_ltl()
    specs_path = os.path.join(args.output_dir, "specs.ltl")
    with open(specs_path, "w") as f:
        f.write(specs)
    print(f"Generated: {specs_path}")

    ver = chase_gen.verification_chase()
    ver_path = os.path.join(args.output_dir, "verification.chase")
    with open(ver_path, "w") as f:
        f.write(ver)
    print(f"Generated: {ver_path}")

    syn = chase_gen.synthesis_chase()
    syn_path = os.path.join(args.output_dir, "synthesis.chase")
    with open(syn_path, "w") as f:
        f.write(syn)
    print(f"Generated: {syn_path}")

    # Summary
    print(f"\nConfiguration:")
    print(f"  Grid: {N}x{N}, Obstacles: {obstacles}")
    print(f"  Robots: {args.num_robots}, Control: {args.control_mode}")
    if args.variable_obstacles > 0:
        print(f"  Variable obstacles: {args.variable_obstacles}")
    print(f"  Targets: {'variable' if args.variable_targets else 'fixed'}")
    for r in robots:
        st = "variable" if r.variable_targets else str(r.station)
        print(f"  Robot {r.robot_id}: depot={r.depot}, station={st}")


if __name__ == "__main__":
    main()
