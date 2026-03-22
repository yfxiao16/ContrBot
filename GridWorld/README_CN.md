# GridWorld 多机器人合约综合演示

基于合约的框架，用于在离散二维网格上通过 GR(1) 反应式综合生成经过验证的机器人控制器。

## 概述

本演示为多机器人网格世界场景生成时序逻辑合约，并使用 CHASE/Slugs 工具链综合有限状态机（FSM）控制器。支持以下特性：

- **可变机器人数量**（1 到 N 个）
- **可变网格大小**（NxN，可配置障碍物）
- **可变目标分配**（调度器在运行时指定目标坐标）
- **可变位置障碍物**（位置固定但运行时可配置）
- **分布式控制**（每个机器人独立合约，并行组合）
- **集中式控制**（请求-授权架构，配合仲裁器）

## 网格世界模型

```
         x=0         x=1         x=2
       +----------+----------+----------+
  y=0  | (0,0)    | (1,0)    | (2,0)    |
       | R1 基地  |   空闲   | R2 基地  |
       +----------+----------+----------+
  y=1  | (0,1)    | (1,1)    | (2,1)    |
       |   空闲   |  障碍物  |   空闲   |
       +----------+----------+----------+
  y=2  | (0,2)    | (1,2)    | (2,2)    |
       |   空闲   |   空闲   |   空闲   |
       +----------+----------+----------+
```

- **5 种动作**：停留 (0)、北 (1)、南 (2)、东 (3)、西 (4)
- 移向边界或障碍物时机器人原地不动（边界截断）
- 默认障碍物在 (1,1)；可通过 `--obstacles` 配置

## 快速开始

### 前置依赖

- `logics_tool`（CHASE 二进制文件）
- `slugs`（GR(1) 反应式综合器）
- Python 3
- 可选：`nuxmv` 用于精化检查

### 生成合约并综合

```bash
cd demos/GridWorld
export DYLD_LIBRARY_PATH=../../third_party/antlr4/lib

# 1. 生成合约文件
python3 generate_gridworld.py --num-robots 2 --variable-targets --output-dir out/

# 2. 解析并生成 structuredSlugs 规范
../../chase/bin/logics_tool -i out/specs.ltl -o out/ -c out/synthesis.chase

# 3. 编译并综合每个机器人
for r in robot1 robot2; do
    python3 ../../slugs/tools/StructuredSlugsParser/compiler.py \
        out/${r}.structuredSlugs > out/${r}.slugsin
    ../../slugs/src/slugs --explicitStrategy --jsonOutput \
        out/${r}.slugsin > out/${r}.json
done

# 4. （可选）验证组合精化关系
../../chase/bin/logics_tool -i out/specs.ltl -o out/ -c out/verification.chase -V
nuxmv out/refinement.smv
```

或使用一体化演示脚本：

```bash
./run_demo.sh /tmp/gridworld_output
```

## 合约生成器用法

```
python3 generate_gridworld.py [选项]
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--grid-size N` | 3 | 网格维度（NxN） |
| `--num-robots N` | 1 | 机器人数量 |
| `--obstacles x,y;...` | `1,1` | 障碍物坐标 |
| `--depots x,y;...` | 自动（网格角落） | 每个机器人的基地位置 |
| `--control-mode` | `distributed` | `distributed`（分布式）或 `centralized`（集中式） |
| `--variable-targets` | 关 | 使用可变目标分配（target_x, target_y 作为输入） |
| `--variable-obstacles N` | 0 | 可变位置障碍物数量 |
| `--fixed-stations x,y;...` | 自动（对角位置） | 每个机器人的固定工作站 |
| `--output-dir DIR` | `.` | 输出目录 |

### 使用示例

```bash
# 单机器人，可变目标，3x3 网格
python3 generate_gridworld.py --num-robots 1 --variable-targets --output-dir out/single/

# 2 个机器人，分布式，固定工作站
python3 generate_gridworld.py --num-robots 2 --fixed-stations "2,2;0,2" --output-dir out/dist_fixed/

# 2 个机器人，集中式仲裁器，可变目标
python3 generate_gridworld.py --num-robots 2 --control-mode centralized \
    --variable-targets --output-dir out/central/

# 2 个机器人，带可变位置障碍物
python3 generate_gridworld.py --num-robots 2 --variable-obstacles 1 \
    --variable-targets --output-dir out/with_blocks/
```

## 合约结构

### 假设-保证范式

每个合约由**假设（Assumptions）**和**保证（Guarantees）**组成。假设描述环境需满足的条件，保证描述控制器的承诺。两种控制模式下合约结构有所不同。

### 分布式模式

每个机器人拥有独立合约，观测其他所有机器人的位置。

**机器人假设**：
- 运动动力学（动作 → 位置更新）
- 任务生命周期（到达、持续直到完成、被清除）
- 目标有效性和持久性（不在障碍物上，活跃任务期间不变）
- 可变障碍物约束（位置不变、不与静态障碍物/基地重叠）
- 其他机器人的公平性（它们最终会返回基地）

**机器人保证**：
- 障碍物规避：永不进入障碍物格子
- 完成语义：`done <-> (在目标处 & 任务活跃)`
- 方向性碰撞规避：不向已被其他机器人占据的邻格移动（四个方向各一条约束）
- 活性：无限次完成任务并返回基地

```
Robot1 || Robot2 || ... || RobotN  精化  System
```

### 集中式模式（请求-授权）

每个机器人独立决策，但移动需经仲裁器批准。仲裁器不包含任何动力学知识——仅比较下一位置。

**机器人假设**：
- 条件动力学（`grant` 时位置更新为 `next_x/next_y`，否则不动）
- 任务生命周期（同分布式）
- 目标有效性和持久性（同分布式）
- 可变障碍物约束（同分布式）

**机器人保证**：
- 下一位置计算：根据动作和障碍物正确计算 `next_x, next_y`
- 请求语义：`action ≠ 0 ↔ request`
- 障碍物规避：`next_x/next_y` 永不指向障碍物
- 完成语义：`done <-> (在目标处 & 任务活跃)`
- 活性：无限次完成任务并返回基地

**仲裁器假设**：
- 初始无请求

**仲裁器保证**：
- 碰撞自由：同时批准的移动不会导致两个机器人到达同一格子
- 请求守卫：只在收到请求时才授权
- 公平性：每个请求最终被批准

```
Robot1 || Robot2 || ... || RobotN || Arbiter  精化  System
```

## 生成文件说明

| 文件 | 说明 |
|------|------|
| `specs.ltl` | LTL 合约规范 |
| `verification.chase` | CHASE 命令：组合与精化检查 |
| `synthesis.chase` | CHASE 命令：逐机器人综合 |
| `*.structuredSlugs` | logics_tool 生成的中间文件 |
| `*.slugsin` | 编译后的 Slugs 输入（中间文件） |
| `*.json` | 综合得到的 FSM 控制器 |
| `refinement.smv` | 用于精化检查的 NuSMV 模型 |

## 工具链流程

```
specs.ltl ──[logics_tool]──> *.structuredSlugs ──[compiler.py]──> *.slugsin ──[slugs]──> *.json (FSM)
    │
    └──[logics_tool -V]──> refinement.smv ──[nuxmv]──> 精化检查结果
```
