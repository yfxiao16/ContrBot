# Natural Language Requirements for NL2LTL Contracts

Each LTL formula is paired with its natural language equivalent, organized by contract.

---

## 1. Single-Robot Warehouse AGV (`agv_warehouse.ltl`)

### Environment Contract

**Initial conditions:**
- The robot starts idle — not navigating, not picking up, not dropping off.
- No items are held, no locations are reached, no task is assigned.

**Guarantees (what the environment promises):**

| # | Natural Language | LTL |
|---|-----------------|-----|
| E1 | If the robot navigates to the shelf, it will eventually arrive at the shelf. | `G(go_to_shelf -> F(at_shelf))` |
| E2 | If the robot attempts a pickup, it will eventually have the item. | `G(pickup -> F(has_item))` |
| E3 | If the robot navigates to the dock, it will eventually arrive at the dock. | `G(go_to_dock -> F(at_dock))` |
| E4 | If the robot drops off the item, it will eventually be delivered. | `G(dropoff -> F(delivered))` |
| E5 | New delivery tasks are assigned infinitely often. | `G F(task)` |
| E6 | A task remains active until the delivery is completed. | `G(task & !delivered -> X(task))` |

### Robot Contract

**Guarantees (what the robot promises):**

| # | Natural Language | LTL |
|---|-----------------|-----|
| R1 | When a task is assigned and the robot doesn't have an item and isn't at the shelf, it will eventually go to the shelf. | `G(task & !has_item & !at_shelf -> F(go_to_shelf))` |
| R2 | When the robot is at the shelf, it will eventually pick up the item. | `G(at_shelf -> F(pickup))` |
| R3 | When the robot has the item, it will eventually go to the dock. | `G(has_item -> F(go_to_dock))` |
| R4 | When the robot is at the dock with the item, it will eventually drop it off. | `G(at_dock & has_item -> F(dropoff))` |
| R5 | The robot never picks up unless it is at the shelf. | `G(!at_shelf -> !pickup)` |
| R6 | The robot never drops off unless it is at the dock with the item. | `G(!at_dock \| !has_item -> !dropoff)` |
| R7 | The robot does not go to the dock before having the item. | `(!go_to_dock) U has_item` |
| R8 | The robot does not drop off before arriving at the dock. | `(!dropoff) U at_dock` |
| R9 | The robot never navigates to the shelf and picks up at the same time. | `G(!(go_to_shelf & pickup))` |
| R10 | The robot never navigates to the shelf and to the dock at the same time. | `G(!(go_to_shelf & go_to_dock))` |
| R11 | The robot never picks up and navigates to the dock at the same time. | `G(!(pickup & go_to_dock))` |
| R12 | The robot never picks up and drops off at the same time. | `G(!(pickup & dropoff))` |
| R13 | The robot never navigates to the dock and drops off at the same time. | `G(!(go_to_dock & dropoff))` |
| R14 | The robot is done if and only if the item has been delivered. | `G(delivered <-> done)` |
| R15 | The robot eventually completes a task, and this happens infinitely often. | `G F(done)` |

### System Contract

| # | Natural Language | LTL |
|---|-----------------|-----|
| S1 | Whenever a task is assigned, the item is eventually delivered. | `G(task -> F(delivered))` |
| S2 | The robot does not go to the dock before having the item. | `(!go_to_dock) U has_item` |
| S3 | The robot does not drop off before arriving at the dock. | `(!dropoff) U at_dock` |
| S4 | No two actions happen simultaneously. | `G(!(go_to_shelf & pickup))` etc. |
| S5 | The robot completes tasks infinitely often. | `G F(done)` |

---

## 2. Multi-Robot Warehouse AGV (`agv_multi_robot.ltl`)

### Robot1 / Robot2 Contracts

Each robot has the same structure. Below uses Robot 1 as example; Robot 2 is symmetric.

**Assumptions (what the robot expects from the environment):**

| # | Natural Language | LTL |
|---|-----------------|-----|
| A1 | If Robot 1 navigates to the shelf while holding a shelf grant, it eventually arrives. | `G(r1_go_to_shelf & r1_shelf_grant -> F(r1_at_shelf))` |
| A2 | If Robot 1 attempts pickup, it eventually has the item. | `G(r1_pickup -> F(r1_has_item))` |
| A3 | If Robot 1 navigates to the dock while holding a dock grant, it eventually arrives. | `G(r1_go_to_dock & r1_dock_grant -> F(r1_at_dock))` |
| A4 | If Robot 1 drops off the item, it is eventually delivered. | `G(r1_dropoff -> F(r1_delivered))` |
| A5 | Tasks for Robot 1 are assigned infinitely often. | `G F(r1_task)` |
| A6 | A task remains active until delivery is completed. | `G(r1_task & !r1_delivered -> X(r1_task))` |
| A7 | If Robot 1 requests the shelf, it is eventually granted access. | `G(r1_shelf_request -> F(r1_shelf_grant))` |
| A8 | If Robot 1 requests the dock, it is eventually granted access. | `G F(r1_dock_request -> r1_dock_grant)` |

**Guarantees (what the robot promises):**

| # | Natural Language | LTL |
|---|-----------------|-----|
| G1 | When tasked without item or shelf access, eventually navigate to shelf. | `G(r1_task & !r1_has_item & !r1_at_shelf -> F(r1_go_to_shelf))` |
| G2 | When at the shelf, eventually pick up the item. | `G(r1_at_shelf -> F(r1_pickup))` |
| G3 | When holding the item, eventually go to the dock. | `G(r1_has_item -> F(r1_go_to_dock))` |
| G4 | When at the dock with the item, eventually drop it off. | `G(r1_at_dock & r1_has_item -> F(r1_dropoff))` |
| G5 | Never pick up unless at the shelf. | `G(!r1_at_shelf -> !r1_pickup)` |
| G6 | Never drop off unless at the dock with the item. | `G(!r1_at_dock \| !r1_has_item -> !r1_dropoff)` |
| G7 | Always request shelf access before navigating to the shelf. | `G(r1_go_to_shelf -> r1_shelf_request)` |
| G8 | Always request dock access before navigating to the dock. | `G(r1_go_to_dock -> r1_dock_request)` |
| G9 | Never navigate to the shelf without a shelf grant. | `G(!r1_shelf_grant -> !r1_go_to_shelf)` |
| G10 | Never navigate to the dock without a dock grant. | `G(!r1_dock_grant -> !r1_go_to_dock)` |
| G11 | Do not go to the dock before having the item. | `(!r1_go_to_dock) U r1_has_item` |
| G12 | Do not drop off before arriving at the dock. | `(!r1_dropoff) U r1_at_dock` |
| G13 | Never navigate to shelf and dock at the same time. | `G(!(r1_go_to_shelf & r1_go_to_dock))` |
| G14 | Never pick up and drop off at the same time. | `G(!(r1_pickup & r1_dropoff))` |
| G15 | Robot 1 is done if and only if the item is delivered. | `G(r1_delivered <-> r1_done)` |
| G16 | Robot 1 completes tasks infinitely often. | `G F(r1_done)` |

### Arbiter Contract

**Guarantees (what the arbiter promises):**

| # | Natural Language | LTL |
|---|-----------------|-----|
| AB1 | Robot 1 and Robot 2 are never both granted shelf access at the same time. | `G(!(r1_shelf_grant & r2_shelf_grant))` |
| AB2 | Robot 1 and Robot 2 are never both granted dock access at the same time. | `G(!(r1_dock_grant & r2_dock_grant))` |
| AB3 | A shelf grant is only given when the robot has requested it. | `G(r1_shelf_grant -> r1_shelf_request)` |
| AB4 | A dock grant is only given when the robot has requested it. | `G(r1_dock_grant -> r1_dock_request)` |
| AB5 | (Same for Robot 2.) | `G(r2_shelf_grant -> r2_shelf_request)` etc. |
| AB6 | If Robot 1 requests the shelf, it is eventually granted. | `G(r1_shelf_request -> F(r1_shelf_grant))` |
| AB7 | If Robot 1 requests the dock, it is eventually granted. | `G F(r1_dock_request -> r1_dock_grant)` |
| AB8 | (Same fairness for Robot 2.) | `G F(r2_shelf_request -> r2_shelf_grant)` etc. |

### System Contract

| # | Natural Language | LTL |
|---|-----------------|-----|
| MS1 | Whenever Robot 1 receives a task, it eventually completes it. | `G(r1_task -> F(r1_done))` |
| MS2 | Whenever Robot 2 receives a task, it eventually completes it. | `G(r2_task -> F(r2_done))` |
| MS3 | Robot 1 and Robot 2 never navigate to the shelf at the same time. | `G(!(r1_go_to_shelf & r2_go_to_shelf))` |
| MS4 | Robot 1 and Robot 2 never navigate to the dock at the same time. | `G(!(r1_go_to_dock & r2_go_to_dock))` |
| MS5 | Both robots complete tasks infinitely often. | `G F(r1_done)`, `G F(r2_done)` |

---

## 3. Patrol-and-Respond AGV (`agv_patrol.ltl`)

### Patrol Environment Contract

**Guarantees (what the environment promises):**

| # | Natural Language | LTL |
|---|-----------------|-----|
| PE1 | If the robot navigates to zone A, it eventually arrives at zone A. | `G(go_to_A -> F(at_A))` |
| PE2 | If the robot navigates to zone B, it eventually arrives at zone B. | `G(go_to_B -> F(at_B))` |
| PE3 | If the robot navigates to zone C, it eventually arrives at zone C. | `G(go_to_C -> F(at_C))` |
| PE4 | If the robot inspects, the inspection eventually completes. | `G(inspect -> F(inspection_done))` |
| PE5 | If the robot reports, the report is eventually acknowledged. | `G(report -> F(report_ack))` |
| PE6 | An alert in zone A persists until the inspection is done. | `G(alert_A & !inspection_done -> X(alert_A))` |
| PE7 | An alert in zone B persists until the inspection is done. | `G(alert_B & !inspection_done -> X(alert_B))` |
| PE8 | An alert in zone C persists until the inspection is done. | `G(alert_C & !inspection_done -> X(alert_C))` |
| PE9 | The robot is never at two zones at the same time. | `G(!(at_A & at_B))`, `G(!(at_A & at_C))`, `G(!(at_B & at_C))` |

### Patrol Robot Contract

**Guarantees (what the robot promises):**

| # | Natural Language | LTL |
|---|-----------------|-----|
| PR1 | The robot visits zone A infinitely often (patrol). | `G F(at_A)` |
| PR2 | The robot visits zone B infinitely often (patrol). | `G F(at_B)` |
| PR3 | The robot visits zone C infinitely often (patrol). | `G F(at_C)` |
| PR4 | If there is an alert in zone A, the robot eventually goes to zone A. | `G(alert_A -> F(go_to_A))` |
| PR5 | If there is an alert in zone B, the robot eventually goes to zone B. | `G(alert_B -> F(go_to_B))` |
| PR6 | If there is an alert in zone C, the robot eventually goes to zone C. | `G(alert_C -> F(go_to_C))` |
| PR7 | If the robot is at zone A and there is an alert, it eventually inspects. | `G(at_A & alert_A -> F(inspect))` |
| PR8 | If the robot is at zone B and there is an alert, it eventually inspects. | `G(at_B & alert_B -> F(inspect))` |
| PR9 | If the robot is at zone C and there is an alert, it eventually inspects. | `G(at_C & alert_C -> F(inspect))` |
| PR10 | After an inspection is done, the robot eventually reports. | `G(inspection_done -> F(report))` |
| PR11 | The robot does not inspect unless it is at some zone. | `G(!at_A & !at_B & !at_C -> !inspect)` |
| PR12 | The robot does not inspect at zone A if there is no alert there. | `G(at_A & !alert_A -> !inspect)` |
| PR13 | The robot does not inspect at zone B if there is no alert there. | `G(at_B & !alert_B -> !inspect)` |
| PR14 | The robot does not inspect at zone C if there is no alert there. | `G(at_C & !alert_C -> !inspect)` |
| PR15 | The robot does not report before an inspection is done. | `(!report) U inspection_done` |
| PR16 | The robot does not navigate while inspecting. | `G(!(inspect & go_to_A))` etc. |
| PR17 | The robot does not navigate to two zones at the same time. | `G(!(go_to_A & go_to_B))` etc. |
| PR18 | After a report is acknowledged, the robot resumes patrol (no further inspections until a new alert or navigation). | `G(report_ack -> (!inspect U (alert_A \| alert_B \| alert_C \| go_to_A \| go_to_B \| go_to_C)))` |
| PR19 | If the robot is at zone A with an alert, it must inspect within 2 steps or leave. | `G(at_A & alert_A -> X(X(inspect \| !at_A)))` |
| PR20 | If the robot is at zone B with an alert, it must inspect within 2 steps or leave. | `G(at_B & alert_B -> X(X(inspect \| !at_B)))` |
| PR21 | If the robot is at zone C with an alert, it must inspect within 2 steps or leave. | `G(at_C & alert_C -> X(X(inspect \| !at_C)))` |

### System Contract

| # | Natural Language | LTL |
|---|-----------------|-----|
| PS1 | The robot visits every zone infinitely often. | `G F(at_A)`, `G F(at_B)`, `G F(at_C)` |
| PS2 | If there is an alert in any zone, the inspection is eventually completed. | `G(alert_A -> F(inspection_done))` etc. |
| PS3 | After inspection, the report is eventually acknowledged. | `G(inspection_done -> F(report_ack))` |
| PS4 | The robot is never at two zones simultaneously. | `G(!(at_A & at_B))` etc. |
