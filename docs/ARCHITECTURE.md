# Architecture

This document specifies the DEVS models that make up the framework and maps them onto the
figures and tables of the accompanying article. Everything below is derived directly from the
source; file and line references are clickable from the repository root.

## Contents

1. [Model hierarchy](#1-model-hierarchy)
2. [Simulation engine](#2-simulation-engine)
3. [Atomic model specification](#3-atomic-model-specification)
4. [Event flow](#4-event-flow)
5. [Shared state](#5-shared-state)
6. [Article-to-code map](#6-article-to-code-map)

---

## 1. Model hierarchy

The outermost coupled model is `AMRSimModel`. It couples the Experimental Frame to the
Simulation Model, exactly as in Fig. 1 of the article.

```
AMRSimModel                                    modeling/AMRSimModel.py
├── EF : ExperimentalFrame                     modeling/experiment/experimental_frame.py
│   ├── Data_generator          (atomic)       modeling/experiment/atomic/Data_generator.py
│   └── Data_collector          (atomic)       modeling/experiment/atomic/Data_collector.py
└── SM : SimulationModel                       modeling/simulation/simulation_model.py
    ├── SystemController                       modeling/simulation/SystemController/SystemController.py
    │   ├── FleetManagement     (atomic)       modeling/simulation/SystemController/FleetManagement.py
    │   └── Scheduler           (atomic)       modeling/simulation/SystemController/Scheduler.py
    ├── AMR × numVehicles                      modeling/simulation/PhysicalSystem/Vehicle.py
    │   ├── Sensor              (atomic)       .../atomic/Sensor.py
    │   ├── Maneuver            (atomic)       .../atomic/Maneuver.py
    │   └── Planner_AMR                        .../Vehicle_Planner.py
    │       ├── Global_Planner  (atomic)       .../atomic/Global_Planner.py
    │       └── Local_Planner   (atomic)       .../atomic/Local_Planner.py
    └── Equipment × N           (atomic)       modeling/simulation/PhysicalSystem/Equipment.py
```

`SimulationModel` builds three families of couplings ([`simulation_model.py`](../modeling/simulation/simulation_model.py)):

| Coupling family | Purpose |
|---|---|
| AMR ↔ SystemController | robot pose reporting up, transport/move commands down |
| AMR ↔ AMR (all pairs) | every robot receives every other robot's pose, so peers act as dynamic obstacles |
| AMR ↔ Equipment | docking, job handover and undocking handshakes (broadcast, filtered by ID) |

The all-pairs AMR coupling is what makes the robots mutually avoiding: a robot's `Sensor`
consumes the peers' `MyManeuverState_O` events and republishes them to the `Local_Planner` as
moving obstacles.

## 2. Simulation engine

`SimulationEngine/` is a domain-independent DEVS kernel.

| Component | File | Role |
|---|---|---|
| `DEVSModel` | [`ClassicDEVS/DEVSModel.py`](../SimulationEngine/ClassicDEVS/DEVSModel.py) | common base: identity, ports, visual node/edge export |
| `DEVSAtomicModel` | [`ClassicDEVS/DEVSAtomicModel.py`](../SimulationEngine/ClassicDEVS/DEVSAtomicModel.py) | state variables, `funcExternalTransition`, `funcInternalTransition`, `funcOutput`, `funcTimeAdvance` |
| `DEVSCoupledModel` | [`ClassicDEVS/DEVSCoupledModel.py`](../SimulationEngine/ClassicDEVS/DEVSCoupledModel.py) | submodel registry, external input/output and internal couplings |
| `DEVSCoupling` | [`ClassicDEVS/DEVSCoupling.py`](../SimulationEngine/ClassicDEVS/DEVSCoupling.py) | a single (source model, source port) → (target model, target port) link |
| `CouplingGraph` | [`CouplingGraph.py`](../SimulationEngine/CouplingGraph.py) | resolves couplings into a routable graph |
| `SimulationEngine` | [`SimulationEngine.py`](../SimulationEngine/SimulationEngine.py) | event scheduler, time advance, `run()` loop |
| `DynamicDEVSCoupledModel` | [`DynamicDEVS/`](../SimulationEngine/DynamicDEVS/) | structural change at run time |
| `MRDEVSAtomicModel` / `MRDEVSCoupledModel` | [`MRDEVS/`](../SimulationEngine/MRDEVS/) | multi-resolution modelling |
| `Configurator`, `Event`, `Logger` | [`Utility/`](../SimulationEngine/Utility/) | configuration store, event payloads, trace logging |
| `Visualizer` | [`Visualzer/Visualizer.py`](../SimulationEngine/Visualzer/Visualizer.py) | live Matplotlib rendering during a run |

An atomic model subclasses `DEVSAtomicModel` and implements the four DEVS functions. The
engine's `run()` accepts the stop time and a set of trace switches:

```python
engine = SimulationEngine()
engine.setOutmostModel(objModels)
engine.run(maxTime=9999, ta=-1, logFileName='log.txt',
           logGeneral=False, logActivateState=False, logActivateMessage=False,
           logActivateTA=False, logStructure=False)
```

Setting any `logActivate*` flag to `True` writes a full event trace to `logFileName`, which is
the primary tool for inspecting model behaviour.

## 3. Atomic model specification

This section corresponds to **Table 2** of the article. `ta` is the time-advance function; all
times are in simulated seconds.

### 3.1 Physical System — AMR

#### Sensor — [`atomic/Sensor.py`](../modeling/simulation/PhysicalSystem/atomic/Sensor.py)

Perception. The model keeps a `PoseStorage` of the peer robots it has been told about and
republishes it to the planners every 100 ms.

| | |
|---|---|
| States | `INIT`, `WAIT`, `ACTIVE` |
| Input ports | `OtherManeuverState_I`, `Complete_job_I` |
| Output ports | `OtherManeuverState_O` |
| `ta` | `INIT`, `WAIT` → ∞ · `ACTIVE` → 0.1 |

> **Divergence from the article.** §3.2 and Fig. 2(a) describe the sensor as a virtual 2-D
> ray-casting device that estimates a surrounding distance distribution and filters objects
> beyond a detection radius. This release contains no ray casting: `Sensor` relays exact peer
> poses, and the range and field-of-view filtering happens downstream in `Local_Planner`
> (an 8 m detection range and a roughly 120° forward cone). The observable effect on the
> local planner is similar, but the perception model is not the one the article specifies.

#### Global_Planner — [`atomic/Global_Planner.py`](../modeling/simulation/PhysicalSystem/atomic/Global_Planner.py)

Fig. 3(a) of the article. On receiving a destination the model moves `INIT` → `GPP`, computes a
global path, emits it, and parks in `PLAN` while the robot drives. A `Replan` event from the
local layer returns it to `GPP`.

| | |
|---|---|
| States | `INIT`, `WAIT`, `GPP`, `SEND_PATH`, `PLAN` |
| Input ports | `Task_I`, `amrGoCommand`, `ManeuverState_I`, `Replan`, `StopSim` |
| Output ports | `GlobalWaypoint_O`, `RequestManeuver` |
| `ta` | `INIT`, `WAIT`, `PLAN` → ∞ · `GPP`, `SEND_PATH` → 0 |

`GPP` is where `GPP_Algorithm()` of Fig. 3(a) runs; the algorithm is chosen by the `algorithm`
constructor argument and resolved in the dispatch blocks at lines 586–597 (initial planning)
and 647–658 (replanning). Planning wall-clock time is accumulated into
`GlobalVar.GlobalPlanner_algorithm_time`, which is what the reported planner-time metric
measures. Raw paths are post-processed by `simplify_path_with_los()` (line-of-sight shortcutting)
before being emitted.

#### Local_Planner — [`atomic/Local_Planner.py`](../modeling/simulation/PhysicalSystem/atomic/Local_Planner.py)

Fig. 3(b) of the article. Follows the global path, re-evaluating linear and angular velocity
every 100 ms, and requests a global replan when the reference path becomes infeasible.

| | |
|---|---|
| States | `WAIT`, `PLAN`, `SEND`, `REPLAN`, `DOCKING`, `UNDOCKING`, `Undocking_to_fleetmanagement` |
| Input ports | `GlobalWaypoint_I`, `agent_pose_I`, `ManeuverState_I`, `OtherManeuverState_I`, `jobExchange_I`, `UndockingComplete_I`, `amrCommand` |
| Output ports | `RequestManeuver_O`, `Replan`, `DeliveryComplete`, `Docking`, `EquipmentDocking`, `UndockingComplete`, `Undocking_O`, `EmergencyBackup_O` |
| `ta` | `WAIT` → ∞ · `SEND`, `REPLAN`, `Undocking_to_fleetmanagement` → 0 · `PLAN` → 0.1 · `UNDOCKING` → 1.0 · otherwise → 1.0 |

The 0.1 s in `PLAN` is the local control period: `calc_dwa()` is evaluated once per tick against
the current obstacle set (static polygons plus peer robots) and produces the velocity command
`(v, ω)` sent to `Maneuver`.

#### Maneuver — [`atomic/Maneuver.py`](../modeling/simulation/PhysicalSystem/atomic/Maneuver.py)

Differential-drive kinematics (Fig. 2(b), Eq. 1):
`ẋ = v·cos θ`, `ẏ = v·sin θ`, `θ̇ = ω`.

| | |
|---|---|
| States | `INIT`, `Move`, `WAIT`, `Docking`, `Undocking` |
| Input ports | `RequestManeuver_I`, `Docking_I`, `Undocking`, `StopSim`, `amrCommand` |
| Output ports | `MyManeuverState_O`, `Complete_O` |
| `ta` | `Move` → 0.1 · `WAIT` → ∞ · `Docking` → 1 · `Undocking`, `INIT` → 0 |

`Move` integrates the pose at 100 ms and publishes `MyManeuverState_O`, which fans out to the
robot's own `Sensor` and planners, to every peer robot, to `FleetManagement` and to
`Data_collector`. This single event is the backbone of the whole model.

### 3.2 Physical System — Equipment

#### Equipment — [`Equipment.py`](../modeling/simulation/PhysicalSystem/Equipment.py)

Abstraction of a process machine. The state set matches the article exactly, with two extra
states for the docking handshake.

| | |
|---|---|
| States | `EMPTY`, `LOAD`, `BUSY`, `DONE`, `UNLOAD`, `INFORM`, `DOCKING`, `UNDOCKING` |
| Input ports | `job`, `EquipmentDocking`, `amrDocking` |
| Output ports | `informDone`, `informFree`, `jobExchange`, `UndockingComplete` |
| `ta` | `EMPTY`, `DONE` → ∞ · `LOAD`, `UNLOAD` → `exchangeTime` · `BUSY` → `processTime` · `INFORM` → 0 · `DOCKING`, `UNDOCKING` → 1 |

`processTime` and `exchangeTime` come from `JSON/map.json`, scaled by the machine's performance
factor in `JSON/processInfo.json`. `DONE` blocks indefinitely until a robot arrives — this is
what generates job waiting time.

### 3.3 Control System

#### FleetManagement — [`FleetManagement.py`](../modeling/simulation/SystemController/FleetManagement.py)

Maintains pose, velocity and duty state for every robot, and translates transport orders from
the scheduler into per-robot move commands.

| | |
|---|---|
| States | `WAIT`, `UPDATE`, `SEND_GO_COMMAND`, `SEND_COMMAND` |
| Input ports | `amrPosition`, `taskAssign`, `undockingComplete` |
| Output ports | `fleetInfo`, `amrCommand`, `amrGoCommand` |
| `ta` | `WAIT` → ∞ · all others → 0 |

`amrGoCommand` is a bare move order (no job attached); `amrCommand` carries a job. Splitting the
two is what lets a robot be repositioned without being considered loaded.

#### Scheduler — [`Scheduler.py`](../modeling/simulation/SystemController/Scheduler.py)

Matches waiting jobs, free machines and idle robots, and issues transport orders.

| | |
|---|---|
| States | `WAIT`, `COMMAND`, `COMPLETE` |
| Input ports | `informDone`, `informFree`, `fleetInfo` |
| Output ports | `jobAssign`, `taskAssign` |
| `ta` | `WAIT` → ∞ · all others → 0 |

`setNextProcess()` advances a job through the routing defined by `numStages`
(`SOURCE → STAGE_B [→ STAGE_C [→ STAGE_D]] → SINK`). Transport orders are tracked by
[`TransportCommand.py`](../modeling/simulation/SystemController/TransportCommand.py).

Separating fleet control from scheduling is deliberate: routing policy and dispatching policy
can be changed independently.

### 3.4 Experimental Frame

#### Data_generator — [`atomic/Data_generator.py`](../modeling/experiment/atomic/Data_generator.py)

Injects jobs into `SOURCE` equipment, subject to the source being `EMPTY`.

| | |
|---|---|
| States | `GEN`, `WAIT` |
| Output ports | `job` |
| `ta` | `GEN` → 1 · `WAIT` → ∞ |

#### Data_collector — [`atomic/Data_collector.py`](../modeling/experiment/atomic/Data_collector.py)

Records every pose update, computes per-job lead time, wait time and throughput, and writes the
trajectory CSVs and per-replication figures.

| | |
|---|---|
| States | `INIT`, `SAVE`, `DONE` |
| Input ports | `MyManeuverState_I`, `OtherManeuverState_I`, `Complete_I` |
| `ta` | `INIT` → ∞ · `SAVE`, `DONE` → 0 |

The output directory is resolved by the inner `_MinimalPoseBuffer`, which shares one timestamp
across all replications of a run so a whole Monte Carlo campaign lands in a single folder. See
[OUTPUTS.md](OUTPUTS.md).

#### MonteCarloAnalyzer — [`MonteCarloAnalyzer.py`](../modeling/experiment/MonteCarloAnalyzer.py)

Not a DEVS model — a plain analysis class driven from `main.py`. It accumulates each
replication's metrics, then computes min/max/mean/standard deviation across replications and
across fleet sizes, and renders the aggregate CSVs and figures.

## 4. Event flow

A full job cycle, in event order:

1. `Data_generator` emits `job` → a `SOURCE` `Equipment` accepts it and enters `BUSY`.
2. On completion the machine emits `informDone` → `Scheduler`.
3. `Scheduler` matches the job to a destination machine and an idle robot, emits `taskAssign`
   → `FleetManagement`.
4. `FleetManagement` emits `amrCommand` → the chosen robot's `Global_Planner`.
5. `Global_Planner` enters `GPP`, runs the selected algorithm, emits `GlobalWaypoint_O`
   → `Local_Planner`.
6. `Local_Planner` enters `PLAN`, and every 100 ms emits `RequestManeuver_O` (a velocity
   command) → `Maneuver`.
7. `Maneuver` integrates the pose and emits `MyManeuverState_O` → its own `Sensor` and planners,
   all peer robots, `FleetManagement`, and `Data_collector`.
8. If an obstacle enters the safety radius `r_s = R + S_min` (Eq. 2), `Local_Planner` produces an
   avoidance trajectory; if the reference path is no longer usable it emits `Replan`
   → `Global_Planner`, which returns to `GPP`.
9. On arrival, `Local_Planner` emits `EquipmentDocking` → `Equipment`; the machine hands the job
   over (`jobExchange`) and the pair completes the undocking handshake.
10. `UndockingComplete` → `FleetManagement` returns the robot to `IDLE`, and the cycle repeats
    until the job reaches `SINK`.

## 5. Shared state

[`SharedData/GlobalVar.py`](../SharedData/GlobalVar.py) holds state that is genuinely global to a
replication — the equipment registry, job registry, vehicle registry and the planner
computation-time counters (`GlobalPlanner_algorithm_time`, `LocalPlanner_algorithm_time` and
their call counts). A fresh `GlobalVar` is constructed for every replication by `AMRSimModel`,
which is what keeps Monte Carlo replications independent.

This is a pragmatic departure from strict DEVS: state that the formalism would pass as events is
instead read directly from a shared object. It is worth knowing about when extending the models.

## 6. Article-to-code map

| Article element | Implementation |
|---|---|
| Fig. 1 — overall architecture | [`modeling/AMRSimModel.py`](../modeling/AMRSimModel.py), [`modeling/simulation/simulation_model.py`](../modeling/simulation/simulation_model.py) |
| Fig. 2(a) — ray-casting perception | [`atomic/Sensor.py`](../modeling/simulation/PhysicalSystem/atomic/Sensor.py) — **partial**, no ray casting; see [§3.1](#31-physical-system--amr) |
| Fig. 2(b), Eq. 1 — differential-drive kinematics | [`atomic/Maneuver.py`](../modeling/simulation/PhysicalSystem/atomic/Maneuver.py) |
| Eq. 2 — safety radius `r_s = R + S_min` | `robot_radius` + `safety_margin` in [`DWA.py`](../Algorithm/PathPlanning/Local_path/DWA.py) |
| Fig. 3(a) — Global Planner DEVS diagram | [`atomic/Global_Planner.py`](../modeling/simulation/PhysicalSystem/atomic/Global_Planner.py) |
| Fig. 3(b) — Local Planner DEVS diagram | [`atomic/Local_Planner.py`](../modeling/simulation/PhysicalSystem/atomic/Local_Planner.py) |
| Table 2 — atomic model specification | [§3 of this document](#3-atomic-model-specification) |
| §4.2 — unified planner interface | [`Algorithm/PathPlanning/global_path/`](../Algorithm/PathPlanning/global_path/) — see [README](../README.md#swapping-path-planning-algorithms) |
| Fig. 7 — cell-type flexible flow shop | [`JSON/map.json`](../JSON/map.json), [`JSON/processInfo.json`](../JSON/processInfo.json) |
| Fig. 10 — operation-level results | [`main.py`](../main.py) sweep mode + [`MonteCarloAnalyzer.py`](../modeling/experiment/MonteCarloAnalyzer.py) |

Elements of the article that are **not** in this release — the ACO/GA dispatcher entries, the
MPC and SAC local planners, and the trigger-event harness of §4.3 — are listed under
[Scope of this release](../README.md#scope-of-this-release).
