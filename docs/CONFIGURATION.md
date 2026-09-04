# Configuration reference

Every experiment condition is expressed in the four JSON files under [`JSON/`](../JSON/).
No source edit is needed to change fleet size, replication count, layout or robot dynamics.

## How configuration is loaded

[`Environment/EnvironmentLoader.py`](../Environment/EnvironmentLoader.py) reads the files named
in `main.py` and flattens them into a single `Configurator` object that every model queries with
`objConfiguration.getConfiguration(key)`:

```python
strPath      = "JSON/"
lstFileNames = ["setup", "map", "processInfo", "vehicleInfo"]
envLoader    = EnvironmentLoader(strPath, lstFileNames)
objConfiguration = envLoader.getConfiguration()
```

Two properties of the loader matter in practice:

- **Files are dispatched on their `fileName` field, not on the path.** Every configuration file
  must contain a top-level `"fileName"` matching one of `setup`, `map`, `processInfo`,
  `vehicleInfo`; anything else is skipped with a warning.
- **Load order is significant.** `setup` must be read before `map` and `processInfo`, because
  `numStages` is used to filter equipment, waiting areas and process sequences as those files are
  read. Keep `"setup"` first in `lstFileNames`.

Paths are resolved relative to the current working directory, so run `python main.py` from the
repository root.

---

## `setup.json` — run control

```json
{
    "fileName": "setup",
    "numVehicles": 5,
    "numJob": 1,
    "numStages": 1,
    "isTerminalOn": true,
    "isVisualizerOn": true,
    "Vehiclechange": true,
    "monteCarlo": 2,
    "renderTime": 0.1
}
```

| Key | Type | Default | Meaning |
|---|---|---|---|
| `numVehicles` | int | — | Fleet size. In sweep mode (`Vehiclechange: true`) this is the **upper bound**: the run sweeps 1, 2, …, `numVehicles`. Must not exceed the number of entries in `vehicleInfo.json`. |
| `numJob` | int | — | Jobs injected per process sequence. |
| `numStages` | int | `3` | Number of intermediate process stages. `1` → `SOURCE → STAGE_B → SINK`; `2` adds `STAGE_C`; `3` adds `STAGE_D`. Equipment, waiting areas and sequences are filtered to match. |
| `isTerminalOn` | bool | — | Verbose per-event tracing to the console. |
| `isVisualizerOn` | bool | — | Live Matplotlib rendering during the run. Turn off for batch experiments — it dominates wall-clock time. |
| `renderTime` | float | — | Frame interval of the live visualiser, in seconds. |
| `monteCarlo` | int | `1` | Monte Carlo replications per scenario. |
| `Vehiclechange` | bool | `false` | `true` → fleet-size sweep 1…`numVehicles`, each with `monteCarlo` replications. `false` → a single fleet size of `numVehicles`. |

**Total simulation runs** = `monteCarlo` in single mode, or `monteCarlo × numVehicles` in sweep
mode. The shipped configuration therefore performs 2 × 5 = 10 runs.

---

## `map.json` — shop-floor layout

Describes the physical environment: process machines with their docking ports, and robot
waiting areas. This is the layout of Fig. 7 in the article.

```json
{
    "fileName": "map",
    "equipmentInfo": [
        {
            "equipmentID": "A-1",
            "processType": "SOURCE",
            "stageID": "STAGE_A",
            "processTime": 0.0,
            "exchangeTime": 1.0,
            "performance": "A",
            "inputPort":  { "nodeID": "A-1_IN",  "position": {"x": 10, "y": 50},
                            "boundingBox": {"width": 5.0, "height": 5.0} },
            "outputPort": { "nodeID": "A-1_OUT", "position": {"x": 20, "y": 50},
                            "boundingBox": {"width": 5.0, "height": 5.0} },
            "workPosition": { "...": "..." }
        }
    ],
    "WatingareaInfo": [
        { "areaID": "WAITING_AREA_A", "position": {"x": 35, "y": 50},
          "boundingBox": {"width": 8.0, "height": 8.0} }
    ]
}
```

### `equipmentInfo[]`

| Field | Meaning |
|---|---|
| `equipmentID` | Unique machine identifier, e.g. `B-2`. |
| `processType` | `SOURCE`, `SINK`, or `PROCESS_<stage><index>` (e.g. `PROCESS_C1`). Jobs enter at `SOURCE` and leave at `SINK`. |
| `stageID` | `STAGE_A` … `STAGE_OUT`. Machines sharing a `stageID` are interchangeable, which is what makes the shop a *flexible* flow shop. |
| `processTime` | Machining duration in simulated seconds; the `BUSY` time advance. |
| `exchangeTime` | Load/unload handover duration; the `LOAD`/`UNLOAD` time advance. |
| `performance` | Key into `processInfo.json → performanceInfo`, scaling this machine's speed. |
| `inputPort` / `outputPort` | Docking node the robot drives to, with its position and footprint. Robots deliver to `inputPort` and pick up from `outputPort`. |
| `workPosition` | The machine body footprint, treated as a static obstacle by the planners. |

The shipped layout has 11 machines: one `SOURCE` (`A-1`), three parallel machines at each of
`STAGE_B`, `STAGE_C` and `STAGE_D`, and one `SINK` (`OUT`).

### `WatingareaInfo[]`

Parking areas an idle robot is sent to. Each has an `areaID`, a `position` and a `boundingBox`.
Note the spelling — the key is `WatingareaInfo` in the file and is exposed to models as
`watingAreaInfo`.

### Stage filtering

When `numStages < 3`, `EnvironmentLoader` removes the unused stages, shifts the remaining
machines and waiting areas into position, and writes the resulting layout to
`Visualizations/<run>/<...>/map.json` so each replication records the layout it actually used.

---

## `processInfo.json` — routing and machine performance

```json
{
    "fileName": "processInfo",
    "seqInfo": [
        { "seqNum": 1, "sequenceList": ["SOURCE", "PROCESS_B1", "PROCESS_C1", "PROCESS_D1", "SINK"] },
        { "seqNum": 2, "sequenceList": ["SOURCE", "PROCESS_B2", "PROCESS_C2", "PROCESS_D2", "SINK"] },
        { "seqNum": 3, "sequenceList": ["SOURCE", "PROCESS_B3", "PROCESS_C3", "PROCESS_D3", "SINK"] }
    ],
    "performanceInfo": [ { "A": 1.0, "B": 0.9, "C": 0.8, "S": 1.2 } ]
}
```

| Key | Meaning |
|---|---|
| `seqInfo[].seqNum` | Sequence identifier. |
| `seqInfo[].sequenceList` | Ordered process types a job of this sequence visits. Truncated to match `numStages`. |
| `performanceInfo` | Multipliers applied to `processTime` by machine performance grade. A machine with grade `B` (0.9) is slower than one with grade `A` (1.0). |

`numJob` jobs are injected per sequence, so the shipped configuration produces 3 jobs in total
(3 sequences × `numJob` = 1).

---

## `vehicleInfo.json` — robots and planner parameters

> The filename is case-sensitive on Linux and macOS. It must be `vehicleInfo.json`, matching
> `lstFileNames` in `main.py`.

```json
{
    "fileName": "vehicleInfo",
    "vehicleInfo": [
        { "vehicleID": "VEHICLE0000000000001", "coordinates": [55, 30] },
        { "vehicleID": "VEHICLE0000000000002", "coordinates": [55, 50] }
    ],
    "vehicleParam": {
        "robot_radius": 1.0,
        "max_speed": 1.5,
        "...": "..."
    }
}
```

### `vehicleInfo[]`

One entry per robot: a unique `vehicleID` and its `coordinates` `[x, y]` spawn pose. The first
`numVehicles` entries are instantiated, so the ordering of this list determines which spawn
poses a smaller fleet uses. Twenty robots are predefined; add entries to run larger fleets.

### `vehicleParam`

Every key here is also promoted to a **top-level configuration key**, so models read
`objConfiguration.getConfiguration('max_speed')` directly. These are the "common vehicle and
planner parameters" of Table 5 in the article, fixed across all experiments so that performance
differences are attributable to the planner rather than to the robot dynamics.

| Parameter | Shipped value | Meaning |
|---|---|---|
| `robot_radius` | 1.0 | Robot radius `R`, used for the safety radius `r_s = R + S_min` (Eq. 2). |
| `obstacle_radius` | 0.7 | Radius assumed for a detected dynamic obstacle. |
| `max_speed` | 1.5 | Maximum linear velocity (m/s). |
| `min_speed` | 0.0 | Minimum linear velocity; `0.0` forbids reversing. |
| `max_accel` | 0.2 | Maximum linear acceleration (m/s²). |
| `max_yaw_rate` | 0.698 (40°/s) | Maximum angular velocity (rad/s). |
| `max_delta_yaw_rate` | 6.283 (360°/s²) | Maximum angular acceleration (rad/s²). |
| `dt` | 0.1 | Control period (s). Matches the `PLAN`/`Move` time advance of the local planner and maneuver models. |
| `predict_time` | 2.0 | DWA trajectory prediction horizon (s). |
| `v_resolution` | 0.1 | Linear-velocity sampling resolution of the dynamic window. |
| `yaw_rate_resolution` | 0.017 (≈1°) | Angular-velocity sampling resolution of the dynamic window. |
| `to_goal_cost_gain` | 1.0 | DWA weight on progress toward the goal. |
| `speed_cost_gain` | 1.0 | DWA weight on maintaining speed. |
| `obstacle_cost_gain` | 1.0 | DWA weight on obstacle clearance. |

The last three gains are the *local planner gain modes* of §5.2 in the article. Lowering
`v_resolution` and `yaw_rate_resolution` makes the dynamic window finer at a proportional cost
in planning time.

### Optional keys

These are read with a fallback if absent, and can be added to `vehicleParam` to tune obstacle
handling without touching the code:

| Parameter | Fallback | Meaning |
|---|---|---|
| `safety_margin` | 1.5 | Extra clearance added to `robot_radius`, i.e. `S_min` in Eq. 2. |
| `static_inflation` | 1.0 | Inflation applied to static obstacle polygons. |
| `static_soft_zone` | 2.0 | Width of the soft cost buffer outside an inflated polygon. |
| `terrain_cost_gain` | 1.0 | DWA weight on static-terrain proximity. |

---

## Selecting a path planning algorithm

The planner is **not** in JSON — it is chosen where the `AMR` models are constructed, in
[`modeling/simulation/simulation_model.py:29`](../modeling/simulation/simulation_model.py#L29):

```python
objVehicle = AMR(vehicleID, self.globalVar.objConfiguration,
                 self.globalVar, algorithm="RRT")
```

Accepted values are `"A_star"`, `"Theta_star"`, `"RRT"` and `"PRM"`; any other string falls back
to a straight-line path. See
[Swapping path planning algorithms](../README.md#swapping-path-planning-algorithms) in the README
for how to register additional planners.

---

## Worked examples

**Single-robot algorithm-level run, no visualiser**

```json
{ "fileName": "setup", "numVehicles": 1, "numJob": 1, "numStages": 1,
  "isTerminalOn": false, "isVisualizerOn": false,
  "Vehiclechange": false, "monteCarlo": 1, "renderTime": 0.1 }
```

**Operation-level fleet sweep, 3 stages, 30 replications per fleet size**

```json
{ "fileName": "setup", "numVehicles": 5, "numJob": 5, "numStages": 3,
  "isTerminalOn": false, "isVisualizerOn": false,
  "Vehiclechange": true, "monteCarlo": 30, "renderTime": 0.1 }
```

This is the shape of the Fig. 10 experiment; it produces 150 runs, so leave `isVisualizerOn`
and `isTerminalOn` off.
