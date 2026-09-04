# Simulation outputs

Every run writes to a single timestamped directory under `Visualizations/`. The timestamp is
generated once per invocation of `main.py` and shared by all replications, so one Monte Carlo
campaign always lands in one folder.

A complete worked example ships with the repository:
[`Visualizations/20260106_142518/`](../Visualizations/20260106_142518/) — a fleet-size sweep over
1–5 robots with 2 replications each.

---

## 1. Directory layout

The layout depends on the run mode set in [`JSON/setup.json`](../JSON/setup.json).

### Fleet-size sweep (`Vehiclechange: true`)

```
Visualizations/20260106_142518/
├── vehicle_num_1/                    # scenario: 1 robot
│   ├── iteration_1/
│   │   ├── Agent/
│   │   │   └── VEHICLE0000000000001.csv
│   │   └── map.json                  # layout actually used by this replication
│   └── iteration_2/
│       └── …
├── vehicle_num_2/ … vehicle_num_5/
└── analysis/                         # aggregated across all scenarios
    ├── *_by_vehicle.csv
    ├── *_by_vehicle.png
    ├── vehicle_comparison.png
    └── vehicle_comparison_summary.txt
```

### Plain Monte Carlo (`Vehiclechange: false`)

```
Visualizations/20251107_201404/
├── iteration_1/
│   ├── Agent/
│   │   └── VEHICLE….csv
│   ├── map.json
│   └── analysis/                     # per-replication job analysis
│       ├── job_summary.csv
│       ├── lead_time.png
│       ├── wait_time.png
│       ├── job_throughput.png
│       └── summary_report.txt
├── iteration_2/ …
└── analysis/                         # aggregated across replications
    ├── *_montecarlo.csv
    ├── montecarlo_*.png
    └── montecarlo_summary.txt
```

The two `analysis` levels answer different questions:

| Level | Location | Use |
|---|---|---|
| Per-replication | `iteration_<n>/analysis/` | Inspect one run in detail; find outliers; check individual job histories. |
| Aggregated | `<run>/analysis/` | Report system performance with min / max / mean / standard deviation across replications. |

Directory creation is handled by `_MinimalPoseBuffer` inside
[`Data_collector.py`](../modeling/experiment/atomic/Data_collector.py); aggregation is handled by
[`MonteCarloAnalyzer.py`](../modeling/experiment/MonteCarloAnalyzer.py).

---

## 2. Robot trajectories — `Agent/VEHICLE*.csv`

One file per robot per replication. One row per `Maneuver` update, i.e. one row per 0.1 s of
simulated time while the robot is moving.

```csv
x,y,yaw,linear_velocity,angular_velocity
55,30,0.0,0.0,0.0
55.15,30,0.0,1.5,0.0
```

| Column | Unit | Meaning |
|---|---|---|
| `x`, `y` | m | Robot position in map coordinates. |
| `yaw` | rad | Heading `θ`. |
| `linear_velocity` | m/s | Commanded linear velocity `v`. |
| `angular_velocity` | rad/s | Commanded angular velocity `ω`. |

These are the trajectories integrated from Eq. 1, and are what
[`Visualizations/visualize.py`](../Visualizations/visualize.py) replays. The actual travelled
distance reported in the article is the path length of the `(x, y)` sequence — as opposed to the
*planned* path length produced by the global planner, and the difference between the two is the
correction contributed by the local layer.

## 3. Replication layout — `map.json`

A copy of the stage-filtered layout used by that specific replication, written so that a run
remains interpretable without knowing what `JSON/map.json` contained at the time. `visualize.py`
prefers this file over the project-level one when rendering.

---

## 4. Per-replication analysis — `iteration_<n>/analysis/`

Produced only in plain Monte Carlo mode.

### `job_summary.csv`

One row per completed job.

```csv
job_id,start_time,complete_time,lead_time,total_wait_time,num_processes
1,1,147.7,146.7,43.6,3
```

| Column | Meaning |
|---|---|
| `job_id` | Job identifier. |
| `start_time` | Simulated time the job was injected at the `SOURCE`. |
| `complete_time` | Simulated time the job reached the `SINK`. |
| `lead_time` | `complete_time − start_time`. |
| `total_wait_time` | Time the job spent waiting for a robot, summed over all stages. |
| `num_processes` | Number of process stages the job visited. |

`lead_time − total_wait_time` is the time the job spent being machined or transported, so
`total_wait_time` isolates the part of lead time that fleet sizing and path planning can improve.

### Figures and report

`lead_time.png`, `wait_time.png` and `job_throughput.png` plot the per-job distributions;
`summary_report.txt` is the same content as text.

---

## 5. Aggregated analysis — `<run>/analysis/`

### Metrics

Both modes aggregate the same eleven metrics, collected per replication by
`Data_collector.get_iteration_results()` and extended in `main.py` with wall-clock timings:

| Metric | Meaning |
|---|---|
| `avg_lead_time` | Mean job lead time (s). |
| `avg_wait_time` | Mean job wait time (s). |
| `avg_throughput` | Completed jobs per simulated second. |
| `total_jobs` | Jobs completed in the replication. |
| `sim_time` | Simulated duration of the replication (s). |
| `real_time` | Wall-clock duration of the replication (s). |
| `time_ratio` | `real_time / sim_time`. Below 1.0 means the run was faster than real time. |
| `global_planner_time` | Total wall-clock time spent inside the global planning algorithm (s). |
| `global_planner_calls` | Number of global planning invocations, i.e. initial plans plus replans. |
| `global_planner_avg` | `global_planner_time / global_planner_calls`. |
| `local_planner_time` / `_calls` / `_avg` | The same three quantities for the local planner. |

The planner-time metrics are *computation* cost, accumulated in
[`SharedData/GlobalVar.py`](../SharedData/GlobalVar.py) as the algorithms run. They are measured
in wall-clock seconds and therefore depend on the host machine — report the hardware alongside
them, and compare planners only within a single machine's results.

`global_planner_calls` doubles as the replanning count: one call per initial plan plus one per
`Replan` event raised by the local layer.

### File formats

**Sweep mode** — `<metric>_by_vehicle.csv`, one row per fleet size:

```csv
Vehicle_Count,Mean,Std,Min,Max
1,163.65,1.05,162.6,164.7
2,167.65,1.15,166.5,168.8
```

**Plain Monte Carlo mode** — `<metric>_montecarlo.csv`, one row per replication:

```csv
[#1] avg_lead_time,150.5
[#2] avg_lead_time,147.4
```

### Figures

| File | Mode | Content |
|---|---|---|
| `<metric>_by_vehicle.png` | sweep | Metric against fleet size, with standard-deviation error bars. |
| `vehicle_comparison.png` | sweep | All metrics against fleet size on one sheet. |
| `vehicle_comparison_summary.txt` | sweep | Text report of every metric per fleet size. |
| `montecarlo_<metric>.png` | Monte Carlo | Metric per replication with the mean and ±1σ band. |
| `montecarlo_algorithm_performance.png` | Monte Carlo | Global and local planner computation time. |
| `montecarlo_all_metrics.png` | Monte Carlo | All metrics on one sheet. |
| `montecarlo_summary.txt` | Monte Carlo | Text report including the planner statistics. |

`vehicle_comparison.png` in the bundled example is the operation-level figure of the article
(Fig. 10) in raw form.

---

## 6. Replaying a run

```bash
python Visualizations/visualize.py                    # most recent run
python Visualizations/visualize.py --timestamp 20260106_142518
```

The script locates the trajectory CSVs and the replication `map.json`, and animates the robots
over the layout. It requires `pandas` in addition to the simulation dependencies. Pass `--help`
for the available options.

---

## 7. What is version-controlled

Outputs are regenerated by any run, so `.gitignore` excludes everything under `Visualizations/`
except `visualize.py` and the single bundled example run. To keep a run of your own, either
commit it explicitly:

```bash
git add -f Visualizations/<timestamp>
```

or archive it outside the repository. Long campaigns are large — a 150-run sweep produces
several hundred megabytes of trajectory CSVs.
