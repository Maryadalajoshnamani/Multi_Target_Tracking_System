# Multi-Target Radar Tracking System Using Kalman Filtering and C

A software radar-tracking application. It reads real automotive radar detections
(RadarScenes), groups nearby detections into clusters, associates the clusters with
tracks, estimates each track's position and velocity with a Kalman filter, stores the
results in SQLite and shows them in a web dashboard.

The project covers the **software** side of radar tracking only. It does not deal with
radar hardware, antennas or embedded electronics.

## Pipeline

```
RadarScenes CSV  ->  reader  ->  frame buffer (same timestamp)
                                      |
                          DBSCAN-style clustering (per sensor)
                                      |
                  track association (Kalman-predicted gate, greedy one-to-one)
                                      |
                       Kalman filter update (x, y, vx, vy)
                          /                        \
                   SQLite database          targets.json + validation_results.json
                                                         |
                                                  web dashboard
```

| Stage | File | What it does |
| --- | --- | --- |
| Reading | `src/radar_reader.c` | Parses the CSV, converts timestamps to seconds |
| Clustering | `src/clustering.c` | Groups detections of one sensor and one frame within 1.25 m (min. 2 points) and computes a centroid. A cluster receives a reference ID only if all labelled points in it agree |
| Association | `src/tracker.c`, `src/assignment.c` | Mahalanobis (chi-square 9.21) gating against Kalman predictions, optimal one-to-one matching with the Hungarian algorithm, new tracks start tentative and are confirmed after 3 hits, tentative tracks expire after 0.5 s and confirmed ones after 1.5 s. Tracks are shared across all four sensors |
| Filtering | `src/tracker.c` | Constant-velocity Kalman filter, state `[x, y, vx, vy]`, continuous white-noise-acceleration process noise |
| Storage | `src/database.c` | `radar_target_latest` (current state) and `radar_target_history` (all observations) |
| Orchestration | `src/pipeline.c`, `src/main.c` | `pipeline.c` runs the real-dataset pipeline; `main.c` is the menu and command-line entry |
| Evaluation | `tools/evaluate_pipeline.py` | Identity metrics against the RadarScenes ground-truth labels |
| Dashboard | `dashboard/` | Radar view, tables, Kalman analysis, prediction and validation results |

## Build and run

Requires GCC and the SQLite development library (the commands below are for MSYS2 UCRT64 on Windows).

```bash
gcc -std=c11 -O2 -Wall -Wextra -Iinclude src/*.c -o radar_tracker.exe -lm -lsqlite3
./radar_tracker.exe                 # interactive menu
./radar_tracker.exe --process       # headless: run the pipeline and exit
./radar_tracker.exe --help          # --data <csv>, --db <file>, --verbose
```

Menu option **9** processes `data/radarscenes/sequence_151/radar_detections.csv`
(roughly half a minute). It clears the tracking tables first, so every run starts clean.
It writes `radar_processing_report.txt`, `radar_association_log_frame.csv`,
`targets.json`, `dashboard/targets.json` and the SQLite database.

Then evaluate and refresh the dashboard data (Python 3 with pandas and scipy):

```bash
python tools/evaluate_pipeline.py
python tools/export_dashboard_results.py
```

Open `dashboard/index.html` with the VS Code Live Server extension.

The RadarScenes data is not stored in this repository. Download it from
https://zenodo.org/records/4559821 and place `sequence_151` under `data/radarscenes/`.

## Results on RadarScenes sequence_151

| Measurement | Value |
| --- | --- |
| Radar detections / frames | 345,410 / 3,691 |
| Detections placed in clusters | 71,909; the rest are treated as noise |
| Tracks created / confirmed | 835 / 258 (only confirmed tracks are stored and shown) |
| Labelled moving detections (ground truth) | 8,205 from 5 reference objects |
| Labelled detections that reach a track | 7,102 (86.6%) |
| Identity F1 (IDF1), fused across sensors | **89.3%** (IDR 83.3%, IDP 96.2%) |
| Identity switches | 42; one track mixes two objects (weighted purity 99.4%) |
| Most heavily labelled object (5,541 detections) | 3 tracks (was 29 before the changes) |
| First half / second half of the sequence | IDF1 87.8% / 90.5% |

Before these changes (per-sensor tracking, greedy matching, a nearly-zero process
noise) IDF1 was 22-42% depending on how it was measured. The biggest single cause was an
overconfident filter that overshot after detection gaps; fusing the sensors and
using optimal assignment did the rest. See `docs/VALIDATION_REPORT.md`.

## Tests

```bash
gcc -Iinclude tests/test_assignment.c src/assignment.c -lm -o test_assignment && ./test_assignment
python tests/test_evaluate.py
```

## Limitations

- Results come from **one sequence with five labelled moving objects**. They are not a general benchmark, and the tuned parameters (process noise, gate, confirmation count) were chosen on this same sequence. Sensitivity runs show a fairly narrow good region, so expect lower numbers on other data.
- **MOTA and MOTP are not reported.** RadarScenes has no per-frame object positions.
- IDP is optimistic: unlabelled clutter that shares a track with a labelled object is not visible in the log.
- 13.4% of labelled detections never reach a confirmed track (mostly isolated single-point returns removed by clustering).
- The dashboard on GitHub Pages is a published snapshot; it does not connect to a running C program and does not yet replay the sequence.
- The `reference_track_id` ground truth is used only for evaluation, never for association.
- Developed and tested on Linux; not re-tested on Windows/MSYS2.

## Possible future work

Evaluation on more RadarScenes sequences; automatic parameter selection on held-out sequences; a dashboard replay of the real sequence; Doppler-aware association (a Doppler initialisation was tried and gave no benefit).

## Author

Joshna Manili
