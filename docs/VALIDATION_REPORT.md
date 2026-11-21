# Validation Report: RadarScenes sequence_151

This report documents how the pipeline was validated, what the numbers mean and where the tracker falls short.

## 1. Pipeline run

Menu option 9 was run on the full sequence (345,410 detections, 3,691 frames).
The run was repeated three times with identical output (72,546 clustered detections, 1,410 tracks).

| Check | Result |
| --- | --- |
| Build | Compiles without errors or warnings (`-std=c11 -Wall -Wextra`) |
| Reference IDs reach clusters and tracks | Yes: 47 tracks carry a reference ID in `radar_target_latest`, 2,496 history rows carry one |
| Track purity | 100% weighted purity; 0 tracks contain labelled detections of two different objects |
| SQLite consistency | 1,410 latest rows, 31,209 history rows, 1,410 distinct IDs; no duplicate (id, timestamp) rows; every latest ID exists in history |

## 2. Problems found and fixed

1. **Option 9 did not finish in practical time.** The history insert checks for duplicates with a lookup on `(target_id, timestamp, x, y)` and there was no index, so every frame scanned the whole history table. An index on `(target_id, timestamp)` reduced the run from over 3 minutes (never finished in testing) to about 30 seconds.
2. **Database rows from earlier runs were mixed with new ones.** Track IDs restart at 1 on every run, but the tables were never cleared. Option 9 now clears `radar_target_latest` and `radar_target_history` first (`reset_tracking_database()`). The legacy `radar_targets` table is untouched.
3. **The report said "Internal tracks created: 124".** That number is the count of track slots in memory; inactive slots are reused. The real number of tracks created is 1,410. The report now prints both.
4. **Undefined behaviour in `qsort`** when there were no candidate pairs (null pointer). Fixed with a guard.

The original files were saved in `backup/before_validation/`.

## 3. Evaluation method

Ground truth: detections with a non-empty `track_id` that are not the static class (label 11): 8,205 detections of 5 objects.
Hypotheses: the internal track IDs in `radar_association_log_frame.csv`.
Because each sensor is tracked separately, identity metrics are computed per (sensor, object). Within a sensor, reference objects are matched to track IDs with the Hungarian algorithm to obtain IDTP, then IDR, IDP and IDF1.

MOTA and MOTP are **not** computed. RadarScenes has no per-frame object positions, only labelled detections.

## 4. Results

| Metric | Value |
| --- | --- |
| Ground-truth detections | 8,205 |
| Placed in a track | 7,108 (86.63%) |
| Dropped as clustering noise | 1,097 |
| IDTP / IDFN / IDFP | 3,212 / 4,993 / 3,896 |
| IDR / IDP / IDF1 | 39.15% / 45.19% / 41.95% |
| Identity switches / continuity | 36 / 99.49% |
| Tracks mixing two objects | 0 |

The earlier figures (98.65% continuity, 96 switches) came from an older log. The same style of measure on the current run gives 99.49% and 36 switches. Neither is a formal MOTA or IDF1 score.

Fragmentation per object (distinct tracks, all sensors): `f6a76074` 29 tracks (5,541 detections, seen by all four sensors), `fcf2aae8` 8, `a6776696` 5, `f6a3bd8e` 4, `f0d0097e` 1.

## 5. Why tracks fragment

The ego vehicle is stationary in this sequence (mean speed 0.01 m/s), so coordinate frames are not the cause. Tracing the cases where a new track started although an earlier track of the same object existed gave three causes:

- **Detection gaps.** The object is missing from the raw data for 0.5 to 1.4 s several times. Clustering needs two detections, so frames where the main object produced a single detection are dropped as noise: on one sensor, 152 of its 816 frames never reached a track (155 frames contain only one labelled detection). This lengthens the gaps.
- **Prediction overshoot after a gap.** After about 1 s without a match, extrapolating the Kalman velocity overshoots. The new detection is 3 to 4.6 m from the last known position but more than 5 m from the predicted one, so it misses the gate (7 of 15 traced cases).
- **Competing clusters.** The old track was already matched to another cluster in the same frame (5 of 15 cases).

Three more cases involved a track that had already expired.
This trace covered only 15 events and is an indication, not a full breakdown.

## 6. Parameter sensitivity

Each row changes parameters from the baseline (gate 5 m, max gap 1.5 s, epsilon 1.25 m, min points 2).

| Configuration | Tracks | GT coverage | IDF1 | Switches | Mixed tracks |
| --- | --- | --- | --- | --- | --- |
| Baseline | 1,410 | 86.6% | 42.0% | 36 | 0 |
| Max gap 3.0 s | 956 | 86.6% | 45.0% | 25 | 1 |
| Gate 8 m, gap 3.0 s | 677 | 86.6% | 43.0% | 57 | 5 |
| Gate 3 m, gap 3.0 s | 1,193 | 86.6% | 35.9% | 52 | 1 |
| Epsilon 1.75 m, gap 3.0 s | 1,141 | 87.4% | 41.6% | 34 | 2 |
| Epsilon 2.0 m, gap 3.0 s | 1,245 | 87.8% | 42.8% | 63 | 1 |
| Min points 1, gap 3.0 s | 3,958 | 100% | 46.6% | 157 | 5 |
| Fallback to last position when coasting (> 0.5 s) | 1,382 | 86.6% | 44.2% | 46 | 1 |

No setting raised IDF1 above 47%, and every improvement came with more mixed tracks or more switches.
The defaults were therefore left unchanged. With five labelled objects, differences of a few points are within noise, and tuning on this sequence would risk overfitting.
Better results most likely need structural changes (see the README's future work), not parameter tuning.

## 7. Dashboard

The dashboard shows the real-data results in a new section, "RadarScenes Validation Results", loaded from `dashboard/validation_results.json`.
The existing radar view, tables and Kalman panels display the targets exported to `targets.json` by option 9.
The Kalman raw-versus-filtered noise comparison only has data after menu option 8 (measurement-noise simulation), because the real dataset has no simulated noise.

## Round 2: tracking quality improvements

Changes: continuous white-noise-acceleration process noise; Mahalanobis gating; Hungarian
assignment; 3-hit track confirmation with fast expiry of tentative tracks; tracks shared
across the four sensors; pipeline split into `pipeline.c`; command-line options; unit tests.

| Metric (fused) | Before | After |
| --- | --- | --- |
| IDF1 | 22.3% | 89.3% |
| IDR / IDP | - | 83.3% / 96.2% |
| Tracks for the main object (5,541 detections) | 29 | 3 |
| Tracks created / confirmed | 1,410 / n.a. | 835 / 258 |

Consistency check: scoring t<34 s and t>=34 s separately gives IDF1 87.8% and 90.5%.

Honest caveats: parameters were tuned on this one sequence; sensitivity sweeps show a narrow
plateau, so the headline number is likely optimistic for new data. Doppler-based track
initialisation and allowing single-point clusters gave no benefit and are disabled.
Not done: dashboard replay of the real sequence and identity timeline.
