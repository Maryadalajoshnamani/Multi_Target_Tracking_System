"""
Identity-based evaluation of the RadarScenes tracking pipeline.

Ground truth  : detections in the raw RadarScenes CSV that have a
                non-empty track_id and are not the STATIC class (label 11).
Hypotheses    : internal track IDs written to radar_association_log_frame.csv
                by option 9 of the C program.

Run from the project root:
    python tools/evaluate_pipeline.py

Outputs:
    evaluation_report.txt
    evaluation_summary.json

Important limitations (also written into the report):
  * RadarScenes provides ground truth only as labelled radar detections,
    not as per-frame object positions. MOTA and MOTP therefore cannot be
    computed; the metrics below are identity based.
  * The association log contains only labelled detections that were placed
    in a cluster. Unlabelled clutter that shares a track with a labelled
    object is invisible to the log, so IDFP / IDP are measured over
    labelled detections only and are optimistic.
  * The tracker runs per sensor, so the same physical object seen by
    several sensors legitimately receives several internal IDs.
    Identity metrics are therefore computed per (sensor, reference object).
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment

RAW = Path("data/radarscenes/sequence_151/radar_detections.csv")
LOG = Path("radar_association_log_frame.csv")
STATIC_LABEL = 11


def load_ground_truth(raw_path, tmin=None, tmax=None):
    raw = pd.read_csv(
        raw_path,
        usecols=["timestamp", "sensor_id", "track_id", "label_id"],
    )
    # Same time base as the C program: seconds since the first row.
    raw["t"] = (raw["timestamp"] - raw["timestamp"].iloc[0]) / 1e6
    if tmin is not None:
        raw = raw[raw["t"] >= tmin]
    if tmax is not None:
        raw = raw[raw["t"] < tmax]
    gt = raw[raw["track_id"].notna() & (raw["label_id"] != STATIC_LABEL)]
    return raw, gt


def load_log(log_path, tmin=None, tmax=None):
    log = pd.read_csv(log_path, dtype={"Reference_ID": str})
    log = log[log["Reference_ID"].notna() & (log["Label_ID"] != STATIC_LABEL)]
    if tmin is not None:
        log = log[log["Timestamp"] >= tmin]
    if tmax is not None:
        log = log[log["Timestamp"] < tmax]
    return log.sort_values(["Sensor_ID", "Reference_ID", "Timestamp"])


def idf1_for_sensor(group, gt_total_for_sensor):
    """Hungarian matching between reference objects and internal IDs."""
    table = pd.crosstab(group["Reference_ID"], group["Internal_ID"])
    if table.empty:
        return 0, 0
    rows, cols = linear_sum_assignment(-table.values)
    idtp = int(table.values[rows, cols].sum())
    return idtp, int(len(group))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--raw", default=str(RAW), help="RadarScenes radar_detections.csv")
    parser.add_argument("--log", default=str(LOG), help="association log written by option 9")
    parser.add_argument("--out-dir", default=".", help="where to write the report files")
    parser.add_argument("--tmin", type=float, default=None, help="first second to evaluate")
    parser.add_argument("--tmax", type=float, default=None, help="end second (exclusive)")
    parser.add_argument("--quiet", action="store_true", help="do not print the report")
    args = parser.parse_args()

    raw_path, log_path = Path(args.raw), Path(args.log)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for p in (raw_path, log_path):
        if not p.exists():
            sys.exit(f"Missing file: {p}")

    raw, gt = load_ground_truth(raw_path, args.tmin, args.tmax)
    log = load_log(log_path, args.tmin, args.tmax)

    gt_total = len(gt)
    logged = len(log)

    lines = []
    out = lines.append
    summary = {}

    out("TRACKING EVALUATION REPORT")
    out("=" * 60)
    out(f"Raw detections                      : {len(raw):,}")
    out(f"Labelled non-static detections (GT) : {gt_total:,}")
    out(f"Reference objects (non-static)      : {gt['track_id'].nunique()}")
    out(f"GT detections placed in a track     : {logged:,}")
    coverage = 100.0 * logged / gt_total if gt_total else 0.0
    out(f"Coverage of GT detections           : {coverage:.2f}%")
    out(f"GT detections dropped as noise      : {gt_total - logged:,}")
    out("")

    summary.update(
        raw_detections=int(len(raw)),
        gt_detections=int(gt_total),
        gt_objects=int(gt["track_id"].nunique()),
        gt_in_tracks=int(logged),
        coverage_percent=round(coverage, 2),
    )

    # ---------- per (sensor, reference object) ----------
    per_object = []
    total_switches = 0
    total_transitions = 0
    for (sensor, ref), g in log.groupby(["Sensor_ID", "Reference_ID"]):
        ids = g["Internal_ID"].tolist()
        switches = sum(1 for a, b in zip(ids, ids[1:]) if a != b)
        counts = g["Internal_ID"].value_counts()
        gt_n = int(((gt["sensor_id"] == sensor) & (gt["track_id"] == ref)).sum())
        per_object.append(
            dict(
                sensor=int(sensor),
                reference=ref[:8],
                gt_detections=gt_n,
                logged=len(ids),
                distinct_tracks=int(counts.size),
                main_track_share=round(100.0 * counts.iloc[0] / len(ids), 1),
                id_switches=int(switches),
            )
        )
        total_switches += switches
        total_transitions += max(len(ids) - 1, 0)

    po = pd.DataFrame(per_object).sort_values(
        ["gt_detections"], ascending=False
    )
    out("PER (SENSOR, REFERENCE OBJECT)")
    out(po.to_string(index=False))
    out("")

    # ---------- identity metrics ----------
    idtp_total = 0
    for sensor, g in log.groupby("Sensor_ID"):
        idtp, _ = idf1_for_sensor(g, None)
        idtp_total += idtp

    idfn = gt_total - idtp_total
    idfp = logged - idtp_total
    idr = idtp_total / gt_total if gt_total else 0.0
    idp = idtp_total / logged if logged else 0.0
    idf1 = (
        2 * idtp_total / (2 * idtp_total + idfp + idfn)
        if (2 * idtp_total + idfp + idfn)
        else 0.0
    )
    continuity = (
        100.0 * (total_transitions - total_switches) / total_transitions
        if total_transitions
        else 0.0
    )

    out("IDENTITY METRICS (per sensor, detection level)")
    out(f"IDTP (detections on the best-matching track) : {idtp_total:,}")
    out(f"IDFN (GT detections not on matched track)    : {idfn:,}")
    out(f"IDFP (logged detections not on matched track): {idfp:,}")
    out(f"IDR  (identity recall)                       : {100*idr:.2f}%")
    out(f"IDP  (identity precision, optimistic)        : {100*idp:.2f}%")
    out(f"IDF1                                         : {100*idf1:.2f}%")
    out("")
    out("CONTINUITY (adjacent logged detections of one object per sensor)")
    out(f"Transitions        : {total_transitions:,}")
    out(f"Identity switches  : {total_switches:,}")
    out(f"Continuity         : {continuity:.2f}%")
    out("(Continuity ignores detections dropped as noise, so it is higher than IDR.)")
    out("")

    # ---------- fused identity metrics (all sensors together) ----------
    # One physical object should keep one ID no matter which sensor sees it.
    ftable = pd.crosstab(log["Reference_ID"], log["Internal_ID"])
    frows, fcols = linear_sum_assignment(-ftable.values)
    f_idtp = int(ftable.values[frows, fcols].sum())
    f_idfn = gt_total - f_idtp
    f_idfp = logged - f_idtp
    f_idr = f_idtp / gt_total if gt_total else 0.0
    f_idp = f_idtp / logged if logged else 0.0
    f_den = 2 * f_idtp + f_idfp + f_idfn
    f_idf1 = 2 * f_idtp / f_den if f_den else 0.0

    # identity switches over time, one decision per frame (mode of IDs)
    f_switches = 0
    f_tracks = {}
    for ref, g in log.groupby("Reference_ID"):
        per_frame = (
            g.groupby(["Timestamp", "Sensor_ID"])["Internal_ID"]
            .agg(lambda x: x.mode().iloc[0])
            .reset_index()
            .sort_values("Timestamp")["Internal_ID"]
            .tolist()
        )
        f_switches += sum(1 for a, b in zip(per_frame, per_frame[1:]) if a != b)
        f_tracks[ref[:8]] = int(g["Internal_ID"].nunique())

    out("FUSED IDENTITY METRICS (all four sensors together)")
    out(f"IDTP / IDFN / IDFP        : {f_idtp:,} / {f_idfn:,} / {f_idfp:,}")
    out(f"IDR / IDP / IDF1          : {100*f_idr:.2f}% / {100*f_idp:.2f}% / {100*f_idf1:.2f}%")
    out(f"Identity switches (frame) : {f_switches}")
    out(f"Tracks per object         : {f_tracks}")
    out("")

    summary.update(
        fused_idr_percent=round(100 * f_idr, 2),
        fused_idp_percent=round(100 * f_idp, 2),
        fused_idf1_percent=round(100 * f_idf1, 2),
        fused_identity_switches=int(f_switches),
        fused_tracks_per_object=f_tracks,
    )

    # ---------- track purity ----------
    purity_rows = []
    for tid, g in log.groupby("Internal_ID"):
        vc = g["Reference_ID"].value_counts()
        purity_rows.append(
            dict(
                track=int(tid),
                refs=int(vc.size),
                rows=len(g),
                purity=vc.iloc[0] / len(g),
            )
        )
    pur = pd.DataFrame(purity_rows)
    mixed = int((pur["refs"] > 1).sum())
    weighted_purity = float((pur["purity"] * pur["rows"]).sum() / pur["rows"].sum())

    out("TRACK PURITY (labelled detections only)")
    out(f"Tracks containing labelled detections        : {len(pur)}")
    out(f"Tracks mixing more than one reference object : {mixed}")
    out(f"Weighted purity                              : {100*weighted_purity:.2f}%")
    out("")

    # ---------- fragmentation ----------
    frag = po.groupby("reference").agg(
        sensors=("sensor", "nunique"),
        tracks=("distinct_tracks", "sum"),
    )
    out("FRAGMENTATION (distinct internal tracks per reference object, all sensors)")
    out(frag.to_string())
    out("")

    out("NOT COMPUTED")
    out("MOTA / MOTP: RadarScenes has no per-frame object positions, only")
    out("labelled detections, so these standard metrics are not defined here.")
    out("IDP is optimistic because unlabelled clutter on a track is not logged.")

    summary.update(
        idtp=int(idtp_total),
        idfn=int(idfn),
        idfp=int(idfp),
        idr_percent=round(100 * idr, 2),
        idp_percent=round(100 * idp, 2),
        idf1_percent=round(100 * idf1, 2),
        transitions=int(total_transitions),
        identity_switches=int(total_switches),
        continuity_percent=round(continuity, 2),
        tracks_with_labelled_detections=int(len(pur)),
        mixed_tracks=mixed,
        weighted_purity_percent=round(100 * weighted_purity, 2),
        per_object=per_object,
    )

    (out_dir / "evaluation_report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (out_dir / "evaluation_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if not args.quiet:
        print("\n".join(lines))


if __name__ == "__main__":
    main()
