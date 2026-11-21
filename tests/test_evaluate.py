"""Tests for tools/evaluate_pipeline.py on tiny synthetic data (no RadarScenes needed)."""
import json, subprocess, sys, tempfile
from pathlib import Path
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent

def run(log_rows):
    d = Path(tempfile.mkdtemp())
    n = 40
    raw = pd.DataFrame({"timestamp": [i * 100000 for i in range(n)], "sensor_id": 1,
                        "track_id": "objA", "label_id": 5})
    raw.to_csv(d / "raw.csv", index=False)
    pd.DataFrame(log_rows).to_csv(d / "log.csv", index=False)
    subprocess.run([sys.executable, str(ROOT / "tools/evaluate_pipeline.py"), "--raw", str(d / "raw.csv"),
                    "--log", str(d / "log.csv"), "--out-dir", str(d), "--quiet"], check=True)
    return json.load(open(d / "evaluation_summary.json"))

def rows(ids):
    return [{"Timestamp": i * 0.1, "Sensor_ID": 1, "Reference_ID": "objA", "Label_ID": 5, "Internal_ID": t, "X": 0.0, "Y": 0.0}
            for i, t in enumerate(ids)]

def test_perfect_tracking():
    s = run(rows([1] * 40))
    assert s["fused_idf1_percent"] > 99

def test_fragmentation_lowers_idf1():
    s = run(rows([1] * 10 + [2] * 10 + [3] * 10 + [4] * 10))
    assert s["fused_idf1_percent"] < 60 and s["fused_identity_switches"] >= 3

if __name__ == "__main__":
    test_perfect_tracking(); test_fragmentation_lowers_idf1(); print("evaluate tests: all passed")
