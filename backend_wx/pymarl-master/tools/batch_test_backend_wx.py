import csv
import subprocess
import sys
import time
from pathlib import Path


def _list_sacred_runs(sacred_root):
    if not sacred_root.exists():
        return set()
    return {p.name for p in sacred_root.iterdir() if p.is_dir() and p.name.isdigit()}


def _pick_sacred_run(sacred_root, before_runs):
    after_runs = _list_sacred_runs(sacred_root)
    new_runs = sorted(after_runs - before_runs)
    if new_runs:
        return new_runs[-1]
    if not after_runs:
        return None
    return max(
        (sacred_root / run_id for run_id in after_runs),
        key=lambda p: p.stat().st_mtime,
    ).name


def _wait_for_file(path, timeout_s=30):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if path.exists():
            return True
        time.sleep(0.2)
    return path.exists()


def _load_metrics_rows(csv_path):
    if not csv_path.exists():
        return [], []
    with open(csv_path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows, reader.fieldnames or []


def _filter_test_rows(rows):
    test_rows = [row for row in rows if row.get("mode") == "test"]
    return test_rows if test_rows else rows


def _mean(values):
    if not values:
        return None
    return sum(values) / len(values)


def _average_metrics(rows, columns):
    if not rows:
        return {}
    ignore_cols = {"source", "episode", "t_env", "mode"}
    metrics = {}
    for col in columns:
        if col in ignore_cols:
            continue
        numeric_vals = []
        for row in rows:
            val = row.get(col, "")
            if val == "":
                continue
            try:
                numeric_vals.append(float(val))
            except ValueError:
                numeric_vals = []
                break
        if numeric_vals:
            metrics[col] = _mean(numeric_vals)
    return metrics


def main():
    pymarl_root = Path(__file__).resolve().parents[1]
    workspace_root = Path(__file__).resolve().parents[3]
    sacred_root = pymarl_root / "results" / "sacred"
    compare_csv = workspace_root / "results" / "compare" / "backend_wx_metrics.csv"

    # ---- Batch test configuration ----
    algs = ["qmix", "iql", "vdn"]
    unique_flags = [False, True]

    test_nepisode = 4
    batch_size_run = 1
    use_cuda = False
    save_model = False

    # Per-combination checkpoint path (relative to pymarl_root).
    # Keys are (alg, unique_flag).
    checkpoints = {
        ("qmix", False): "results/models/qmix__2026-05-10_20-33-19",
        ("qmix", True): "",
        ("iql", False): "",
        ("iql", True): "",
        ("vdn", False): "",
        ("vdn", True): "",
    }

    label_template = "{alg}_unique_{unique}_test{test_nepisode}"

    results = []
    all_metric_keys = set()

    for alg in algs:
        for unique in unique_flags:
            checkpoint_path = checkpoints.get((alg, bool(unique)), "")
            if not checkpoint_path:
                print(f"Skipping {alg} unique={int(unique)}: checkpoint_path not set.")
                continue
            label = label_template.format(
                alg=alg,
                unique=int(unique),
                test_nepisode=test_nepisode,
            )
            before_runs = _list_sacred_runs(sacred_root)
            cmd = [
                sys.executable,
                str(pymarl_root / "src" / "main.py"),
                f"--config={alg}",
                "--env-config=env_drone",
                "with",
                f"unique_task_assignment={str(bool(unique))}",
                "evaluate=True",
                f"checkpoint_path={checkpoint_path}",
                f"test_nepisode={test_nepisode}",
                f"batch_size_run={batch_size_run}",
                f"use_cuda={str(bool(use_cuda))}",
                f"save_model={str(bool(save_model))}",
                f"label={label}",
            ]
            print("Running:", " ".join(cmd))
            subprocess.run(cmd, cwd=str(pymarl_root), check=True)

            sacred_run_id = _pick_sacred_run(sacred_root, before_runs)
            if not sacred_run_id:
                print("Warning: sacred run directory not found; skip metrics.")
                continue

            metrics_csv = sacred_root / sacred_run_id / "backend_wx_metrics.csv"
            if not _wait_for_file(metrics_csv, timeout_s=30):
                print(f"Warning: metrics CSV not found at {metrics_csv}; skip.")
                continue

            rows, columns = _load_metrics_rows(metrics_csv)
            rows = _filter_test_rows(rows)
            metric_means = _average_metrics(rows, columns)
            all_metric_keys.update(metric_means.keys())

            results.append({
                "alg": alg,
                "unique_task_assignment": int(unique),
                "checkpoint_path": checkpoint_path,
                "test_nepisode": test_nepisode,
                "batch_size_run": batch_size_run,
                "use_cuda": int(use_cuda),
                "save_model": int(save_model),
                "label": label,
                "sacred_run_id": sacred_run_id,
                **{f"avg_{k}": v for k, v in metric_means.items()},
            })

    compare_csv.parent.mkdir(parents=True, exist_ok=True)
    metric_columns = [f"avg_{k}" for k in sorted(all_metric_keys)]
    base_columns = [
        "alg",
        "unique_task_assignment",
        "checkpoint_path",
        "test_nepisode",
        "batch_size_run",
        "use_cuda",
        "save_model",
        "label",
        "sacred_run_id",
    ]
    with open(compare_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=base_columns + metric_columns)
        writer.writeheader()
        for row in results:
            writer.writerow(row)

    print(f"Wrote summary to: {compare_csv}")


if __name__ == "__main__":
    main()
