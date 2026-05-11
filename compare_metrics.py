import csv
import json
from pathlib import Path


def _load_paths(project_root):
    cfg_path = project_root / "config" / "simulation.json"
    compare_dir = project_root / "results" / "compare"
    files = {
        "frontend_greedy": "frontend_greedy_metrics.csv",
        "backend_si": "backend_si_metrics.csv",
        "backend_wx": "backend_wx_metrics.csv",
    }

    if cfg_path.exists():
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        metrics_cfg = cfg.get("metrics", {})
        compare_dir = project_root / metrics_cfg.get("compare_dir", "results/compare")
        files["frontend_greedy"] = metrics_cfg.get("frontend_greedy_file", files["frontend_greedy"])
        files["backend_si"] = metrics_cfg.get("backend_si_file", files["backend_si"])
        files["backend_wx"] = metrics_cfg.get("backend_wx_file", files["backend_wx"])

    return {k: compare_dir / v for k, v in files.items()}


def _read_rows(path):
    if not path.exists():
        return []
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _mean(rows, key):
    vals = [float(r[key]) for r in rows if key in r and r[key] != ""]
    if not vals:
        return 0.0
    return sum(vals) / len(vals)


def _to_float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except Exception:
        return float(default)


def _best_test_row(source, rows):
    if not rows:
        return None

    # backend_wx has explicit train/test split; others are evaluation rows and treated as test.
    if source == "backend_wx":
        test_rows = [r for r in rows if str(r.get("mode", "")).lower() == "test"]
        if not test_rows:
            return None
    else:
        test_rows = rows

    # Higher completion_rate is better, then higher on_time_rate, then lower avg_delay.
    return max(
        test_rows,
        key=lambda r: (
            _to_float(r, "completion_rate"),
            _to_float(r, "on_time_rate"),
            -_to_float(r, "avg_delay"),
        ),
    )


def main():
    project_root = Path(__file__).resolve().parent
    paths = _load_paths(project_root)

    print("=" * 72)
    print("Metrics Comparison")
    print("=" * 72)
    print("source, test_rows, best_completion_rate, best_on_time_rate, best_avg_delay")

    for source, path in paths.items():
        rows = _read_rows(path)
        best = _best_test_row(source, rows)
        if best is None:
            print(f"{source}, 0, 0.0000, 0.0000, 0.0000")
            print(f"  file: {path}")
            continue

        if source == "backend_wx":
            test_rows = [r for r in rows if str(r.get("mode", "")).lower() == "test"]
        else:
            test_rows = rows

        print(
            f"{source}, {len(test_rows)}, {_to_float(best, 'completion_rate'):.4f}, "
            f"{_to_float(best, 'on_time_rate'):.4f}, {_to_float(best, 'avg_delay'):.4f}"
        )
        extra = []
        for k in ("episode", "t_env", "mode"):
            if k in best and str(best.get(k)) != "":
                extra.append(f"{k}={best.get(k)}")
        if extra:
            print(f"  best_row: {', '.join(extra)}")
        print(f"  file: {path}")

    print("=" * 72)


if __name__ == "__main__":
    main()
