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


def main():
    project_root = Path(__file__).resolve().parent
    paths = _load_paths(project_root)

    print("=" * 72)
    print("Metrics Comparison")
    print("=" * 72)
    print("source, episodes, completion_rate_mean, on_time_rate_mean, avg_delay_mean")

    for source, path in paths.items():
        rows = _read_rows(path)
        print(
            f"{source}, {len(rows)}, {_mean(rows, 'completion_rate'):.4f}, "
            f"{_mean(rows, 'on_time_rate'):.4f}, {_mean(rows, 'avg_delay'):.4f}"
        )
        print(f"  file: {path}")

    print("=" * 72)


if __name__ == "__main__":
    main()
