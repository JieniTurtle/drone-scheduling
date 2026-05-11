import argparse
import csv
import json
from pathlib import Path


def _safe_float(v, default=None):
    try:
        if v is None or v == "":
            return default
        return float(v)
    except Exception:
        return default


def _load_metrics_csv(csv_path):
    if not csv_path.exists():
        return []
    with open(csv_path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _load_info_json(info_path):
    if not info_path.exists():
        return {}
    with open(info_path, "r", encoding="utf-8") as f:
        return json.load(f)


def _split_mode_rows(rows):
    train_rows = []
    test_rows = []
    for r in rows:
        mode = str(r.get("mode", "")).strip().lower()
        if mode == "test":
            test_rows.append(r)
        else:
            train_rows.append(r)
    return train_rows, test_rows


def _extract_xy(rows, x_key, y_key):
    xs, ys = [], []
    for r in rows:
        x = _safe_float(r.get(x_key), default=None)
        y = _safe_float(r.get(y_key), default=None)
        if x is None:
            x = float(len(xs) + 1)
        if y is None:
            continue
        xs.append(x)
        ys.append(y)
    return xs, ys


def _find_best_test_row(test_rows):
    if not test_rows:
        return None

    def sort_key(r):
        c = _safe_float(r.get("completion_rate"), default=0.0)
        o = _safe_float(r.get("on_time_rate"), default=0.0)
        d = _safe_float(r.get("avg_delay"), default=1e18)
        return (c, o, -d)

    return max(test_rows, key=sort_key)


def _plot_metrics(rows, out_dir):
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        raise RuntimeError(
            "matplotlib 不可用，请先安装：pip install matplotlib"
        ) from e

    train_rows, test_rows = _split_mode_rows(rows)

    metrics = ["completion_rate", "on_time_rate", "avg_delay", "total_completed"]
    for metric in metrics:
        fig, ax = plt.subplots(figsize=(9, 5))
        tx, ty = _extract_xy(train_rows, "t_env", metric)
        vx, vy = _extract_xy(test_rows, "t_env", metric)

        if ty:
            ax.plot(tx, ty, marker="o", linewidth=1.6, label="train")
        if vy:
            ax.plot(vx, vy, marker="s", linewidth=1.8, label="test")

        ax.set_title(f"backend_wx {metric} vs t_env")
        ax.set_xlabel("t_env")
        ax.set_ylabel(metric)
        ax.grid(alpha=0.25)
        if ty or vy:
            ax.legend()

        fig.tight_layout()
        fig.savefig(out_dir / f"metrics_{metric}.png", dpi=160)
        plt.close(fig)


def _plot_info_curves(info_data, out_dir):
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        raise RuntimeError(
            "matplotlib 不可用，请先安装：pip install matplotlib"
        ) from e

    plotted = 0
    for key, vals in info_data.items():
        if key.endswith("_T"):
            continue
        t_key = f"{key}_T"
        if t_key not in info_data:
            continue
        if not isinstance(vals, list) or not isinstance(info_data[t_key], list):
            continue
        if len(vals) == 0 or len(vals) != len(info_data[t_key]):
            continue

        xs = [_safe_float(x, None) for x in info_data[t_key]]
        ys = [_safe_float(y, None) for y in vals]
        pairs = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
        if len(pairs) < 2:
            continue

        x2 = [p[0] for p in pairs]
        y2 = [p[1] for p in pairs]

        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot(x2, y2, linewidth=1.6)
        ax.set_title(f"info.json: {key}")
        ax.set_xlabel("t_env")
        ax.set_ylabel(key)
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(out_dir / f"info_{key}.png", dpi=160)
        plt.close(fig)
        plotted += 1

    return plotted


def _write_summary(run_dir, rows, info_data, out_dir):
    train_rows, test_rows = _split_mode_rows(rows)
    best_test = _find_best_test_row(test_rows)

    summary = {
        "run_dir": str(run_dir),
        "metrics_rows": len(rows),
        "train_rows": len(train_rows),
        "test_rows": len(test_rows),
        "best_test": best_test,
        "info_keys": sorted(list(info_data.keys()))[:200],
    }

    with open(out_dir / "analysis_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)


def analyze(run_dir):
    run_dir = Path(run_dir).resolve()
    csv_path = run_dir / "backend_wx_metrics.csv"
    info_path = run_dir / "info.json"
    out_dir = run_dir / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = _load_metrics_csv(csv_path)
    info_data = _load_info_json(info_path)

    if rows:
        _plot_metrics(rows, out_dir)
    if info_data:
        _plot_info_curves(info_data, out_dir)

    _write_summary(run_dir, rows, info_data, out_dir)

    print("=" * 72)
    print(f"Run Dir: {run_dir}")
    print(f"Metrics CSV exists: {csv_path.exists()}, rows: {len(rows)}")
    print(f"Info JSON exists: {info_path.exists()}, keys: {len(info_data.keys()) if isinstance(info_data, dict) else 0}")
    print(f"Output charts dir: {out_dir}")
    print("Generated files:")
    for p in sorted(out_dir.glob("*.png")):
        print(f"  - {p.name}")
    print("  - analysis_summary.json")
    print("=" * 72)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze one Sacred run folder that contains backend_wx_metrics.csv and info.json"
    )
    parser.add_argument(
        "--run-dir",
        type=str,
        default="results/sacred/12",
        help="Run folder path, e.g. results/sacred/12",
    )
    args = parser.parse_args()
    analyze(args.run_dir)


if __name__ == "__main__":
    main()
