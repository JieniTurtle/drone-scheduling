"""通用 CSV 数值列均值统计工具。

功能：
- 自动识别 CSV 中的数值列
- 统计每一列的 count / mean / min / max / sum
- 统计所有数值单元格的整体均值
- 支持输出为文本、JSON 或汇总 CSV

用法示例：
    python csv_average.py --input results/compare/backend_wx_metrics.csv
    python csv_average.py --input backend_wx/pymarl-master/results/sacred/43/backend_wx_metrics.csv --format json
    python csv_average.py --input xxx.csv --output summary.csv
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import OrderedDict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


def _safe_float(value):
    try:
        if value is None:
            return None
        text = str(value).strip()
        if text == "":
            return None
        return float(text)
    except Exception:
        return None


def _read_rows(csv_path: Path) -> List[Dict[str, str]]:
    with open(csv_path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _collect_numeric_stats(rows: List[Dict[str, str]], exclude: Iterable[str]) -> Tuple[Dict[str, Dict[str, float]], float, int]:
    exclude_set = {str(item) for item in exclude}
    if not rows:
        return {}, 0.0, 0

    fieldnames = list(rows[0].keys())
    stats: Dict[str, Dict[str, float]] = OrderedDict()
    overall_sum = 0.0
    overall_count = 0

    for column in fieldnames:
        if column in exclude_set:
            continue

        values: List[float] = []
        for row in rows:
            number = _safe_float(row.get(column))
            if number is not None:
                values.append(number)

        if not values:
            continue

        column_sum = sum(values)
        column_count = len(values)
        stats[column] = {
            "count": float(column_count),
            "mean": column_sum / column_count,
            "min": min(values),
            "max": max(values),
            "sum": column_sum,
        }
        overall_sum += column_sum
        overall_count += column_count

    return stats, overall_sum, overall_count


def _format_text(stats: Dict[str, Dict[str, float]], overall_sum: float, overall_count: int) -> str:
    lines = []
    lines.append("=" * 72)
    lines.append("CSV Column Averages")
    lines.append("=" * 72)
    if not stats:
        lines.append("No numeric columns found.")
        lines.append("=" * 72)
        return "\n".join(lines)

    lines.append(f"{'column':<32} {'count':>8} {'mean':>14} {'min':>14} {'max':>14} {'sum':>14}")
    for column, metric in stats.items():
        lines.append(
            f"{column:<32} {int(metric['count']):>8} {metric['mean']:>14.6f} {metric['min']:>14.6f} "
            f"{metric['max']:>14.6f} {metric['sum']:>14.6f}"
        )

    overall_mean = overall_sum / overall_count if overall_count else 0.0
    lines.append("-" * 72)
    lines.append(f"overall_numeric_cells: {overall_count}")
    lines.append(f"overall_numeric_mean:  {overall_mean:.6f}")
    lines.append("=" * 72)
    return "\n".join(lines)


def _format_json(stats: Dict[str, Dict[str, float]], overall_sum: float, overall_count: int) -> str:
    overall_mean = overall_sum / overall_count if overall_count else 0.0
    payload = {
        "columns": stats,
        "overall": {
            "numeric_cells": overall_count,
            "mean": overall_mean,
            "sum": overall_sum,
        },
    }
    return json.dumps(payload, ensure_ascii=False, indent=2)


def _write_summary_csv(output_path: Path, stats: Dict[str, Dict[str, float]], overall_sum: float, overall_count: int) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["column", "count", "mean", "min", "max", "sum"])
        for column, metric in stats.items():
            writer.writerow([
                column,
                int(metric["count"]),
                metric["mean"],
                metric["min"],
                metric["max"],
                metric["sum"],
            ])
        overall_mean = overall_sum / overall_count if overall_count else 0.0
        writer.writerow(["__overall__", overall_count, overall_mean, "", "", overall_sum])


def parse_args():
    parser = argparse.ArgumentParser(description="对 CSV 中的数值列求平均")
    parser.add_argument("--input", required=True, help="输入 CSV 文件路径")
    parser.add_argument(
        "--exclude",
        type=str,
        default="",
        help="不参与统计的列名，逗号分隔，比如 source,mode,t_env",
    )
    parser.add_argument(
        "--format",
        choices=("text", "json"),
        default="text",
        help="输出格式：text 或 json",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="可选：将汇总结果保存为 CSV 文件",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    csv_path = Path(args.input).expanduser().resolve()
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV 文件不存在: {csv_path}")

    rows = _read_rows(csv_path)
    exclude = [item.strip() for item in args.exclude.split(",") if item.strip()]
    stats, overall_sum, overall_count = _collect_numeric_stats(rows, exclude)

    if args.output:
        _write_summary_csv(Path(args.output).expanduser().resolve(), stats, overall_sum, overall_count)

    if args.format == "json":
        print(_format_json(stats, overall_sum, overall_count))
    else:
        print(_format_text(stats, overall_sum, overall_count))


if __name__ == "__main__":
    main()