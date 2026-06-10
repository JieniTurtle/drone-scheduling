from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
COMPARE_DIR = ROOT / "results" / "compare"
OUT_DIR = COMPARE_DIR / "plots"

CSV_FILES = [
    "frontend_greedy_metrics.csv",
    "backend_si_metrics.csv",
    "backend_wx_metrics.csv",
]

ALGORITHM_ORDER = ["greedy", "pso", "iql", "iql_u", "vdn", "vdn_u", "qmix", "qmix_u"]

COLUMN_ALIASES = {
    "算法": "Algorithm",
    "总步数": "Total Steps",
    "完成任务数": "Completed Tasks",
    "生成任务数": "Generated Tasks",
    "完成率": "Completion Rate",
    "从生成到分配等待时间": "Generation-to-Assignment Wait",
    "从分配到实际装载上机等待时间": "Assignment-to-Loading Wait",
    "从上机到送达平均时间": "Loading-to-Delivery Time",
    "从生成到完成总时间平均": "Avg Generation-to-Completion Time",
    "从生成到完成总时间平均值": "Avg Generation-to-Completion Time",
    "从生成到完成总时间最大": "Max Generation-to-Completion Time",
    "从生成到完成总时间最大值": "Max Generation-to-Completion Time",
    "超时率": "Timeout Rate",
    "平均时延": "Average Delay",
    "优先级1平均时延": "Priority-1 Average Delay",
    "优先级2平均时延": "Priority-2 Average Delay",
    "优先级3平均时延": "Priority-3 Average Delay",
    "总能量消耗": "Total Energy Consumption",
}

BAR_METRICS = [
    "Completion Rate",
    "Generation-to-Assignment Wait",
    "Avg Generation-to-Completion Time",
    "Max Generation-to-Completion Time",
    "Timeout Rate",
    "Average Delay",
    "Total Energy Consumption",
]

PRIORITY_DELAY_METRICS = [
    "Priority-1 Average Delay",
    "Priority-2 Average Delay",
    "Priority-3 Average Delay",
]

REQUIRED_METRICS = BAR_METRICS + PRIORITY_DELAY_METRICS

HIGHER_IS_BETTER = {"Completion Rate"}

TRUNCATED_Y_METRICS = {
    "Avg Generation-to-Completion Time",
    "Total Energy Consumption",
}

PRIORITY_DELAY_WEIGHTS = np.array([1.0, 1.5, 2.2])
OVERALL_SCORE_WEIGHTS = {
    "Completion Rate": 0.18,
    "Timeout Rate": 0.22,
    "Timeout Duration": 0.15,
    "Priority Timeout Duration": 0.15,
    "Avg Completion Time": 0.14,
    "Energy Consumption": 0.16,
}

COLORS = {
    "greedy": "#4E79A7",
    "pso": "#F28E2B",
    "iql": "#E15759",
    "iql_u": "#E15759",
    "vdn": "#59A14F",
    "vdn_u": "#59A14F",
    "qmix": "#4C6FBF",
    "qmix_u": "#4C6FBF",
}

DISPLAY_NAMES = {
    "greedy": "Greedy",
    "pso": "PSO",
    "iql": "IQL",
    "iql_u": "IQL-U",
    "vdn": "VDN",
    "vdn_u": "VDN-U",
    "qmix": "QMIX",
    "qmix_u": "QMIX-U",
}


def configure_matplotlib():
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "axes.unicode_minus": False,
        "figure.dpi": 140,
        "savefig.dpi": 300,
        "font.size": 10,
        "axes.titlesize": 13,
        "axes.labelsize": 11,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "axes.linewidth": 0.8,
        "grid.linewidth": 0.5,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })


def normalize_columns(df):
    renamed = df.rename(columns={col: COLUMN_ALIASES.get(col, col) for col in df.columns})
    if "Algorithm" not in renamed.columns:
        raise KeyError("Missing algorithm column. Expected column: 算法")
    return renamed


def load_compare_data():
    frames = []
    for name in CSV_FILES:
        path = COMPARE_DIR / name
        if path.exists():
            frames.append(normalize_columns(pd.read_csv(path, encoding="utf-8-sig")))
    if not frames:
        raise FileNotFoundError(f"No compare CSV files found in {COMPARE_DIR}")

    data = pd.concat(frames, ignore_index=True)
    data["Algorithm"] = data["Algorithm"].astype(str).str.strip()
    data = data[data["Algorithm"].isin(ALGORITHM_ORDER)].copy()

    missing = [metric for metric in REQUIRED_METRICS if metric not in data.columns]
    if missing:
        raise KeyError(f"Missing metric columns after alias normalization: {missing}")

    for metric in REQUIRED_METRICS:
        data[metric] = pd.to_numeric(data[metric], errors="coerce")

    data["_row_order"] = np.arange(len(data))
    data = data.sort_values("_row_order").drop_duplicates("Algorithm", keep="last")
    data = data.set_index("Algorithm").reindex(ALGORITHM_ORDER).dropna(how="all").reset_index()
    data["Display Name"] = data["Algorithm"].map(DISPLAY_NAMES)
    return data


def ability_scores(data, metrics):
    scores = data[["Algorithm", "Display Name"]].copy()
    for metric in metrics:
        values = data[metric].astype(float)
        min_v = values.min()
        max_v = values.max()
        if np.isclose(max_v, min_v):
            scores[metric] = 1.0
        elif metric in HIGHER_IS_BETTER:
            scores[metric] = (values - min_v) / (max_v - min_v)
        else:
            scores[metric] = (max_v - values) / (max_v - min_v)
    return scores


def priority_delay_score(data):
    priority_matrix = data[PRIORITY_DELAY_METRICS].astype(float).to_numpy()
    return (priority_matrix @ PRIORITY_DELAY_WEIGHTS) / PRIORITY_DELAY_WEIGHTS.sum()


def normalized_overall_scores(data):
    profile = data[["Algorithm", "Display Name"]].copy()
    values_by_name = {
        "Completion Rate": data["Completion Rate"].astype(float).to_numpy(),
        "Timeout Rate": data["Timeout Rate"].astype(float).to_numpy(),
        "Timeout Duration": data["Average Delay"].astype(float).to_numpy(),
        "Priority Timeout Duration": priority_delay_score(data),
        "Avg Completion Time": data["Avg Generation-to-Completion Time"].astype(float).to_numpy(),
        "Energy Consumption": data["Total Energy Consumption"].astype(float).to_numpy(),
    }

    for display_metric, values in values_by_name.items():
        values = np.asarray(values, dtype=float)
        min_v = np.nanmin(values)
        max_v = np.nanmax(values)
        if np.isclose(max_v, min_v):
            profile[display_metric] = 1.0
        elif display_metric == "Completion Rate":
            profile[display_metric] = values
        else:
            profile[display_metric] = (max_v - values) / (max_v - min_v)
    return profile


def weighted_overall_score(data):
    profiles = normalized_overall_scores(data)
    score = np.zeros(len(profiles), dtype=float)
    for metric, weight in OVERALL_SCORE_WEIGHTS.items():
        score += weight * profiles[metric].astype(float).to_numpy()
    out = profiles[["Algorithm", "Display Name"]].copy()
    out["Weighted Overall Score"] = score
    return out


def style_axes(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color("#333333")
    ax.spines["bottom"].set_color("#333333")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.set_axisbelow(True)


def metric_label(value, metric):
    if metric in {"Completion Rate", "Timeout Rate"}:
        return f"{value:.3f}"
    if abs(value) >= 1000:
        return f"{value:.0f}"
    return f"{value:.1f}"


def file_slug(metric):
    return (
        metric.lower()
        .replace(" ", "_")
        .replace("-", "_")
        .replace("/", "_")
    )


def save_bar_charts(data):
    labels = data["Display Name"].tolist()
    x = np.arange(len(labels))
    colors = [COLORS.get(alg, "#777777") for alg in data["Algorithm"]]

    for metric in BAR_METRICS:
        fig, ax = plt.subplots(figsize=(8.8, 4.8))
        values = data[metric].astype(float).to_numpy()
        bars = ax.bar(
            x,
            values,
            color=colors,
            edgecolor="#222222",
            linewidth=0.6,
            width=0.68,
        )
        ax.set_title(metric)
        ax.set_xlabel("Algorithm")
        ax.set_ylabel(metric)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=25, ha="right")
        style_axes(ax)

        if metric in TRUNCATED_Y_METRICS:
            ymin = np.nanmin(values)
            ymax = np.nanmax(values)
            span = max(ymax - ymin, 1.0)
            ax.set_ylim(max(0.0, ymin - 0.18 * span), ymax + 0.18 * span)
            ax.text(
                0.01,
                0.97,
                "Truncated y-axis",
                transform=ax.transAxes,
                ha="left",
                va="top",
                fontsize=8,
                color="#555555",
            )

        ymax = np.nanmax(values)
        offset = ymax * 0.012 if ymax > 0 else 0.02
        for bar, value in zip(bars, values):
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + offset,
                metric_label(value, metric),
                ha="center",
                va="bottom",
                fontsize=7.5,
            )

        fig.tight_layout()
        fig.savefig(OUT_DIR / f"bar_{file_slug(metric)}.png", bbox_inches="tight")
        plt.close(fig)


def save_bar_overview(data):
    labels = data["Display Name"].tolist()
    x = np.arange(len(labels))
    colors = [COLORS.get(alg, "#777777") for alg in data["Algorithm"]]

    ncols = 4
    nrows = int(np.ceil(len(BAR_METRICS) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 3.4 * nrows))
    axes = np.atleast_1d(axes).flat
    for ax, metric in zip(axes, BAR_METRICS):
        values = data[metric].astype(float).to_numpy()
        ax.bar(x, values, color=colors, edgecolor="#222222", linewidth=0.4, width=0.68)
        ax.set_title(metric, fontsize=10)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=7.5)
        style_axes(ax)
    for ax in list(axes)[len(BAR_METRICS):]:
        ax.axis("off")

    fig.suptitle("Comparison of Key Metrics Across Algorithms", fontsize=15, y=1.01)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "bar_all_metrics_overview.png", bbox_inches="tight")
    plt.close(fig)


def save_priority_delay_grouped_bar(data):
    labels = data["Display Name"].tolist()
    group_gap = 0.9
    bar_width = 0.24
    group_x = np.arange(len(labels)) * (len(PRIORITY_DELAY_METRICS) * bar_width + group_gap)
    offsets = np.array([-bar_width, 0.0, bar_width])
    priority_colors = ["#6BAED6", "#FD8D3C", "#74C476"]
    priority_labels = ["Priority 1", "Priority 2", "Priority 3"]

    fig, ax = plt.subplots(figsize=(10.8, 5.8))
    priority_values = []
    for idx, metric in enumerate(PRIORITY_DELAY_METRICS):
        values = data[metric].astype(float).to_numpy()
        priority_values.append(values)
        bars = ax.bar(
            group_x + offsets[idx],
            values,
            width=bar_width,
            label=priority_labels[idx],
            color=priority_colors[idx],
            edgecolor="#222222",
            linewidth=0.5,
        )

    weighted_delay = priority_delay_score(data)
    ax.plot(
        group_x,
        weighted_delay,
        color="#6A3D9A",
        marker="D",
        markersize=5.0,
        linewidth=2.1,
        label="Weighted Priority Delay Score",
        zorder=5,
    )

    ax.set_title("Priority-wise Delay and Weighted Score", pad=12)
    ax.set_xlabel("Algorithm")
    ax.set_ylabel("Average Delay")
    ax.set_xticks(group_x)
    ax.set_xticklabels(labels, rotation=25, ha="right")
    ax.legend(frameon=False, ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.18))
    style_axes(ax)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "bar_priority_delay_grouped.png", bbox_inches="tight")
    plt.close(fig)


def save_radar_chart(data):
    metrics = BAR_METRICS
    scores = ability_scores(data, metrics)
    angles = np.linspace(0, 2 * np.pi, len(metrics), endpoint=False).tolist()
    angles += angles[:1]

    fig, ax = plt.subplots(figsize=(9.2, 9.2), subplot_kw={"polar": True})
    for _, row in scores.iterrows():
        alg = row["Algorithm"]
        values = [float(row[m]) for m in metrics]
        values += values[:1]
        ax.plot(
            angles,
            values,
            label=row["Display Name"],
            color=COLORS.get(alg, "#777777"),
            linestyle="--" if alg.endswith("_u") else "-",
            linewidth=1.9,
        )

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(metrics, fontsize=8)
    ax.set_ylim(0, 1)
    ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_yticklabels(["0.2", "0.4", "0.6", "0.8", "1.0"], fontsize=8)
    ax.grid(alpha=0.35)
    ax.set_title("Normalized Capability Radar Chart", fontsize=14, pad=22)
    ax.legend(loc="upper right", bbox_to_anchor=(1.28, 1.10), frameon=False)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "radar_algorithm_capability.png", bbox_inches="tight")
    plt.close(fig)


def save_weighted_overall_score(data):
    scores = weighted_overall_score(data)
    scores = scores.set_index("Algorithm").reindex(ALGORITHM_ORDER).reset_index()
    labels = scores["Display Name"].tolist()
    values = scores["Weighted Overall Score"].astype(float).to_numpy()
    colors = [COLORS.get(alg, "#777777") for alg in scores["Algorithm"]]

    fig, ax = plt.subplots(figsize=(9.0, 4.8))
    bars = ax.bar(
        np.arange(len(labels)),
        values,
        color=colors,
        edgecolor="#222222",
        linewidth=0.6,
        width=0.68,
    )
    ax.set_title("Weighted Overall Score Across Algorithms")
    ax.set_xlabel("Algorithm")
    ax.set_ylabel("Weighted overall score (higher is better)")
    ax.set_xticks(np.arange(len(labels)))
    ax.set_xticklabels(labels, rotation=25, ha="right")
    ax.set_ylim(0, max(1.0, np.nanmax(values) + 0.08))
    style_axes(ax)
    for bar, value in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(np.nanmax(values) * 0.012, 0.01),
            f"{value:.3f}",
            ha="center",
            va="bottom",
            fontsize=8,
        )
    fig.tight_layout()
    fig.savefig(OUT_DIR / "bar_weighted_overall_score.png", bbox_inches="tight")
    plt.close(fig)


def save_processed_table(data):
    data.drop(columns=["_row_order"], errors="ignore").to_csv(
        OUT_DIR / "combined_compare_metrics.csv",
        index=False,
        encoding="utf-8-sig",
    )
    ability_scores(data, BAR_METRICS).to_csv(
        OUT_DIR / "normalized_capability_scores.csv",
        index=False,
        encoding="utf-8-sig",
    )
    weighted_overall_score(data).to_csv(
        OUT_DIR / "weighted_overall_score.csv",
        index=False,
        encoding="utf-8-sig",
    )


def main():
    configure_matplotlib()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    data = load_compare_data()
    save_processed_table(data)
    save_bar_charts(data)
    save_bar_overview(data)
    save_priority_delay_grouped_bar(data)
    save_weighted_overall_score(data)
    print(f"Saved plots to {OUT_DIR}")
    print("Algorithms:", ", ".join(data["Algorithm"].tolist()))


if __name__ == "__main__":
    main()
