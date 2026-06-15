"""PSO (backend_si) scheduler evaluation entrypoint.

与 frontend/greedy/run_greedy.py 对齐：跑多个 episode，对指标求均值，并把汇总
追加到 results/compare/backend_si_metrics.csv（中文 15 列、utf-8-sig）。

为了和 qmix（backend_wx）使用相同的任务场景，本脚本对第 ep 个 episode 使用
seed = base_seed + ep（base_seed 默认 100），与 qmix 的
episode_runner.py 中 `episode_seed = seed + episode_id`（seed=100）完全一致。

用法:
    cd frontend
    python run_pso.py --episodes 5            # 默认 base seed=100
    python run_pso.py --episodes 5 --seed 200 # 换一组场景
    python run_pso.py --no-csv                # 只打印不写 CSV
"""

import argparse
import csv
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FRONTEND_ROOT = PROJECT_ROOT / "frontend"

sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(FRONTEND_ROOT))

from config.config_loder import get_shared_config
from environment import Environment
from backend_si.pso_scheduler import PSOScheduler


CSV_OUTPUT = PROJECT_ROOT / "results" / "compare" / "backend_si_metrics.csv"

# 与 run_greedy.py 完全一致的中文 15 列，保证 greedy / pso CSV 列对齐
METRIC_COLUMNS = [
    "总步数",
    "完成任务数",
    "生成任务数",
    "完成率",
    "从生成到分配等待时间",
    "从分配到实际装载上机等待时间",
    "从上机到送达平均时间",
    "从生成到完成总时间平均",
    "从生成到完成总时间最大",
    "超时率",
    "平均时延",
    "优先级1平均时延",
    "优先级2平均时延",
    "优先级3平均时延",
    "总能量消耗",
]


def _resolve_episode_steps(default_steps):
    cfg = get_shared_config()
    env_cfg = cfg.get("environment", {}) if isinstance(cfg, dict) else {}
    return int(env_cfg.get("episode_max_steps", default_steps))


def _to_output_metrics(stats):
    return {
        "总步数": float(stats.get("episode_step", 0.0)),
        "完成任务数": float(stats.get("total_completed", 0.0)),
        "生成任务数": float(stats.get("total_generated", 0.0)),
        "完成率": float(stats.get("completion_rate", 0.0)),
        "从生成到分配等待时间": float(stats.get("avg_generation_to_assignment_wait", 0.0)),
        "从分配到实际装载上机等待时间": float(stats.get("avg_assignment_to_load_wait", 0.0)),
        "从上机到送达平均时间": float(stats.get("avg_load_to_delivery_time", 0.0)),
        "从生成到完成总时间平均": float(stats.get("avg_generation_to_completion_time", 0.0)),
        "从生成到完成总时间最大": float(stats.get("max_generation_to_completion_time", 0.0)),
        "超时率": float(stats.get("timeout_rate", 0.0)),
        "平均时延": float(stats.get("avg_delay", 0.0)),
        "优先级1平均时延": float(stats.get("avg_delay_priority_1", 0.0)),
        "优先级2平均时延": float(stats.get("avg_delay_priority_2", 0.0)),
        "优先级3平均时延": float(stats.get("avg_delay_priority_3", 0.0)),
        "总能量消耗": float(stats.get("total_energy_consumed", 0.0)),
    }


def _mean_metrics(rows):
    if not rows:
        return {col: 0.0 for col in METRIC_COLUMNS}
    return {
        col: sum(float(row.get(col, 0.0)) for row in rows) / len(rows)
        for col in METRIC_COLUMNS
    }


def run_one_episode(osm_path, episode_steps, seed):
    env = Environment(str(osm_path), visualize=False, episode_max_steps=episode_steps)
    # 用固定 seed 复现与 qmix 相同的任务场景
    obs = env.reset(seed=seed)
    # PSOScheduler 持有队列/已知任务等内部状态，必须每个 episode 新建
    scheduler = PSOScheduler(num_drones=len(env.drones), verbose=False, seed=seed)
    done = False
    while not done:
        action = scheduler.step(obs, current_time=env.current_time)
        obs, _, done, _ = env.step(action)

    stats = env.get_statistics()
    stats["episode_step"] = int(env.current_time)
    stats["_scheduler_stats"] = scheduler.get_stats()
    return stats


def _write_csv(output_path, stats):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = ",".join(METRIC_COLUMNS)
    existing_header = None
    if output_path.exists():
        with open(output_path, "r", encoding="utf-8-sig", newline="") as f:
            existing_header = f.readline().strip()

    write_mode = "a" if existing_header == header else "w"
    with open(output_path, write_mode, newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        if write_mode == "w":
            writer.writerow(METRIC_COLUMNS)
        writer.writerow([stats[col] for col in METRIC_COLUMNS])


def main():
    parser = argparse.ArgumentParser(description="Run PSO (backend_si) scheduler evaluation.")
    parser.add_argument("--episodes", type=int, default=1, help="Number of episodes to run.")
    parser.add_argument("--episode-steps", type=int, default=None, help="Max steps per episode.")
    parser.add_argument("--osm", type=str, default="data/map/part_of_yangpu.osm", help="OSM path relative to frontend.")
    parser.add_argument("--seed", type=int, default=100, help="Base seed; episode ep uses seed+ep (aligns with qmix, default 100).")
    parser.add_argument("--no-csv", action="store_true", help="Do not write the averaged CSV summary.")
    args = parser.parse_args()

    episode_steps = args.episode_steps or _resolve_episode_steps(default_steps=1200)
    osm_path = FRONTEND_ROOT / args.osm

    output_rows = []
    for ep in range(args.episodes):
        episode_seed = args.seed + ep + 1
        stats = run_one_episode(osm_path, episode_steps, episode_seed)
        output = _to_output_metrics(stats)
        output_rows.append(output)
        print(
            f"Episode {ep + 1} (seed={episode_seed}): "
            f"完成率={output['完成率']:.4f}, "
            f"超时率={output['超时率']:.4f}, "
            f"平均时延={output['平均时延']:.4f}, "
            f"从上机到送达平均时间={output['从上机到送达平均时间']:.4f}, "
            f"完成={output['完成任务数']:.0f}/{output['生成任务数']:.0f}, "
            f"总步数={output['总步数']:.0f}"
        )
        sched_stats = stats.get("_scheduler_stats")
        if sched_stats:
            print(f"    [scheduler] {sched_stats}")

    mean_stats = _mean_metrics(output_rows)

    print("=" * 60)
    print("Mean Metrics")
    for col in METRIC_COLUMNS:
        print(f"{col}: {mean_stats[col]:.4f}")
    if not args.no_csv:
        _write_csv(CSV_OUTPUT, mean_stats)
        print(f"csv_file: {CSV_OUTPUT}")
    print("=" * 60)


if __name__ == "__main__":
    main()
