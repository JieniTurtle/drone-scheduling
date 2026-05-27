"""纯贪心算法测试入口。

用法:
    cd frontend
    python greedy/run_greedy.py --episodes 5 --episode-steps 1200 --csv
"""

import argparse
import csv
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FRONTEND_ROOT = PROJECT_ROOT / "frontend"

sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(FRONTEND_ROOT))

from environment import Environment
from greedy.scheduler import greedy_action_from_observation
from config.config_loder import get_shared_config


CSV_OUTPUT = PROJECT_ROOT / "results" / "compare" / "frontend_greedy_metrics.csv"


def _resolve_episode_steps(default_steps):
    cfg = get_shared_config()
    env_cfg = cfg.get("environment", {}) if isinstance(cfg, dict) else {}
    return int(env_cfg.get("episode_max_steps", default_steps))


def run_one_episode(osm_path, episode_steps):
    """运行一个贪心调度 episode，返回完整统计数据。"""
    env = Environment(str(osm_path), visualize=False, episode_max_steps=episode_steps)
    obs = env.reset()
    done = False
    while not done:
        action = greedy_action_from_observation(obs)
        obs, _, done, _ = env.step(action)

    stats = env.get_statistics()
    total_generated = int(getattr(env, "total_generated_tasks", 0))
    result = {
        "completion_rate": float(stats.get("completion_rate", 0.0)),
        "on_time_rate": float(stats.get("on_time_rate", 0.0)),
        "avg_delay": float(stats.get("avg_delay", 0.0)),
        "avg_delivery_time": float(stats.get("avg_delivery_time", 0.0)),
        "total_completed": int(stats.get("total_completed", 0)),
        "total_generated": total_generated,
        "episode_steps": int(env.current_time),
    }
    if total_generated > 0:
        result["order_generate_rate"] = result["episode_steps"] / total_generated
    else:
        result["order_generate_rate"] = 0.0
    return result


def _write_csv(output_path, episode, stats):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    need_header = not output_path.exists()
    with open(output_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if need_header:
            writer.writerow([
                "source", "episode", "completion_rate", "on_time_rate",
                "avg_delay", "avg_delivery_time", "total_completed",
                "total_generated", "order_generate_rate", "episode_steps",
            ])
        writer.writerow([
            "frontend_greedy",
            int(episode),
            float(stats["completion_rate"]),
            float(stats["on_time_rate"]),
            float(stats["avg_delay"]),
            float(stats["avg_delivery_time"]),
            int(stats["total_completed"]),
            int(stats["total_generated"]),
            float(stats["order_generate_rate"]),
            int(stats["episode_steps"]),
        ])


def main():
    parser = argparse.ArgumentParser(description="纯贪心算法测试入口")
    parser.add_argument("--episodes", type=int, default=5, help="运行 episode 数量")
    parser.add_argument("--episode-steps", type=int, default=None, help="每个 episode 最大步数")
    parser.add_argument("--osm", type=str, default="data/map/part_of_yangpu.osm", help="OSM 地图相对路径 (frontend 目录下)")
    parser.add_argument("--csv", action="store_true", default=False, help="是否输出 CSV 文件")
    args = parser.parse_args()

    episode_steps = args.episode_steps or _resolve_episode_steps(default_steps=1200)
    osm_path = FRONTEND_ROOT / args.osm

    all_stats = []
    for ep in range(args.episodes):
        stats = run_one_episode(osm_path, episode_steps)
        all_stats.append(stats)
        if args.csv:
            _write_csv(CSV_OUTPUT, ep + 1, stats)
        print(
            f"Episode {ep + 1}: "
            f"completion={stats['completion_rate']:.4f}, "
            f"on_time={stats['on_time_rate']:.4f}, "
            f"avg_delay={stats['avg_delay']:.4f}, "
            f"avg_delivery={stats['avg_delivery_time']:.4f}, "
            f"completed={stats['total_completed']}/{stats['total_generated']}, "
            f"generate_rate={stats['order_generate_rate']:.4f}, "
            f"steps={stats['episode_steps']}"
        )

    n = max(len(all_stats), 1)
    mean_completion = sum(s["completion_rate"] for s in all_stats) / n
    mean_on_time = sum(s["on_time_rate"] for s in all_stats) / n
    mean_delay = sum(s["avg_delay"] for s in all_stats) / n
    mean_delivery = sum(s["avg_delivery_time"] for s in all_stats) / n
    mean_generate_rate = sum(s["order_generate_rate"] for s in all_stats) / n

    print("=" * 60)
    print("Mean Metrics")
    print(f"completion_rate:       {mean_completion:.4f}")
    print(f"on_time_rate:          {mean_on_time:.4f}")
    print(f"avg_delay:             {mean_delay:.4f}")
    print(f"avg_delivery_time:     {mean_delivery:.4f}")
    print(f"order_generate_rate:   {mean_generate_rate:.4f}")
    if args.csv:
        print(f"csv_file:              {CSV_OUTPUT}")
    print("=" * 60)


if __name__ == "__main__":
    main()
