import argparse
import os
import sys
import csv
import json
from pathlib import Path

# Allow importing backend scheduler from project root.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT))

from environment import Environment
from backend_si.pso_scheduler import PSOScheduler
from greedy.scheduler import greedy_action_from_observation


def _get_metrics_output(policy):
    cfg_path = PROJECT_ROOT / "config" / "simulation.json"
    compare_dir = PROJECT_ROOT / "results" / "compare"
    filename_map = {
        "greedy": "frontend_greedy_metrics.csv",
        "pso": "backend_si_metrics.csv",
    }
    filename = filename_map[policy]

    if cfg_path.exists():
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        metrics_cfg = cfg.get("metrics", {})
        compare_dir = PROJECT_ROOT / metrics_cfg.get("compare_dir", "results/compare")
        if policy == "greedy":
            filename = metrics_cfg.get("frontend_greedy_file", filename)
        else:
            filename = metrics_cfg.get("backend_si_file", filename)

    return compare_dir / filename


def _write_metrics_row(output_path, source, episode, stats):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "source", "episode", "episode_step", "completion_rate", "on_time_rate",
        "avg_delay", "avg_wait_time_to_load", "avg_delivery_time",
        "avg_generation_to_completion_time", "avg_generation_time",
        "total_completed", "total_generated", "avg_steps_per_order",
        "total_energy_consumed", "max_delivery_time",
    ]
    existing_header = None
    if output_path.exists():
        with open(output_path, "r", encoding="utf-8") as existing:
            existing_header = existing.readline().strip()
    need_header = existing_header != ",".join(header)
    write_mode = "w" if need_header else "a"
    with open(output_path, write_mode, newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if need_header:
            writer.writerow(header)
        writer.writerow([
            source,
            int(episode),
            int(stats.get("episode_step", 0)),
            float(stats["completion_rate"]),
            float(stats["on_time_rate"]),
            float(stats["avg_delay"]),
            float(stats.get("avg_wait_time_to_load", 0.0)),
            float(stats.get("avg_delivery_time", 0.0)),
            float(stats.get("avg_generation_to_completion_time", 0.0)),
            float(stats.get("avg_generation_time", 0.0)),
            int(stats.get("total_completed", 0)),
            int(stats.get("total_generated", 0)),
            float(stats.get("avg_steps_per_order", 0.0)),
            float(stats.get("total_energy_consumed", 0.0)),
            float(stats.get("max_delivery_time", 0.0)),
        ])


def run_one_episode(osm_path, episode_steps, policy):
    env = Environment(osm_path, visualize=False, episode_max_steps=episode_steps)
    scheduler = None
    if policy == "pso":
        scheduler = PSOScheduler(num_drones=len(env.drones), verbose=False)

    obs = env.reset()
    done = False
    while not done:
        if policy == "greedy":
            action = greedy_action_from_observation(obs)
        else:
            action = scheduler.step(obs, current_time=env.current_time)
        obs, _, done, _ = env.step(action)

    stats = env.get_statistics()
    return {
        "episode_step": int(env.current_time),
        "completion_rate": float(stats.get("completion_rate", 0.0)),
        "on_time_rate": float(stats.get("on_time_rate", 0.0)),
        "avg_delay": float(stats.get("avg_delay", 0.0)),
        "avg_wait_time_to_load": float(stats.get("avg_wait_time_to_load", 0.0)),
        "avg_delivery_time": float(stats.get("avg_delivery_time", 0.0)),
        "avg_generation_to_completion_time": float(stats.get("avg_generation_to_completion_time", 0.0)),
        "avg_generation_time": float(stats.get("avg_generation_time", 0.0)),
        "avg_steps_per_order": float(stats.get("avg_steps_per_order", 0.0)),
        "total_completed": int(stats.get("total_completed", 0)),
        "total_generated": int(stats.get("total_generated", getattr(env, "total_generated_tasks", 0))),
        "total_energy_consumed": float(stats.get("total_energy_consumed", 0.0)),
        "max_delivery_time": float(stats.get("max_delivery_time", 0.0)),
    }


def main():
    parser = argparse.ArgumentParser(description="Evaluate frontend environment metrics.")
    parser.add_argument("--policy", type=str, default="greedy", choices=["greedy", "pso"], help="Policy type for evaluation.")
    parser.add_argument("--episodes", type=int, default=5, help="Number of evaluation episodes.")
    parser.add_argument("--episode-steps", type=int, default=None, help="Max steps per episode.")
    parser.add_argument("--osm", type=str, default="data/map/part_of_yangpu.osm", help="Relative path to OSM map file.")
    args = parser.parse_args()

    episode_steps = args.episode_steps or 1200

    all_stats = []
    metrics_path = _get_metrics_output(args.policy)
    source = "frontend_greedy" if args.policy == "greedy" else "backend_si"
    for ep in range(args.episodes):
        stats = run_one_episode(args.osm, episode_steps, args.policy)
        all_stats.append(stats)
        _write_metrics_row(metrics_path, source, ep + 1, stats)
        print(
            f"Episode {ep + 1}: "
            f"completion={stats['completion_rate']:.4f}, "
            f"on_time={stats['on_time_rate']:.4f}, "
            f"avg_delay={stats['avg_delay']:.4f}, "
            f"avg_wait={stats['avg_wait_time_to_load']:.4f}, "
            f"avg_delivery={stats['avg_delivery_time']:.4f}, "
            f"avg_gen_to_done={stats['avg_generation_to_completion_time']:.4f}, "
            f"completed={stats['total_completed']}/{stats['total_generated']}, "
            f"avg_gen_time={stats['avg_generation_time']:.4f}, "
            f"steps={stats['episode_step']}"
        )

    n = max(len(all_stats), 1)
    mean_completion = sum(s["completion_rate"] for s in all_stats) / n
    mean_on_time = sum(s["on_time_rate"] for s in all_stats) / n
    mean_avg_delay = sum(s["avg_delay"] for s in all_stats) / n
    mean_avg_wait = sum(s["avg_wait_time_to_load"] for s in all_stats) / n
    mean_avg_delivery = sum(s["avg_delivery_time"] for s in all_stats) / n
    mean_gen_to_done = sum(s["avg_generation_to_completion_time"] for s in all_stats) / n
    mean_avg_gen_time = sum(s["avg_generation_time"] for s in all_stats) / n

    print("=" * 60)
    print("Mean Metrics")
    print(f"completion_rate:       {mean_completion:.4f}")
    print(f"on_time_rate:          {mean_on_time:.4f}")
    print(f"avg_delay:             {mean_avg_delay:.4f}")
    print(f"avg_wait_time_to_load: {mean_avg_wait:.4f}")
    print(f"avg_delivery_time:     {mean_avg_delivery:.4f}")
    print(f"avg_gen_to_done:       {mean_gen_to_done:.4f}")
    print(f"avg_generation_time:   {mean_avg_gen_time:.4f}")
    print(f"metrics_file:          {metrics_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
