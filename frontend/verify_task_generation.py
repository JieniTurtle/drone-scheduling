import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FRONTEND_ROOT = PROJECT_ROOT / "frontend"

sys.path.append(str(PROJECT_ROOT))

from environment import Environment
from scheduler import greedy_action_from_observation
from config.config_loder import get_shared_config


def _resolve_episode_steps(default_steps):
    cfg = get_shared_config()
    env_cfg = cfg.get("environment", {}) if isinstance(cfg, dict) else {}
    return int(env_cfg.get("episode_max_steps", default_steps))


def run_one_episode(osm_path, episode_steps):
    env = Environment(str(osm_path), visualize=False, episode_max_steps=episode_steps)
    obs = env.reset()
    done = False
    while not done:
        action = greedy_action_from_observation(obs)
        obs, _, done, _ = env.step(action)

    stats = env.get_statistics()
    result = {
        "avg_delivery_time": float(stats.get("avg_delivery_time", 0.0)),
        "avg_delay": float(stats.get("avg_delay", 0.0)),
        "total_completed": int(stats.get("total_completed", 0)),
        "total_generated": int(getattr(env, "total_generated_tasks", 0)),
        "episode_steps": int(env.current_time),
    }
    if result["total_generated"] > 0:
        result["order_generate_rate"] =  result["episode_steps"] / result["total_generated"]
    else:
        result["order_generate_rate"] = 0.0
    return result


def main():
    parser = argparse.ArgumentParser(description="Verify task generation with greedy scheduling.")
    parser.add_argument("--episodes", type=int, default=3, help="Number of episodes to run.")
    parser.add_argument("--episode-steps", type=int, default=None, help="Max steps per episode.")
    parser.add_argument("--osm", type=str, default="data/map/part_of_yangpu.osm", help="Relative OSM path under frontend.")
    args = parser.parse_args()

    episode_steps = args.episode_steps or _resolve_episode_steps(default_steps=1200)
    osm_path = FRONTEND_ROOT / args.osm

    all_stats = []
    for ep in range(args.episodes):
        stats = run_one_episode(osm_path, episode_steps)
        all_stats.append(stats)
        print(
            "Episode {}: avg_delivery_time={:.4f}, avg_delay={:.4f}, order_generate_rate={:.4f}, total_completed={}, total_generated={}, steps={}".format(
                ep + 1,
                stats["avg_delivery_time"],
                stats["avg_delay"],
                stats["order_generate_rate"],
                stats["total_completed"],
                stats["total_generated"],
                stats["episode_steps"],
            )
        )

    n = max(len(all_stats), 1)
    mean_delivery = sum(s["avg_delivery_time"] for s in all_stats) / n
    mean_delay = sum(s["avg_delay"] for s in all_stats) / n
    mean_order_generate_rate = sum(s["order_generate_rate"] for s in all_stats) / n

    print("=" * 60)
    print("Mean Metrics")
    print("avg_delivery_time: {:.4f}".format(mean_delivery))
    print("avg_delay:        {:.4f}".format(mean_delay))
    print("order_generate_rate:  {:.4f}".format(mean_order_generate_rate))
    print("=" * 60)


if __name__ == "__main__":
    main()
