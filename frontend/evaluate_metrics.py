import argparse
import os
import sys

# Allow importing backend scheduler from project root.
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from environment import Environment
from backend_si.pso_scheduler import PSOScheduler


def run_one_episode(osm_path, episode_steps):
    env = Environment(osm_path, visualize=False, episode_max_steps=episode_steps)
    scheduler = PSOScheduler(num_drones=len(env.drones), verbose=False)

    obs = env.reset()
    done = False
    while not done:
        action = scheduler.step(obs, current_time=env.current_time)
        obs, _, done, _ = env.step(action)

    return env.get_statistics()


def main():
    parser = argparse.ArgumentParser(description="Evaluate frontend environment metrics.")
    parser.add_argument("--episodes", type=int, default=5, help="Number of evaluation episodes.")
    parser.add_argument("--episode-steps", type=int, default=1200, help="Max steps per episode.")
    parser.add_argument("--osm", type=str, default="data/map/part_of_yangpu.osm", help="Relative path to OSM map file.")
    args = parser.parse_args()

    all_stats = []
    for ep in range(args.episodes):
        stats = run_one_episode(args.osm, args.episode_steps)
        all_stats.append(stats)
        print(
            f"Episode {ep + 1}: completion_rate={stats['completion_rate']:.4f}, "
            f"on_time_rate={stats['on_time_rate']:.4f}, avg_delay={stats['avg_delay']:.4f}"
        )

    n = max(len(all_stats), 1)
    mean_completion = sum(s["completion_rate"] for s in all_stats) / n
    mean_on_time = sum(s["on_time_rate"] for s in all_stats) / n
    mean_avg_delay = sum(s["avg_delay"] for s in all_stats) / n

    print("=" * 60)
    print("Mean Metrics")
    print(f"completion_rate: {mean_completion:.4f}")
    print(f"on_time_rate:    {mean_on_time:.4f}")
    print(f"avg_delay:       {mean_avg_delay:.4f}")
    print("=" * 60)


if __name__ == "__main__":
    main()
