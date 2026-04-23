from envs import REGISTRY as env_REGISTRY
from functools import partial
from components.episode_buffer import EpisodeBatch
import numpy as np
import torch as th
import csv
import os


class EpisodeRunner:

    def __init__(self, args, logger):
        self.args = args
        self.logger = logger
        self.batch_size = self.args.batch_size_run
        assert self.batch_size == 1

        self.env = env_REGISTRY[self.args.env](**self.args.env_args)
        self.episode_limit = self.env.episode_limit
        self.t = 0

        self.t_env = 0

        self.train_returns = []
        self.test_returns = []
        self.train_stats = {}
        self.test_stats = {}
        self.metrics_file = os.path.join(self.args.local_results_path, "env_metrics.csv")
        self._metrics_header_written = False

        # Log the first run
        self.log_train_stats_t = -1000000

    def setup(self, scheme, groups, preprocess, mac):
        self.new_batch = partial(EpisodeBatch, scheme, groups, self.batch_size, self.episode_limit + 1,
                                 preprocess=preprocess, device=self.args.device)
        self.mac = mac

    def get_env_info(self):
        return self.env.get_env_info()

    def save_replay(self):
        self.env.save_replay()

    def close_env(self):
        self.env.close()

    def reset(self):
        self.batch = self.new_batch()
        self.env.reset()
        self.t = 0

    def run(self, test_mode=False):
        if hasattr(self.env, "set_test_mode"):
            self.env.set_test_mode(test_mode)
        self.reset()

        terminated = False
        episode_return = 0
        noop_fastpath_steps = 0
        self.mac.init_hidden(batch_size=self.batch_size)

        while not terminated:

            avail_actions = self.env.get_avail_actions()

            pre_transition_data = {
                "state": [self.env.get_state()],
                "avail_actions": [avail_actions],
                "obs": [self.env.get_obs()]
            }

            self.batch.update(pre_transition_data, ts=self.t)

            avail_actions_arr = np.asarray(avail_actions)
            only_noop = (
                avail_actions_arr.ndim == 2
                and avail_actions_arr.shape[1] > 0
                and np.all(avail_actions_arr[:, 0] == 1)
                and np.all(avail_actions_arr.sum(axis=1) == 1)
            )

            if only_noop:
                actions = th.zeros((self.batch_size, self.env.n_agents), dtype=th.long, device=self.args.device)
                noop_fastpath_steps += 1
            else:
                # Pass the entire batch of experiences up till now to the agents
                # Receive the actions for each agent at this timestep in a batch of size 1
                actions = self.mac.select_actions(self.batch, t_ep=self.t, t_env=self.t_env, test_mode=test_mode)

            reward, terminated, env_info = self.env.step(actions[0])
            episode_return += reward

            post_transition_data = {
                "actions": actions,
                "reward": [(reward,)],
                "terminated": [(terminated != env_info.get("episode_limit", False),)],
            }

            self.batch.update(post_transition_data, ts=self.t)

            self.t += 1

        env_info["noop_fastpath_steps"] = noop_fastpath_steps

        self._write_episode_metrics(env_info, test_mode)

        if hasattr(self.env, "pop_episode_reward_adjustments"):
            adjustments, _ = self.env.pop_episode_reward_adjustments(self.t)
            if adjustments:
                add = th.tensor(adjustments, dtype=self.batch["reward"].dtype, device=self.batch["reward"].device)
                self.batch.data.transition_data["reward"][0, : self.t, 0] += add
                episode_return += float(np.sum(adjustments))

        last_data = {
            "state": [self.env.get_state()],
            "avail_actions": [self.env.get_avail_actions()],
            "obs": [self.env.get_obs()]
        }
        self.batch.update(last_data, ts=self.t)

        # Select actions in the last stored state
        actions = self.mac.select_actions(self.batch, t_ep=self.t, t_env=self.t_env, test_mode=test_mode)
        self.batch.update({"actions": actions}, ts=self.t)

        cur_stats = self.test_stats if test_mode else self.train_stats
        cur_returns = self.test_returns if test_mode else self.train_returns
        log_prefix = "test_" if test_mode else ""
        cur_stats.update({k: cur_stats.get(k, 0) + env_info.get(k, 0) for k in set(cur_stats) | set(env_info)})
        cur_stats["n_episodes"] = 1 + cur_stats.get("n_episodes", 0)
        cur_stats["ep_length"] = self.t + cur_stats.get("ep_length", 0)

        if not test_mode:
            self.t_env += self.t

        cur_returns.append(episode_return)

        if test_mode and (len(self.test_returns) == self.args.test_nepisode):
            self._log(cur_returns, cur_stats, log_prefix)
        elif self.t_env - self.log_train_stats_t >= self.args.runner_log_interval:
            self._log(cur_returns, cur_stats, log_prefix)
            if hasattr(self.mac.action_selector, "epsilon"):
                self.logger.log_stat("epsilon", self.mac.action_selector.epsilon, self.t_env)
            self.log_train_stats_t = self.t_env

        return self.batch

    def _log(self, returns, stats, prefix):
        self.logger.log_stat(prefix + "return_mean", np.mean(returns), self.t_env)
        self.logger.log_stat(prefix + "return_std", np.std(returns), self.t_env)
        returns.clear()

        for k, v in stats.items():
            if k != "n_episodes":
                self.logger.log_stat(prefix + k + "_mean" , v/stats["n_episodes"], self.t_env)
        stats.clear()

    def _write_episode_metrics(self, env_info, test_mode):
        if not isinstance(env_info, dict):
            return

        required_keys = ("completion_rate", "on_time_rate", "avg_delay")
        if not all(k in env_info for k in required_keys):
            return

        os.makedirs(os.path.dirname(self.metrics_file), exist_ok=True)
        mode = "test" if test_mode else "train"

        need_header = (not self._metrics_header_written) and (not os.path.exists(self.metrics_file))
        with open(self.metrics_file, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if need_header:
                writer.writerow(["t_env", "mode", "completion_rate", "on_time_rate", "avg_delay"])
                self._metrics_header_written = True
            writer.writerow([
                int(self.t_env),
                mode,
                float(env_info["completion_rate"]),
                float(env_info["on_time_rate"]),
                float(env_info["avg_delay"]),
            ])
