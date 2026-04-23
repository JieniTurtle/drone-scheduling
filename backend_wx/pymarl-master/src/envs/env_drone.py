import os
import sys
import json
from pathlib import Path

import numpy as np

from .multiagentenv import MultiAgentEnv


class EnvDroneEnv(MultiAgentEnv):
    """PyMARL env wrapper for direct Python integration with frontend Environment."""

    def __init__(
        self,
        episode_limit=None,
        max_tasks=None,
        max_remaining_time=None,
        frontend_root=None,
        map_relative_path=None,
        seed=None,
        **kwargs,
    ):
        del kwargs

        self._project_root = self._resolve_project_root(frontend_root)
        shared_cfg = self._load_shared_config()
        env_cfg = shared_cfg.get("environment", {})
        wx_cfg = shared_cfg.get("backend_wx", {})

        if episode_limit is None:
            episode_limit = env_cfg.get("episode_max_steps", 1200)
        if max_tasks is None:
            max_tasks = wx_cfg.get("max_tasks", env_cfg.get("max_unassigned_tasks", 5))
        if max_remaining_time is None:
            max_remaining_time = wx_cfg.get("max_remaining_time", 600.0)
        if map_relative_path is None:
            map_relative_path = wx_cfg.get("map_relative_path", "data/map/part_of_yangpu.osm")

        self.episode_limit = int(episode_limit)
        self.max_tasks = int(max_tasks)
        self.max_remaining_time = float(max_remaining_time)
        self._rng = np.random.RandomState(seed if seed is not None else 0)

        self._frontend_dir = self._project_root / "frontend"
        self._map_path = self._frontend_dir / map_relative_path

        self._prepare_frontend_import_paths()
        self._frontend_env = self._create_frontend_env()

        self.n_agents = len(self._frontend_env.drones)
        self.n_actions = self.max_tasks + 1  # action 0 is NoOp
        self.noop_action = 0

        self._task_feature_dim = 8
        self._agent_feature_dim = 3
        self._obs_size = self._agent_feature_dim + self.max_tasks * self._task_feature_dim
        self._state_size = self.n_agents * self._agent_feature_dim + self.max_tasks * self._task_feature_dim

        self._bounds = self._frontend_env.get_global_bounds()
        self._t = 0
        self._last_obs = None
        self.reset()

    def _resolve_project_root(self, frontend_root):
        if frontend_root:
            root = Path(frontend_root).resolve()
        else:
            root = Path(__file__).resolve().parents[4]
        if not (root / "frontend").exists():
            raise FileNotFoundError("Cannot find frontend directory under project root.")
        return root

    def _load_shared_config(self):
        cfg_path = self._project_root / "config" / "simulation.json"
        if not cfg_path.exists():
            return {}
        with open(cfg_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _prepare_frontend_import_paths(self):
        frontend_dir_str = str(self._frontend_dir)
        root_dir_str = str(self._project_root)
        if frontend_dir_str not in sys.path:
            sys.path.insert(0, frontend_dir_str)
        if root_dir_str not in sys.path:
            sys.path.insert(0, root_dir_str)

    def _create_frontend_env(self):
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        from environment import Environment

        prev_cwd = os.getcwd()
        try:
            # frontend Environment relies on relative paths for local assets.
            os.chdir(str(self._frontend_dir))
            return Environment(str(self._map_path), visualize=False, episode_max_steps=self.episode_limit)
        finally:
            os.chdir(prev_cwd)

    def reset(self):
        self._t = 0
        self._last_obs = self._frontend_env.reset()
        return self.get_obs(), self.get_state()

    def step(self, actions):
        action_list = self._to_action_list(actions)
        obs_before = self._last_obs if self._last_obs is not None else self._frontend_env._obs()

        assignments = {}
        selected_task_ids = set()
        unassigned_tasks = obs_before.get("unassigned_tasks", [])
        drone_is_free = obs_before.get("drone_is_free", [True] * self.n_agents)

        for agent_id, action in enumerate(action_list):
            if action <= 0:
                continue
            if agent_id >= len(drone_is_free) or not drone_is_free[agent_id]:
                continue
            task_slot = action - 1
            if task_slot >= len(unassigned_tasks):
                continue

            task_id = unassigned_tasks[task_slot]["task_id"]
            if task_id in selected_task_ids:
                continue

            selected_task_ids.add(task_id)
            assignments[agent_id] = [task_id]

        next_obs, reward, frontend_done, frontend_info = self._frontend_env.step(assignments)
        self._last_obs = next_obs
        self._t += 1

        hit_episode_limit = self._t >= self.episode_limit
        terminated = bool(hit_episode_limit or frontend_done)
        env_info = {
            "episode_limit": bool(hit_episode_limit),
        }

        if terminated:
            if isinstance(frontend_info, dict):
                for k in ("completion_rate", "on_time_rate", "avg_delay"):
                    if k in frontend_info:
                        env_info[k] = float(frontend_info[k])
            if hasattr(self._frontend_env, "get_statistics"):
                stats = self._frontend_env.get_statistics()
                env_info["completion_rate"] = float(stats.get("completion_rate", env_info.get("completion_rate", 0.0)))
                env_info["on_time_rate"] = float(stats.get("on_time_rate", env_info.get("on_time_rate", 0.0)))
                env_info["avg_delay"] = float(stats.get("avg_delay", env_info.get("avg_delay", 0.0)))

        return float(reward), terminated, env_info

    def _to_action_list(self, actions):
        if hasattr(actions, "detach"):
            arr = actions.detach().cpu().numpy()
        else:
            arr = np.asarray(actions)
        arr = np.array(arr).reshape(-1)
        out = []
        for i in range(self.n_agents):
            if i < arr.shape[0]:
                out.append(int(arr[i]))
            else:
                out.append(0)
        return out

    def _norm_coord(self, x, y):
        if not self._bounds:
            return 0.0, 0.0
        min_x, min_y, max_x, max_y = self._bounds
        dx = max(max_x - min_x, 1e-6)
        dy = max(max_y - min_y, 1e-6)
        nx = float((x - min_x) / dx)
        ny = float((y - min_y) / dy)
        return nx, ny

    def _task_vector(self, task):
        sx, sy = task.get("source", [0.0, 0.0])
        dx, dy = task.get("destination", [0.0, 0.0])
        nsx, nsy = self._norm_coord(sx, sy)
        ndx, ndy = self._norm_coord(dx, dy)

        rem = task.get("remaining_time", self.max_remaining_time)
        if np.isinf(rem):
            rem = self.max_remaining_time
        rem = float(np.clip(rem / max(self.max_remaining_time, 1.0), 0.0, 1.0))

        priority = float(task.get("priority", 0.0)) / 5.0
        weight = float(task.get("weight", 0.0)) / 10.0
        active = 1.0
        return [nsx, nsy, ndx, ndy, rem, priority, weight, active]

    def _flatten_tasks(self):
        task_features = []
        tasks = [] if self._last_obs is None else self._last_obs.get("unassigned_tasks", [])
        for i in range(self.max_tasks):
            if i < len(tasks):
                task_features.extend(self._task_vector(tasks[i]))
            else:
                task_features.extend([0.0] * self._task_feature_dim)
        return task_features

    def get_obs(self):
        if self._last_obs is None:
            return [[0.0] * self._obs_size for _ in range(self.n_agents)]

        tasks_flat = self._flatten_tasks()
        drone_positions = self._last_obs.get("drone_positions", [[0.0, 0.0]] * self.n_agents)
        drone_is_free = self._last_obs.get("drone_is_free", [True] * self.n_agents)

        obs = []
        for agent_id in range(self.n_agents):
            x, y = drone_positions[agent_id]
            nx, ny = self._norm_coord(x, y)
            is_free = 1.0 if drone_is_free[agent_id] else 0.0
            obs.append([nx, ny, is_free] + tasks_flat)
        return obs

    def get_obs_agent(self, agent_id):
        return self.get_obs()[agent_id]

    def get_obs_size(self):
        return self._obs_size

    def get_state(self):
        if self._last_obs is None:
            return [0.0] * self._state_size

        state = []
        drone_positions = self._last_obs.get("drone_positions", [[0.0, 0.0]] * self.n_agents)
        drone_is_free = self._last_obs.get("drone_is_free", [True] * self.n_agents)

        for agent_id in range(self.n_agents):
            x, y = drone_positions[agent_id]
            nx, ny = self._norm_coord(x, y)
            state.extend([nx, ny, 1.0 if drone_is_free[agent_id] else 0.0])

        state.extend(self._flatten_tasks())
        return state

    def get_state_size(self):
        return self._state_size

    def get_avail_actions(self):
        return [self.get_avail_agent_actions(agent_id) for agent_id in range(self.n_agents)]

    def get_avail_agent_actions(self, agent_id):
        avail = [0] * self.n_actions
        avail[self.noop_action] = 1

        if self._last_obs is None:
            return avail

        drone_is_free = self._last_obs.get("drone_is_free", [True] * self.n_agents)
        if agent_id >= len(drone_is_free) or not drone_is_free[agent_id]:
            return avail

        num_tasks = min(len(self._last_obs.get("unassigned_tasks", [])), self.max_tasks)
        for action in range(1, num_tasks + 1):
            avail[action] = 1
        return avail

    def get_total_actions(self):
        return self.n_actions

    def render(self):
        return None

    def close(self):
        return None

    def seed(self):
        return None

    def save_replay(self):
        return None
