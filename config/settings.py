import json
from pathlib import Path


_DEFAULT_CONFIG = {
    "environment": {
        "max_unassigned_tasks": 5,
        "episode_max_steps": 1200,
        "num_drones": 3,
        "initial_task_count": 8,
        "allow_multi_task": True,
        "max_obs_tasks": 20,
    },
    "task": {
        "clicked_positions_file": "clicked_positions.txt",
        "weight_min": 1,
        "weight_max": 4,
        "priority_min": 1,
        "priority_max": 5,
        "deadline_offset_min": 200,
        "deadline_offset_max": 500,
        "warehouse_route_id": "1364970737#0",
        "charger_route_id": "125465016",
        "warehouse_pos": [357600.574872369, 3462308.772003661],
    },
    "drone": {
        "battery_capacity": 15000.0,
        "battery_consumption_base": 0.5,
        "battery_load_penalty_factor": 0.3,
        "battery_low_threshold": 0.2,
        "speed": 200.0,
        "time_step": 0.1,
        "carrying_capacity": 5,
    },
    "charging_station": {
        "station_id": 0,
        "x": 356000.0,
        "y": 3463000.0,
        "charging_power": 50,
    },
    "backend_wx": {
        "max_tasks": 5,
        "max_remaining_time": 600.0,
        "map_relative_path": "data/map/part_of_yangpu.osm",
    },
    "metrics": {
        "compare_dir": "results/compare",
        "frontend_greedy_file": "frontend_greedy_metrics.csv",
        "backend_si_file": "backend_si_metrics.csv",
        "backend_wx_file": "backend_wx_metrics.csv",
    },
}


def _deep_merge(base, override):
    if not isinstance(base, dict) or not isinstance(override, dict):
        return override

    merged = dict(base)
    for k, v in override.items():
        if k in merged and isinstance(merged[k], dict) and isinstance(v, dict):
            merged[k] = _deep_merge(merged[k], v)
        else:
            merged[k] = v
    return merged


def get_shared_config(config_path=None):
    cfg = dict(_DEFAULT_CONFIG)
    if config_path is None:
        config_path = Path(__file__).resolve().with_name("simulation.json")
    else:
        config_path = Path(config_path)

    if config_path.exists():
        with open(config_path, "r", encoding="utf-8") as f:
            user_cfg = json.load(f)
        cfg = _deep_merge(cfg, user_cfg)

    return cfg
