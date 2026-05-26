import json
import subprocess
import sys
from pathlib import Path


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _write_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


def main():
    project_root = Path(__file__).resolve().parents[3]
    sim_path = project_root / "config" / "simulation.json"

    original_cfg = _load_json(sim_path)

    algs = ["qmix", "iql", "vdn"]
    scales = [4, 3, 2, 1]
    unique_flags = [True, False]

    try:
        for scale in scales:
            cfg = _load_json(sim_path)
            cfg.setdefault("task_generation", {}).setdefault("realistic", {})["interval_scale"] = float(scale)
            _write_json(sim_path, cfg)

            for alg in algs:
                for unique in unique_flags:
                    unique_str = "true" if unique else "false"
                    label = f"{alg}_unique_{int(unique)}_scale_{scale}"
                    cmd = [
                        sys.executable,
                        str(project_root / "backend_wx" / "pymarl-master" / "src" / "main.py"),
                        f"--config={alg}",
                        "--env-config=env_drone",
                        "with",
                        f"unique_task_assignment={unique_str}",
                        f"label={label}",
                    ]
                    print("Running:", " ".join(cmd))
                    subprocess.run(cmd, cwd=str(project_root), check=True)
    finally:
        _write_json(sim_path, original_cfg)


if __name__ == "__main__":
    main()
