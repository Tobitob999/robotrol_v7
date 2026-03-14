import json
import os


CONFIG_FILES = {
    "system": os.path.join("configs", "system.json"),
    "camera": os.path.join("configs", "camera.json"),
    "markers": os.path.join("configs", "markers.json"),
    "cube": os.path.join("configs", "cube.yaml"),
    "grid": os.path.join("configs", "grid.json"),
    "robot": os.path.join("configs", "robot.json"),
    "transforms": os.path.join("configs", "transforms.json"),
    "perception": os.path.join("configs", "perception.json"),
    "simulation": os.path.join("configs", "simulation.json"),
    "calibration": os.path.join("configs", "calibration.json"),
    "tnt": os.path.join("configs", "tnt.json"),
    "chess": os.path.join("configs", "chess.json"),
    "project_flags": os.path.join("configs", "project_flags.json"),
    "tnt_learning": os.path.join("configs", "tnt_learning.json"),
}


def load_config(path: str):
    with open(path, "r", encoding="utf-8-sig") as f:
        content = f.read()
    return json.loads(content)


def load_all_configs(base_dir: str):
    configs = {}
    for key, rel_path in CONFIG_FILES.items():
        path = os.path.join(base_dir, rel_path)
        configs[key] = load_config(path)
    return configs
