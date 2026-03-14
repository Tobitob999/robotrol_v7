import json
import os


def load_config(base_dir: str):
    path = os.path.join(base_dir, "configs", "chess.json")
    with open(path, "r", encoding="utf-8-sig") as f:
        return json.loads(f.read())
