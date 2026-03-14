import json
import os
import sys

DEFAULT_PROFILE = "Moveo"
PROFILE_FILES = {
    "Moveo": "Moveo.json",
    "EB15_red": "EB15_red.json",
    "EB300": "EB300.json",
}

LEGACY_FILES = {
    "endstops": "endstops.json",
    "cam_to_base": "cam_to_base.json",
    "gamepad": "gamepad_config.json",
    "dh_model": os.path.join("model", "dh.json"),
}


def _read_json(path):
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def _write_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=True)


def _candidate_base_dirs(base_dir=None):
    candidates = []
    if base_dir:
        candidates.append(base_dir)
    if getattr(sys, "frozen", False):
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass:
            candidates.append(meipass)
        exe_dir = os.path.dirname(sys.executable)
        if exe_dir:
            candidates.append(exe_dir)
    candidates.append(os.getcwd())
    seen = set()
    result = []
    for base in candidates:
        if not base:
            continue
        norm = os.path.abspath(base)
        if norm in seen:
            continue
        seen.add(norm)
        result.append(norm)
    return result


def _resolve_write_base_dir(base_dir=None):
    if base_dir:
        if getattr(sys, "frozen", False):
            meipass = getattr(sys, "_MEIPASS", None)
            if meipass and os.path.abspath(base_dir) == os.path.abspath(meipass):
                exe_dir = os.path.dirname(sys.executable)
                if exe_dir:
                    return exe_dir
        return base_dir
    if getattr(sys, "frozen", False):
        exe_dir = os.path.dirname(sys.executable)
        if exe_dir:
            return exe_dir
    return os.getcwd()


def _find_existing_profile_path(name, base_dir=None):
    filename = PROFILE_FILES.get(name, f"{name}.json")
    for base in _candidate_base_dirs(base_dir):
        path = os.path.join(base, filename)
        if os.path.isfile(path):
            return path
    return None


def _find_base_dir_with_legacy_files(base_dir=None):
    legacy_paths = list(LEGACY_FILES.values())
    for base in _candidate_base_dirs(base_dir):
        for rel in legacy_paths:
            if os.path.isfile(os.path.join(base, rel)):
                return base
    return _resolve_write_base_dir(base_dir)


def profile_path(name, base_dir=None):
    base = _resolve_write_base_dir(base_dir)
    filename = PROFILE_FILES.get(name, f"{name}.json")
    return os.path.join(base, filename)


def build_profile_from_legacy(name, base_dir=None):
    base = _find_base_dir_with_legacy_files(base_dir)
    data = {"name": name, "version": 1}
    for key, rel in LEGACY_FILES.items():
        payload = _read_json(os.path.join(base, rel))
        if payload is not None:
            data[key] = payload
    return data


def load_profile(name, base_dir=None, create_from_legacy=True):
    path = _find_existing_profile_path(name, base_dir=base_dir)
    data = _read_json(path) if path else None
    if data is None and create_from_legacy and name == DEFAULT_PROFILE:
        data = build_profile_from_legacy(name, base_dir=base_dir)
        if data:
            save_profile(name, data, base_dir=base_dir)
    return data


def save_profile(name, data, base_dir=None):
    path = profile_path(name, base_dir=base_dir)
    _write_json(path, data)
    return path
