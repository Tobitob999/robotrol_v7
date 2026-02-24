import json
import os
import random
import time
from typing import Any, Dict, Optional


DEFAULT_SETTINGS = {
    "enabled": False,
    "mode": "shadow",
    "explore_prob": 0.25,
    "explore_decay": 0.995,
    "explore_min": 0.05,
    "reward": {
        "success": 1.0,
        "failure": -1.5,
        "sim_scale": 0.5,
        "confidence_weight": 0.4,
        "time_target_s": 2.5,
        "time_penalty_per_s": 0.1,
        "error_penalty": 0.2
    },
    "safety": {
        "enable_rollback": True,
        "rollback_failure_streak": 3,
        "fallback_mode": "shadow"
    },
    "bounds": {
        "grasp_offset_mm": [-20.0, 20.0],
        "approach_distance_mm": [5.0, 120.0],
        "lift_distance_mm": [10.0, 180.0],
        "approach_speed": [10.0, 400.0],
        "grasp_speed": [5.0, 250.0],
        "close_force": [0.1, 5.0],
        "min_confidence": [0.3, 0.99],
        "min_area_px": [100, 30000]
    },
    "sigma": {
        "grasp_offset_mm": 1.0,
        "approach_distance_mm": 3.0,
        "lift_distance_mm": 4.0,
        "approach_speed": 10.0,
        "grasp_speed": 6.0,
        "close_force": 0.1,
        "min_confidence": 0.02,
        "min_area_px": 80.0
    },
    "seed": 42
    ,
    "context": {
        "confidence_bins": [0.6, 0.85]
    }
}


DEFAULT_POLICY = {
    "version": 2,
    "epsilon": 0.25,
    "best_reward": -1e9,
    "last_reward": 0.0,
    "failure_streak": 0,
    "stats": {"attempts": 0, "successes": 0},
    "params": {
        "grasp_offset_mm": 0.0,
        "approach_distance_mm": 50.0,
        "lift_distance_mm": 80.0,
        "approach_speed": 60.0,
        "grasp_speed": 30.0,
        "close_force": 1.0,
        "min_confidence": 0.8,
        "min_area_px": 500
    },
    "stable_params": {
        "grasp_offset_mm": 0.0,
        "approach_distance_mm": 50.0,
        "lift_distance_mm": 80.0,
        "approach_speed": 60.0,
        "grasp_speed": 30.0,
        "close_force": 1.0,
        "min_confidence": 0.8,
        "min_area_px": 500
    },
    "contexts": {}
}


class TntSelfLearningManager:
    def __init__(self, base_dir: str, logger=None):
        self._base_dir = base_dir
        self._logger = logger
        self._cfg_path = os.path.join(base_dir, "configs", "tnt_learning.json")
        self._policy_path = os.path.join(base_dir, "configs", "tnt_learning_policy.json")
        self._log_path = os.path.join(base_dir, "data", "learning_tnt_log.jsonl")
        self.settings = self._load_json(self._cfg_path, DEFAULT_SETTINGS)
        self.policy = self._load_json(self._policy_path, DEFAULT_POLICY)
        self._pending: Optional[Dict[str, Any]] = None
        self._rng = random.Random(int(self.settings.get("seed", 42)))
        self._last_context_key = "global"
        self._normalize_policy()

    @staticmethod
    def _merge_defaults(data: Dict[str, Any], defaults: Dict[str, Any]) -> Dict[str, Any]:
        out = dict(defaults)
        for k, v in data.items():
            if isinstance(v, dict) and isinstance(out.get(k), dict):
                out[k] = TntSelfLearningManager._merge_defaults(v, out[k])
            else:
                out[k] = v
        return out

    def _normalize_policy(self):
        self.policy = self._merge_defaults(self.policy, DEFAULT_POLICY)
        if int(self.policy.get("version", 1)) < 2:
            self.policy["version"] = 2
        if not self.policy.get("stable_params"):
            self.policy["stable_params"] = dict(self.policy.get("params", {}))
        if "contexts" not in self.policy or not isinstance(self.policy.get("contexts"), dict):
            self.policy["contexts"] = {}

    def _context_key(self, cycle_ctx: Dict[str, Any], simulated: bool) -> str:
        bins = self.settings.get("context", {}).get("confidence_bins", [0.6, 0.85])
        try:
            b0 = float(bins[0])
            b1 = float(bins[1])
        except Exception:
            b0, b1 = 0.6, 0.85
        conf = cycle_ctx.get("confidence")
        if conf is None:
            band = "unknown"
        else:
            try:
                c = float(conf)
                if c < b0:
                    band = "low"
                elif c < b1:
                    band = "mid"
                else:
                    band = "high"
            except Exception:
                band = "unknown"
        phase = "sim" if simulated else "real"
        return f"{phase}:{band}"

    def _get_context_bucket(self, key: str) -> Dict[str, Any]:
        contexts = self.policy.setdefault("contexts", {})
        if key not in contexts or not isinstance(contexts.get(key), dict):
            contexts[key] = {
                "best_reward": -1e9,
                "attempts": 0,
                "successes": 0,
                "params": dict(self.policy.get("params", {})),
            }
        return contexts[key]

    def _load_json(self, path: str, defaults: Dict[str, Any]) -> Dict[str, Any]:
        if not os.path.exists(path):
            return json.loads(json.dumps(defaults))
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return self._merge_defaults(data, defaults)
        except Exception:
            return json.loads(json.dumps(defaults))

    def _save_json(self, path: str, payload: Dict[str, Any]):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

    def save(self):
        self._save_json(self._policy_path, self.policy)

    def save_settings(self):
        self._save_json(self._cfg_path, self.settings)

    def set_enabled(self, enabled: bool):
        self.settings["enabled"] = bool(enabled)

    def set_mode(self, mode: str):
        mode = (mode or "shadow").strip().lower()
        self.settings["mode"] = "active" if mode == "active" else "shadow"

    def sync_baseline_from_configs(self, configs: Dict[str, Any]):
        params = self.policy.get("params", {})
        cube = configs.get("cube", {})
        robot = configs.get("robot", {})
        gripper = robot.get("gripper", {})
        perception = configs.get("perception", {})
        tnt = configs.get("tnt", {})
        params["grasp_offset_mm"] = float(cube.get("grasp_offset_mm", params.get("grasp_offset_mm", 0.0)))
        params["approach_distance_mm"] = float(cube.get("approach_distance_mm", params.get("approach_distance_mm", 50.0)))
        params["lift_distance_mm"] = float(cube.get("lift_distance_mm", params.get("lift_distance_mm", 80.0)))
        params["approach_speed"] = float(robot.get("approach_speed", params.get("approach_speed", 60.0)))
        params["grasp_speed"] = float(robot.get("grasp_speed", params.get("grasp_speed", 30.0)))
        params["close_force"] = float(gripper.get("close_force", params.get("close_force", 1.0)))
        params["min_confidence"] = float(perception.get("min_confidence", params.get("min_confidence", 0.8)))
        params["min_area_px"] = int(tnt.get("min_area_px", params.get("min_area_px", 500)))
        self.policy["stable_params"] = dict(params)

    def reset_policy_from_configs(self, configs: Dict[str, Any]):
        self.policy = json.loads(json.dumps(DEFAULT_POLICY))
        self.sync_baseline_from_configs(configs)
        self.save()

    def _clamp(self, key: str, value: float) -> float:
        b = self.settings.get("bounds", {}).get(key)
        if not b or len(b) != 2:
            return value
        lo, hi = float(b[0]), float(b[1])
        return max(lo, min(hi, value))

    def _propose_candidate(self) -> Dict[str, Any]:
        ctx_bucket = self._get_context_bucket(self._last_context_key)
        seed_params = ctx_bucket.get("params") or self.policy.get("params", {})
        params = dict(seed_params)
        epsilon = float(self.policy.get("epsilon", self.settings.get("explore_prob", 0.25)))
        explore = self._rng.random() < epsilon
        if explore:
            sigmas = self.settings.get("sigma", {})
            for key, base in list(params.items()):
                sigma = float(sigmas.get(key, 0.0))
                if sigma <= 0.0:
                    continue
                proposal = float(base) + self._rng.gauss(0.0, sigma)
                if key == "min_area_px":
                    proposal = int(round(proposal))
                params[key] = self._clamp(key, float(proposal))
        return {"params": params, "explore": explore}

    def _apply_params_to_configs(self, configs: Dict[str, Any], params: Dict[str, Any]):
        cube = configs.setdefault("cube", {})
        robot = configs.setdefault("robot", {})
        gripper = robot.setdefault("gripper", {})
        perception = configs.setdefault("perception", {})
        tnt = configs.setdefault("tnt", {})

        cube["grasp_offset_mm"] = float(params["grasp_offset_mm"])
        cube["approach_distance_mm"] = float(params["approach_distance_mm"])
        cube["lift_distance_mm"] = float(params["lift_distance_mm"])
        robot["approach_speed"] = float(params["approach_speed"])
        robot["grasp_speed"] = float(params["grasp_speed"])
        gripper["close_force"] = float(params["close_force"])
        perception["min_confidence"] = float(params["min_confidence"])
        tnt["min_area_px"] = int(round(float(params["min_area_px"])))

    def before_cycle(self, configs: Dict[str, Any], simulated: bool, context_hint: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        if not bool(self.settings.get("enabled", False)):
            self._pending = None
            return {"enabled": False, "applied": False, "mode": self.settings.get("mode", "shadow")}

        self._last_context_key = self._context_key(context_hint or {}, bool(simulated))
        candidate = self._propose_candidate()
        mode = str(self.settings.get("mode", "shadow")).lower()
        applied = mode == "active"
        if applied:
            self._apply_params_to_configs(configs, candidate["params"])
        self._pending = {
            "timestamp": time.time(),
            "simulated": bool(simulated),
            "mode": mode,
            "applied": applied,
            "params": candidate["params"],
            "explore": bool(candidate["explore"]),
            "context_key": self._last_context_key,
        }
        return {
            "enabled": True,
            "applied": applied,
            "mode": mode,
            "explore": bool(candidate["explore"]),
            "context_key": self._last_context_key,
        }

    def _reward(self, success: bool, simulated: bool, cycle_ctx: Dict[str, Any], error_text: str) -> float:
        rw = self.settings.get("reward", {})
        score = float(rw.get("success", 1.0) if success else rw.get("failure", -1.5))

        confidence = cycle_ctx.get("confidence")
        if confidence is not None:
            try:
                score += float(rw.get("confidence_weight", 0.0)) * max(0.0, float(confidence) - 0.5)
            except Exception:
                pass

        duration_s = cycle_ctx.get("duration_s")
        if duration_s is not None:
            try:
                excess = max(0.0, float(duration_s) - float(rw.get("time_target_s", 2.5)))
                score -= excess * float(rw.get("time_penalty_per_s", 0.1))
            except Exception:
                pass

        if error_text:
            score -= float(rw.get("error_penalty", 0.2))

        if simulated:
            score *= float(rw.get("sim_scale", 0.5))
        return score

    def _maybe_rollback(self, success: bool) -> Dict[str, Any]:
        out = {"rolled_back": False, "mode_changed": False}
        if success:
            self.policy["failure_streak"] = 0
            self.policy["stable_params"] = dict(self.policy.get("params", {}))
            return out

        self.policy["failure_streak"] = int(self.policy.get("failure_streak", 0)) + 1
        safety = self.settings.get("safety", {})
        if not bool(safety.get("enable_rollback", True)):
            return out
        trigger = int(safety.get("rollback_failure_streak", 3))
        if int(self.policy.get("failure_streak", 0)) < max(1, trigger):
            return out

        stable = dict(self.policy.get("stable_params", {}))
        if stable:
            self.policy["params"] = stable
            out["rolled_back"] = True

        fallback_mode = str(safety.get("fallback_mode", "shadow")).lower()
        if fallback_mode in {"shadow", "active"} and self.settings.get("mode") != fallback_mode:
            self.settings["mode"] = fallback_mode
            out["mode_changed"] = True
            self.save_settings()
        return out

    def after_cycle(self, success: bool, error_text: str = "", cycle_ctx: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        pending = self._pending
        if not pending:
            return {"updated": False}

        ctx = cycle_ctx or {}
        simulated = bool(pending.get("simulated", False))
        context_key = str(pending.get("context_key", self._context_key(ctx, simulated)))
        self._last_context_key = context_key
        bucket = self._get_context_bucket(context_key)
        reward = self._reward(bool(success), simulated, ctx, error_text)
        self.policy["last_reward"] = float(reward)

        stats = self.policy.setdefault("stats", {"attempts": 0, "successes": 0})
        stats["attempts"] = int(stats.get("attempts", 0)) + 1
        if success:
            stats["successes"] = int(stats.get("successes", 0)) + 1
        bucket["attempts"] = int(bucket.get("attempts", 0)) + 1
        if success:
            bucket["successes"] = int(bucket.get("successes", 0)) + 1

        best_reward = float(self.policy.get("best_reward", -1e9))
        best_reward_ctx = float(bucket.get("best_reward", -1e9))
        accepted = False
        if pending.get("mode") == "active" and reward >= best_reward_ctx:
            bucket["best_reward"] = reward
            bucket["params"] = dict(pending["params"])
            self.policy["params"] = dict(pending["params"])
            self.policy["best_reward"] = max(best_reward, reward)
            accepted = True
        elif pending.get("mode") == "shadow" and reward > best_reward_ctx:
            bucket["best_reward"] = reward
            self.policy["best_reward"] = max(best_reward, reward)

        rb = self._maybe_rollback(bool(success))

        eps = float(self.policy.get("epsilon", self.settings.get("explore_prob", 0.25)))
        eps *= float(self.settings.get("explore_decay", 0.995))
        eps = max(float(self.settings.get("explore_min", 0.05)), eps)
        self.policy["epsilon"] = eps

        self.save()
        self._append_log(
            {
                "timestamp": pending.get("timestamp"),
                "mode": pending.get("mode"),
                "applied": pending.get("applied"),
                "explore": pending.get("explore"),
                "simulated": simulated,
                "success": bool(success),
                "reward": reward,
                "accepted": accepted,
                "epsilon": eps,
                "error": error_text or "",
                "params": pending.get("params"),
                "cycle_ctx": ctx,
                "context_key": context_key,
                "rollback": rb,
            }
        )
        self._pending = None
        return {
            "updated": True,
            "accepted": accepted,
            "reward": reward,
            "epsilon": eps,
            "rolled_back": bool(rb.get("rolled_back", False)),
            "mode_changed": bool(rb.get("mode_changed", False)),
        }

    def _append_log(self, entry: Dict[str, Any]):
        os.makedirs(os.path.dirname(self._log_path), exist_ok=True)
        with open(self._log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")

    def get_status(self) -> Dict[str, Any]:
        stats = self.policy.get("stats", {})
        attempts = int(stats.get("attempts", 0))
        successes = int(stats.get("successes", 0))
        ratio = (float(successes) / float(attempts)) if attempts > 0 else 0.0
        return {
            "enabled": bool(self.settings.get("enabled", False)),
            "mode": str(self.settings.get("mode", "shadow")),
            "epsilon": float(self.policy.get("epsilon", 0.0)),
            "attempts": attempts,
            "successes": successes,
            "success_rate": ratio,
            "best_reward": float(self.policy.get("best_reward", -1e9)),
            "last_reward": float(self.policy.get("last_reward", 0.0)),
            "failure_streak": int(self.policy.get("failure_streak", 0)),
            "active_context": self._last_context_key,
            "context_count": len(self.policy.get("contexts", {})),
            "params": dict(self.policy.get("params", {})),
        }
