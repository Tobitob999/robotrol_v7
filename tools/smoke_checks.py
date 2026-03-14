#!/usr/bin/env python3
"""Project smoke checks (non-UI, non-hardware).

Runs:
- English text check (strict mode)
- Short mock simulation cycle run
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from control.app import build_pipeline
from learning.tnt_self_learning import TntSelfLearningManager
from simulation.simulation_loop import SimulationRunner


def _run_language_check(repo_root: str) -> None:
    cmd = [sys.executable, os.path.join(repo_root, "tools", "check_english_text.py"), "--strict"]
    rc = subprocess.call(cmd, cwd=repo_root)
    if rc != 0:
        raise RuntimeError("Language check failed.")


def _run_mock_simulation(repo_root: str, cycles: int = 5) -> None:
    pipeline, configs = build_pipeline(repo_root)
    # Force deterministic mock-perception mode for CI/local smoke checks.
    pipeline.context.perception.mode = "mock"
    pipeline.context.perception.min_confidence = 0.0
    sim = SimulationRunner(pipeline, configs.get("perception", {}))
    successes = sim.run(cycles)
    if successes != cycles:
        raise RuntimeError(f"Simulation smoke failed: {successes}/{cycles} cycles succeeded.")


def _run_learner_logic_check() -> None:
    with tempfile.TemporaryDirectory(prefix="tnt_learn_smoke_") as tmp:
        os.makedirs(os.path.join(tmp, "configs"), exist_ok=True)
        os.makedirs(os.path.join(tmp, "data"), exist_ok=True)
        learner = TntSelfLearningManager(tmp)
        configs = {
            "cube": {"grasp_offset_mm": 0.0, "approach_distance_mm": 50.0, "lift_distance_mm": 80.0},
            "robot": {"approach_speed": 60.0, "grasp_speed": 30.0, "gripper": {"close_force": 1.0}},
            "perception": {"min_confidence": 0.8},
            "tnt": {"min_area_px": 500},
        }
        learner.reset_policy_from_configs(configs)
        learner.set_enabled(True)
        learner.set_mode("active")
        learner.save_settings()

        learner.before_cycle(configs, simulated=False)
        out_ok = learner.after_cycle(True, "", {"duration_s": 1.5, "confidence": 0.95})
        if not out_ok.get("updated"):
            raise RuntimeError("Learner smoke failed: no update on success cycle.")

        # Force rollback behavior by repeated failures.
        rolled_back = False
        for _ in range(4):
            learner.before_cycle(configs, simulated=False, context_hint={"confidence": 0.2})
            out_fail = learner.after_cycle(False, "forced failure", {"duration_s": 4.0, "confidence": 0.2})
            if out_fail.get("rolled_back"):
                rolled_back = True
                break
        if not rolled_back:
            raise RuntimeError("Learner smoke failed: rollback was not triggered.")

        # Context bucket check (at least low/high buckets should be created)
        learner.before_cycle(configs, simulated=False, context_hint={"confidence": 0.9})
        learner.after_cycle(True, "", {"duration_s": 1.2, "confidence": 0.9})
        st = learner.get_status()
        if int(st.get("context_count", 0)) < 2:
            raise RuntimeError("Learner smoke failed: context buckets were not created.")


def main() -> int:
    repo_root = REPO_ROOT
    _run_language_check(repo_root)
    _run_mock_simulation(repo_root, cycles=5)
    _run_learner_logic_check()
    print("Smoke checks passed: language(strict) + simulation(5/5) + learner(v3-context).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
