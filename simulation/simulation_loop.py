import time

from simulation.mock_world import MockWorld


class SimulationRunner:
    def __init__(self, pipeline, perception_cfg: dict):
        self.pipeline = pipeline
        self.world = MockWorld(perception_cfg.get("mock_pose", {}))

    def run(self, cycles: int, per_cycle=None, before_cycle=None):
        successes = 0
        perception = self.pipeline.context.perception
        original_mode = getattr(perception, "mode", None)
        try:
            if hasattr(perception, "mode"):
                perception.mode = "mock"
            for i in range(int(cycles)):
                if before_cycle is not None:
                    try:
                        before_cycle(i)
                    except Exception:
                        pass
                _T, pos, quat = self.world.pose_for_cycle(i)
                self.pipeline.context.perception.set_mock_pose(pos, quat, 0.99)
                t0 = time.time()
                ok = self.pipeline.run_cycle()
                dt = time.time() - t0
                if ok:
                    successes += 1
                if per_cycle is not None:
                    try:
                        err = self.pipeline.state_machine.last_error
                        conf = None
                        obj_pose = getattr(self.pipeline.context, "object_pose", None)
                        if obj_pose is not None and hasattr(obj_pose, "confidence"):
                            conf = float(obj_pose.confidence)
                        per_cycle(i, ok, err, {"duration_s": dt, "confidence": conf})
                    except Exception:
                        pass
        finally:
            if hasattr(perception, "mode"):
                perception.mode = original_mode
        return successes
