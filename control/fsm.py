from enum import Enum
from control.errors import PickPlaceError


class State(str, Enum):
    IDLE = "IDLE"
    DETECT = "DETECT"
    PLAN_GRASP = "PLAN_GRASP"
    EXECUTE_PICK = "EXECUTE_PICK"
    PLAN_PLACE = "PLAN_PLACE"
    EXECUTE_PLACE = "EXECUTE_PLACE"
    SUCCESS = "SUCCESS"
    SAFE_STOP = "SAFE_STOP"


class PickPlaceStateMachine:
    def __init__(self):
        self.state = State.IDLE
        self.last_error = None

    def run_cycle(self, context) -> bool:
        self.state = State.IDLE
        self.last_error = None

        while True:
            if self.state == State.IDLE:
                self.state = State.DETECT
                continue
            if self.state == State.DETECT:
                if not self._call(context.detect):
                    return self._safe_stop(context)
                self.state = State.PLAN_GRASP
                continue
            if self.state == State.PLAN_GRASP:
                if not self._call(context.plan_grasp):
                    return self._safe_stop(context)
                self.state = State.EXECUTE_PICK
                continue
            if self.state == State.EXECUTE_PICK:
                if not self._call(context.execute_pick):
                    return self._safe_stop(context)
                self.state = State.PLAN_PLACE
                continue
            if self.state == State.PLAN_PLACE:
                if not self._call(context.plan_place):
                    return self._safe_stop(context)
                self.state = State.EXECUTE_PLACE
                continue
            if self.state == State.EXECUTE_PLACE:
                if not self._call(context.execute_place):
                    return self._safe_stop(context)
                self.state = State.SUCCESS
                continue
            if self.state == State.SUCCESS:
                return True
            if self.state == State.SAFE_STOP:
                return self._safe_stop(context)

    def _call(self, fn):
        try:
            fn()
            return True
        except PickPlaceError as exc:
            self.last_error = exc
            return False
        except Exception as exc:
            self.last_error = exc
            return False

    def _safe_stop(self, context):
        try:
            context.safe_stop()
        finally:
            self.state = State.IDLE
        return False
