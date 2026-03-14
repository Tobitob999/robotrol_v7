class PickPlaceError(Exception):
    pass


class PerceptionError(PickPlaceError):
    pass


class PlanningError(PickPlaceError):
    pass


class ControlError(PickPlaceError):
    pass
