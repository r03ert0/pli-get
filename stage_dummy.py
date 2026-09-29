"""Dummy stage backend for testing without hardware."""

from stage_interface import StageInterface


class DummyStage(StageInterface):
    """Dummy XY stage that tracks position in memory."""

    def __init__(self):
        self.x_mm = 0.0
        self.y_mm = 0.0
        self._homed = False

    def connect(self) -> None:
        pass

    def home(self) -> None:
        self.x_mm = 0.0
        self.y_mm = 0.0
        self._homed = True

    def move_to(self, x_mm: float, y_mm: float) -> None:
        self.x_mm = x_mm
        self.y_mm = y_mm

    def get_position(self) -> tuple:
        return (self.x_mm, self.y_mm)

    def is_homed(self) -> bool:
        return self._homed

    def close(self) -> None:
        pass
