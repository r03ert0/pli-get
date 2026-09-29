"""Stage interface for PLI acquisition.

Defines the abstract stage interface and a factory function
that creates the appropriate stage backend.
"""

from abc import ABC, abstractmethod


class StageLimitsError(Exception):
    """Raised when a move would exceed stage limits."""
    pass


class StageInterface(ABC):
    """Abstract base class for all PLI XY stage backends.

    All positions are in millimeters.
    Subclasses can set x_limit and y_limit to enforce soft limits.
    """

    # Soft limits in mm. None means no limit.
    x_limit = None  # (min_mm, max_mm) or None
    y_limit = None  # (min_mm, max_mm) or None

    # True if home() sends a command asynchronously and the caller must wait
    # for a "Homing done" confirmation before proceeding.
    home_is_async: bool = False

    def check_limits(self, x_mm: float, y_mm: float) -> None:
        '''Raise StageLimitsError if (x_mm, y_mm) is outside soft limits.'''
        if self.x_limit is not None:
            if x_mm < self.x_limit[0] or x_mm > self.x_limit[1]:
                raise StageLimitsError(
                    f"X={x_mm:.3f} mm is outside limits [{self.x_limit[0]:.1f}, {self.x_limit[1]:.1f}] mm")
        if self.y_limit is not None:
            if y_mm < self.y_limit[0] or y_mm > self.y_limit[1]:
                raise StageLimitsError(
                    f"Y={y_mm:.3f} mm is outside limits [{self.y_limit[0]:.1f}, {self.y_limit[1]:.1f}] mm")

    @abstractmethod
    def connect(self) -> None:
        '''Open the connection to the stage controller.'''

    @abstractmethod
    def home(self) -> None:
        '''Home the stage (move to origin / reference position).'''

    @abstractmethod
    def move_to(self, x_mm: float, y_mm: float) -> None:
        '''Move the stage to an absolute position in mm.'''

    @abstractmethod
    def get_position(self) -> tuple:
        '''Return the current (x_mm, y_mm) position.'''

    @abstractmethod
    def is_homed(self) -> bool:
        '''Return True if the stage has been homed.'''

    @abstractmethod
    def close(self) -> None:
        '''Close the connection to the stage controller.'''


def get_stage(stage_type, pli_serial=None, steps_per_mm=None, **kwargs):
    """Factory function to create the appropriate stage backend.

    Parameters:
        stage_type: "arduino", "thorlabs", or "dummy"
        pli_serial: serial connection (required for arduino)
        steps_per_mm: conversion factor (required for arduino)
        **kwargs: additional backend-specific parameters

    Returns:
        StageInterface instance
    """
    x_limit = kwargs.get("x_limit")
    y_limit = kwargs.get("y_limit")

    if stage_type == "arduino":
        from stage_arduino import ArduinoStage
        if pli_serial is None:
            raise ValueError("pli_serial is required for arduino stage")
        if steps_per_mm is None:
            raise ValueError("steps_per_mm is required for arduino stage")
        return ArduinoStage(pli_serial, steps_per_mm,
                            x_limit=x_limit, y_limit=y_limit)

    elif stage_type == "thorlabs":
        from stage_thorlabs import ThorlabsStage
        device_id = kwargs.get("device_id")
        stage = ThorlabsStage(device_id=device_id)
        stage.x_limit = x_limit
        stage.y_limit = y_limit
        return stage

    else:
        from stage_dummy import DummyStage
        return DummyStage()
