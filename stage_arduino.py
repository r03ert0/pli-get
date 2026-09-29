"""Arduino-based XY stage backend for PLI acquisition.

Wraps the Arduino serial protocol's goto/home commands
for the core-XY stage used in XYPLI machines.
"""

import time
from stage_interface import StageInterface


class ArduinoStage(StageInterface):
    """XY stage controlled via Arduino CNC shield (core-XY kinematics).

    The Arduino firmware handles core-XY kinematics internally.
    This backend sends goto/home commands and tracks position.

    Parameters:
        pli_serial: serial connection to the Arduino
        steps_per_mm: conversion factor from mm to stepper steps
    """

    home_is_async: bool = True  # home() sends serial command; wait for "> Homing done."

    def __init__(self, pli_serial, steps_per_mm, x_limit=None, y_limit=None):
        self.pli_serial = pli_serial
        self.steps_per_mm = steps_per_mm
        self.x_limit = x_limit  # (min_mm, max_mm) or None
        self.y_limit = y_limit  # (min_mm, max_mm) or None
        self.x_mm = 0.0
        self.y_mm = 0.0
        self._homed = False

    def _write(self, string, sleep=0.01):
        '''Send a command to the Arduino.'''
        bin_cmd = bytes(string, "ascii")
        if string.startswith(("delay", "dir", "enable", "disable")):
            bin_cmd += b"\r\n"
        else:
            bin_cmd += b"\n"
        self.pli_serial.write(bin_cmd)
        if sleep:
            time.sleep(sleep)

    def connect(self) -> None:
        '''No-op: connection is managed by pli_serial.'''
        pass

    def home(self) -> None:
        '''Send the home command to the Arduino.'''
        self._write("home")
        self.x_mm = 0.0
        self.y_mm = 0.0
        self._homed = True

    def move_to(self, x_mm: float, y_mm: float) -> None:
        '''Move to an absolute position in mm. Checks soft limits first.'''
        self.check_limits(x_mm, y_mm)
        # round rather than truncate: int() biases every move toward zero by up
        # to one step, which accumulates into a systematic offset across a grid.
        x_steps = int(round(x_mm * self.steps_per_mm))
        y_steps = int(round(y_mm * self.steps_per_mm))
        print(f"stage move from ({self.x_mm:.3f}, {self.y_mm:.3f}) to ({x_mm:.3f}, {y_mm:.3f}) mm")
        self._write(f"goto x{x_steps} y{y_steps}")
        self.x_mm = x_mm
        self.y_mm = y_mm

    def get_position(self) -> tuple:
        '''Return the tracked (x_mm, y_mm) position.'''
        return (self.x_mm, self.y_mm)

    def is_homed(self) -> bool:
        '''Return True if the stage has been homed.'''
        return self._homed

    def close(self) -> None:
        '''No-op: serial connection lifecycle is managed externally.'''
        pass
