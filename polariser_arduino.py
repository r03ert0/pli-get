"""Arduino-based polariser controller for PLI acquisition.

Controls one or two polariser motors via Arduino serial commands.
Supports both single-polariser (XYPLI) and dual-polariser (Cerna) configurations.
"""

import time
from polariser_interface import PolariserInterface


class ArduinoPolariser(PolariserInterface):
    """Polariser rotation via Arduino stepper motors.

    Parameters:
        pli_serial: serial connection to the Arduino
        motor_ids: list of motor indices (e.g. [4] for XYPLI, [1, 2] for Cerna)
        directions: list of direction multipliers per motor (e.g. [1] or [1, -1])
        n_stepper_steps: steps per full revolution of the stepper motor
        n_large_gear_teeth: teeth on the large (polariser) gear
        n_small_gear_teeth: teeth on the small (stepper) gear
        motor_busy: shared motor_busy array from PLI (for status tracking)
        set_status: callback to set PLI status (e.g. "moving")
    """

    def __init__(self, pli_serial, motor_ids, directions,
                 n_stepper_steps, n_large_gear_teeth, n_small_gear_teeth,
                 motor_busy=None, set_status=None):
        self.pli_serial = pli_serial
        self.motor_ids = motor_ids
        self.directions = directions
        self.n_stepper_steps = n_stepper_steps
        self.n_large_gear_teeth = n_large_gear_teeth
        self.n_small_gear_teeth = n_small_gear_teeth
        self.motor_busy = motor_busy
        self.set_status = set_status
        self._current_step = 0

    @property
    def steps_whole_turn(self) -> float:
        return self.n_large_gear_teeth / self.n_small_gear_teeth * self.n_stepper_steps

    @property
    def current_step(self) -> int:
        return self._current_step

    def _write(self, string, sleep=0.01):
        '''Send a command to the Arduino.'''
        bin_cmd = bytes(string, "ascii")
        bin_cmd += b"\n"
        self.pli_serial.write(bin_cmd)
        if sleep:
            time.sleep(sleep)

    def _rotate_motor(self, motor_id, delta):
        '''Rotate a single motor by delta steps.'''
        direction = "+" if delta >= 0 else "-"
        self._write(f"{motor_id}{direction}{abs(delta)}\n")
        if self.motor_busy is not None:
            self.motor_busy[motor_id] = True
        if self.set_status is not None:
            self.set_status("moving")

    def rotate_to(self, step: int) -> bool:
        '''Rotate to an absolute step position. Returns True if movement was sent.'''
        delta = round(step - self._current_step)
        if delta == 0:
            return False
        # Primary motor (first in list) tracks the step count
        self._rotate_motor(self.motor_ids[0], self.directions[0] * delta)
        self._current_step += delta
        # Additional motors (dual-polariser setup)
        for i in range(1, len(self.motor_ids)):
            self._rotate_motor(self.motor_ids[i], self.directions[i] * delta)
        return True

    def rotate_home(self) -> bool:
        '''Rotate to home, taking the shortest path around. Returns True if movement was sent.'''
        swt = self.steps_whole_turn
        if self._current_step > swt / 2:
            moved = self.rotate_to(int(swt))
            self._current_step = self._current_step - int(swt)
            return moved
        else:
            return self.rotate_to(0)

    def reset(self) -> None:
        '''Reset the step counter to 0 without moving.'''
        self._current_step = 0


class DummyPolariser(PolariserInterface):
    """Dummy polariser for testing without hardware."""

    def __init__(self, n_stepper_steps=800,
                 n_large_gear_teeth=96, n_small_gear_teeth=42):
        self.n_stepper_steps = n_stepper_steps
        self.n_large_gear_teeth = n_large_gear_teeth
        self.n_small_gear_teeth = n_small_gear_teeth
        self._current_step = 0

    @property
    def steps_whole_turn(self) -> float:
        return self.n_large_gear_teeth / self.n_small_gear_teeth * self.n_stepper_steps

    @property
    def current_step(self) -> int:
        return self._current_step

    def rotate_to(self, step: int) -> bool:
        moved = (step != self._current_step)
        self._current_step = step
        return moved

    def rotate_home(self) -> bool:
        moved = (self._current_step != 0)
        self._current_step = 0
        return moved

    def reset(self) -> None:
        self._current_step = 0
