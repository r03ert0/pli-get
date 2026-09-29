"""Thorlabs MLS203 motorized stage backend for PLI acquisition.

Uses pylablib to control the Thorlabs Kinesis-based XY stage
used in the Cerna microscope setup.

Requires: pylablib (pip install pylablib)
"""

from stage_interface import StageInterface

# Conversion factor: 20000 internal steps = 1 mm
FACTOR = 20000


class ThorlabsStage(StageInterface):
    """XY stage controlled via Thorlabs Kinesis (MLS203).

    Parameters:
        device_id: Thorlabs device serial number string.
            If None, auto-detects the first available Kinesis device.
    """

    def __init__(self, device_id=None):
        self.device_id = device_id
        self.stage = None
        self._homed = False

    def connect(self) -> None:
        '''Connect to the Thorlabs stage.'''
        from pylablib.devices import Thorlabs

        if self.device_id is None:
            devices = Thorlabs.list_kinesis_devices(filter_ids=False)
            if not devices:
                raise RuntimeError("No Thorlabs Kinesis devices found")
            self.device_id = devices[0][0]
            print(f"Auto-detected Thorlabs device: {self.device_id}")

        self.stage = Thorlabs.KinesisMotor(
            self.device_id, scale="stage", is_rack_system=True
        )
        print(f"Connected to Thorlabs stage: {self.device_id}")

        # Check if already homed
        try:
            status = self.stage.get_full_status()
            if status['status'][1] == 'homed':
                self._homed = True
        except Exception:
            pass

    def home(self) -> None:
        '''Home both axes (channel 1 = X, channel 2 = Y).

        Note: axes need to be enabled via the controller box first.
        Channel 1 must be homed before channel 2.
        '''
        if self.stage is None:
            self.connect()
        print("Homing Thorlabs stage...")
        self.stage.home(channel=1)
        self.stage.wait_move(channel=1)
        self.stage.home(channel=2)
        self.stage.wait_move(channel=2)
        self._homed = True
        print("Thorlabs stage homed.")

    def move_to(self, x_mm: float, y_mm: float) -> None:
        '''Move to an absolute position in mm.'''
        if self.stage is None:
            self.connect()

        x_pos, y_pos = self.get_position()
        print(f"stage move from ({x_pos:.3f}, {y_pos:.3f}) to ({x_mm:.3f}, {y_mm:.3f}) mm")

        # Convert mm to internal units
        x_steps = x_mm * FACTOR
        y_steps = y_mm * FACTOR
        self.stage.move_to(x_steps, channel=1)
        self.stage.move_to(y_steps, channel=2)
        self.stage.wait_move(channel=1)
        self.stage.wait_move(channel=2)

    def get_position(self) -> tuple:
        '''Return the current (x_mm, y_mm) position.'''
        if self.stage is None:
            return (0.0, 0.0)
        x = self.stage.get_position(channel=1) / FACTOR
        y = self.stage.get_position(channel=2) / FACTOR
        return (x, y)

    def is_homed(self) -> bool:
        '''Return True if the stage has been homed.'''
        return self._homed

    def close(self) -> None:
        '''Close the connection to the stage.'''
        if self.stage is not None:
            self.stage.close()
            self.stage = None
            print("Thorlabs stage closed.")
