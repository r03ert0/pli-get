# Adding a new stage backend

This guide explains how to add support for a new XY stage to pli_get.

## 1. Create a new file

Create `stage_<name>.py` in the `pli_get/` directory (e.g. `stage_marzhauser.py`).

## 2. Implement StageInterface

Your class must inherit from `StageInterface` and implement all abstract methods:

```python
from stage_interface import StageInterface

class MarzhauserStage(StageInterface):
    """Example: Marzhauser SCAN stage backend."""

    def __init__(self, port=None):
        # Store configuration, don't connect yet
        self.port = port
        self._homed = False

    def connect(self) -> None:
        """Open the connection to the stage controller."""
        # Connect to your hardware here

    def home(self) -> None:
        """Home the stage."""
        # Send home/reference command
        self._homed = True

    def move_to(self, x_mm: float, y_mm: float) -> None:
        """Move to an absolute position in millimeters."""
        # Convert mm to your controller's units and send move command

    def get_position(self) -> tuple:
        """Return (x_mm, y_mm) in millimeters."""
        # Read position from controller and convert to mm
        return (x_mm, y_mm)

    def is_homed(self) -> bool:
        return self._homed

    def close(self) -> None:
        """Close the connection."""
        # Cleanup
```

### Key points

- All positions are in **millimeters**. Convert to/from your controller's native units internally.
- `move_to()` should block until the move is complete (or at minimum until the command is sent, if the PLI async loop handles waiting).
- `connect()` should handle auto-detection if possible.
- `close()` should be safe to call multiple times.

## 3. Register in the factory

Add your backend to `get_stage()` in `stage_interface.py`:

```python
elif stage_type == "marzhauser":
    from stage_marzhauser import MarzhauserStage
    return MarzhauserStage(port=kwargs.get("port"))
```

## 4. Add a platform profile (optional)

If this stage is part of a specific microscope setup, add a profile to `platform_profiles.py`:

```python
"my_microscope": {
    "stage_type": "marzhauser",
    "polariser_motors": [4],
    ...
}
```

## Existing backends

| Backend | File | Hardware | Units |
|---------|------|----------|-------|
| `ArduinoStage` | `stage_arduino.py` | Arduino CNC shield (core-XY) | steps (converted via steps_per_mm) |
| `ThorlabsStage` | `stage_thorlabs.py` | Thorlabs MLS203 (Kinesis) | mm natively (factor 20000) |
| `DummyStage` | `stage_dummy.py` | No hardware (testing) | mm (in memory) |
