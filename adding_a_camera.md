# Adding a new camera

This document describes how to add support for a new camera to the PLI acquisition system.

## Overview

All cameras are accessed through `camera_interface.py`, which defines:
- `CameraInterface` — an abstract base class that every camera backend must implement
- `GrabResult` — a helper class for wrapping continuous grab results
- `get_camera()` — a factory function that auto-detects the connected camera

`pli_get.py` never imports a specific camera module. It calls `get_camera()`, which tries each backend in order and returns the first one that succeeds, falling back to `DummyCamera`.

## Steps

### 1. Create a new camera module

Create a file named `camera_<vendor>.py` (e.g. `camera_flir.py`). Import and inherit from `CameraInterface`:

```python
from camera_interface import CameraInterface, GrabResult

class Camera(CameraInterface):
    model = None

    def __init__(self):
        # Connect to the camera. If no camera of this type is found,
        # let the exception propagate — the factory will catch it
        # and try the next backend.
        ...
        self.model = ...  # set the model name for identification
```

The class must be named `Camera` and placed at module level.

### 2. Implement the interface methods

All methods from `CameraInterface` must be implemented:

| Method | Purpose |
|--------|---------|
| `open()` | Open the camera connection |
| `close()` | Close the camera connection |
| `set_color_mode(color_mode)` | Set the pixel format (`"RGB8"`, `"Mono8"`, `"Mono12"`) |
| `get_gain()` / `set_gain(gain)` | Read/write gain (camera is opened/closed internally) |
| `set_gain_online(gain)` | Set gain while the camera is already grabbing |
| `get_exposure()` / `set_exposure(exposure)` | Read/write exposure time |
| `set_exposure_online(exposure)` | Set exposure while the camera is already grabbing |
| `get_gamma()` / `set_gamma(gamma)` | Read/write gamma |
| `set_gamma_online(gamma)` | Set gamma while the camera is already grabbing |
| `grab_image(channel=1)` | Grab a single image, return a numpy array |
| `start_grabbing()` | Start continuous grabbing (used by interactive live view) |
| `retrieve_result()` | Return a `GrabResult` (context manager with `.Array` attribute) |
| `stop_grabbing()` | Stop continuous grabbing |
| `max()` | Return the maximum pixel value for the current format (e.g. 255 for 8-bit) |

Notes:
- `grab_image()` should handle the full open/grab/close cycle internally and return a numpy array.
- `start_grabbing()` / `retrieve_result()` / `stop_grabbing()` are used for the interactive live view (`--interactive` mode). If the camera SDK doesn't support continuous streaming, you can raise `NotImplementedError` — the camera will still work for acquisition.
- The `*_online` methods set parameters while the camera is already open and grabbing. If the SDK doesn't distinguish between online and offline parameter changes, they can simply call the regular setter.
- `retrieve_result()` must return a context manager with an `.Array` attribute containing a numpy array. Use the `GrabResult` helper from `camera_interface`:

```python
def retrieve_result(self):
    frame = ...  # get a frame from the SDK
    return GrabResult(frame_as_numpy_array)
```

### 3. Register the backend in the factory

Edit `camera_interface.py` and add a new `try/except` block in `get_camera()`:

```python
def get_camera():
    # Try Basler
    try:
        from camera_basler import Camera as BaslerCamera
        cam = BaslerCamera()
        return cam
    except Exception:
        pass

    # Try Allied Vision
    try:
        from camera_allied_vision import Camera as AlliedVisionCamera
        cam = AlliedVisionCamera()
        return cam
    except Exception:
        pass

    # Try FLIR  <-- add your new backend here
    try:
        from camera_flir import Camera as FlirCamera
        cam = FlirCamera()
        return cam
    except Exception:
        pass

    # Fallback to dummy
    from camera_dummy import DummyCamera
    print("No camera found: using dummy camera.")
    return DummyCamera()
```

The detection order matters: cameras are tried top to bottom, and the first successful connection wins. Place your backend at the appropriate position.

### 4. Reference: existing backends

- `camera_basler.py` — complete implementation using `pypylon`, good reference for a fully working backend
- `camera_allied_vision.py` — implementation using `vmbpy`, streaming methods not yet implemented
- `camera_dummy.py` — minimal no-op implementation, useful as a starting template
