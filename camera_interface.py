"""Camera interface for PLI acquisition.

Defines the abstract camera interface and a factory function
that auto-detects the connected camera.
"""

from abc import ABC, abstractmethod
import numpy as np


class GrabResult:
    """Wrapper for continuous grab results, usable as a context manager.
    Provides a .Array attribute containing the image as a numpy array."""
    def __init__(self, array):
        self.Array = array
    def __enter__(self):
        return self
    def __exit__(self, *args):
        pass


class CameraInterface(ABC):
    """Abstract base class for all PLI camera backends."""
    model = None

    @abstractmethod
    def open(self):
        '''Open the camera connection.'''

    @abstractmethod
    def close(self):
        '''Close the camera connection.'''

    @abstractmethod
    def set_color_mode(self, color_mode):
        '''Set the pixel format. Valid values: "RGB8", "Mono8", "Mono12".'''

    @abstractmethod
    def get_gain(self):
        '''Return the current gain value.'''

    @abstractmethod
    def set_gain(self, gain):
        '''Set the gain (camera is opened/closed internally).'''

    @abstractmethod
    def set_gain_online(self, gain):
        '''Set the gain while the camera is already open/grabbing.'''

    @abstractmethod
    def get_exposure(self):
        '''Return the current exposure time.'''

    @abstractmethod
    def set_exposure(self, exposure):
        '''Set the exposure time (camera is opened/closed internally).'''

    @abstractmethod
    def set_exposure_online(self, exposure):
        '''Set the exposure time while the camera is already open/grabbing.'''

    @abstractmethod
    def get_gamma(self):
        '''Return the current gamma value.'''

    @abstractmethod
    def set_gamma(self, gamma):
        '''Set the gamma value.'''

    @abstractmethod
    def set_gamma_online(self, gamma):
        '''Set the gamma value while the camera is already open/grabbing.'''

    @abstractmethod
    def grab_image(self, channel=1):
        '''Grab a single image. Returns a numpy array.'''

    @abstractmethod
    def start_grabbing(self):
        '''Start continuous grabbing (for interactive/live view).'''

    @abstractmethod
    def retrieve_result(self):
        '''Retrieve one frame during continuous grabbing.
        Returns a GrabResult context manager with a .Array attribute.'''

    @abstractmethod
    def stop_grabbing(self):
        '''Stop continuous grabbing.'''

    @abstractmethod
    def max(self):
        '''Return the maximum pixel value for the current pixel format.'''


def get_camera():
    """Auto-detect and return an available camera.
    Tries Basler, then Allied Vision, then falls back to DummyCamera."""

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

    # Fallback to dummy
    from camera_dummy import DummyCamera
    print("No camera found: using dummy camera.")
    return DummyCamera()
