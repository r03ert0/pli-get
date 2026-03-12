"""Dummy camera for testing without hardware."""

import numpy as np
from camera_interface import CameraInterface, GrabResult


class DummyCamera(CameraInterface):
    '''Dummy camera used for testing when no real camera is connected.'''
    model = "DummyCamera"

    def open(self):
        pass

    def close(self):
        pass

    def set_color_mode(self, color_mode):
        print(f"DummyCamera: set_color_mode({color_mode})")

    def get_gain(self):
        return 0

    def set_gain(self, gain):
        print(f"DummyCamera: set_gain({gain})")

    def set_gain_online(self, gain):
        print(f"DummyCamera: set_gain_online({gain})")

    def get_exposure(self):
        return 0

    def set_exposure(self, exposure):
        print(f"DummyCamera: set_exposure({exposure})")

    def set_exposure_online(self, exposure):
        print(f"DummyCamera: set_exposure_online({exposure})")

    def get_gamma(self):
        return 1

    def set_gamma(self, gamma):
        print(f"DummyCamera: set_gamma({gamma})")

    def set_gamma_online(self, gamma):
        print(f"DummyCamera: set_gamma_online({gamma})")

    def grab_image(self, channel=1):
        return np.zeros((100, 100, 3), dtype=np.uint8)

    def start_grabbing(self):
        print("DummyCamera: start_grabbing")

    def retrieve_result(self):
        return GrabResult(np.zeros((100, 100), dtype=np.uint8))

    def stop_grabbing(self):
        print("DummyCamera: stop_grabbing")

    def max(self):
        return 255
