# Allied Vision camera

import numpy as np
import vmbpy
from camera_interface import CameraInterface, GrabResult

vmb = vmbpy.VmbSystem.get_instance()


class Camera(CameraInterface):
    '''Camera interface for Allied Vision cameras (via vmbpy).'''
    model = None
    camera = None

    def __init__(self, camera_id=None):
        print("\nLook for Allied Vision camera...")
        with vmb:
            cams = vmb.get_all_cameras()
            if camera_id is not None:
                matching = [c for c in cams if c.get_id() == camera_id]
                if not matching:
                    available = [c.get_id() for c in cams]
                    raise RuntimeError(f"Camera {camera_id} not found. Available: {available}")
                self.camera = matching[0]
            else:
                self.camera = cams[0]
        self.camera_id = camera_id
        self.model = self.camera.get_model()
        print(f"Got Allied Vision camera: {self.camera.get_name()}, model: {self.model}")
        self.open()

    def open(self):
        pass

    def close(self):
        pass

    def set_color_mode(self, color_mode):
        self.camera.set_pixel_format(color_mode)

    def get_gain(self):
        return self.camera.get_gain()

    def set_gain(self, gain):
        self.camera.set_gain(gain)

    def set_gain_online(self, gain):
        self.camera.set_gain(gain)

    def get_exposure(self):
        return self.camera.get_exposure()

    def set_exposure(self, exposure):
        self.camera.set_exposure(exposure)

    def set_exposure_online(self, exposure):
        self.camera.set_exposure(exposure)

    def get_gamma(self):
        return self.camera.get_gamma()

    def set_gamma(self, gamma):
        self.camera.set_gamma(gamma)

    def set_gamma_online(self, gamma):
        self.camera.set_gamma(gamma)

    def grab_image(self, channel=1):
        with vmb:
            with self.camera:
                frame = self.camera.get_frame(timeout_ms=1000)
                img = frame.as_numpy_ndarray()
        return img

    def start_grabbing(self):
        raise NotImplementedError("Continuous grabbing not yet implemented for Allied Vision cameras")

    def retrieve_result(self):
        raise NotImplementedError("Continuous grabbing not yet implemented for Allied Vision cameras")

    def stop_grabbing(self):
        raise NotImplementedError("Continuous grabbing not yet implemented for Allied Vision cameras")

    def max(self):
        # vmbpy doesn't expose PixelDynamicRangeMax directly;
        # return the max value based on pixel format bit depth
        fmt = str(self.camera.get_pixel_format())
        if "12" in fmt:
            return 4095
        return 255
