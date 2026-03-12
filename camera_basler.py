# Basler camera

import numpy as np
from pypylon import pylon
from camera_interface import CameraInterface


class Camera(CameraInterface):
    '''Camera interface for Basler cameras (via pypylon).'''
    model = None
    camera = None

    def __init__(self):
        print("\nLook for Basler camera...")
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.model = self.camera.GetDeviceInfo().GetModelName()
        print("Got Basler camera:", self.model)
        self.open()

    def open(self):
        self.camera.Open()

    def close(self):
        self.camera.Close()

    def set_color_mode(self, color_mode):
        self.camera.Open()
        if color_mode == "RGB8":
            self.camera.PixelFormat.SetValue("RGB8")
        elif color_mode == "Mono8":
            self.camera.PixelFormat.SetValue("Mono8")
        elif color_mode == "Mono12":
            self.camera.PixelFormat.SetValue("Mono12")
        else:
            raise ValueError("color_mode must be RGB8 or Mono8 or Mono12")
        self.camera.Close()

    def get_gain(self):
        return self.camera.Gain.Value

    def set_gain(self, gain):
        self.camera.Open()
        self.camera.GainAuto.SetValue("Off")
        self.camera.Gain.SetValue(gain)
        self.camera.Close()

    def set_gain_online(self, gain):
        self.camera.GainAuto.SetValue("Off")
        self.camera.Gain.SetValue(gain)

    def get_exposure(self):
        return self.camera.ExposureTime.Value

    def set_exposure(self, exposure):
        self.camera.Open()
        self.camera.ExposureAuto.SetValue("Off")
        self.camera.ExposureTime.SetValue(exposure)
        self.camera.Close()

    def set_exposure_online(self, exposure):
        self.camera.ExposureAuto.SetValue("Off")
        self.camera.ExposureTime.SetValue(exposure)

    def get_gamma(self):
        return self.camera.Gamma.Value

    def set_gamma(self, gamma):
        self.camera.Open()
        self.camera.Gamma.SetValue(gamma)

    def set_gamma_online(self, gamma):
        self.camera.Gamma.SetValue(gamma)

    def grab_image(self, channel=1):
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        grab_result = self.camera.RetrieveResult(500, pylon.TimeoutHandling_ThrowException)
        img = None
        if grab_result.GrabSucceeded():
            img = grab_result.Array[:, :]
        grab_result.Release()
        self.camera.StopGrabbing()
        return img

    def start_grabbing(self):
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    def retrieve_result(self):
        return self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    def stop_grabbing(self):
        self.camera.StopGrabbing()

    def max(self):
        return self.camera.PixelDynamicRangeMax.Value
