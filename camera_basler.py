# Basler camera

import time

import numpy as np
from pypylon import pylon
from camera_interface import CameraInterface

GRAB_MAX_ATTEMPTS = 4       # 1 initial try + 3 retries
GRAB_RETRY_SLEEP_S = 0.05   # 50 ms


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
        self.fix_white_balance()

    def open(self):
        self.camera.Open()

    def fix_white_balance(self, ratios=(1.0, 1.0, 1.0)):
        '''Switch off automatic white balance and fix the balance ratios.

        Colour Basler cameras ship with BalanceWhiteAuto = Continuous. During an
        acquisition that loop re-estimates the red/blue gains from each frame,
        so it swings them as the polariser turns and hunts between two states
        across acquisitions: red and blue varied ~10x more than the polariser
        signal itself, alternating from one FOV to the next (docs/report-7).
        Ratios of 1.0 give the raw sensor response. Mono models have no white
        balance; this is then a no-op.
        '''
        was_open = self.camera.IsOpen()
        if not was_open:
            self.camera.Open()
        try:
            if not hasattr(self.camera, "BalanceWhiteAuto"):
                return
            try:
                self.camera.BalanceWhiteAuto.SetValue("Off")
            except Exception:
                return          # feature not available in this pixel format
            for channel, value in zip(("Red", "Green", "Blue"), ratios):
                self.camera.BalanceRatioSelector.SetValue(channel)
                self.camera.BalanceRatio.SetValue(value)
        finally:
            if not was_open:
                self.camera.Close()

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
        # The pixel format can re-enable auto functions; pin the balance again.
        self.fix_white_balance()

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
        self.camera.Close()

    def set_gamma_online(self, gamma):
        self.camera.Gamma.SetValue(gamma)

    def grab_image(self, channel=1):
        if not self.camera.IsOpen():
            self.camera.Open()

        last_error = "unknown error"
        for attempt in range(1, GRAB_MAX_ATTEMPTS + 1):
            grab_result = None
            try:
                grab_result = self.camera.GrabOne(5000)
                if grab_result.GrabSucceeded():
                    # Copy the array before Release(): grab_result.Array is a
                    # view onto the driver's buffer, which Release() returns
                    # to the pool for reuse. Returning it uncopied would risk
                    # handing the caller a buffer that gets overwritten by a
                    # subsequent grab.
                    return grab_result.GetArray().copy()
                error_code = grab_result.GetErrorCode()
                error_desc = grab_result.GetErrorDescription()
                last_error = f"0x{error_code:08x}: {error_desc}"
                print(
                    f"Basler grab failed (attempt {attempt}/{GRAB_MAX_ATTEMPTS}): "
                    f"error code {last_error}"
                )
            except (pylon.GenericException, RuntimeError) as e:
                last_error = str(e)
                print(
                    f"Basler grab raised an exception "
                    f"(attempt {attempt}/{GRAB_MAX_ATTEMPTS}): {last_error}"
                )
            finally:
                if grab_result is not None:
                    grab_result.Release()

            if attempt < GRAB_MAX_ATTEMPTS:
                time.sleep(GRAB_RETRY_SLEEP_S)

        raise RuntimeError(
            f"Basler grab failed after {GRAB_MAX_ATTEMPTS} attempts: {last_error}"
        )

    def start_grabbing(self):
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    def retrieve_result(self):
        return self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    def stop_grabbing(self):
        self.camera.StopGrabbing()

    def max(self):
        return self.camera.PixelDynamicRangeMax.Value
