# Remote camera server backend
#
# Connects to a camera server (e.g. samsung-s21-camera/server.py) via its
# HTTP API, which must already be running with the device connected.
#
# Requirements (in addition to the main project):
#   Pillow

import base64
import io
import json
import ssl
import urllib.request

import numpy as np
from PIL import Image

from camera_interface import CameraInterface, GrabResult

# Shared SSL context that skips certificate verification (self-signed certs)
_SSL_CTX = ssl.create_default_context()
_SSL_CTX.check_hostname = False
_SSL_CTX.verify_mode = ssl.CERT_NONE


class Camera(CameraInterface):
    """Camera interface for a remote device via the camera server HTTP API.

    Communicates with the camera server (e.g. samsung-s21-camera/server.py)
    which relays commands to the device and returns images.
    """

    model = "Remote Camera"

    def __init__(self, server_url="https://localhost:5000", timeout=10):
        """Connect to a running camera server.

        Args:
            server_url: URL of the server (e.g. "https://192.168.1.42:5000").
            timeout: Seconds to wait for each server call.
        """
        print(f"\nLook for remote camera at {server_url} ...")
        self._url = server_url.rstrip("/")
        self._timeout = timeout

        # Internal state
        self._color_mode = "RGB8"
        self._gain = 0.0
        self._exposure = 0.0
        self._gamma = 1.0
        self._grabbing = False

        # Verify the phone camera is connected
        status = self._get("/api/status")
        if not status.get("connected"):
            raise RuntimeError(
                "Server is reachable but no phone camera is connected. "
                "Open the server URL on the phone and start the camera first."
            )

        print("Got remote camera (via server)")

        # Read initial settings
        settings = status.get("settings", {})
        self._exposure = settings.get("exposureTime", 0.0)
        self._gain = settings.get("iso", 0.0)

    # ------------------------------------------------------------------
    # HTTP helpers
    # ------------------------------------------------------------------

    def _get(self, path, timeout=None):
        """HTTP GET and return parsed JSON."""
        t = timeout or self._timeout
        req = urllib.request.Request(self._url + path)
        with urllib.request.urlopen(req, timeout=t, context=_SSL_CTX) as resp:
            return json.loads(resp.read())

    def _post(self, path, data=None, timeout=None):
        """HTTP POST JSON and return parsed JSON.  Raises on server error."""
        t = timeout or self._timeout
        body = json.dumps(data or {}).encode()
        req = urllib.request.Request(
            self._url + path, data=body,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=t, context=_SSL_CTX) as resp:
            result = json.loads(resp.read())
        if not result.get("ok"):
            raise RuntimeError(result.get("error", "unknown server error"))
        return result

    def _dataurl_to_array(self, data_url):
        """Decode a base64 data-URL JPEG into a numpy array."""
        img_data = base64.b64decode(data_url.split(",", 1)[1])
        img = Image.open(io.BytesIO(img_data))

        if self._color_mode == "RGB8":
            arr = np.array(img.convert("RGB"), dtype=np.uint8)
        elif self._color_mode == "Mono8":
            arr = np.array(img.convert("L"), dtype=np.uint8)
        elif self._color_mode == "Mono12":
            # Phone captures 8-bit JPEG; scale to 12-bit range
            arr = np.array(img.convert("L"), dtype=np.uint16) * 16
        else:
            arr = np.array(img.convert("RGB"), dtype=np.uint8)

        # Apply gamma correction in post-processing
        if self._gamma != 1.0 and self._gamma > 0:
            max_val = self.max()
            arr = np.clip(
                np.power(arr.astype(np.float64) / max_val, 1.0 / self._gamma)
                * max_val,
                0, max_val,
            ).astype(arr.dtype)

        return arr

    # ------------------------------------------------------------------
    # CameraInterface implementation
    # ------------------------------------------------------------------

    def open(self):
        pass  # HTTP is stateless; nothing to open

    def close(self):
        self._grabbing = False

    def set_color_mode(self, color_mode):
        if color_mode not in ("RGB8", "Mono8", "Mono12"):
            raise ValueError("color_mode must be RGB8, Mono8, or Mono12")
        self._color_mode = color_mode

    # -- gain (mapped to ISO) --

    def get_gain(self):
        return self._gain

    def set_gain(self, gain):
        self._gain = gain
        self._post("/api/set_settings", {"iso": float(gain)})

    def set_gain_online(self, gain):
        self.set_gain(gain)

    # -- exposure --

    def get_exposure(self):
        return self._exposure

    def set_exposure(self, exposure):
        self._exposure = exposure
        self._post("/api/set_settings", {
            "exposureMode": "manual",
            "exposureTime": float(exposure),
        })

    def set_exposure_online(self, exposure):
        self.set_exposure(exposure)

    # -- gamma (post-processing only) --

    def get_gamma(self):
        return self._gamma

    def set_gamma(self, gamma):
        self._gamma = gamma

    def set_gamma_online(self, gamma):
        self._gamma = gamma

    # -- image capture --

    def grab_image(self, channel=1):
        result = self._post("/api/take_photo", timeout=20)
        return self._dataurl_to_array(result["dataUrl"])

    def start_grabbing(self):
        self._grabbing = True

    def retrieve_result(self):
        if not self._grabbing:
            raise RuntimeError("Not currently grabbing")
        result = self._post("/api/take_photo", timeout=20)
        return GrabResult(self._dataurl_to_array(result["dataUrl"]))

    def stop_grabbing(self):
        self._grabbing = False

    def max(self):
        if self._color_mode == "Mono12":
            return 4095
        return 255

    # -- phone-specific extras (not part of CameraInterface) --

    def set_zoom(self, level):
        return self._post("/api/set_settings", {"zoom": float(level)})

    def set_focus(self, mode="manual", distance=None):
        payload = {"focusMode": mode}
        if distance is not None:
            payload["focusDistance"] = float(distance)
        return self._post("/api/set_settings", payload)

    def set_white_balance(self, mode="manual", temperature=None):
        payload = {"whiteBalanceMode": mode}
        if temperature is not None:
            payload["colorTemperature"] = float(temperature)
        return self._post("/api/set_settings", payload)

    def set_torch(self, enabled=True):
        return self._post("/api/set_settings", {"torch": bool(enabled)})

    def get_capabilities(self):
        result = self._get("/api/get_capabilities")
        return result.get("capabilities", {})

    def get_settings(self):
        result = self._get("/api/get_settings")
        return result.get("settings", {})
