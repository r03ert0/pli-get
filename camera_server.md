# Remote Camera Server for PLI Acquisition

Use a phone (or any device with a browser and camera) as a camera for the
PLI microscope. The setup has two independent parts:

1. **Server** (e.g. `samsung-s21-camera/server.py`) — runs on the computer,
   serves a web page to the device, and relays commands between the device
   and any Python client.
2. **PLI camera backend** (`pli_get/camera_server.py`) — connects to
   the server over SocketIO and implements the standard `CameraInterface`.

## Prerequisites

Python packages (in addition to the main project dependencies):

```bash
# For the server
pip install flask flask-socketio python-socketio Pillow

# For the PLI camera backend (SocketIO client)
pip install "python-socketio[client]" websocket-client Pillow
```

The device and the computer must be on the **same local network** (Wi-Fi).

## Step-by-step Usage

### 1. Start the server

In one terminal:

```bash
cd samsung-s21-camera
python server.py                  # HTTPS on port 5000 (default)
python server.py --port 8080      # custom port
python server.py --no-cli         # headless (no interactive CLI)
```

The server prints a URL like `https://192.168.1.42:5000`.

### 2. Connect the device

1. Open the server URL on the device's browser (Chrome or Samsung Internet
   recommended).
2. Accept the self-signed certificate warning.
3. Tap **Start Camera** and grant camera permissions.
4. The server prints `[+] Camera ready!`.

You can use the server's interactive CLI to verify the connection and
adjust settings before starting the PLI acquisition:

```
cam> status
cam> photo
cam> zoom 2.0
cam> focus manual 0.5
```

### 3. Run pli_get with the remote camera

#### Direct instantiation

```python
from camera_server import Camera

cam = Camera(server_url="https://192.168.1.42:5000")
img = cam.grab_image()   # numpy array
```

| Parameter    | Default                     | Description                                |
|--------------|-----------------------------|--------------------------------------------|
| `server_url` | `https://localhost:5000`    | URL of the running server                  |
| `timeout`    | 10                          | Seconds to wait for each server call       |

#### Via `get_camera()` factory

Pass the server URL to enable remote camera detection:

```python
from camera_interface import get_camera

cam = get_camera(camera_server_url="https://192.168.1.42:5000")
```

If no Basler or Allied Vision camera is found, it will connect to the
camera server. Without the URL argument, the remote camera is skipped.

#### Via `pli_get.py`

```bash
# Interactive mode
python pli_get.py --interactive --camera_server https://192.168.1.42:5000

# Acquisition
python pli_get.py --acquire --camera_server https://192.168.1.42:5000 \
  --base_path ./data --n_angles 36 \
  --n_polarisers 1 --n_stepper_steps 800 \
  --n_large_gear_teeth 96 --n_small_gear_teeth 42 \
  --color_mode Mono8 --gain 100 --exposure 10000 --gamma 1.0
```

## Settings Mapping

| PLI setting      | Device control           | Notes                                    |
|------------------|--------------------------|------------------------------------------|
| `gain`           | ISO sensitivity          | Passed to the device as ISO              |
| `exposure`       | Exposure time (us)       | Manual exposure mode, microseconds       |
| `gamma`          | Post-processing          | Applied in software after capture        |
| `color_mode`     | JPEG conversion          | Device captures JPEG; converted to RGB8, Mono8, or Mono12 |

### Color modes

- **RGB8** — 3-channel uint8 numpy array.
- **Mono8** — single-channel uint8 (luminance conversion).
- **Mono12** — single-channel uint16, values in 0–4095. Since the device
  captures 8-bit JPEGs, the 8-bit values are scaled to 12-bit range.

## Device-specific Controls

Beyond the standard `CameraInterface`, extra methods are available:

```python
cam = Camera(server_url="https://192.168.1.42:5000")

cam.set_zoom(2.0)
cam.set_focus(mode="manual", distance=0.5)
cam.set_white_balance(mode="manual", temperature=5500)
cam.set_torch(True)

print(cam.get_capabilities())   # supported controls and ranges
print(cam.get_settings())       # current camera settings
```

## Integration with pli_get.py

The `--camera_server` argument is already wired into `pli_get.py`.
No additional code changes are needed.

## Architecture

```
 Device (browser)          Server (computer)           PLI script
 ┌──────────────┐         ┌──────────────────┐        ┌─────────────────┐
 │  index.html  │◄──SIO──►│   server.py      │◄──SIO──│ camera_server   │
 │  camera API  │         │  (Flask+SocketIO) │        │  .py            │
 │  (JS)        │         │                  │        │ (CameraInterface)│
 └──────────────┘         └──────────────────┘        └─────────────────┘
     device WiFi              port 5000                  pli_get.py
```

- The **device** captures images and applies hardware settings via the
  browser's MediaDevices / ImageCapture API.
- The **server** relays `pli_*` events from the PLI client to the device
  and returns results as SocketIO acknowledgements.
- The **PLI backend** is a pure SocketIO client — it does not import or
  start the server.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `ConnectionError` from `camera_server.py` | Make sure `server.py` is running and the URL is correct. |
| "Server is reachable but no phone camera is connected" | Open the URL on the device, tap Start Camera, grant permissions. |
| Device can't open the URL | Check both devices are on the same Wi-Fi. Try disabling firewall for the server port. |
| Certificate warning on the device | Normal for self-signed certs. Tap Advanced → Proceed. |
| Slow capture speed | Each `grab_image()` is a full-resolution photo over WiFi (~1–3s). Lower resolution with `cam.set_resolution()` via the server CLI. |
| Device disconnects mid-acquisition | Keep the screen on and the browser tab in the foreground. Disable auto-lock. |
| Controls not available | Depends on the browser. Chrome and Samsung Internet expose the most. Check `cam.get_capabilities()`. |

## Limitations

- **Capture speed**: each image is a full photo transferred over Wi-Fi
  as JPEG (~1–3 seconds per frame).
- **Bit depth**: 8-bit JPEG source. Mono12 scales values but doesn't add
  true dynamic range.
- **Gamma**: post-processing only, not a hardware sensor setting.
- **Continuous grabbing**: `start_grabbing()`/`retrieve_result()` take
  sequential photos; no true hardware streaming.
- **Device must stay awake**: browser tab active, screen on.
- **Server must be started separately**: the PLI backend does not start
  or manage the server process.
