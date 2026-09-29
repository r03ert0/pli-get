"""Platform profiles for known PLI hardware configurations.

Each profile provides default values for a specific microscope setup.
CLI arguments override profile defaults when provided.
"""

PROFILES = {
    "xypli_small": {
        "stage_type": "arduino",
        "polariser_motors": [4],
        "polariser_directions": [1],
        "n_polarisers": 1,
        "n_stepper_steps": 800,
        "n_large_gear_teeth": 96,
        "n_small_gear_teeth": 42,
        "known_arduino_ports": ["AB0LS12X", "cu.usbmodem", "ttyUSB0"],
        "steps_per_mm": None,  # needs calibration for mm mode
        "x_limit": None,  # (min_mm, max_mm) — set after calibration
        "y_limit": None,
    },
    "xypli_large": {
        "stage_type": "arduino",
        "polariser_motors": [4],
        "polariser_directions": [1],
        "n_polarisers": 1,
        "n_stepper_steps": 6400,
        "n_large_gear_teeth": 170,
        "n_small_gear_teeth": 20,
        "known_arduino_ports": ["AB0LS12X", "cu.usbmodem", "ttyUSB0"],
        "steps_per_mm": 320,
        # How stage axes appear in the camera image: (image right, image down).
        # Measured 2026-09-21 from the overlap of neighbouring FOVs: moving the
        # stage to -y shifts the view to the right, to -x shifts it upward.
        # Fixed by the camera mounting; re-measure if the camera is re-seated.
        "camera_image_axes": ("-y", "+x"),
        "x_limit": (0, 40.625),   # mm, measure and adjust
        "y_limit": (0, 58), # 53.125),   # mm, measure and adjust
    },
    "cerna": {
        "stage_type": "thorlabs",
        "polariser_motors": [1, 2],
        "polariser_directions": [1, 1],
        "n_polarisers": 2,
        "n_stepper_steps": 6400,
        "n_large_gear_teeth": 96,
        "n_small_gear_teeth": 42,
        "known_arduino_ports": ["COM10"],
        "steps_per_mm": None,  # Thorlabs uses mm natively
        "x_limit": (0, 110),  # MLS203 travel range
        "y_limit": (0, 75),
    },
}
