"""
pli acquisition
RT, 27 june 2023, 10h53
RT, VS, 14 july 2023, 21h29
RT, 24 july 2023, 22h31
RT, 29 may 2024, adapt to xypli
RT, 6 june 2024, implement multi-fov acquisition
RT, 12 march 2026, big cleanup
RT, 26 march 2026, refactor for multi-platform support (xypli + cerna)

use env py310
"""

import pathlib
import os
import asyncio
import threading
import argparse
import re
import json
import time
import numpy as np
from skimage import io
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
from datetime import datetime as dt
import code

from camera_interface import get_camera, get_cameras
from serial_utils import find_arduino_serial, DummySerial
from stage_interface import get_stage
from polariser_arduino import ArduinoPolariser, DummyPolariser
from platform_profiles import PROFILES


## PLI machine

class PLI:
    '''PLI machine class.

    Parameters:
        stage: StageInterface instance for XY motion
        polariser: PolariserInterface instance for polariser rotation
        cameras: list of CameraInterface instances
        pli_serial: serial connection for Arduino communication
    '''

    debug = 1

    def _init_state(self):
        '''Initialize mutable instance state.'''
        self.images = []
        self.channel = 1  # green, default image channel to capture

        self.status = "idle"  # possible values: idle, moving, ready, done
        self.motor_busy = [False, False, False, False, False]
        self._is_homing = False

        self.n_angles = None
        self.interactive_mode = False

        self.xy_roi_rect = (0, 0, 1, 1)
        self.xy_steps = (1, 1)
        self.polariser_delay = DEFAULT_POLARISER_DELAY_US  # microseconds between polariser steps
        self.settle_ms = 0        # pause after each polariser move, before the grab

    # PLI machine serial communication

    def process_pli_machine_message(self, message):
        '''Process a message received from the PLI machine.'''
        if self.debug: print("PLI>", message)
        if message == "ready":
            self.status = "ready"
            return
        if "Homing done" in message:
            self._is_homing = False
            self.status = "ready"
            return
        arr = re.split(r"\W+", message)
        while len(arr)>1 and arr[0] == "done":
            motor_number =  int(arr[1])
            self.motor_busy[motor_number] = False
            if not self._is_homing \
               and self.motor_busy[1] is False \
               and self.motor_busy[2] is False \
               and self.motor_busy[3] is False \
               and self.motor_busy[4] is False:
                self.status = "ready"
            arr = arr[2:]

    async def listen_to_pli_machine_messages(self, ser):
        '''Listen to messages from the PLI machine.'''
        while self.status != "done":
            if ser.in_waiting:
                message = ser.readline().decode().strip()
                if self.debug > 1:
                    print("MSG:", message)
                self.process_pli_machine_message(message)
            # Poll fast: every move waits on this loop to see "done: N", and a
            # 0.1 s poll added ~0.2 s to each of the thousands of moves in a run.
            await asyncio.sleep(0.005)

    async def echo_pli_machine_messages(self, ser):
        '''Echo PLI machine messages without processing them.'''
        while self.status != "done":
            if ser.in_waiting:
                message = ser.readline().decode().strip()
                print(f"PLI> {message}")
            await asyncio.sleep(0.005)

    async def wait_for_ready(self, timeout=30):
        '''Wait for the PLI machine to be ready.'''
        t0 = time.time()
        while self.status != "ready":
            if time.time() - t0 > timeout:
                raise TimeoutError(
                    f"wait_for_ready timed out after {timeout}s "
                    f"(status={self.status!r})")
            await asyncio.sleep(0.005)

    def write(self, string, sleep=0.01):
        '''Send raw commands to the PLI machine.
        Commands:
            delay n: delay between steps of the machine in ms.
              From 1 (fast) to 30 (slow) are reasonable values
            dir n{+, -}: if n is 1, 2 or 3, sets the direction of the x, y, z motors.
              If n is 4, turns the polariser motor
            i{+, -}n: i is the index of the motor (1 or 2),
              + and - indicate the direction of the motion, and
              n indicates the number of steps to rotate.
              For example, "1+100" rotates motor 1, ccw, 100 steps.
            wait n: pauses the execution of commands for n seconds.
              This command is not sent to the PLI machine but
              executed locally.
        '''
        bin_cmd = bytes(string, "ascii")
        if string.startswith(("delay", "dir", "enable", "disable")):
            bin_cmd += b"\r\n"
        else:
            bin_cmd += b"\n"
        self.pli_serial.write(bin_cmd)
        if sleep:
            time.sleep(sleep)

    ## Settings

    def set_n_angles(self, n_angles):
        '''Set the number of angles.'''
        self.n_angles = n_angles

    def set_xy_roi_rect(self, xy_roi_rect):
        '''Set the XY region of interest.'''
        self.xy_roi_rect = xy_roi_rect

    def set_xy_steps(self, xy_steps):
        '''Set the number of steps in the XY stage.'''
        self.xy_steps = xy_steps

    ## Acquisition functions

    def grab(self, image_name = "test.png"):
        '''Grab one image from each camera. Appends images to self.images.
        Returns:
            img: numpy array of the first camera's image
        '''
        first_img = None
        for cam_idx, cam in enumerate(self.cameras):
            img = cam.grab_image()
            if len(self.cameras) > 1:
                # multi-camera: include camera index in filename
                base, ext = os.path.splitext(image_name)
                name = f"{base}_cam{cam_idx}{ext}"
            else:
                name = image_name
            self.images.append([img, name])
            if first_img is None:
                first_img = img
        return first_img

    def save_all_images(self, check_contrast=False, raw=False):
        '''Save all images in Image_Array.'''
        print("saving images...", end=" ")
        _, computed_image_name = self.images[0]
        pathlib.Path(
            os.path.dirname(f"{computed_image_name}")
        ).mkdir(
            parents=True,
            exist_ok=True
        )

        for img, computed_image_name in self.images:
            if raw is False:
                io.imsave(f"{computed_image_name}", img, check_contrast=check_contrast)
            else:
                raw_image_name = computed_image_name.replace(".tif",f".{img.shape[0]}x{img.shape[1]}x{img.shape[2]}.{img.dtype}.raw")
                img.tofile(f"{raw_image_name}")
        print("all images saved.")

    ## Autocalibration

    def mean_value_image(self, img=None):
        '''Get the mean value of an image for calibration. If no image is passed, acquire a new one.'''
        if img is None:
            img = self.cameras[0].grab_image()[::10, ::10]
        else:
            img = img[::10, ::10]
        res = np.mean(img.ravel())
        return res

    async def calibrate(self, n_per_turn=72, n_turns=1, out_dir=None,
                        min_modulation=0.3, measure_only=False):
        '''Calibrate the polariser against a reference polariser sheet.

        Put a piece of polariser sheet in the slide holder first: it acts as a
        fixed analyser, so the transmitted intensity follows Malus' law as the
        machine's polariser turns. The polariser is swept forward through
        `n_turns` turns in `n_per_turn` steps per turn; each frame is averaged
        in memory (nothing is saved) and the curve is fitted by
        polariser_calibration.summarise(). That yields:

          - the measured steps per turn, to check the gearing in the profile;
          - extinction (crossed with the reference sheet), which becomes the
            polariser's zero -- the polariser is moved there, forward only so
            backlash is taken up, and the step counter reset;
          - any periodic drive-angle error, and its period. One motor
            revolution points at the pinion.

        One turn is enough: a two-turn sweep on xypli_large (2026-09-21) gave
        identical turns to within the noise, and one turn of 72 frames matched
        its extinction to 0.1 deg. Thinning to 36 frames per turn cost
        0.2-0.6 deg, so 72 is the floor for a uniform sweep.

        Returns the summary dict, or None if the modulation was too weak to
        trust (in which case the polariser is not moved to a new zero).
        '''
        import json
        import polariser_calibration as pc

        if self.pli_serial is None or len(self.cameras) == 0:
            print("Calibrate: no camera or machine")
            return None

        swt = self.polariser.steps_whole_turn
        motor_rev = getattr(self.polariser, "n_stepper_steps", None) or 6400
        n = n_per_turn * n_turns
        print(f"sweeping {n_turns} turns, {n_per_turn} frames per turn "
              f"({swt} steps per turn configured)")

        self.write(f"delay {self.polariser_delay}")
        self.write("enable")

        # Pre-roll a quarter turn forward before recording. It takes up slack
        # in the drive after the motors re-enable, and it keeps the sweep from
        # starting ON a dip: a dip split across the sweep's two ends was biased
        # by ~2 deg in testing. Starting from a calibrated zero, a quarter turn
        # also puts BOTH dips mid-sweep, so their agreement can be checked.
        pre = int(swt / 4)
        if self.polariser.rotate_to(pre):
            await self.wait_for_ready()

        steps, means = [], []
        cam = self.cameras[0]
        t_move = t_grab = 0.0
        t_start = time.time()
        for i in range(n):
            s = pre + int(i * swt / n_per_turn)
            t0 = time.time()
            if self.polariser.rotate_to(s):
                await self.wait_for_ready()
            t1 = time.time()
            img = cam.grab_image()
            t_grab += time.time() - t1
            t_move += t1 - t0
            h, w = img.shape[:2]
            core = img[h // 4:3 * h // 4:8, w // 4:3 * w // 4:8]
            if core.ndim == 2:
                core = core[..., None]
            means.append(core.reshape(-1, core.shape[-1]).mean(axis=0))
            steps.append(s)
            if i % n_per_turn == 0 or i == n - 1:
                print(f"  frame {i + 1}/{n}  step {s}  mean {means[-1].round(1)}")

        t_sweep = time.time() - t_start
        print(f"sweep took {t_sweep:.1f} s: {t_move / n:.2f} s/frame moving, "
              f"{t_grab / n:.2f} s/frame grabbing, "
              f"{(t_sweep - t_move - t_grab) / n:.2f} s/frame other")
        steps = np.array(steps, float)
        means = np.array(means, float)
        # green for RGB, the only channel for mono
        y = means[:, 1] if means.shape[1] >= 3 else means[:, 0]
        saturated = float(np.max(y)) > 0.97 * (255 if img.dtype == np.uint8 else 65535)

        r = pc.summarise(steps, y, swt, motor_rev)
        use_drive = r["rms_with_motor_period_error"] < 0.95 * r["rms_plain"]
        fit = r["fit_drive"] if use_drive else r["fit_plain"]

        print(f"""
steps per turn          : {swt:.0f} (from the gearing; used for all estimates)
free-fit check          : {r['measured_steps_per_turn']:.1f}  ({r['turn_error_pct']:+.3f} %; one sweep resolves ~+/-0.03 %)
modulation              : {r['modulation']:.2f}
fit RMS                 : {r['rms_plain']:.3f} plain, {r['rms_with_motor_period_error']:.3f} with a one-motor-turn angle error
angle error, motor turn : {r['motor_period_error_deg']:.3f} deg  (period {motor_rev} steps)
extinction              : {r['extinction_deg']:.2f} deg into each half-turn from the flanks \
(spread {r['extinction_spread_deg']:.2f} across levels; model minimum {r['extinction_model_deg']:.2f})
dips used               : {r['n_dips']}, agreeing to {r['dip_disagreement_deg']:.2f} deg
excluded samples        : {r['n_excluded']} (black-level clipped, plus 2 settle frames)""")
        if means.shape[1] >= 3:
            ext = [pc.channel_fit(steps, means[:, c], swt / 2.0)[1] for c in range(3)]
            print(f"extinction per channel  : red {ext[0]:.2f}, green {ext[1]:.2f}, "
                  f"blue {ext[2]:.2f} deg (green sets the zero)")
        if r["lost_steps_suspected"]:
            print(f"WARNING: the free-fit period is {r['turn_error_pct']:+.3f} % off the "
                  "gearing. A toothed drive can't slip, so the stepper may be missing "
                  "steps -- try a slower --polariser_delay.")
        if saturated:
            print("WARNING: the brightest frames are near saturation; "
                  "lower the exposure and repeat for an accurate fit.")

        ok = r["modulation"] >= min_modulation and not measure_only
        if measure_only:
            print("\nmeasure-only: the polariser zero was NOT changed")
        if ok and r["n_dips"] >= 2 and r["dip_disagreement_deg"] > 1.0:
            print(f"WARNING: the two dips disagree by {r['dip_disagreement_deg']:.2f} "
                  "deg; the sweep is not trustworthy. Zero NOT changed.")
            ok = False
        if ok:
            target = pc.next_extinction(fit, steps[-1])
            print(f"\nmoving forward to extinction at step {target}")
            if self.polariser.rotate_to(target):
                await self.wait_for_ready()
            self.polariser.reset()
            print("polariser zero set at extinction (crossed with the reference sheet)")
        if not ok:
            # No new zero: put the polariser back where the sweep started, i.e.
            # forward to the next whole turn, so the existing zero stays valid.
            # (A measure-only sweep once left it ~85 deg off for later runs.)
            back = int(np.ceil(self.polariser.current_step / swt) * swt)
            if self.polariser.rotate_to(back):
                await self.wait_for_ready()
            self.polariser.reset()
            print("polariser returned to its starting angle")
        if not ok and not measure_only:
            print(f"\nWARNING: modulation {r['modulation']:.2f} is below "
                  f"{min_modulation}. Is the reference polariser sheet in the "
                  "slide holder? The polariser zero was NOT changed.")

        if out_dir:
            out = pathlib.Path(out_dir)
            out.mkdir(parents=True, exist_ok=True)
            keep = {k: v for k, v in r.items()
                    if k not in ("scan", "fit_plain", "fit_drive")}
            keep.update(sweep_seconds=t_sweep, move_s_per_frame=t_move / n,
                        grab_s_per_frame=t_grab / n,
                        zero_set=bool(ok), used_drive_error_model=bool(use_drive),
                        saturated=saturated, polariser_delay_us=self.polariser_delay)
            (out / "calibration.json").write_text(json.dumps(keep, indent=1))
            np.savetxt(out / "sweep.csv",
                       np.column_stack([steps, means]), delimiter=",",
                       header="step," + ",".join(f"ch{c}" for c in range(means.shape[1])))
            self._plot_calibration(out / "calibration.png", steps, y, r, fit, means)
            print(f"results written to {out}")

        return r

    @staticmethod
    def _plot_calibration(path, steps, y, r, fit, means=None):
        import polariser_calibration as pc
        fig, ax = plt.subplots(3, 1, figsize=(10, 9))
        fine = np.linspace(steps.min(), steps.max(), 3000)
        if means is not None and means.shape[1] >= 3:
            # RGB: each channel with its own fit. Green is the one summarised
            # above and used for the zero.
            for c, (name, col) in enumerate([("red", "tab:red"), ("green", "tab:green"),
                                             ("blue", "tab:blue")]):
                ch_fit, ext = pc.channel_fit(steps, means[:, c], fit["P"])
                ax[0].plot(steps, means[:, c], ".", c=col, ms=4)
                if ch_fit is not None:
                    ax[0].plot(fine, pc.model(fine, ch_fit), "-", c=col, lw=1,
                               label=f"{name}: extinction {ext:.2f} deg")
            # headroom for a one-row legend above the curves
            ax[0].set_ylim(top=float(np.max(means[:, :3])) * 1.2)
            ax[0].legend(loc="upper center", ncol=3, fontsize=9)
        else:
            ax[0].plot(steps, y, ".", label="measured")
            ax[0].plot(fine, pc.model(fine, fit), "-", lw=1, label="fit")
            ax[0].legend()
        ax[0].set_xlabel("polariser step"); ax[0].set_ylabel("mean intensity")
        ax[1].plot(steps, y - pc.model(steps, r["fit_plain"]), ".-", lw=0.5,
                   label="residual, plain fit")
        ax[1].plot(steps, y - pc.model(steps, r["fit_drive"]), ".-", lw=0.5,
                   label="residual, with motor-turn angle error")
        ax[1].set_xlabel("polariser step"); ax[1].legend()
        if r["scan"]:
            pm, rms = zip(*r["scan"])
            ax[2].semilogx(pm, rms, "-")
        ax[2].axvline(r["motor_steps_per_rev"], ls="--", c="k", lw=0.8,
                      label="one motor revolution")
        ax[2].set_xlabel("angle-error period tested (steps)")
        ax[2].set_ylabel("fit RMS"); ax[2].legend()
        fig.tight_layout(); fig.savefig(path, dpi=100); plt.close(fig)

    async def interactive(self):
        '''Interactive mode for configuring exposure, gain and gamma.'''
        print("interactive mode")
        if len(self.cameras) == 0:
            print("Empty PLI and camera: Interactive mode")
            return

        time.sleep(1)
        print("set delay")
        self.write("delay 5")
        await self.wait_for_ready()

        time.sleep(1)
        print("enable the motors")
        self.write("enable")
        print("WAIT 2")
        await self.wait_for_ready()

        time.sleep(1)
        print("turn on the light")

        cam = self.cameras[0]  # interactive mode uses first camera

        self.interactive_mode = True
        plt.rcParams['toolbar'] = 'None'
        fig, ax_dict = plt.subplot_mosaic("AAB;AAB;AAB;AAB;AAC;AAD;AAE", figsize=(10, 5))

        def on_close(event):
            time.sleep(1)
            print("disable the motors")
            self.write("disable")

            print(f"--exposure {cam.get_exposure()} --gain {cam.get_gain()} --gamma {cam.get_gamma()}")
            self.interactive_mode = False
        fig.canvas.mpl_connect('close_event', on_close)

        fig.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.05, hspace=0.05)
        cam.close()
        cam.open()

        try:
            cam.set_color_mode("Mono8")
        except Exception as e:
            print(f"ERROR: cannot set pixel format: {e}")
            exit(1)

        cam.open()

        input_box1 = TextBox(ax_dict["C"], 'Exposure', initial=cam.get_exposure())
        def exposure(val):
            cam.set_exposure_online(float(val))
        input_box1.on_submit(exposure)

        input_box2 = TextBox(ax_dict["D"], 'Gain', initial=cam.get_gain())
        def gain(val):
            cam.set_gain_online(float(val))
        input_box2.on_submit(gain)

        input_box3 = TextBox(ax_dict["E"], 'Gamma', initial=cam.get_gamma())
        def gamma(val):
            cam.set_gamma(float(val))
        input_box3.on_submit(gamma)

        cam.start_grabbing()
        while self.interactive_mode is True:
            with cam.retrieve_result() as grab_result:
                img = grab_result.Array[::10, ::10]
                ax_dict["A"].clear()
                ax_dict["A"].imshow(img, cmap="gray")
                ax_dict["B"].clear()
                ax_dict["B"].hist(
                    img.flatten(),
                    bins=64,
                    range=(0, cam.max()),
                    density=True)
                ax_dict["B"].get_yaxis().set_visible(False)
            plt.pause(0.1)
        plt.close(fig)
        cam.stop_grabbing()
        cam.close()

    async def acquire(self, base_path, verbose=False):
        '''Acquire one set of polarised images for all angles.'''

        steps_whole_turn = self.polariser.steps_whole_turn
        if verbose: print("steps for a whole turn:", steps_whole_turn)

        # angles
        angle_step = steps_whole_turn/self.n_angles
        if verbose: print("steps in one angular displacement:", angle_step)

        self.images = []
        dark_array = []

        await self.wait_for_ready()
        # Inter-step delay for the polariser, in MICROSECONDS (the firmware
        # echoes e.g. "500us delay"). xypli_large turns 54400 steps per
        # revolution (170/20 gearing x 6400), so one angle is ~6000 steps; at
        # the default 5 us that is close to the stepper's limit and can
        # resonate, visibly shaking the polariser. Raise it if that happens --
        # it costs acquisition time and nothing else.
        self.write(f"delay {self.polariser_delay}")

        if verbose: print("rotate to home")
        if self.polariser.rotate_home():
            await self.wait_for_ready()

        if verbose: print("turn on the light")

        for iteration in range(self.n_angles):
            if verbose: print(f"angle: {iteration}")

            if self.polariser.rotate_to(int(iteration * angle_step)):
                await self.wait_for_ready()
                # Optional pause before grabbing: lets the polariser stop
                # ringing after the move (see --settle_ms).
                if self.settle_ms:
                    await asyncio.sleep(self.settle_ms / 1000)

            img = self.grab(f"{base_path}/{iteration}.tif")

            dark_array.append(self.mean_value_image(img))

        if verbose: print("move home")
        if self.polariser.rotate_home():
            await self.wait_for_ready()

        if verbose: print("turn off the light")

        if verbose: print(f"mean dark value: {np.mean(dark_array)}")
        if verbose: print(f"dark value range: {np.min(dark_array)} - {np.max(dark_array)}")

        print(dt.now())
        self.save_all_images(raw=False)
        print(dt.now())

    ## Constructor

    def __init__(self, stage, polariser, cameras, pli_serial):
        self._init_state()
        self.stage = stage
        self.polariser = polariser
        self.cameras = cameras if cameras is not None else []
        self.pli_serial = pli_serial
        print("")

# Utility
def pli_for_repl():
    '''Return a pli object with a parallel thread for getting
    and displaying its serial messages. Useful for using in interactive mode (REPL)
    To run async commands do, for example:
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(pli.stage.move_to(5.0, 6.0))
    '''

    def start_loop(loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()
    pli_serial = find_arduino_serial()
    stage = get_stage("dummy")
    polariser = DummyPolariser()
    pli = PLI(stage, polariser, [], pli_serial)
    new_loop = asyncio.new_event_loop()
    thread = threading.Thread(target=start_loop, args=(new_loop,), daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(pli.echo_pli_machine_messages(pli.pli_serial), new_loop)
    return pli

# Tasks

async def calibrate_task(pli):
    '''Calibrate the orientation of the PLI machine.'''

    print("=> Calibration worflow\n")

    await pli.calibrate(n_per_turn=pli.calib_samples, n_turns=pli.calib_turns,
                        out_dir=pli.calib_out, measure_only=pli.calib_measure_only)
    pli.write("disable")
    pli.status = "done" # this stops the listener task and ends the script

NO_REPLY_COMMANDS = {"enable", "disable", "status"}

# Inter-step delay (us) for polariser moves. At 5 us the xypli_large stepper
# drops ~0.3-0.45 % of its steps: alternating measure-only calibrations on
# 2026-09-26 gave a free-fit turn of -0.28/-0.30 % at 5 us against
# -0.04/-0.03 % at 50 us, and 50 us dips agreed ~10x better. It costs
# ~0.04 s per move.
DEFAULT_POLARISER_DELAY_US = 50

async def commands_task(pli, commands):
    '''Send raw commands to the PLI machine.'''

    print("=> Commands workflow\n", commands)

    await pli.wait_for_ready()

    for command in commands:
        command = command.strip()
        print(f"command: [{command}]")
        if command.startswith("wait"):
            secs = float(command.split(" ")[1])
            print(f"waiting for {secs} seconds")
            time.sleep(secs)
        else:
            base = command.split()[0]
            if base == "home":
                # The firmware reports `done: N` while backing off the
                # endstops; without this flag the first of those would mark the
                # machine ready long before "Homing done" arrives.
                pli._is_homing = True
                pli.status = "moving"
            elif base == "goto":
                # Status is still "ready" from the previous command, so without
                # this the wait below returned at once and the script could
                # exit mid-move. Mark both XY motors busy until their done: N.
                pli.status = "moving"
                pli.motor_busy[1] = pli.motor_busy[2] = True
            elif re.match(r"^\d[+-]\d+$", base):
                pli.status = "moving"
                pli.motor_busy[int(base[0])] = True
            pli.write(command)
            if base not in NO_REPLY_COMMANDS:
                await pli.wait_for_ready()
                print("check!")
        time.sleep(1)

    pli.status = "done"

async def interactive_task(pli):
    '''Interactive mode for setting exposure, gain and gamma.'''

    print("=> Interactive camera settings worflow\n")

    await pli.interactive()
    pli.status = "done" # this stops the listener task and ends the script

def grid_count(extent, pitch):
    '''Number of FOV positions spanning `extent` at spacing `pitch`.

    `extent` is the ROI width or height (may be negative; direction is handled
    by the caller). Both are in mm.

    The count is ceil(|extent| / pitch) + 1: the grid starts at the ROI origin
    and steps until it covers the whole extent, so a 10 mm ROI at 5 mm pitch
    needs 3 positions (0, 5, 10), not 2.

    The epsilon absorbs float error so an extent that is an exact multiple of
    the pitch does not gain a spurious extra column. It matters because
    svg_acquire rounds its coordinates, turning an exact 4.0 into 3.9994.
    Truncating that ratio instead of rounding it up used to drop the last row
    and column of every grid.
    '''
    return int(np.ceil(np.abs(extent) / pitch - 1e-9)) + 1


def roi_items(pli):
    '''(label, (x, y, w, h), (dx, dy)) for every ROI that is not skipped.

    An ROI given as "label:x,y,w,h@dx,dy" has its own pitch (svg_acquire uses
    it for grids squeezed inside the stage limits); the others use --xy_steps.
    '''
    for item in pli.xy_roi_rect:
        if item is None:
            continue
        label, rect = item[0], item[1]
        steps = item[2] if len(item) > 2 and item[2] is not None else pli.xy_steps
        yield label, rect, steps


def validate_acquisition_grid(pli):
    '''Validate that all FOV positions are within stage limits.
    Raises StageLimitsError if any position is out of bounds.
    Returns a summary of the grid for logging.
    '''
    from stage_interface import StageLimitsError
    all_positions = []

    for label, xy_roi_rect, xy_steps in roi_items(pli):

        n_cols = grid_count(xy_roi_rect[2], xy_steps[0])
        n_rows = grid_count(xy_roi_rect[3], xy_steps[1])

        x, y, w, h = xy_roi_rect
        dx, dy = 1, 1
        if w < 0: dx = -1
        if h < 0: dy = -1

        for j in range(n_rows):
            for i in range(n_cols):
                x_target = x + dx * i * xy_steps[0]
                y_target = y + dy * j * xy_steps[1]
                all_positions.append((x_target, y_target, label, j, i))

    # Check all positions against stage limits
    errors = []
    for x_target, y_target, label, row, col in all_positions:
        try:
            pli.stage.check_limits(x_target, y_target)
        except StageLimitsError as e:
            roi_str = f"roi {label} " if label else ""
            errors.append(f"  {roi_str}row {row}, col {col}: ({x_target:.3f}, {y_target:.3f}) mm — {e}")

    if errors:
        limits_str = ""
        if pli.stage.x_limit:
            limits_str += f"  X: [{pli.stage.x_limit[0]:.1f}, {pli.stage.x_limit[1]:.1f}] mm\n"
        if pli.stage.y_limit:
            limits_str += f"  Y: [{pli.stage.y_limit[0]:.1f}, {pli.stage.y_limit[1]:.1f}] mm\n"
        msg = (f"Acquisition grid has {len(errors)} position(s) outside stage limits:\n"
               + "\n".join(errors)
               + f"\n\nStage limits:\n{limits_str}"
               + "Aborting. Fix --xy_roi_rect or adjust limits in platform_profiles.py.")
        raise StageLimitsError(msg)

    # Print summary
    if not all_positions:
        raise StageLimitsError(
            "No acquisition positions. Every --xy_roi_rect was skipped "
            "(bracketed as [x,y,w,h]) or none was given.")

    x_vals = [p[0] for p in all_positions]
    y_vals = [p[1] for p in all_positions]
    print(f"Grid validation OK: {len(all_positions)} FOVs, "
          f"X range [{min(x_vals):.3f}, {max(x_vals):.3f}] mm, "
          f"Y range [{min(y_vals):.3f}, {max(y_vals):.3f}] mm")


async def acquire_task(pli, base_path):
    '''Acquire polarised images.'''

    print("=> Data acquisition workflow\n")

    time.sleep(0.1)

    print("enable the motors")
    pli.write("enable")

    pli.write("status")
    # Wait up to 5s for the machine to respond, then drain the buffer.
    # The boot "ready" arrives here if the machine reset on port open;
    # if already running, we still get the status dump — either way we proceed.
    t0 = time.time()
    while time.time() - t0 < 5:
        if pli.pli_serial.in_waiting:
            while pli.pli_serial.in_waiting:
                message = pli.pli_serial.readline().decode().strip()
                pli.process_pli_machine_message(message)
            if pli.status == "ready":
                break
        await asyncio.sleep(0.1)

    pli._is_homing = pli.stage.home_is_async
    pli.stage.home()

    if pli.stage.home_is_async:
        pli.status = "moving"
        await pli.wait_for_ready(timeout=120)

    for label, xy_roi_rect, xy_steps in roi_items(pli):

        n_cols = grid_count(xy_roi_rect[2], xy_steps[0])
        n_rows = grid_count(xy_roi_rect[3], xy_steps[1])

        # move to ROI origin
        x, y, w, h = xy_roi_rect
        dx, dy = 1, 1
        if w < 0: w, dx = -w, -1
        if h < 0: h, dy = -h, -1

        pli.write("status")
        while pli.pli_serial.in_waiting:
            message = pli.pli_serial.readline().decode().strip()
            pli.process_pli_machine_message(message)
            await asyncio.sleep(0.1)

        await pli.wait_for_ready()

        # Record where every FOV was, so tools like pli_preview can lay tiles
        # out from the actual motion instead of guessing from the fov_j_i
        # names. Rewritten after each FOV so an interrupted run keeps it.
        roi_dir = pathlib.Path(base_path) / (f"roi_{label}" if label else "")
        positions = {"platform": getattr(pli, "platform", None), "units": "mm",
                     "xy_steps": list(xy_steps), "fovs": {}}

        for j in range(n_rows):
            for i in range(n_cols):
                print(f"\nACQUIRING ROW: {j}, COL: {i}")

                pli.write("status")
                while pli.pli_serial.in_waiting:
                    message = pli.pli_serial.readline().decode().strip()
                    pli.process_pli_machine_message(message)
                    await asyncio.sleep(0.1)

                pli.write("delay 500", sleep=0)
                x_target = x + dx * i * xy_steps[0]
                y_target = y + dy * j * xy_steps[1]

                pli.stage.move_to(x_target, y_target)
                pli.status = "moving"
                pli.motor_busy[1] = True
                pli.motor_busy[2] = True
                await pli.wait_for_ready()

                pli.write("delay 5", sleep=0)
                if label:
                    await pli.acquire(f"{base_path}/roi_{label}/fov_{j}_{i}")
                else:
                    await pli.acquire(f"{base_path}/fov_{j}_{i}")

                positions["fovs"][f"fov_{j}_{i}"] = [round(x_target, 4),
                                                     round(y_target, 4)]
                roi_dir.mkdir(parents=True, exist_ok=True)
                with open(roi_dir / "positions.json", "w") as f:
                    json.dump(positions, f, indent=1)

    # move home
    pli.write("delay 500", sleep=0)
    await pli.wait_for_ready()

    print("disable the motors")
    pli.write("disable")
    time.sleep(0.1)

    # Quick-look preview. Done after the motors are off so the machine is idle
    # while it computes, and wrapped so that a preview failure can never cost
    # us the acquisition that just finished.
    preview = getattr(pli, "preview", None)
    if preview:
        mode, reduce_factor = preview
        try:
            from pli_preview import preview_run
            print(f"\n=> Generating {mode} preview")
            preview_run(base_path, mode=mode, reduce_factor=reduce_factor,
                        platform=getattr(pli, "platform", None))
        except Exception as e:
            print(f"WARNING: preview failed ({type(e).__name__}: {e})")
            print("         The acquired images are unaffected; run "
                  "pli_preview.py on the data directory to retry.")

    pli.status = "done" # this stops the listener task and ends the script


def create_pli(args):
    '''Create and configure a PLI instance from CLI arguments.

    Handles platform selection, serial detection, stage/polariser/camera creation.
    '''
    platform = getattr(args, 'platform', None)
    camera_server_url = getattr(args, 'camera_server', None)
    camera_ids = getattr(args, 'camera_ids', None)
    no_camera = getattr(args, 'no_camera', False)

    # Load profile defaults
    profile = PROFILES.get(platform, {}) if platform else {}

    # Cameras first: USB3 camera init can cause a brief USB bus reset that
    # disrupts any already-open CDC serial connection. Opening cameras before
    # the Arduino serial port ensures the bus is settled before we connect.
    # PLI_DUMMY_HARDWARE=1 forces the dummy camera and serial port even when
    # real hardware is attached. The test suite sets it: without it, running
    # the tests with the machine plugged in would home and drive the real
    # stage.
    dummy_hardware = os.environ.get("PLI_DUMMY_HARDWARE") == "1"

    cameras = []
    if dummy_hardware:
        from camera_dummy import DummyCamera
        cameras = [DummyCamera()]
    elif not no_camera:
        if camera_ids:
            ids = [c.strip() for c in camera_ids.split(",")]
            cameras = get_cameras(camera_server_url=camera_server_url, camera_ids=ids)
        else:
            cameras = get_cameras(camera_server_url=camera_server_url)

    # Serial connection (after cameras so USB bus is stable)
    known_ports = profile.get("known_arduino_ports", None)
    if dummy_hardware:
        print("PLI_DUMMY_HARDWARE=1: using dummy serial port")
        pli_serial = DummySerial()
    else:
        pli_serial = find_arduino_serial(known_ports)

    # Stage
    stage_type = profile.get("stage_type", "arduino")
    steps_per_mm = getattr(args, 'steps_per_mm', None) or profile.get("steps_per_mm", None)

    if stage_type == "arduino" and steps_per_mm is None and isinstance(pli_serial, DummySerial):
        stage_type = "dummy"
    elif stage_type == "arduino" and steps_per_mm is None:
        # For backward compat: if no steps_per_mm, use step units (1 step = 1 unit)
        steps_per_mm = 1.0

    x_limit = profile.get("x_limit", None)
    y_limit = profile.get("y_limit", None)
    stage = get_stage(stage_type, pli_serial=pli_serial, steps_per_mm=steps_per_mm,
                      x_limit=x_limit, y_limit=y_limit)

    # Polariser
    motor_ids = profile.get("polariser_motors", [4])
    directions = profile.get("polariser_directions", [1])
    n_stepper_steps = getattr(args, 'n_stepper_steps', None) or profile.get("n_stepper_steps", 800)
    n_large_gear_teeth = getattr(args, 'n_large_gear_teeth', None) or profile.get("n_large_gear_teeth", 96)
    n_small_gear_teeth = getattr(args, 'n_small_gear_teeth', None) or profile.get("n_small_gear_teeth", 42)

    if isinstance(pli_serial, DummySerial) and stage_type == "dummy":
        polariser = DummyPolariser(n_stepper_steps, n_large_gear_teeth, n_small_gear_teeth)
    else:
        def set_status(s):
            # will be set on pli after construction
            pass
        polariser = ArduinoPolariser(
            pli_serial, motor_ids, directions,
            n_stepper_steps, n_large_gear_teeth, n_small_gear_teeth,
            motor_busy=None, set_status=set_status
        )

    pli = PLI(stage, polariser, cameras, pli_serial)

    # Wire up polariser's motor_busy and set_status to the PLI instance
    if isinstance(polariser, ArduinoPolariser):
        polariser.motor_busy = pli.motor_busy
        polariser.set_status = lambda s: setattr(pli, 'status', s)

    # find_arduino_serial() already consumed the "ready" boot message, so
    # pre-set status to avoid commands_task hanging on wait_for_ready().
    if not isinstance(pli_serial, DummySerial):
        pli.status = "ready"

    return pli


def main(args):
    '''Main function.'''

    pli = None
    loop = None
    listener_task = None

    if args.interactive is False:
        pli = create_pli(args)
        pli.polariser_delay = getattr(args, "polariser_delay", DEFAULT_POLARISER_DELAY_US)
        pli.settle_ms = getattr(args, "settle_ms", 0) or 0
        pli.platform = getattr(args, "platform", None)
        pli.calib_samples = getattr(args, "calib_samples", 72)
        pli.calib_turns = getattr(args, "calib_turns", 1)
        pli.calib_measure_only = getattr(args, "calib_measure_only", False)
        pli.calib_out = getattr(args, "base_path", None) or \
            f"calibration_{dt.now():%Y%m%d_%H%M%S}"
        if args.calibrate:
            for cam in pli.cameras:
                if getattr(args, "color_mode", None): cam.set_color_mode(args.color_mode)
                if getattr(args, "gain", None) is not None: cam.set_gain(args.gain)
                if getattr(args, "exposure", None) is not None: cam.set_exposure(args.exposure)
                if getattr(args, "gamma", None) is not None: cam.set_gamma(args.gamma)
        loop = asyncio.get_event_loop()
        listener_task = loop.create_task(
            pli.listen_to_pli_machine_messages(pli.pli_serial))

    if args.calibrate is True:
        calibrate_commands_task = loop.create_task(calibrate_task(pli))
        tasks = [listener_task, calibrate_commands_task]
        # FIRST_EXCEPTION: if the calibration raises, stop the listener and
        # surface the error instead of waiting on it forever.
        done, pending = loop.run_until_complete(
            asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION))
        pli.status = "done"
        for p in pending:
            p.cancel()
        loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        for task in done:
            if not task.cancelled() and task.exception():
                raise task.exception()
        print("done calibrating")

    elif args.commands:
        commands = args.commands.split(";")
        raw_commands_task = loop.create_task(commands_task(pli, commands))
        tasks = [listener_task, raw_commands_task]
        loop.run_until_complete(asyncio.wait(tasks))
        print("done processing commands")

    elif args.interactive is True:
        banner = "xypli interactive"
        code.interact(banner=f"{'='*len(banner)}\nXypli interactive\n{'='*len(banner)}\n", local = globals())

    elif args.acquire is True:
        # required arguments
        if args.base_path is None:
            raise ValueError("base_path is required")
        if args.n_angles is None:
            raise ValueError("n_angles is required")
        if args.color_mode is None:
            raise ValueError("color_mode is required")
        if args.gain is None:
            raise ValueError("gain is required")
        if args.exposure is None:
            raise ValueError("exposure is required")
        if args.gamma is None:
            raise ValueError("gamma is required")
        base_path = args.base_path
        color_mode = args.color_mode
        gain = args.gain
        exposure = args.exposure
        gamma = args.gamma

        # optional arguments
        xy_roi_rect = [(None, (0, 0, 1, 1))]
        xy_steps = (1, 1)
        if args.xy_roi_rect is not None:
            xy_roi_rect = args.xy_roi_rect
        if args.xy_steps is not None:
            xy_steps = args.xy_steps

        # apply the settings
        pli.write("delay 5", sleep=0)
        pli.polariser.reset()

        pli.set_n_angles(args.n_angles)
        pli.set_xy_roi_rect(xy_roi_rect)
        pli.set_xy_steps(xy_steps)
        pli.preview = ((args.preview_mode, args.preview_reduce)
                       if getattr(args, "preview", False) else None)
        pli.polariser_delay = getattr(args, "polariser_delay", DEFAULT_POLARISER_DELAY_US)
        pli.platform = getattr(args, "platform", None)
        for cam in pli.cameras:
            cam.set_color_mode(color_mode)
            cam.set_gain(gain)
            cam.set_exposure(exposure)
            cam.set_gamma(gamma)

        # validate grid before starting async tasks
        try:
            validate_acquisition_grid(pli)
        except Exception as e:
            pli.status = "done"
            print(f"\nERROR: {e}")
            raise SystemExit(1)

        # start the acquisition
        acquisition_commands_task = loop.create_task(acquire_task(pli, base_path))
        tasks = [listener_task, acquisition_commands_task]
        try:
            done, pending = loop.run_until_complete(
                asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION))
            for p in pending:
                p.cancel()
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            for task in done:
                if not task.cancelled() and task.exception():
                    raise task.exception()
        finally:
            for cam in pli.cameras:
                try:
                    cam.close()
                except Exception:
                    pass

def rect_string(value):
    '''Parse a xy_roi_rect string
    The following are valid rect_strings:
    --xy_roi_rect=1,2,3,4, an ROI of coordinates x,y,w,h=1,2,3,4
    --xy_roi_rect=2:1,2,3,4, an ROI with label=2 and coordinates x,y,w,h=1,2,3,4. The label is added to the base_path where the images are saved.
    --xy_roi_rect=3:[1,2,3,4] an ROI with label=3 and coordinates that are ignored.
    --xy_roi_rect=4:1,2,3,4@0.5,0.5, an ROI with its own x,y distance between
        FOVs, overriding --xy_steps for this ROI only.
    '''

    if "[" in value:
        # ignore
        return

    label = None
    if ":" in value:
        # ROI with label
        label = value.split(":")[0]
        value = value.split(":")[1]

    steps = None
    if "@" in value:
        value, steps = value.split("@")
        steps = tuple(map(float, steps.split(',')))
        if len(steps) != 2 or min(steps) <= 0:
            raise argparse.ArgumentTypeError(f"bad ROI steps {steps!r}: need dx,dy > 0")

    rect = tuple(map(float, value.split(',')))

    return label, rect, steps

def floattuple(value):
    '''Convert a string to a tuple of floats.'''
    return tuple(map(float, value.split(',')))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Acquire PLI data.')

    # platform selection
    parser.add_argument('--platform', type=str, choices=list(PROFILES.keys()),
                        default=None, help='hardware platform profile')

    # automatic calibration of the polariser's angle
    parser.add_argument('--calibrate', action='store_true',
                        help='calibrate the polariser against a reference polariser '
                             'sheet placed in the slide holder: sweep, fit, and set '
                             'the polariser zero at extinction')
    parser.add_argument('--calib_samples', type=int, default=72,
                        help='frames per polariser turn during --calibrate (default 72; '
                             'fewer costs precision on the dip flanks)')
    parser.add_argument('--calib_measure_only', action='store_true',
                        help='sweep and analyse only; never move to or reset the '
                             'polariser zero (for diagnostics without the sheet)')
    parser.add_argument('--calib_turns', type=int, default=1,
                        help='polariser turns swept during --calibrate (default 1)')

    # interactive interface to set exposure, gain and gamma
    parser.add_argument('--interactive', '-i', action='store_true', help='start interactive mode for camera configuration')

    # acquire images
    parser.add_argument('--acquire', action='store_true', help='acquire an image')
    parser.add_argument('--base_path', type=str, help='base path for saving the images')
    parser.add_argument('--n_angles', type=int, help='number of angles to acquire')
    parser.add_argument('--n_polarisers', type=int, help='number of polarisers in the machine (1 or 2)')
    parser.add_argument('--n_stepper_steps', type=int, help='number of steps in a whole turn of the stepper motors')
    parser.add_argument('--n_large_gear_teeth', type=int, help='number of teeth in the large gear of the machine')
    parser.add_argument('--n_small_gear_teeth', type=int, help='number of teeth in the small stepper gear')
    parser.add_argument('--color_mode', type=str, help='camera color mode')
    parser.add_argument('--gain', type=float, help='camera gain')
    parser.add_argument('--exposure', type=float, help='camera exposure time')
    parser.add_argument('--gamma', type=float, help='camera gamma')
    parser.add_argument('--xy_roi_rect', type=rect_string, action='append', help='[label:]x,y,width,height[@dx,dy] of the ROI in mm (default: 0,0,1,1); '
                             '@dx,dy overrides --xy_steps for this ROI')
    parser.add_argument('--xy_steps', type=floattuple, help='x,y distance in mm between FOVs (default: 1,1)')

    parser.add_argument('--polariser_delay', type=int, default=DEFAULT_POLARISER_DELAY_US,
                        help='inter-step delay in MICROSECONDS while rotating '
                             'the polariser; below ~50 the xypli_large stepper '
                             f'misses steps (default {DEFAULT_POLARISER_DELAY_US})')

    parser.add_argument('--settle_ms', type=float, default=0,
                        help='pause (ms) after each polariser move before grabbing, '
                             'to let the polariser stop ringing (default 0)')

    # quick-look preview after an acquisition
    parser.add_argument('--preview', action='store_true',
                        help='generate a retardation/transmittance preview '
                             'mosaic after the acquisition')
    parser.add_argument('--preview_mode', choices=['retardation', 'transmittance'],
                        default='retardation',
                        help='preview type (default retardation; transmittance '
                             'skips the model fit and is cheaper)')
    parser.add_argument('--preview_reduce', type=int, default=8,
                        help='downsample factor for the preview (default 8)')

    # send raw commands
    parser.add_argument('--commands', type=str, help='raw commands to send to the PLI machine, separated by a semi-colon')

    # phone camera server
    parser.add_argument('--camera_server', type=str, default=None,
                        help='URL of a remote camera server (e.g. https://192.168.1.42:5000)')

    # multi-camera and stage
    parser.add_argument('--camera_ids', type=str, default=None,
                        help='comma-separated camera IDs for multi-camera acquisition')
    parser.add_argument('--steps_per_mm', type=float, default=None,
                        help='conversion factor from mm to steps for Arduino stage')

    main(parser.parse_args())

#-----------------------------------------------------------------------------
# IMPORTANT: To use this, comment out the main(parser.parse_args()) line above
#-----------------------------------------------------------------------------
# my_dict = {
#     "calibrate": False,
#     "interactive": False,
#     "commands": False,
#     "acquire": True,
#     "platform": "xypli_small",
#     "base_path": "/Users/roberto/Desktop/xypli-test",
#     "n_angles": 9*2, # 36*2,
#     "n_polarisers": 1,
#     "n_stepper_steps": 200 * 2 * 16, # 800
#     "n_large_gear_teeth": 96, # 112,
#     "n_small_gear_teeth": 42,
#     "color_mode": "RGB8", # "Mono8",
#     "gain": 10, # 0,
#     "exposure": 11000, # 11000,
#     "gamma": 1,
#     "xy_roi_rect": [rect_string("1:19.569,33.197,-5.080,-5.080")],  # mm
#     "xy_steps": (5.08, 5.08),  # mm
#     "camera_server": None,
#     "camera_ids": None,
#     "steps_per_mm": None,
# }
# args = argparse.Namespace(**my_dict)
# main(args)
#-----------------------------------------------------------------------------

# python pli_get.py --platform xypli_large --acquire --base_path /Users/roberto/Desktop/test-xypli --n_angles 9 --color_mode Mono8 --gain 0 --exposure 11000 --gamma 1 --xy_roi_rect 0,0,10,10 --xy_steps 5,5

# python pli_get.py --platform cerna --acquire --base_path "/Users/roberto/Desktop/" --n_angles 72 --color_mode Mono12 --gain 0 --exposure 5000 --gamma 1 --xy_steps 4.06,4.06 --xy_roi_rect 27.625,22.897,11.369,10.706
