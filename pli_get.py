"""
pli acquisition
RT, 27 june 2023, 10h53
RT, VS, 14 july 2023, 21h29
RT, 24 july 2023, 22h31
RT, 29 may 2024, adapt to xypli
RT, 6 june 2024, implement multi-fov acquisition

use env py39
"""

import pathlib
import os
import asyncio
import threading
import argparse
import re
import time
import numpy as np
from skimage import io
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
from datetime import datetime as dt
import code

from camera_interface import get_camera


## PLI machine

class DummySerial:
    '''Dummy serial port for testing.'''
    in_waiting = True
    message = "ready"
    prev_command = None
    def __init__(self):
        pass
    def write(self, bytes_data, verbose=False):
        '''Write data to the serial port.'''
        message = "ready"
        data = bytes_data.decode('utf8').strip()
        if data.startswith(("status")):
            message = f"""\
> steppers: enabled
> currentSteps: 0,0,0,0
> xy: 0,0
> x endstop: off
> y endstop: off
> delay: 500
> idle time: 0
"""
            self.prev_command = "status"
        elif data.startswith(("goto")):
            cmds = data.split("\n")
            message = "done: 1\ndone: 2\n"
        elif data.startswith(("delay", "dir", "enable", "disable", "home")) is False:
            cmds = data.split("\n")
            message = ""
            for cmd in cmds:
                motor_index = int(cmd[0])
                message += f"done: {motor_index}\n"
        if verbose: print(f"DummySerial > {bytes_data.strip()}")
        self.message = message
        self.in_waiting = True
    def readline(self):
        '''Read a line from the serial port.'''
        tmp = self.message
        if self.prev_command == "status":
            self.message = "ready"
            self.in_waiting = True
            self.prev_command = None
        elif self.message == "ready":
            self.in_waiting = False
            self.message = ""
        else:
            # After "done: N" messages, follow up with "ready"
            self.message = "ready"
            self.in_waiting = True
        return tmp.encode(encoding='utf-8')

class PLI:
    '''PLI machine class.'''

    pli_serial = None # serial connection to the PLI machine
    camera = None
    xy_stage_serial = None # XY stage

    images = [] # array to store all images
    channel = 1 # green, default image channel to capture

    polariser_steps = 0 # current number of steps in the PLI machine
    x_steps = 0 # current number of steps in X direction of the XY stage
    y_steps = 0 # current number of steps in Y direction of the XY stage

    status = "idle" # possible values: idle, moving, ready, done

    #----------------------------
    # control assignments

    # One polariser pli
    # motor_polariser_top = 1

    # Two polarisers pli
    # motor_polariser_top = 1
    # motor_polariser_bottom = 2

    # xypli motor assignments
    motor_x = 1
    motor_y = 2
    motor_polariser_top = 4
    light = 4
    #----------------------------

    motor_busy = [False, False, False, False, False]
    debug = 1

    n_polarisers = None
    n_stepper_steps = None
    n_large_gear_teeth = None
    n_small_gear_teeth = None

    n_rows = 1
    n_cols = 1
    xy_roi_rect = (0, 0, 1, 1)
    xy_steps = (1, 1)

    n_angles = None

    interactive_mode = False

    # PLI machine

    def process_pli_machine_message(self, message):
        '''Process a message received from the PLI machine.'''
        if self.debug: print("PLI>", message)
        if message == "ready":
            self.status = "ready"
            return
        arr = re.split(r"\W+", message)
        while len(arr)>1 and arr[0] == "done":
            motor_number =  int(arr[1])
            self.motor_busy[motor_number] = False
            if self.motor_busy[1] is False \
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
            await asyncio.sleep(0.1)

    async def echo_pli_machine_messages(self, ser):
        '''Echo PLI machine messages without processing them.'''
        while self.status != "done":
            if ser.in_waiting:
                message = ser.readline().decode().strip()
                print(f"PLI> {message}")
            await asyncio.sleep(0.1)

    async def wait_for_ready(self):
        '''Wait for the PLI machine to be ready.'''
        while self.status != "ready":
            await asyncio.sleep(0.1)

    def new(self):
        '''Create a new serial connection to the PLI machine.'''
        print("\nLook for PLI machine...")
        list_of_known_arduino_ports = [
            "AB0LS12X", # Paris
            "cu.usbmodem", # Paris (fits usbmodem1101, usbmodem11201, usbmodem11301)
            "ttyUSB0"   # Alicante
        ]
        ports = serial.tools.list_ports.comports()
        # display candidate ports
        candidate_ports = []
        for port, desc, hwid in sorted(ports):
            if max([p in port for p in list_of_known_arduino_ports]) is True:
                candidate_ports.append((port, desc, hwid))
                print(f"* {port}: {desc} [{hwid}]")
            else:
                print(f"  {port}: {desc} [{hwid}]")
        arduino_port = None
        if len(candidate_ports) == 0:
            print("No candidate ports found.")
        else:
            print(f"Candidate ports: {len(candidate_ports)}")
            print("Using the first one.")
            print(f"Got pli machine: {candidate_ports[0][0]}")
            arduino_port = candidate_ports[0][0]
        ser = None
        if arduino_port is None:
            print("Arduino not found: returning dummy serial port.")
            ser = DummySerial()
        else:
            try:
                ser = serial.Serial(arduino_port, 115200)
            except Exception as exc:
                raise serial.serialutil.SerialException('Serial port is busy') from exc
        return ser

    def write(self, string, sleep=0.01):
        '''Send raw commands to the PLI machine.
        Commands:
            delay n: delay between steps of the machine in ms.
              From 1 (fast) to 30 (slow) are reasonable values
            dir n{+, -}: if n is 1, 2 or 3, sets the direction of the x, y, z motors.
              If n is 4, [turns on or off the light.] turns the polariser mitir
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

    # Motion of the polariser stepper

    def rotate(self, motor_number, delta):
        '''Rotate one motor by delta steps.'''
        direction = "+" if delta >= 0 else "-"
        string = f"{motor_number}{direction}{abs(delta)}\n"
        self.write(string)
        if motor_number == self.motor_polariser_top:
            self.polariser_steps += delta
        self.status = "moving"
        self.motor_busy[motor_number] = True

    def rotate_to(self, pos):
        '''Rotate to a given position.'''
        delta = round(pos - self.polariser_steps)
        # if self.debug: print(f"rotate_to delta: {delta}")
        if delta == 0:
            return
        self.rotate(self.motor_polariser_top, delta)
        if self.n_polarisers == 2:
            self.rotate(self.motor_polariser_bottom, -delta)

    def reset_polariser_steps(self):
        '''Reset the number of polariser_steps.'''
        self.polariser_steps = 0

    # def rotate_to_home(self):
    #    '''rotate to home position.'''
    #    if self.debug:
    #        print(f"rotate_to_home from {self.polariser_steps} to 0")
    #    self.rotate_to(0)

    def rotate_to_home(self):
        '''Rotate to home position.'''
        steps_whole_turn = self.n_large_gear_teeth/self.n_small_gear_teeth * self.n_stepper_steps
        # steps_whole_turn = round(steps_whole_turn)
        if self.polariser_steps > steps_whole_turn/2:
            self.rotate_to(steps_whole_turn)
            # self.reset_polariser_steps()
            self.polariser_steps = self.polariser_steps - steps_whole_turn
            if self.debug: print(f"cumulated polariser steps: {self.polariser_steps}")
        else:
            self.rotate_to(0)

    # Motion of the XY stage
    # def move_dx(self, dx):
    #     '''Move the XY stage in the X direction. Values dx>0 are
    #     towards the polariser's stepper (right in the slice image).'''

    #     if dx == 0:
    #         return

    #     self.x_steps += dx
    #     sign = "+" if dx >= 0 else ""
    #     string = f"""1{sign}{dx}ƒimsave\n2{sign}{dx}"""
    #     self.write(string)
    #     self.status = "moving"
    #     self.motor_busy[self.motor_x] = True
    #     self.motor_busy[self.motor_y] = True

    def move_to_x(self, x) -> None:
        '''Move the XY stage to a given x position.'''
        # dx = x - self.x_steps
        # if dx == 0:
        #     return
        # self.move_dx(dx)
        if x == self.x_steps:
            return
        self.x_steps = x
        string = f"""goto x{x}"""
        self.write(string)
        self.status = "moving"
        self.motor_busy[self.motor_x] = True
        self.motor_busy[self.motor_y] = True

    # def move_dy(self, dy):
    #     '''Move the XY stage in the Y direction. Values dy>0
    #     are towards the polariser (up in the slice image).'''

    #     if dy == 0:
    #         return

    #     self.y_steps += dy
    #     signx, signy = ("+", "-") if dy >= 0 else ("-", "+")
    #     string = f"""1{signx}{abs(dy)}\n2{signy}{abs(dy)}"""
    #     self.write(string)
    #     self.status = "moving"
    #     self.motor_busy[self.motor_x] = True
    #     self.motor_busy[self.motor_y] = True

    def move_to_y(self, y):
        '''Move the XY stage to a given y position.'''
        # dy = y - self.y_steps
        # if dy == 0:
        #     return
        # self.move_dy(dy)
        if y == self.y_steps:
            return
        self.y_steps = y
        string = f"""goto y{y}"""
        self.write(string)
        self.status = "moving"
        self.motor_busy[self.motor_x] = True
        self.motor_busy[self.motor_y] = True

    def move_to_xy(self, x, y):
        '''Move the XY stage to a given x, y position.'''
        print(f"move from {self.x_steps}, {self.y_steps} to {x}, {y}")
        string = f"""goto x{x} y{y}"""
        self.write(string)
        self.status = "moving"
        self.motor_busy[self.motor_x] = True
        self.motor_busy[self.motor_y] = True


    def move_to_home(self):
        '''Move the XY stage to the home position.'''

        self.write("home")
        self.status = "moving"

    def reset_xy_steps(self):
        '''Reset the number of XY steps.'''
        self.x_steps = 0
        self.y_steps = 0

    ## Settings

    def set_n_angles(self, n_angles):
        '''Set the number of angles.'''
        self.n_angles = n_angles

    def set_n_polarisers(self, n_polarisers):
        '''Set the number of polarisers.'''
        if n_polarisers != 1 and n_polarisers != 2:
            raise ValueError("n_polarisers must be 1 or 2")
        self.n_polarisers = n_polarisers

    def set_n_stepper_steps(self, n_stepper_steps):
        '''Set the number of steps in a whole turn of the stepper motors.'''
        self.n_stepper_steps = n_stepper_steps

    def set_n_large_gear_teeth(self, n_large_gear_teeth):
        '''Set the number of teeth in the large gear.'''
        self.n_large_gear_teeth = n_large_gear_teeth

    def set_n_small_gear_teeth(self, n_small_gear_teeth):
        '''Set the number of teeth in the small gear.'''
        self.n_small_gear_teeth = n_small_gear_teeth

    def set_n_rows(self, n_rows):
        '''Set the number of rows.'''
        self.n_rows = int(n_rows)

    def set_n_cols(self, n_cols):
        '''Set the number of columns.'''
        self.n_cols = int(n_cols)

    def set_xy_roi_rect(self, xy_roi_rect):
        '''Set the XY region of interest.'''
        self.xy_roi_rect = xy_roi_rect

    def set_xy_steps(self, xy_steps):
        '''Set the number of steps in the XY stage.'''
        self.xy_steps = xy_steps

    ## Acquisition functions

    def grab(self, image_name = "test.png"):
        '''Grab one image from the camera. Appends the image in Image_Array.
        Returns:
            img: numpy array of the image
        '''
        img = self.camera.grab_image()
        self.images.append([img, image_name])
        # print(f"Image shape: {img.shape[0]} x {img.shape[1]}")
        return img

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
            img = self.camera.grab_image()[::10, ::10]
        else:
            img = img[::10, ::10]
        res = np.mean(img.ravel())
        return res

    async def calibrate(self, initial_step):
        '''Calibrate the background of the PLI machine.
        Parameters:
            initial_step: initial step size
        Returns:
            vals: array of mean values
            pos: position of the PLI machine
        '''
        if self.pli_serial is None or self.camera is None:
            print("Empty PLI and camera: Calibrate")
            return [0], 0
        time.sleep(1)
        self.write("delay 5")
        self.camera.set_gamma(1)
        pos = 0
        dx = initial_step # pylint: disable=C0103
        val0 = self.mean_value_image()
        print("initial value:", val0)
        vals = [val0]
        while dx != 0:
            self.rotate(1, dx)
            await self.wait_for_ready()
            pos += dx
            val1 = self.mean_value_image()
            vals.append(val1)
            dy = val1 - val0 # pylint: disable=C0103
            if dy > 0:
                dx = -int(dx/2) # pylint: disable=C0103
            val0 = val1
        return vals, pos

    async def interactive(self):
        '''Interactive mode for configuring exposure, gain and gamma.'''
        # if self.pli_serial is None or self.camera is None:
        print("interactive mode")
        if self.camera is None:
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
        # self.write("dir 4+")

        self.interactive_mode = True
        plt.rcParams['toolbar'] = 'None'
        fig, ax_dict = plt.subplot_mosaic("AAB;AAB;AAB;AAB;AAC;AAD;AAE", figsize=(10, 5))

        def on_close(event):
            time.sleep(1)
            print("disable the motors")
            self.write("disable")

            print(f"--exposure {self.camera.get_exposure()} --gain {self.camera.get_gain()} --gamma {self.camera.get_gamma()}")
            self.interactive_mode = False
        fig.canvas.mpl_connect('close_event', on_close)

        fig.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.05, hspace=0.05)
        self.camera.close()
        self.camera.open()

        # self.camera.camera.UserSetSelector = "Default"
        # self.camera.camera.UserSetLoad.Execute()
        try:
            # self.camera.PixelFormat.SetValue("Mono8") # ("Mono12")
            self.camera.set_color_mode("Mono8")
        except:
            print("ERROR: cannot set pixel format")
            exit(1)

        # self.camera.Height = 2000
        # self.camera.Width = 2000
        # self.camera.AcquisitionFrameRateEnable.SetValue(True)
        # self.camera.AcquisitionFrameRate.SetValue(1.0)
        # self.camera.StreamGrabber.MaxBufferSize = 20286016
        # self.camera.StreamGrabber.MaxTransferSize = 262144

        self.camera.open()

        input_box1 = TextBox(ax_dict["C"], 'Exposure', initial=self.camera.get_exposure())
        def exposure(val):
            self.camera.set_exposure_online(float(val))
        #     self.camera.ExposureAuto.SetValue("Off")
        #     self.camera.ExposureTime.SetValue(float(val))
        input_box1.on_submit(exposure)

        input_box2 = TextBox(ax_dict["D"], 'Gain', initial=self.camera.get_gain())
        def gain(val):
            self.camera.set_gain_online(float(val))
        #     self.camera.GainAuto.SetValue("Off")
        #     self.camera.Gain.SetValue(float(val))
        input_box2.on_submit(gain)

        input_box3 = TextBox(ax_dict["E"], 'Gamma', initial=self.camera.get_gamma())
        def gamma(val):
            self.camera.set_gamma(float(val))
        #     self.camera.Gamma.SetValue(float(val))
        input_box3.on_submit(gamma)

        self.camera.start_grabbing()
        # self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        while self.interactive_mode is True:
            #     with self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException) as grab_result:
            with self.camera.retrieve_result() as grab_result:
                img = grab_result.Array[::10, ::10]
                ax_dict["A"].clear()
                ax_dict["A"].imshow(img, cmap="gray")
                ax_dict["B"].clear()
                ax_dict["B"].hist(
                    img.flatten(),
                    bins=64,
                    range=(0, self.camera.max()), # self.camera.PixelDynamicRangeMax.Value),
                    density=True)
                ax_dict["B"].get_yaxis().set_visible(False)
            plt.pause(0.1)
        plt.close(fig)
        self.camera.stop_grabbing()
        self.camera.close()
        # self.camera.StopGrabbing()
        # self.camera.Close()

    async def acquire(self, base_path, verbose=False):
        '''Acquire one set of polarised images for all angles.'''

        steps_whole_turn = self.n_large_gear_teeth/self.n_small_gear_teeth * self.n_stepper_steps
        if verbose: print("steps for a whole turn:", steps_whole_turn)

        # angles
        angle_step = steps_whole_turn/self.n_angles
        if verbose: print("setps in one angular displacement:", angle_step)

        self.images = []
        dark_array = []

        await self.wait_for_ready()
        self.write("delay 5")

        if verbose: print("rotate to home")
        self.rotate_to_home()
        await self.wait_for_ready()

        if verbose: print("turn on the light")
        # self.write("light on")
        # time.sleep(1)

        for iteration in range(self.n_angles):
            if verbose: print(f"angle: {iteration}")

            self.rotate_to(int(iteration * angle_step))
            await self.wait_for_ready()

            img = self.grab(f"{base_path}/{iteration}.tif")

            dark_array.append(self.mean_value_image(img))

        if verbose: print("move home")
        self.rotate_to_home()
        await self.wait_for_ready()

        if verbose: print("turn off the light")
        # self.write("light off")
        # time.sleep(1)

        if verbose: print(f"mean dark value: {np.mean(dark_array)}")
        if verbose: print(f"dark value range: {np.min(dark_array)} - {np.max(dark_array)}")
        # plt.plot(dark_array)

        # self.status = "done" # this stops the listener task and ends the script

        print(dt.now())
        self.save_all_images(raw=False)
        print(dt.now())

    ## Constructor

    def __init__(self, no_camera=False):
        if no_camera is False:
            self.camera = get_camera()
        self.pli_serial = self.new()
        print("")

# Utility
def pli_for_repl():
    '''Return a pli object with a parallel thread for getting
    and displaying its serial messages. Useful for using in interactive mode (REPL)
    To run async commands do, for example:
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(pli.move_to_xy(5000, 6000))
    '''

    def start_loop(loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()
    pli = PLI(no_camera=True)
    new_loop = asyncio.new_event_loop()
    thread = threading.Thread(target=start_loop, args=(new_loop,), daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(pli.echo_pli_machine_messages(pli.pli_serial), new_loop)
    return pli

# Tasks

async def calibrate_task(pli):
    '''Calibrate the orientation of the PLI machine.'''

    print("=> Calibration worflow\n")

    vals, pos = await pli.calibrate(20)
    print(vals)
    print(pos)
    pli.status = "done" # this stops the listener task and ends the script

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
            pli.write(command)
            await pli.wait_for_ready()
            print("check!")
        time.sleep(1)

    pli.status = "done"

async def interactive_task(pli):
    '''Interactive mode for setting exposure, gain and gamma.'''

    print("=> Interactive camera settings worflow\n")

    await pli.interactive()
    pli.status = "done" # this stops the listener task and ends the script

async def acquire_task(pli, base_path):
    '''Acquire polarised images.'''

    print("=> Data acquisition workflow\n")

    time.sleep(0.1)

    # await pli.wait_for_ready()

    print("enable the motors")
    pli.write("enable")

    pli.write("status")
    while pli.pli_serial.in_waiting:
        message = pli.pli_serial.readline().decode().strip()
        print(f"PLI> {message}")
        await asyncio.sleep(0.1)

    await pli.wait_for_ready()

    pli.move_to_home()
    await pli.wait_for_ready()

    await asyncio.sleep(5)

    xy_steps = pli.xy_steps

    for label, xy_roi_rect in [data for data in pli.xy_roi_rect if data is not None]:

        n_cols = int(np.abs(np.ceil(xy_roi_rect[2]) / xy_steps[0])) + 1
        n_rows = int(np.abs(np.ceil(xy_roi_rect[3]) / xy_steps[1])) + 1

        # move to ROI origin
        x, y, w, h = xy_roi_rect
        dx, dy = 1, 1
        if w < 0: w, dx = -w, -1
        if h < 0: h, dy = -h, -1

        pli.write("status")
        while pli.pli_serial.in_waiting:
            message = pli.pli_serial.readline().decode().strip()
            print(f"PLI> {message}")
            await asyncio.sleep(0.1)

        await pli.wait_for_ready()

        for j in range(n_rows):
            for i in range(n_cols):
                print(f"\nACQUIRING ROW: {j}, COL: {i}")

                pli.write("status")
                while pli.pli_serial.in_waiting:
                    message = pli.pli_serial.readline().decode().strip()
                    #if "xy:" in message: print(f"PLI> {message}")
                    if pli.debug > 1: print(f"PLI> {message}")
                    await asyncio.sleep(0.1)

                pli.write("delay 500", sleep=0)
                x_target = x + dx * i * pli.xy_steps[0]
                y_target = y + dy * j * pli.xy_steps[1]

                pli.move_to_xy(x_target, y_target)
                await pli.wait_for_ready()
                pli.x_steps = x_target
                pli.y_steps = y_target

                pli.write("delay 5", sleep=0)
                if label:
                    await pli.acquire(f"{base_path}/roi_{label}/fov_{j}_{i}")
                else:
                    await pli.acquire(f"{base_path}/fov_{j}_{i}")

    # move home
    pli.write("delay 500", sleep=0)
    # pli.move_to_home()
    await pli.wait_for_ready()

    print("disable the motors")
    pli.write("disable")
    time.sleep(0.1)

    pli.status = "done" # this stops the listener task and ends the script

def main(args):
    '''Main function.'''

    # create a new PLI machine instance,
    # and configure it
    pli = None
    loop = None
    listener_task = None

    if args.interactive is False:
        pli = PLI()
        loop = asyncio.get_event_loop()
        listener_task = loop.create_task(
            pli.listen_to_pli_machine_messages(pli.pli_serial))

    if args.calibrate is True:
        calibrate_commands_task = loop.create_task(calibrate_task(pli))
        tasks = [listener_task, calibrate_commands_task]
        loop.run_until_complete(asyncio.wait(tasks))
        print("done calibrating")

    elif args.commands:
        commands = args.commands.split(";")
        raw_commands_task = loop.create_task(commands_task(pli, commands))
        # echo_task = loop.create_task(
        #     pli.echo_pli_machine_messages(pli.pli_serial))
        listener_task = loop.create_task(
            pli.listen_to_pli_machine_messages(pli.pli_serial))
        tasks = [listener_task, raw_commands_task]
        loop.run_until_complete(asyncio.wait(tasks))
        print("done processing commands")

    elif args.interactive is True:
        # interactive_commands_task = loop.create_task(interactive_task(pli))
        # tasks = [listener_task, interactive_commands_task]
        # loop.run_until_complete(asyncio.wait(tasks))
        # print("done interactive mode")

        banner = "xypli interactive"
        code.interact(banner=f"{'='*len(banner)}\nXypli interactive\n{'='*len(banner)}\n", local = globals())

    elif args.acquire is True:
        # required arguments
        if args.base_path is None:
            raise ValueError("base_path is required")
        if args.n_angles is None:
            raise ValueError("n_angles is required")
        if args.n_polarisers is None:
            raise ValueError("n_polarisers is required")
        if args.n_stepper_steps is None:
            raise ValueError("n_stepper_steps is required")
        if args.n_large_gear_teeth is None:
            raise ValueError("n_large_gear_teeth is required")
        if args.n_small_gear_teeth is None:
            raise ValueError("n_small_gear_teeth is required")
        if args.color_mode is None:
            raise ValueError("color_mode is required")
        if args.gain is None:
            raise ValueError("gain is required")
        if args.exposure is None:
            raise ValueError("exposure is required")
        if args.gamma is None:
            raise ValueError("gamma is required")
        base_path = args.base_path
        n_angles = args.n_angles
        n_polarisers = args.n_polarisers
        n_stepper_steps = args.n_stepper_steps
        n_large_gear_teeth = args.n_large_gear_teeth
        n_small_gear_teeth = args.n_small_gear_teeth
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
        pli.reset_polariser_steps()

        pli.set_n_angles(n_angles)
        pli.set_n_polarisers(n_polarisers)
        pli.set_n_stepper_steps(n_stepper_steps)
        pli.set_n_large_gear_teeth(n_large_gear_teeth)
        pli.set_n_small_gear_teeth(n_small_gear_teeth)
        pli.set_xy_roi_rect(xy_roi_rect)
        pli.set_xy_steps(xy_steps)
        pli.camera.set_color_mode(color_mode)
        pli.camera.set_gain(gain)
        pli.camera.set_exposure(exposure)
        pli.camera.set_gamma(gamma)

        # start the acquisition
        acquisition_commands_task = loop.create_task(acquire_task(pli, base_path))
        tasks = [listener_task, acquisition_commands_task]
        loop.run_until_complete(asyncio.wait(tasks))

def rect_string(value):
    '''Parse a xy_roi_rect string
    The following are valid rect_strings:
    --xy_roi_rect=1,2,3,4, an ROI of coordinates x,y,w,h=1,2,3,4
    --xy_roi_rect=2:1,2,3,4, an ROI with label=2 and coordinates x,y,w,h=1,2,3,4. The label is added to the base_path where the images are saved.
    --xy_roi_rect=3:[1,2,3,4] an ROI with label=3 and coordinates that are ignored.
    '''

    if "[" in value:
        # ignore
        print("xy_roi_rect case 1")
        return

    label = None
    if ":" in value:
        # ROI with label
        print("xy_roi_rect case 2")
        label = value.split(":")[0]
        value = value.split(":")[1]

    rect = tuple(map(int, value.split(',')))

    return label, rect
    
def inttuple(value):
    '''Convert a string to a tuple of integers.'''
    return tuple(map(int, value.split(',')))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Acquire PLI data.')

    # automatic calibration of the polariser's angle
    parser.add_argument('--calibrate', action='store_true', help='auto-calibrate the PLI machine')

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
    parser.add_argument('--xy_roi_rect', type=rect_string, action='append', help='x,y,width,height of the region of interest in the image (default: 0,0,1,1)')
    parser.add_argument('--xy_steps', type=inttuple, help='x,y steps in the x and y directions for one image (default: 1,1)')

    # send raw commands
    parser.add_argument('--commands', type=str, help='raw commands to send to the PLI machine, separated by a semi-colon')

    main(parser.parse_args())

#-----------------------------------------------------------------------------
# IMPORTANT: To use this, comment out the main(parser.parse_args()) line above
#-----------------------------------------------------------------------------
# my_dict = {
#     "calibrate": False,
#     "interactive": False,
#     "commands": False,
#     "acquire": True,
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
#     "xy_roi_rect": [rect_string("1:6262,10623,-1626,-1626")],
#     "xy_steps": (1626, 1626)
# }
# args = argparse.Namespace(**my_dict)
# main(args)
#-----------------------------------------------------------------------------

# python pli-get.py --acquire --base_path /Users/roberto/Desktop/test-xypli --n_angles 9 --n_polarisers 1 --n_stepper_steps 6400 --n_large_gear_teeth 96 --n_small_gear_teeth 42 --color_mode Mono8 --gain 0 --exposure 11000 --gamma 1 --xy_roi_rect 0,0,3000,3000 --xy_steps 3000,3000

# python pli-get.py --acquire --base_path "/Users/roberto/Desktop/" --n_angles 72 --n_polarisers 1 --n_stepper_steps 6400 --n_large_gear_teeth 96 --n_small_gear_teeth 42 --color_mode RGB8 --gain 0 --exposure 11000 --gamma 1 --xy_steps 1300,1300 --xy_roi_rect 8840,7327,3638,3426