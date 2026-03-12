# Acquiring PLI data

With a spli6 PLI machine, a Basler camera and an XY stage.

## Usage

### Calibrate
`python pli_get.py --calibrate`

### Unload a sample using the XY stage
`python pli_get.py --xy_commands "G21G91G1X120F2000"`

### Load a sample using the XY stage
`python pli_get.py --xy_commands "G21G91G1X-120F2000"`

## Acquire one set of images with 1 rotating polariser configuration in a small PLI machine (67 mm polarisers)
`python pli_get.py --acquire --base_path path/to/data --n_angles 36 --n_polarisers 1 --n_stepper_steps 800 --n_large_gear_teeth 96 --n_small_gear_teeth 42 --color_mode Mono8 --gain 9.555 --exposure 11000 --gamma 3.999`

## Acquire the same set of images with a 2 rotating polarisers configuration in a large PLI machine (95 mm polarisers)
`python pli_get.py --acquire --base_path path/to/data --n_angles 36 --n_polarisers 2 --n_stepper_steps 800 --n_large_gear_teeth 112 --n_small_gear_teeth 42 --color_mode Mono8 --gain 9.555 --exposure 11000 --gamma 3.999`

## Send raw commands to the PLI machine
`python pli_get.py --command "1+200;wait 1;1-200"`: Make motor 1 move 200 steps clockwise, wait 1 second, move it 200 steps counter clockwise.

`python pli_get.py --command "delay 5"`: Set the delay between steps to 5 ms.

## Notes

For some reason which we haven't figured out, the PLI machine runs artifactual steps during initialisation if the computer controlling is unplugged... work in progress.

## Current configurations
The Basler ac2A4504-18ucBAS camera can acquire grey-scale images in color_mode = Mono12.
The Basler acA5472-17uc camera can acquire grey-scale images in color_mode = Mono8, and colour images with color_mode = RGB8.
The small PLI machine using 67 mm polarisers has gears with 96 teeth (42 in the motors).
The large PLI machine using 95 mm polarisers has gears with 112 teeth (and the same 42 in the motors).
The stepper motors are configure for 800 steps in a full turn.

# API

```python
class DummySerial: fake serial connection used for debugging
```

```python
class DummyCamera: fake camera connection used for debugging
```

```python
class PLI: main class controlling all aspects of the acquisition of pli data
    # hardware connection variables
    pli_serial: serial port of the pli machine
    camera: connection with the camera
    xy_stage_serial: serial port of the xy stage

    images: image array
    channel: channel kept when the camera acquires colour images
    steps: number of steps away from the home position
    status: status of the pli machine: idle, ready, done
    motor_busy: array keeping track of each of the pli machine motors being busy
    debug: debug flag which is set to True to display debug messages

    # pli machine configuration
    n_polarisers: number of rotating polarisers, 1 or 2.
    n_stepper_steps: number of steps the steppers do for a complete turn
    n_large_gear_teeth: number of teeth in the large gear of the polarisers
    n_small_gear_teeth: number of teeth in the small gear of the stepper motors

    # acquisition configuration
    n_angles: number of angles where to acquire images

    # xy stage functions
    listen_for_xy_stage_messages(ser): listen to messages from the xy stage
    xy_stage_write(string, sleep=0.01): send raw messages to the xy stage
    new_xy_stage(): create a connection with the xy stage

    # pli machine functions
    process_message(message): process messages from the pli machine
    listen_for_messages(ser): listen to messages from the pli machine
    wait_for_ready(): wait until the pli machine is ready
    pli_write(string, sleep=0.01): send raw messaged to the pli machine
    move(motor_number, delta): move motor motor_number delta steps
    move2(delta): move the two motors delta steps
    move_to(pos): move the two motors to position pos
    move_to_home(): move the two motors back to their home position
    reset_steps(): set the current position of the machine as home
    new_pli(): create a connection with the pli machine

    # camera functions
    new_camera(): create a connection with the camera
    camera_open(): open the camera
    camera_close(): close the camera

    # set configuration
    set_n_polarisers(n_polarisers): indicate the number of polarisers in the pli machine
    set_n_stepper_steps(n_stepper_steps): indicate the number of steps for a whole turn of the stepper motors in the pli machine
    set_n_large_gear_teeth(n_large_gear_teeth): set the number of teeth in the gear attached to the polarisers
    set_n_small_gear_teeth(n_small_gear_teeth): set the number of teeth in the gear attached to the motor

    # set acquisition configuration
    set_n_angles(n_angles): set the number of angular steps for a whole turn in the acquisition
    set_color_mode(color_mode): set the color mode of the camera for the acquisition
    set_gain(gain): set the gain of the camera for the acquisition
    set_exposure(exposure): set the exposure of the camera for the acquisition
    set_gamma(gamma): set the gamma of the camera for the acquisition

    # acquisition
    grab_image(channel=1): acquire one image
    grab(image_name="test.png"): acquire one image and add it to the image array
    save_all_images(): save all images in the image array

    # autocalibration
    mean_value_image(img=None): compute the mean value of an image, used for calibration.
    calibrate(initial_step): launch an autocalibration sequence, with an initial search step of initial_step
```

```python
calibrate_task(pli): asynchronous calibration task, calls calibrate()

acquire_task(pli, base_path): asynchronous task to acquire images at n_angles. The images are stored in a temporary image array and saved at base_path at the end.

main: parse command line arguments an call the appropriate functions
```
