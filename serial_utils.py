"""Serial port utilities for PLI acquisition.

Provides DummySerial for testing without hardware, and
find_arduino_serial() for auto-detecting the Arduino.
"""

import time
import serial
import serial.tools.list_ports


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
        elif data.startswith("home"):
            message = "> Homing done."
        elif data.startswith(("delay", "dir", "enable", "disable")) is False:
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


DEFAULT_KNOWN_PORTS = [
    "AB0LS12X",     # Paris
    "cu.usbmodem",  # Paris (fits usbmodem1101, usbmodem11201, usbmodem11301)
    "ttyUSB0",      # Alicante
]


def _identify_pli_board(port, baud=115200, boot_wait=2.0, reply_wait=2.0):
    '''Open a candidate port and check whether the PLI machine is on it.

    The joystick controller enumerates as a usbmodem port too, and it echoes
    back whatever it is sent -- so "something replied" is not enough to go on.
    Only the PLI board answers `status` with a `currentSteps:` line.

    Returns an open serial.Serial if this is the PLI board, otherwise None
    (with the port closed again). Raises if the port cannot be opened at all.
    '''
    ser = serial.Serial(port, baud)

    # The board resets when the port is opened and announces itself once with
    # "ready". Consume it here, so any subsequent USB activity (e.g. camera
    # init) cannot disrupt the boot handshake. Its absence is not an error:
    # the board does not reset on open while the joystick controller is wired
    # to the I2C bus (see pli_arduino/readme.md).
    t0 = time.time()
    while time.time() - t0 < boot_wait:
        if ser.in_waiting:
            if ser.readline().decode(errors='replace').strip() == "ready":
                print(f"  {port}: boot message received")
                break
        else:
            time.sleep(0.05)
    while ser.in_waiting:
        ser.readline()

    ser.write(b"status\n")
    t0 = time.time()
    while time.time() - t0 < reply_wait:
        if ser.in_waiting:
            line = ser.readline().decode(errors='replace').strip()
            if "currentSteps" in line:
                time.sleep(0.2)
                while ser.in_waiting:    # drain the rest of the status block
                    ser.readline()
                return ser
        else:
            time.sleep(0.02)

    print(f"  {port}: did not answer 'status' as the PLI machine")
    ser.close()
    return None


def find_arduino_serial(known_ports=None):
    '''Find and connect to the PLI machine's Arduino.

    Every candidate port is asked for its `status`, and the first one that
    answers like the PLI board is used. Taking the first candidate blindly is
    not safe: the joystick controller enumerates as a usbmodem port as well,
    and it answers serial traffic by echoing it back.

    Parameters:
        known_ports: list of port name substrings to match against.
            Defaults to DEFAULT_KNOWN_PORTS.

    Returns:
        serial.Serial or DummySerial
    '''
    if known_ports is None:
        known_ports = DEFAULT_KNOWN_PORTS

    print("\nLook for PLI machine...")
    ports = serial.tools.list_ports.comports()
    candidate_ports = []
    for port, desc, hwid in sorted(ports):
        if max([p in port for p in known_ports]) is True:
            candidate_ports.append((port, desc, hwid))
            print(f"* {port}: {desc} [{hwid}]")
        else:
            print(f"  {port}: {desc} [{hwid}]")

    if len(candidate_ports) == 0:
        print("No candidate ports found.")
        print("Arduino not found: returning dummy serial port.")
        return DummySerial()

    print(f"Candidate ports: {len(candidate_ports)}")
    open_errors = []
    for port, _, _ in candidate_ports:
        try:
            ser = _identify_pli_board(port)
        except Exception as exc:
            # Busy or unreadable: remember it, but keep looking -- the PLI
            # board may be on one of the other candidates.
            print(f"  {port}: cannot open ({exc})")
            open_errors.append(f"{port} ({exc})")
            continue
        if ser is not None:
            print(f"Got pli machine: {port}")
            return ser

    # Do not quietly fall back to a dummy when a port existed but could not be
    # opened -- that produces a full run of duplicate images with no error.
    if open_errors:
        raise serial.serialutil.SerialException(
            "Could not open candidate serial port(s): " + "; ".join(open_errors))

    print("No candidate port answered as the PLI machine: "
          "returning dummy serial port.")
    return DummySerial()
