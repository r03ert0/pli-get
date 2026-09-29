"""Shared port detection for the diagnostic probes."""
import sys, glob, time, serial


def _looks_like_pli(port, baud):
    '''Ask a candidate port for its status and see if the PLI board answers.

    The joystick controller also enumerates as a usbmodem port, and it echoes
    whatever you send it -- so the port order alone is not enough to tell them
    apart.
    '''
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception:
        return None
    time.sleep(1.5)                       # let any reset-on-open banner land
    while ser.in_waiting:
        ser.readline()
    ser.write(b"status\n")
    t0, seen = time.time(), []
    while time.time() - t0 < 2.0:
        if ser.in_waiting:
            seen.append(ser.readline().decode(errors="replace"))
        else:
            time.sleep(0.02)
    if "currentSteps" in " ".join(seen):
        return ser
    ser.close()
    return None


def open_pli(baud=115200, port=None):
    '''Open the PLI machine serial port, or exit with a clear message.'''
    if port is None and len(sys.argv) > 1 and sys.argv[1].startswith("/dev/"):
        port = sys.argv[1]
    if port:
        print(f"Using {port} (given explicitly)")
        return serial.Serial(port, baud)

    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        sys.exit("No PLI machine found. Check the USB cable "
                 "(a charge-only cable enumerates nothing).")
    for p in ports:
        ser = _looks_like_pli(p, baud)
        if ser is not None:
            print(f"Using {p} (answered 'status' as the PLI board)")
            return ser
        print(f"  {p}: not the PLI board")
    sys.exit(f"None of {ports} answered as the PLI board. "
             "Is it powered and running pli_arduino.ino?")
