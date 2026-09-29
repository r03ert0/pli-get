"""Quick test: open serial port, enable steppers, send home, print all messages."""
import time
import serial
import serial.tools.list_ports

PORT_SUBSTRINGS = ["cu.usbmodem", "AB0LS12X", "ttyUSB0"]

def find_port():
    for port, desc, hwid in serial.tools.list_ports.comports():
        if any(s in port for s in PORT_SUBSTRINGS):
            print(f"Found: {port}")
            return port
    raise RuntimeError("No Arduino found")

port = find_port()
ser = serial.Serial(port, 115200, timeout=1)
print("Port opened — waiting for boot message...")

# Drain any buffered data and wait a bit for Arduino to (re)boot
time.sleep(2.5)

def drain(label):
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        print(f"  [{label}] {line!r}")

drain("boot")

print("\nSending: enable")
ser.write(b"enable\r\n")
time.sleep(0.2)
drain("enable")

print("\nSending: home  (waiting up to 60s for 'Homing done'...)")
ser.write(b"home\n")

t0 = time.time()
homing_done = False
while time.time() - t0 < 60:
    if ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        print(f"  [home] {line!r}")
        if "Homing done" in line:
            homing_done = True
            break
    time.sleep(0.05)

if homing_done:
    print("\nSUCCESS: received 'Homing done'")
else:
    print("\nTIMEOUT: 'Homing done' not received within 60s")

ser.close()
