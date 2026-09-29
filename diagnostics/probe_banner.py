"""Confirm which firmware build is actually flashed.

The banner is printed only in setup(), and this board does not reset when the
serial port is opened -- so press the reset button while this is listening.

Expected after the 2026-09-10 fix:
    > ardu-pli-4, dec 2023, rev2026.09.10

Usage: python probe_banner.py [/dev/cu.usbmodemXXXX]
Pass the port explicitly when more than one board is plugged in.
"""
import sys, time, glob, serial
from _port import open_pli

if len(sys.argv) > 1:
    port = sys.argv[1]
    print(f"Using {port}")
    ser = serial.Serial(port, 115200)
else:
    print(f"Ports: {sorted(glob.glob('/dev/cu.usbmodem*'))}"
          "  (pass one as an argument to pick)")
    ser = open_pli()
print("\nPress the RESET button on the PLI board now. Listening for 25s...\n")
t0, seen = time.time(), []
while time.time() - t0 < 25:
    if ser.in_waiting:
        line = ser.readline().decode(errors="replace").rstrip()
        seen.append(line)
        print(f"  [{time.time()-t0:5.1f}] {line}")
    else:
        time.sleep(0.02)

banner = [l for l in seen if "ardu-pli" in l]
print("\n=== banner ===")
print(banner or "NONE seen -- reset not pressed, or board does not re-announce")
if any("rev2026.09.10" in l for l in banner):
    print("OK: the fixed firmware is flashed.")
elif banner:
    print("MISMATCH: this is not the patched build.")
ser.close()
