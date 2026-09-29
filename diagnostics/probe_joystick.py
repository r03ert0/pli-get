"""Regression check for the other direction: joystick motion must stay silent.

The fix makes serial-commanded moves report 'done: N'. Joystick-driven motion
must NOT -- it is continuous, and would spam the client with completions it
never asked for.

Silence alone proves nothing: the firmware prints nothing during joystick
motion either way. So this polls 'status' throughout the window and tracks the
RANGE each motor covers -- moving a stick both ways nets to zero displacement
but still shows up as a range. It requires BOTH that motors moved and that no
'done:' was emitted.

'status' is safe to send during the window: handleStatusMessage() only prints,
it never touches reportDone.

Run it, then push each joystick in BOTH directions during the window.
"""
import re, time
from _port import open_pli

WINDOW = 40
POLL = 1.2
MOTORS = {1: "X/A", 2: "Y/B", 3: "joy2 X", 4: "joy2 Y"}

ser = open_pli()
seen = []


def pump():
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").rstrip()
        seen.append(line)
        m = re.search(r"currentSteps:\s*(-?\d+),(-?\d+),(-?\d+),(-?\d+)", line)
        if m:
            return tuple(int(g) for g in m.groups())
    return None


time.sleep(1)
pump()
ser.write(b"enable\r\n")
time.sleep(0.5)
pump()

print(f"\nMove EACH joystick in BOTH directions over the next {WINDOW}s...\n")
samples, t0, next_poll = [], time.time(), 0.0
while time.time() - t0 < WINDOW:
    if time.time() - t0 >= next_poll:
        ser.write(b"status\n")
        next_poll += POLL
    got = pump()
    if got:
        samples.append(got)
        print(f"  [{time.time()-t0:5.1f}] currentSteps: {got}")
    time.sleep(0.02)

time.sleep(0.5)
pump()
dones = [l for l in seen if "done" in l.lower()]

print("\n=== per-motor travel ===")
ranges = {}
for i, name in MOTORS.items():
    vals = [s[i - 1] for s in samples]
    rng = (max(vals) - min(vals)) if vals else 0
    ranges[i] = rng
    print(f"  motor {i} ({name:6s}): range {rng:6d} steps"
          f"{'' if rng else '   <- never moved'}")

print("\n=== result ===")
if not samples:
    print("INCONCLUSIVE: no status replies -- is the board responding?")
elif not any(ranges.values()):
    print("INCONCLUSIVE: nothing moved -- were the joysticks pushed?")
elif dones:
    print(f"FAIL: {len(dones)} 'done:' lines during joystick motion: {dones}")
    print("handleJoystick() is not clearing reportDone for every driven axis.")
else:
    covered = [f"motor {i}" for i, r in ranges.items() if r]
    print(f"PASS: {', '.join(covered)} moved, and no 'done:' lines were emitted.")
    missing = [f"motor {i} ({MOTORS[i]})" for i, r in ranges.items() if not r]
    if missing:
        print(f"NOTE: {', '.join(missing)} untested -- that axis never moved.")

ser.write(b"disable\r\n")
time.sleep(0.3)
ser.close()
