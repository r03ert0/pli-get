"""Do real moves complete AND report? Moves motors 1, 4 and 3 by 100 steps.

Compare 'currentSteps' before and after: the motors should advance by exactly
100 each. If they do but no 'done:' line appears, motion is fine and only
the reporting is broken.
"""
import time
from _port import open_pli

ser = open_pli()
t0 = time.time()
seen = []


def drain(secs, tag=""):
    end = time.time() + secs
    while time.time() < end:
        if ser.in_waiting:
            line = ser.readline().decode(errors="replace").rstrip()
            seen.append(line)
            print(f"  [{time.time()-t0:6.2f}] {tag} {line}")
        else:
            time.sleep(0.02)


def send(cmd):
    print(f"[{time.time()-t0:6.2f}] --> {cmd!r}")
    tail = b"\r\n" if cmd.startswith(("delay", "dir", "enable", "disable")) else b"\n"
    ser.write(bytes(cmd, "ascii") + tail)


drain(2, "idle")
send("enable"); drain(1)
send("status"); drain(2, "before")
send("delay 500"); drain(0.5)
for cmd in ("1+100", "4+100", "3+100"):
    send(cmd); drain(6, cmd)
send("status"); drain(2, "after")
send("disable")
ser.close()

print("\n=== any 'done:' lines? ===")
print([l for l in seen if "done" in l.lower()]
      or "NONE -- no 'done: N' emitted for any motor")
