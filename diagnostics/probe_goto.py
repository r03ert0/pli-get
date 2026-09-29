"""Home, then send one goto and log raw serial with timestamps.

Reproduces exactly what acquire_task sends for the first FOV. Watch for
'done: 1' / 'done: 2' -- pli_get.py blocks in wait_for_ready() until both
arrive. The trailing 'status' shows whether the move physically happened
regardless of what was reported.
"""
import time
from _port import open_pli

ser = open_pli()
t0 = time.time()


def drain(secs, tag=""):
    end = time.time() + secs
    while time.time() < end:
        if ser.in_waiting:
            print(f"  [{time.time()-t0:6.2f}] {tag} "
                  f"{ser.readline().decode(errors='replace').rstrip()}")
        else:
            time.sleep(0.02)


def send(cmd):
    print(f"[{time.time()-t0:6.2f}] --> {cmd!r}")
    tail = b"\r\n" if cmd.startswith(("delay", "dir", "enable", "disable")) else b"\n"
    ser.write(bytes(cmd, "ascii") + tail)


drain(3, "boot")
send("enable"); drain(1)
send("home"); drain(45, "home")
send("status"); drain(2, "status")

# first FOV of a y=3mm ROI at 320 steps/mm
send("delay 500")
send("goto x0 y960")
print("... waiting 30s for done: 1 / done: 2 ...")
drain(30, "goto")

send("status"); drain(2, "after")
send("disable")
ser.close()
