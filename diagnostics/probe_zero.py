"""Is the firmware emitting 'done: N' at all?

'N+0' takes the steps == 0 branch of handleStepperMessage, which prints
'done: N' synchronously in the same loop() iteration -- no motion, no timing.
So a failure here can only be the reporting gate itself.

Expected: 19-20 out of 20 (attempt 0 races this script's own buffer flush).
A score of 0/20 means 'done:' is suppressed -- see docs/report-2.md.
"""
import time
from _port import open_pli

ser = open_pli()
time.sleep(1)
while ser.in_waiting:
    ser.readline()

hits = 0
for k in range(20):
    ser.write(b"1+0\n")
    t0, got = time.time(), []
    while time.time() - t0 < 0.4:
        if ser.in_waiting:
            got.append(ser.readline().decode(errors="replace").rstrip())
        else:
            time.sleep(0.01)
    if any("done" in g for g in got):
        hits += 1
    print(f"{k:2d}: {got}")

print(f"\n'done: 1' received in {hits}/20 attempts")
ser.close()
