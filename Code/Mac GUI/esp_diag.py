#!/usr/bin/env python3
"""Capture ESP32-S3 boot output over the USB-Serial-JTAG to tell whether the app
is running, stuck in ROM download mode, or boot-looping. Triggers an EN reset via
the Pi GPIO from inside, after the port is open, so the ROM banner is captured."""
import os, sys, time
try:
    import serial
except ImportError:
    print("NO_PYSERIAL"); sys.exit(2)

port = "/dev/ttyACM0"
try:
    s = serial.Serial(port, 115200, timeout=0.3)
except Exception as e:
    print("OPEN_FAIL:", e); sys.exit(1)

time.sleep(0.2)
s.reset_input_buffer()
# EN reset pulse: low 300ms, then high
os.system("gpioset --mode=time --sec=0 --usec=300000 gpiochip0 17=0")
os.system("gpioset --mode=time --sec=2 gpiochip0 17=1 &")
print("reset pulsed, capturing 6s...", flush=True)

buf = b""; t = time.time()
while time.time() - t < 6:
    c = s.read(512)
    if c:
        buf += c
s.close()
print("BYTES:", len(buf))
print("---ASCII---")
sys.stdout.write(buf.decode("latin-1", "replace"))
print("\n---END---")
