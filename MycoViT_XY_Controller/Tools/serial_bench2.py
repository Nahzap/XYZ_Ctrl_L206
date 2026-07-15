import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
mode = sys.argv[4] if len(sys.argv) > 4 else None  # e.g. "A,0,0" or "M"

ser = serial.Serial(port, baud, timeout=0.2)
time.sleep(0.2)
if mode:
    ser.write((mode + "\n").encode())
    time.sleep(0.3)
ser.reset_input_buffer()

t0 = time.time()
buf = b""
total_bytes = 0
lines = good = bad = info = 0
sample = None
while time.time() - t0 < secs:
    chunk = ser.read(65536)
    if not chunk:
        continue
    total_bytes += len(chunk)
    buf += chunk
    while b"\n" in buf:
        raw, buf = buf.split(b"\n", 1)
        line = raw.strip()
        if not line:
            continue
        lines += 1
        s = line.decode("ascii", "replace")
        if s.startswith("INFO"):
            info += 1
            continue
        parts = s.split(",")
        if len(parts) == 6:
            try:
                s1 = int(parts[2]); s2 = int(parts[3])
                if 0 <= s1 <= 4095 and 0 <= s2 <= 4095:
                    good += 1; sample = s
                else:
                    bad += 1
            except ValueError:
                bad += 1
        else:
            bad += 1
dt = time.time() - t0
ser.close()
print(f"mode={mode} baud={baud} window={dt:.2f}s")
print(f"throughput={total_bytes*10/dt/1e6:.2f} Mbit/s  rate={lines/dt:.0f} lines/s  good={good} bad={bad}")
print("sample:", sample)
