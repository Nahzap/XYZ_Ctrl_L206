import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

ser = serial.Serial(port, baud, timeout=0.2)
time.sleep(0.2)
ser.reset_input_buffer()

t0 = time.time()
buf = b""
total_bytes = 0
lines = 0
good = 0
bad = 0
info = 0
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
            print("  ", s)
            continue
        parts = s.split(",")
        if len(parts) == 6:
            try:
                pa = int(parts[0]); pb = int(parts[1])
                s1 = int(parts[2]); s2 = int(parts[3])
                if 0 <= s1 <= 4095 and 0 <= s2 <= 4095:
                    good += 1
                    sample = s
                else:
                    bad += 1
            except ValueError:
                bad += 1
        else:
            bad += 1
dt = time.time() - t0
ser.close()

print("---- serial bench ----")
print(f"port={port} baud={baud} window={dt:.2f}s")
print(f"bytes={total_bytes}  throughput={total_bytes/dt/1024:.1f} KiB/s  ({total_bytes*10/dt/1e6:.2f} Mbit/s incl framing)")
print(f"lines={lines}  rate={lines/dt:.0f} lines/s")
print(f"good(6-field,valid)={good}  bad/malformed={bad}  INFO={info}")
if lines:
    print(f"integrity={100.0*good/max(1,(good+bad)):.2f}%")
print("sample:", sample)
