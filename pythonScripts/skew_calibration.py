"""
ADC inter-channel sampling skew calibration.
Measures the fixed time delay between sequential ADC reads on the ESP32.

The firmware calibration loop emits one line per cycle:
    v0,v1,v2,v3,t0,t1,t2,t3
where t0-t3 are the micros() timestamps taken immediately before each
adc1_get_raw() call.

This script collects NUM_SAMPLES lines, computes the mean (t1-t0), (t2-t0),
(t3-t0) across all samples, and prints the CH_SKEW_US constant ready to
paste into main.cpp.

Requirements:
    pip install pyserial numpy
"""

import sys
import serial
import serial.tools.list_ports
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
BAUD_RATE   = 115200
PORT        = 'COM3'       # fallback if auto-detect fails
NUM_SAMPLES = 500          # number of cycles to average over

# ── Port detection (same logic as mic_calibration.py) ────────────────────────
def find_port() -> str:
    for p in serial.tools.list_ports.comports():
        if any(k in p.description.upper() for k in ('CP210', 'CH340', 'USB', 'ESP')):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else PORT

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"Connecting to {port} at {BAUD_RATE} baud…")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        sys.exit(1)

    print(f"Collecting {NUM_SAMPLES} samples — keep the environment quiet…")

    deltas = []   # each row: [t1-t0, t2-t0, t3-t0] in µs

    while len(deltas) < NUM_SAMPLES:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            parts = line.split(',')
            if len(parts) != 8:
                continue
            vals = [int(p) for p in parts]
        except (ValueError, UnicodeDecodeError):
            continue

        t0, t1, t2, t3 = vals[4], vals[5], vals[6], vals[7]

        # micros() is uint32 on ESP32 — handle wrap-around within a cycle
        # (extremely unlikely in <100µs, but safe)
        d1 = (t1 - t0) & 0xFFFFFFFF
        d2 = (t2 - t0) & 0xFFFFFFFF
        d3 = (t3 - t0) & 0xFFFFFFFF

        # sanity check: inter-channel delay should be < 200µs
        if d1 < 200 and d2 < 200 and d3 < 200:
            deltas.append([d1, d2, d3])

        if len(deltas) % 50 == 0:
            print(f"  {len(deltas)}/{NUM_SAMPLES}")

    ser.close()

    arr   = np.array(deltas, dtype=float)
    means = np.mean(arr, axis=0)
    stds  = np.std(arr, axis=0)

    print("\n── Results ──────────────────────────────────────────────────────")
    print(f"  CH0 skew: 0.00 µs  (reference)")
    print(f"  CH1 skew: {means[0]:.2f} ± {stds[0]:.2f} µs")
    print(f"  CH2 skew: {means[1]:.2f} ± {stds[1]:.2f} µs")
    print(f"  CH3 skew: {means[2]:.2f} ± {stds[2]:.2f} µs")
    print()
    print("── Paste this into main.cpp ──────────────────────────────────────")
    print(f"static constexpr double CH_SKEW_US[4] = "
          f"{{0.0, {means[0]:.2f}, {means[1]:.2f}, {means[2]:.2f}}};")

if __name__ == '__main__':
    main()
