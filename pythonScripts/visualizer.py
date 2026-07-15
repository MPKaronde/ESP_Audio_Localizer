"""
Sound localizer visualizer.
Reads angle values (0-360) from the ESP32 over serial and draws a live direction line.

Requirements:
    pip install pyserial matplotlib numpy
"""

import sys
import queue
import threading
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import serial.tools.list_ports

# ── Config ────────────────────────────────────────────────────────────────────
BAUD_RATE   = 115200
PORT        = 'COM3'      # fallback if auto-detect finds nothing
TRAIL_LEN   = 8           # number of ghost lines shown behind the current reading
UPDATE_MS   = 50          # plot refresh interval in milliseconds

# Mic positions in metres — must match MIC_LOC order in main.cpp
MIC_POS = np.array([
    [ 0.0625, -0.0625],   # MIC 1  VP  (GPIO 36)
    [ 0.0625,  0.0625],   # MIC 2  VN  (GPIO 39)
    [-0.0625,  0.0625],   # MIC 3      (GPIO 34)
    [-0.0625, -0.0625],   # MIC 4      (GPIO 35)
])
MIC_LABELS = ['1 (VP)', '2 (VN)', '3 (34)', '4 (35)']

# ── Serial reader (background thread) ────────────────────────────────────────
angle_queue: queue.Queue = queue.Queue()

def find_port() -> str:
    for p in serial.tools.list_ports.comports():
        if any(k in p.description.upper() for k in ('CP210', 'CH340', 'USB', 'ESP')):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else PORT

def serial_thread(port: str) -> None:
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"Connected on {port} at {BAUD_RATE} baud")
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        sys.exit(1)

    while True:
        try:
            raw = ser.readline().decode('utf-8', errors='ignore').strip()
            val = float(raw)
            if 0.0 <= val <= 360.0:
                angle_queue.put(val)
        except (ValueError, UnicodeDecodeError):
            pass

# ── Plot setup ────────────────────────────────────────────────────────────────
plt.style.use('dark_background')
fig, ax = plt.subplots(figsize=(7, 7))
ax.set_aspect('equal')
ax.axis('off')
ax.set_title('Sound Localizer', fontsize=16, pad=12, color='white')

PLOT_R = 0.145
ax.set_xlim(-PLOT_R, PLOT_R)
ax.set_ylim(-PLOT_R, PLOT_R)

# Compass rings
for r, alpha in [(0.055, 0.25), (0.11, 0.35)]:
    ax.add_patch(plt.Circle((0, 0), r, fill=False, color='white', alpha=alpha, lw=1))

# Cardinal tick marks and labels
for deg in range(0, 360, 30):
    rad = np.radians(deg)
    inner, outer = 0.108, 0.118
    ax.plot([inner*np.cos(rad), outer*np.cos(rad)],
            [inner*np.sin(rad), outer*np.sin(rad)],
            color='white', alpha=0.4, lw=1)
for deg, label in [(0,'0°'), (90,'90°'), (180,'180°'), (270,'270°')]:
    rad = np.radians(deg)
    ax.text(0.135*np.cos(rad), 0.135*np.sin(rad), label,
            ha='center', va='center', fontsize=9, color='gray')

# Microphone squares
for pos, label in zip(MIC_POS, MIC_LABELS):
    ax.plot(*pos, 's', color='#4FC3F7', markersize=13, zorder=5)
    nudge = pos / np.linalg.norm(pos) * 0.024
    ax.text(*(pos + nudge), label, ha='center', va='center',
            fontsize=8, color='#4FC3F7', zorder=6)

# Centre dot
ax.plot(0, 0, 'o', color='white', markersize=5, zorder=6)

# Direction line — one active + TRAIL_LEN ghosts
LINE_LEN   = 0.10
TRAIL_COLOR = '#FF5252'

trail_lines = []
for i in range(TRAIL_LEN):
    alpha = 0.08 + 0.12 * (i / TRAIL_LEN)
    ln, = ax.plot([], [], '-', color=TRAIL_COLOR, lw=1.5, alpha=alpha, zorder=3)
    trail_lines.append(ln)

active_line, = ax.plot([], [], '-',  color=TRAIL_COLOR, lw=2.5, zorder=4)
active_tip,  = ax.plot([], [], '^',  color=TRAIL_COLOR, markersize=11, zorder=4)

angle_text = ax.text(0, -PLOT_R + 0.005, '—', ha='center', va='bottom',
                     fontsize=22, fontweight='bold', color='white')

# ── Animation ─────────────────────────────────────────────────────────────────
history: deque = deque(maxlen=TRAIL_LEN + 1)

def update(_):
    while not angle_queue.empty():
        history.append(angle_queue.get())

    if not history:
        return [active_line, active_tip, angle_text] + trail_lines

    # Current reading
    rad = np.radians(history[-1])
    dx, dy = LINE_LEN * np.cos(rad), LINE_LEN * np.sin(rad)
    active_line.set_data([0, dx], [0, dy])
    active_tip.set_data([dx], [dy])
    angle_text.set_text(f'{history[-1]:.1f}°')

    # Ghost trail (oldest = most faded)
    past = list(history)[:-1]
    for i, ln in enumerate(trail_lines):
        if i < len(past):
            r2 = np.radians(past[i])
            ln.set_data([0, LINE_LEN*np.cos(r2)], [0, LINE_LEN*np.sin(r2)])
        else:
            ln.set_data([], [])

    return [active_line, active_tip, angle_text] + trail_lines

# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    threading.Thread(target=serial_thread, args=(port,), daemon=True).start()
    ani = animation.FuncAnimation(fig, update, interval=UPDATE_MS, blit=True)
    plt.tight_layout()
    plt.show()
