"""
Microphone gain calibration plotter.
Streams raw ADC values from all 4 mics and displays them as a scrolling
oscilloscope. Use this to:
  1. Check idle baseline (all channels should sit at the same flat value)
  2. Match trim-pot gain (peaks should be equal amplitude on the same sound)
  3. Identify clipping (flat-topped waveform)

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
BAUD_RATE    = 115200
PORT         = 'COM3'
WINDOW       = 500        # number of samples shown at once (~1 second at 500 Hz)
EXPECTED_MID = 2750       # expected idle ADC value — shown as dashed reference line
ADC_MAX      = 4095

MIC_COLORS = ['#FF5252', '#4FC3F7', '#69F0AE', '#FFD740']
MIC_NAMES  = ['MIC 1  VP  (GPIO 36)',
               'MIC 2  VN  (GPIO 39)',
               'MIC 3      (GPIO 34)',
               'MIC 4      (GPIO 35)']

# ── Serial reader ─────────────────────────────────────────────────────────────
data_queue: queue.Queue = queue.Queue()

def find_port() -> str:
    for p in serial.tools.list_ports.comports():
        if any(k in p.description.upper() for k in ('CP210', 'CH340', 'USB', 'ESP')):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else PORT

def serial_thread(port: str) -> None:
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"Connected on {port}")
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        sys.exit(1)

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            parts = line.split(',')
            if len(parts) == 4:
                vals = [int(p) for p in parts]
                data_queue.put(vals)
        except (ValueError, UnicodeDecodeError):
            pass

# ── Plot setup ────────────────────────────────────────────────────────────────
plt.style.use('dark_background')
fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
fig.suptitle('Microphone Calibration — Raw ADC Values', fontsize=13, color='white')
fig.subplots_adjust(hspace=0.08, left=0.08, right=0.98, top=0.93, bottom=0.06)

buffers = [deque([EXPECTED_MID] * WINDOW, maxlen=WINDOW) for _ in range(4)]
lines   = []
val_labels = []

x = np.arange(WINDOW)

for i, (ax, color, name) in enumerate(zip(axes, MIC_COLORS, MIC_NAMES)):
    ax.set_ylim(0, ADC_MAX)
    ax.set_xlim(0, WINDOW)
    ax.set_ylabel(name, fontsize=8, color=color, rotation=0,
                  labelpad=145, va='center')
    ax.yaxis.set_label_position('left')
    ax.tick_params(colors='gray', labelsize=8)
    ax.set_yticks([0, 1000, EXPECTED_MID, 3000, ADC_MAX])
    ax.set_yticklabels(['0', '1000', f'{EXPECTED_MID}\n(mid)', '3000', '4095'],
                       fontsize=7, color='gray')
    for spine in ax.spines.values():
        spine.set_edgecolor('#333333')
    ax.set_facecolor('#0d0d0d')

    # reference line at expected midpoint
    ax.axhline(EXPECTED_MID, color='white', alpha=0.2, lw=1, ls='--')

    ln, = ax.plot(x, list(buffers[i]), color=color, lw=1.0)
    lines.append(ln)

    # current value label (top-right of each subplot)
    lbl = ax.text(0.99, 0.88, '', transform=ax.transAxes,
                  ha='right', va='top', fontsize=9, color=color)
    val_labels.append(lbl)

axes[-1].set_xlabel('samples  (newest → right)', fontsize=8, color='gray')

# ── Animation ─────────────────────────────────────────────────────────────────
def update(_):
    new_data = False
    while not data_queue.empty():
        vals = data_queue.get()
        for i, v in enumerate(vals):
            buffers[i].append(v)
        new_data = True

    if not new_data:
        return lines + val_labels

    for i in range(4):
        buf = list(buffers[i])
        lines[i].set_ydata(buf)
        current = buf[-1]
        idle_est = int(np.median(buf))
        val_labels[i].set_text(f'now: {current}   idle≈{idle_est}')

    return lines + val_labels

# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    threading.Thread(target=serial_thread, args=(port,), daemon=True).start()
    ani = animation.FuncAnimation(fig, update, interval=30, blit=True)
    plt.show()
