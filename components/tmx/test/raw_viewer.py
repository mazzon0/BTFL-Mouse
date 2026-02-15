# touch_viewer_7x10.py
# Serial reader and real-time 7x10 matrix viewer with binary violet/yellow display based on raw threshold.

import serial
import threading
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import time


ROWS = 10
COLS = 7
NUM_PADS = ROWS * COLS

PORT = 'COM65'
BAUD = 115200
SER_TIMEOUT = 0.1

RAW_THRESHOLD = 10000.0
TIMER_MS = 50


try:
    ser = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
except Exception:
    ser = None

raw_values = np.zeros(NUM_PADS, dtype=float)


def serial_reader():
    global raw_values
    if ser is None:
        return

    while True:
        try:
            raw = ser.readline()
        except Exception:
            time.sleep(0.2)
            continue

        if not raw:
            continue

        try:
            line = raw.decode(errors='ignore').strip()
        except Exception:
            continue

        parts = [p for p in line.split(',') if p.strip() != '']
        if not parts:
            continue

        values = []
        try:
            for p in parts:
                values.append(float(p.strip()))
        except ValueError:
            continue

        if len(values) == NUM_PADS:
            raw_values = np.array(values, dtype=float)

# Start serial thread
t = threading.Thread(target=serial_reader, daemon=True)
t.start()


app = QtWidgets.QApplication([])

win = pg.GraphicsLayoutWidget(title=f"Touch Viewer {ROWS}x{COLS} (Threshold View)")
view = win.addViewBox()
view.setAspectLocked(True)

img = pg.ImageItem(np.zeros((ROWS, COLS)))
view.addItem(img)


# index 0 → violet (cells under threshold)
# index 1 → yellow (cells over threshold)
lut = np.zeros((2, 3), dtype=np.ubyte)
# Violet: for example RGB(128, 0, 128)
lut[0] = [128, 0, 128]
# Yellow: RGB(255, 255, 0)
lut[1] = [255, 255, 0]

img.setLookupTable(lut)
img.setLevels([0, 1])

win.nextRow()
status_label = QtWidgets.QLabel("Status: initializing")
proxy = QtWidgets.QGraphicsProxyWidget()
proxy.setWidget(status_label)
win.addItem(proxy)


def update():
    """Updates the binary threshold display (violet or yellow) based on raw values."""
    global raw_values

    raw_arr = raw_values.copy()

    try:
        raw_disp2d = raw_arr.reshape(ROWS, COLS)
    except Exception:
        raw_disp2d = np.zeros((ROWS, COLS))

    raw_disp2d_flipped = raw_disp2d[::-1, :]

    mask = (raw_disp2d_flipped > RAW_THRESHOLD).astype(np.uint8)

    img.setImage(mask, autoLevels=False)

    max_raw = raw_arr.max() if raw_arr.size else 0.0
    status_label.setText(
        f"Status: OK — Max Raw Value = {max_raw:.0f}   last_update = {time.strftime('%H:%M:%S')}"
    )

# Timer
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(TIMER_MS)

win.show()

try:
    app.exec()
except Exception:
    pass
finally:
    if ser and ser.is_open:
        ser.close()
