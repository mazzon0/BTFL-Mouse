import os
os.environ["QT_API"] = "pyqt5"   # used by pyqtgraph

import serial
import threading
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import time



PORT = 'COM63'
BAUD = 115200
SER_TIMEOUT = 0.05

TOUCHPAD_WIDTH  = 7
TOUCHPAD_HEIGHT = 10

MAX_AREA_REF = 200000
FINGER_REJECTION_THRESHOLD = 90000000
TIMER_MS = 50



current_frame_data = []
data_lock = threading.Lock()
frame_counter = 0
running = True

# colo map
id_color_map = {}

def get_color_for_id(id_value):
    if id_value not in id_color_map:
        id_color_map[id_value] = pg.intColor(len(id_color_map), 64)
    return id_color_map[id_value]



try:
    ser = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
    print(f"Serial aperta su {PORT}")
except Exception as e:
    print(f"Errore seriale: {e}")
    ser = None



# ==========================
def serial_reader():
    global current_frame_data, running

    if ser is None:
        return

    frame_touches = []

    while running:
        try:
            raw = ser.readline()
        except Exception:
            time.sleep(0.1)
            continue

        if not raw:
            continue

        try:
            line = raw.decode(errors="ignore").strip()
        except:
            continue

        # Fine frame
        if line == "FRAME_END":
            with data_lock:
                current_frame_data = frame_touches
            frame_touches = []
            continue


        # ID, state, currX, currY, startX, startY, Area
        parts = line.split(',')
        if len(parts) == 7:
            try:
                t = {
                    'ID':     int(parts[0]),
                    'state':  int(parts[1]),
                    'X':      float(parts[2]),
                    'Y':      float(parts[3]),
                    'startX': float(parts[4]),
                    'startY': float(parts[5]),
                    'Area':   int(parts[6])
                }
                frame_touches.append(t)
            except:
                continue



if ser:
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()



app = QtWidgets.QApplication([])

win = pg.GraphicsLayoutWidget(title="Touch Blob Viewer", show=True)

plot = win.addPlot()
plot.setAspectLocked(True)

plot.setXRange(0, TOUCHPAD_WIDTH)
plot.setYRange(0, TOUCHPAD_HEIGHT)

plot.setLabel('left', 'Y')
plot.setLabel('bottom', 'X')

scatter = pg.ScatterPlotItem(size=10, pen=None)
plot.addItem(scatter)

win.nextRow()
status_label = QtWidgets.QLabel("Status: init")
proxy = QtWidgets.QGraphicsProxyWidget()
proxy.setWidget(status_label)
win.addItem(proxy)


def update_plot():
    global frame_counter

    with data_lock:
        data = list(current_frame_data)

    frame_counter += 1

    # filtro dita
    filtered = [d for d in data if d['Area'] <= FINGER_REJECTION_THRESHOLD]

    if not filtered:
        scatter.setData([])
        status_label.setText(f"No touches — Frame {frame_counter}")
        return

    x    = np.array([d['X'] for d in filtered])
    y    = np.array([d['Y'] for d in filtered])
    ids  = np.array([d['ID'] for d in filtered])
    area = np.array([d['Area'] for d in filtered])
    sx   = np.array([d['startX'] for d in filtered])
    sy   = np.array([d['startY'] for d in filtered])
    st   = np.array([d['state'] for d in filtered])

    # squared distance
    sqdist = (x - sx)**2 + (y - sy)**2

    # area of blob
    sizes = 5 + 45 * np.sqrt(np.clip(area / MAX_AREA_REF, 0, 1))
    brushes = [pg.mkBrush(get_color_for_id(i)) for i in ids]

    scatter.setData(x=x, y=y, size=sizes, brush=brushes)

    text = f"Frame {frame_counter} | "
    for i in range(len(filtered)):
        text += f"[ID {ids[i]}: state={st[i]} d2={sqdist[i]:.1f}] "

    status_label.setText(text)


timer = QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(TIMER_MS)


try:
    app.exec()
finally:
    running = False
    if ser and ser.is_open:
        ser.close()
