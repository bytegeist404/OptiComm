import mmap
import os
import struct
import sys
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

FILENAME = ".stream"

# ts (uint32 micros), raw (uint16), raw_ema (float), noise_ema (float), transition (uint8), sampled (uint8)
RECORD = struct.Struct("<IHffBB")
RECORD_SIZE = RECORD.size

FPS = 30
WINDOW_SECONDS = 5.0  # set <= 0 to show full history
MAX_POINTS = 50_000

FLAG_TRANSITION = 1 << 0
FLAG_SAMPLED = 1 << 1


class StreamFilePlotter(QtWidgets.QMainWindow):
    def __init__(self, filename: str):
        super().__init__()

        self.setWindowTitle("Live Receiver Stream")
        self.filename = filename

        # ---- file + mmap ----
        self.file = open(self.filename, "rb")
        self.mm = None
        self.mapped_size = 0
        self.offset = 0
        self._refresh_mmap_if_needed()

        # ---- buffers ----
        self.ts = deque(maxlen=MAX_POINTS)  # seconds
        self.raw = deque(maxlen=MAX_POINTS)
        self.raw_ema = deque(maxlen=MAX_POINTS)
        self.noise_ema = deque(maxlen=MAX_POINTS)

        self.transition_ts = deque(maxlen=MAX_POINTS)
        self.sampled_ts = deque(maxlen=MAX_POINTS)

        # ---- plot ----
        self.plot = pg.PlotWidget()
        self.setCentralWidget(self.plot)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.addLegend()

        self.curve_raw = self.plot.plot(pen=pg.mkPen("w", width=1), name="raw")
        self.curve_raw_ema = self.plot.plot(pen=pg.mkPen("y", width=2), name="raw_ema")
        self.curve_noise = self.plot.plot(pen=pg.mkPen("r", width=1), name="noise_ema")

        self.curve_transition_lines = self.plot.plot(
            pen=pg.mkPen("c", width=1), name="transition"
        )
        self.curve_sampled_lines = self.plot.plot(
            pen=pg.mkPen("m", width=1), name="sampled"
        )

        # ---- timer ----
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / FPS))

    # --------------------------------------------------

    def _refresh_mmap_if_needed(self):
        try:
            current_size = os.fstat(self.file.fileno()).st_size
        except Exception:
            current_size = 0

        if current_size == self.mapped_size:
            return

        if self.mm is not None:
            try:
                self.mm.close()
            except Exception:
                pass
            self.mm = None

        if current_size == 0:
            self.mapped_size = 0
            self.offset = 0
            return

        try:
            self.mm = mmap.mmap(
                self.file.fileno(), current_size, access=mmap.ACCESS_READ
            )
            self.mapped_size = current_size
        except Exception:
            self.mm = None
            self.mapped_size = current_size
            if self.offset > self.mapped_size:
                self.offset = 0

        if self.offset > self.mapped_size:
            self.offset = 0

    # --------------------------------------------------

    def read_new_records(self):
        self._refresh_mmap_if_needed()
        records = []

        if self.mm is None or self.mapped_size < RECORD_SIZE:
            return records

        usable_size = self.mapped_size - (self.mapped_size % RECORD_SIZE)

        while self.offset + RECORD_SIZE <= usable_size:
            try:
                rec = RECORD.unpack_from(self.mm, self.offset)
            except struct.error:
                self._refresh_mmap_if_needed()
                break

            records.append(rec)
            self.offset += RECORD_SIZE

        return records

    # --------------------------------------------------

    @staticmethod
    def _build_vertical_lines(ts_iter, ymin, ymax):
        xs = []
        ys = []
        nan = float("nan")

        for t in ts_iter:
            xs.extend([t, t, nan])
            ys.extend([ymin, ymax, nan])

        if not xs:
            return np.array([], dtype=float), np.array([], dtype=float)

        return np.asarray(xs), np.asarray(ys)

    # --------------------------------------------------

    def update(self):
        records = self.read_new_records()
        if not records:
            return

        appended = False

        for ts_u32, raw_u16, raw_ema_f, noise_ema_f, transition_b, sampled_b in records:
            # micros → seconds
            ts_s = ts_u32 * 1e-6

            # defensive corruption check
            if raw_u16 > 4095:
                continue

            self.ts.append(ts_s)
            self.raw.append(float(raw_u16))
            self.raw_ema.append(float(raw_ema_f))
            self.noise_ema.append(float(noise_ema_f))

            if transition_b & FLAG_TRANSITION:
                self.transition_ts.append(ts_s)
            if sampled_b & FLAG_SAMPLED:
                self.sampled_ts.append(ts_s)

            appended = True

        if not appended or not self.ts:
            return

        ts_arr = np.asarray(self.ts)
        raw_arr = np.asarray(self.raw)
        raw_ema_arr = np.asarray(self.raw_ema)
        noise_arr = np.asarray(self.noise_ema)

        self.curve_raw.setData(ts_arr, raw_arr)
        self.curve_raw_ema.setData(ts_arr, raw_ema_arr)
        self.curve_noise.setData(ts_arr, noise_arr)

        ymin = min(raw_arr.min(), raw_ema_arr.min(), noise_arr.min())
        ymax = max(raw_arr.max(), raw_ema_arr.max(), noise_arr.max())
        if ymin == ymax:
            ymin -= 1
            ymax += 1

        tx, ty = self._build_vertical_lines(self.transition_ts, ymin, ymax)
        sx, sy = self._build_vertical_lines(self.sampled_ts, ymin, ymax)

        self.curve_transition_lines.setData(tx, ty)
        self.curve_sampled_lines.setData(sx, sy)

        tmax = ts_arr[-1]
        if WINDOW_SECONDS > 0:
            self.plot.setXRange(tmax - WINDOW_SECONDS, tmax, padding=0)
        else:
            self.plot.setXRange(ts_arr[0], tmax, padding=0)

        pad = max(1.0, 0.05 * (ymax - ymin))
        self.plot.setYRange(ymin - pad, ymax + pad, padding=0)

    # --------------------------------------------------

    def closeEvent(self, event):
        self.timer.stop()
        try:
            if self.mm:
                self.mm.close()
        except Exception:
            pass
        try:
            self.file.close()
        except Exception:
            pass
        event.accept()


def main():
    if not os.path.exists(FILENAME):
        print(f"Waiting for {FILENAME}...")
        while not os.path.exists(FILENAME):
            time.sleep(0.2)

    app = QtWidgets.QApplication(sys.argv)
    win = StreamFilePlotter(FILENAME)
    win.resize(1300, 600)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
