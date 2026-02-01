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

RECORD = struct.Struct("<dfffB7x")
RECORD_SIZE = RECORD.size

FPS = 30
WINDOW_SECONDS = 1.0  # visible time window
MAX_POINTS = 50_000  # safety cap

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
        self.ts = deque(maxlen=MAX_POINTS)
        self.raw = deque(maxlen=MAX_POINTS)
        self.ema = deque(maxlen=MAX_POINTS)
        self.thr = deque(maxlen=MAX_POINTS)

        # store only timestamps for events (floats)
        self.transition_ts = deque(maxlen=MAX_POINTS)
        self.sampled_ts = deque(maxlen=MAX_POINTS)

        # ---- plot ----
        self.plot = pg.PlotWidget()
        self.setCentralWidget(self.plot)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.addLegend()

        self.curve_raw = self.plot.plot(pen=pg.mkPen("w", width=1), name="raw")
        self.curve_ema = self.plot.plot(pen=pg.mkPen("y", width=1), name="ema")
        self.curve_thr = self.plot.plot(pen=pg.mkPen("c", width=1), name="thr")

        # vertical line plots — connect='finite' will break lines on np.nan
        self.lines_transition = pg.PlotDataItem(
            pen=pg.mkPen((255, 0, 0, 200), width=1),
            name="transition",
            connect="finite",
        )
        self.lines_sampled = pg.PlotDataItem(
            pen=pg.mkPen((0, 255, 0, 200), width=1),
            name="sampled",
            connect="finite",
        )

        self.plot.addItem(self.lines_transition)
        self.plot.addItem(self.lines_sampled)

        # ---- timer ----
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / FPS))

    # --------------------------------------------------

    def _refresh_mmap_if_needed(self):
        """Ensure mapping matches current file size (remap on grow/truncate)."""
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
            # transient failure — keep state consistent and retry next tick
            self.mm = None
            self.mapped_size = current_size
            if self.offset > self.mapped_size:
                self.offset = 0

        if self.offset > self.mapped_size:
            self.offset = 0

    # --------------------------------------------------

    def read_new_records(self):
        """Read all full records appended since last call; remap safely if needed."""
        self._refresh_mmap_if_needed()
        records = []

        if self.mm is None or self.mapped_size < RECORD_SIZE:
            return records

        usable_size = self.mapped_size - (self.mapped_size % RECORD_SIZE)

        while self.offset + RECORD_SIZE <= usable_size:
            try:
                rec = RECORD.unpack_from(self.mm, self.offset)
            except struct.error:
                # defensive remap and stop this cycle; next tick will retry
                self._refresh_mmap_if_needed()
                break
            records.append(rec)
            self.offset += RECORD_SIZE

        return records

    # --------------------------------------------------

    def _build_vertical_lines(self, ts_iter, ymin: float, ymax: float):
        """Return (x_array, y_array) with np.nan separators for vertical segments.

        For each timestamp t produce: [t, t, nan] and [ymin, ymax, nan].
        """
        # ts_iter may be deque — iterate and produce floats
        xs = []
        ys = []
        nan = float("nan")
        for t in ts_iter:
            # ensure numeric float
            try:
                tf = float(t)
            except Exception:
                continue
            xs.extend([tf, tf, nan])
            ys.extend([ymin, ymax, nan])

        if not xs:
            return np.array([], dtype=float), np.array([], dtype=float)

        return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)

    # --------------------------------------------------

    def update(self):
        records = self.read_new_records()
        if not records:
            return

        for ts, raw, ema, thr, flags in records:
            # ensure numeric values; drop if corrupt
            try:
                ts_f = float(ts)
                raw_f = float(raw)
                ema_f = float(ema)
                thr_f = float(thr)
            except Exception:
                continue

            self.ts.append(ts_f)
            self.raw.append(raw_f)
            self.ema.append(ema_f)
            self.thr.append(thr_f)

            if flags & FLAG_TRANSITION:
                self.transition_ts.append(ts_f)
            if flags & FLAG_SAMPLED:
                self.sampled_ts.append(ts_f)

        if not self.ts:
            return

        # ---- update curves ----
        # convert deques to numeric arrays (pyqtgraph accepts lists too, but arrays are explicit)
        ts_arr = np.asarray(self.ts, dtype=float)
        raw_arr = np.asarray(self.raw, dtype=float)
        ema_arr = np.asarray(self.ema, dtype=float)
        thr_arr = np.asarray(self.thr, dtype=float)

        self.curve_raw.setData(ts_arr, raw_arr)
        self.curve_ema.setData(ts_arr, ema_arr)
        self.curve_thr.setData(ts_arr, thr_arr)

        # ---- x window ----
        tmax = ts_arr[-1]
        tmin = tmax - WINDOW_SECONDS
        self.plot.setXRange(tmin, tmax, padding=0)

        # ---- y range ----
        ymin = min(raw_arr.min(), ema_arr.min(), thr_arr.min())
        ymax = max(raw_arr.max(), ema_arr.max(), thr_arr.max())
        if ymin == ymax:
            ymin -= 1.0
            ymax += 1.0

        # ---- vertical event lines ----
        tx, ty = self._build_vertical_lines(self.transition_ts, ymin, ymax)
        sx, sy = self._build_vertical_lines(self.sampled_ts, ymin, ymax)

        # setData with numeric arrays (np.nan breaks segments)
        self.lines_transition.setData(tx, ty)
        self.lines_sampled.setData(sx, sy)

    # --------------------------------------------------

    def closeEvent(self, event):
        self.timer.stop()
        try:
            if self.mm is not None:
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
        print(f"Waiting for {FILENAME} to appear...")
        while not os.path.exists(FILENAME):
            time.sleep(0.2)

    app = QtWidgets.QApplication(sys.argv)
    win = StreamFilePlotter(FILENAME)
    win.resize(1200, 500)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
