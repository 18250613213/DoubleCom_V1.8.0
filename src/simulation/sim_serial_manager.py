"""模拟串口管理器 - 继承SerialManager, 以加速模拟时钟回放生成的时间线。

信号协议(data_received/ubx_received/connection_status/error_occurred)与真实
SerialManager完全一致, 上层handle_data/handle_ubx无需感知数据来源。
"""
import time

from PyQt5.QtCore import pyqtSignal

from src.communication.serial_manager import SerialManager


class SimulatedSerialManager(SerialManager):
    sim_finished = pyqtSignal(int)  # (port_id)

    def __init__(self, port_id, timeline, speed=10.0):
        super().__init__(port_id)
        self._timeline = timeline
        self._speed = float(speed)
        self._cursor = 0
        self._t0 = None
        self._done = False

    def start_sim(self):
        self._t0 = time.monotonic()
        self._cursor = 0
        self._done = False
        self._read_timer.start(10)
        self.connection_status.emit(True, self.port_id)

    def _poll_serial(self):
        if self._t0 is None:
            return
        sim_t = (time.monotonic() - self._t0) * self._speed
        tl = self._timeline
        n = len(tl)
        while self._cursor < n and tl[self._cursor][0] <= sim_t:
            _, kind, payload = tl[self._cursor]
            if kind == 'ubx':
                self.ubx_received.emit(payload, self.port_id)
            else:
                self.data_received.emit(payload, self.port_id)
            self._cursor += 1
        if self._cursor >= n and not self._done:
            self._done = True
            self._read_timer.stop()
            self.sim_finished.emit(self.port_id)

    def sim_time(self):
        if self._t0 is None:
            return 0.0
        return (time.monotonic() - self._t0) * self._speed

    def progress(self):
        if not self._timeline:
            return 1.0
        return self._cursor / len(self._timeline)

    def is_done(self):
        return self._done

    def connect(self, *args, **kwargs):
        raise RuntimeError("SimulatedSerialManager不支持真实串口连接, 请使用start_sim()")
