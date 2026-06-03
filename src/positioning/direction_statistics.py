import math
from datetime import datetime


class DirectionStatistics:
    def __init__(self):
        self.total_epochs = 0
        self.successful_epochs = 0
        self.h_mean = 0.0
        self.h_max = 0.0
        self.v_mean = 0.0
        self.v_max = 0.0
        self.start_time = None
        self.duration = 0.0
        self._active = False
        self._enu_saved = False
        self._enu1_times = []
        self._enu1_east = []
        self._enu1_north = []
        self._enu1_up = []
        self._enu2_times = []
        self._enu2_east = []
        self._enu2_north = []
        self._enu2_up = []
        # [新增] 串口3 ENU数据快照
        self._enu3_times = []
        self._enu3_east = []
        self._enu3_north = []
        self._enu3_up = []

    def add_epoch(self, east, north, up, quality):
        self.total_epochs += 1
        h_error = math.sqrt(east ** 2 + north ** 2)
        v_error = abs(up)
        if h_error > self.h_max:
            self.h_max = h_error
        if v_error > self.v_max:
            self.v_max = v_error
        if quality > 0:
            self.successful_epochs += 1
            n = self.successful_epochs
            self.h_mean = self.h_mean + (h_error - self.h_mean) / n
            self.v_mean = self.v_mean + (v_error - self.v_mean) / n

    def save_enu_snapshot(self, t1, e1, n1, u1, t2, e2, n2, u2, t3=None, e3=None, n3=None, u3=None):
        self._enu1_times = list(t1)
        self._enu1_east = list(e1)
        self._enu1_north = list(n1)
        self._enu1_up = list(u1)
        self._enu2_times = list(t2)
        self._enu2_east = list(e2)
        self._enu2_north = list(n2)
        self._enu2_up = list(u2)
        # [新增] 串口3 ENU快照
        if t3 is not None:
            self._enu3_times = list(t3)
            self._enu3_east = list(e3)
            self._enu3_north = list(n3)
            self._enu3_up = list(u3)
        self._enu_saved = True

    def clear_enu(self):
        self._enu1_times = []
        self._enu1_east = []
        self._enu1_north = []
        self._enu1_up = []
        self._enu2_times = []
        self._enu2_east = []
        self._enu2_north = []
        self._enu2_up = []
        # [新增] 清空串口3 ENU数据
        self._enu3_times = []
        self._enu3_east = []
        self._enu3_north = []
        self._enu3_up = []
        self._enu_saved = False

    def get_enu1_snapshot(self):
        return self._enu1_times, self._enu1_east, self._enu1_north, self._enu1_up

    def get_enu2_snapshot(self):
        return self._enu2_times, self._enu2_east, self._enu2_north, self._enu2_up

    # [新增] 获取串口3 ENU快照
    def get_enu3_snapshot(self):
        return self._enu3_times, self._enu3_east, self._enu3_north, self._enu3_up

    def has_enu_data(self):
        """Check both ENU1 and ENU2 snapshots are available."""
        return (self._enu_saved
                and len(self._enu1_times) > 0
                and len(self._enu2_times) > 0)

    # [新增] 检查是否有ENU3数据
    def has_enu3_data(self):
        return self._enu_saved and len(self._enu3_times) > 0

    def reset(self):
        self.total_epochs = 0
        self.successful_epochs = 0
        self.h_mean = 0.0
        self.h_max = 0.0
        self.v_mean = 0.0
        self.v_max = 0.0
        self.start_time = None
        self.duration = 0.0
        self._active = False
        self.clear_enu()

    def start(self):
        self._active = True
        self.start_time = datetime.now()

    def stop(self):
        if self._active:
            self.update_duration()
            self._active = False

    def update_duration(self):
        if self._active and self.start_time:
            self.duration = (datetime.now() - self.start_time).total_seconds()

    def get_duration_str(self):
        total_sec = int(self.duration)
        h = total_sec // 3600
        m = (total_sec % 3600) // 60
        s = total_sec % 60
        return f"{h:02d}:{m:02d}:{s:02d}"

    def success_rate(self):
        if self.total_epochs == 0:
            return 0.0
        return self.successful_epochs / self.total_epochs * 100.0

    def get_stats(self):
        return {
            'total_epochs': self.total_epochs,
            'successful_epochs': self.successful_epochs,
            'success_rate': self.success_rate(),
            'h_mean': self.h_mean,
            'h_max': self.h_max,
            'v_mean': self.v_mean,
            'v_max': self.v_max,
            'duration': self.get_duration_str(),
            'duration_seconds': self.duration,
        }
