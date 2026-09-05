"""
方向误差统计与多端口 ENU 快照管理模块

在车载/机载 GNSS 终端或抗干扰天线阵列的方向敏感性测试中，通常需要在转台上将设备
置于 4 个典型测试方向（如 0°/90°/180°/270° 或 前/右/后/左）。

本模块定义了针对单个方向的统计度量模型，负责记录：
  1. 历元统计：总历元数、有效定位历元数、定位成功率 (Success Rate)。
  2. 误差统计：水平径向误差均值与峰值 (H-Mean, H-Max)、高程绝对误差均值与峰值 (V-Mean, V-Max)。
  3. 测试时长：开始时间、持续时长与格式化时分秒显示。
  4. 坐标快照：保存串口1（基准参考）、串口2（干扰端口1）及串口3（干扰端口2）的 ENU 序列，供报表生成与离线绘图使用。
"""

import math
from datetime import datetime


class DirectionStatistics:
    """
    单方向定位精度与抗干扰性能统计类。
    
    维护特定角度下的历元数、误差极值与均值，并持久化该阶段的三路串口 ENU 时间序列快照。
    """

    def __init__(self):
        """初始化统计指标与快照缓存。"""
        self.total_epochs = 0          # 接收到的总历元数
        self.successful_epochs = 0     # 定位有效历元数 (定位解状态 Quality > 0)
        self.h_mean = 0.0              # 水平径向误差平均值 (米)
        self.h_max = 0.0               # 水平径向误差最大峰值 (米)
        self.v_mean = 0.0              # 天向垂直误差绝对值平均值 (米)
        self.v_max = 0.0               # 天向垂直误差绝对值最大峰值 (米)
        self.start_time = None         # 当前方向测试启动时间戳
        self.duration = 0.0            # 测试持续总秒数
        self._active = False           # 当前方向是否正在测试中
        self._enu_saved = False        # 是否已保存 ENU 轨迹快照

        # 串口 1 (基准参考机) ENU 轨迹快照
        self._enu1_times = []
        self._enu1_east = []
        self._enu1_north = []
        self._enu1_up = []

        # 串口 2 (干扰测试机 1) ENU 轨迹快照
        self._enu2_times = []
        self._enu2_east = []
        self._enu2_north = []
        self._enu2_up = []

        # 串口 3 (干扰测试机 2) ENU 轨迹快照
        self._enu3_times = []
        self._enu3_east = []
        self._enu3_north = []
        self._enu3_up = []

    def add_epoch(self, east, north, up, quality):
        """
        向当前方向累加一个历元的测量误差，并增量更新均值与最大值。

        计算规则:
          - 水平径向误差: h_error = sqrt(east^2 + north^2)
          - 天向绝对误差: v_error = abs(up)
          - 若 quality > 0 则记为有效解算历元，采用递推平均法更新均值：
            mean_{n} = mean_{n-1} + (error - mean_{n-1}) / n

        参数:
            east (float): 东向误差 (米)。
            north (float): 北向误差 (米)。
            up (float): 天向误差 (米)。
            quality (int): 定位质量指示 (0=未定位, >0=有效定位)。
        """
        self.total_epochs += 1
        h_error = math.sqrt(east ** 2 + north ** 2)
        v_error = abs(up)

        # 跟踪最大峰值误差
        if h_error > self.h_max:
            self.h_max = h_error
        if v_error > self.v_max:
            self.v_max = v_error

        # 递推统计有效历元均值
        if quality > 0:
            self.successful_epochs += 1
            n = self.successful_epochs
            self.h_mean = self.h_mean + (h_error - self.h_mean) / n
            self.v_mean = self.v_mean + (v_error - self.v_mean) / n

    def save_enu_snapshot(self, t1, e1, n1, u1, t2, e2, n2, u2, t3=None, e3=None, n3=None, u3=None):
        """
        保存本方向测试阶段多路串口的完整 ENU 坐标序列快照。

        用于在测试结束后独立绘制本方向的误差散点图、时间序列图并导出测试报告。

        参数:
            t1, e1, n1, u1 (iterable): 串口 1 (基准) 的时间与 ENU 坐标列表。
            t2, e2, n2, u2 (iterable): 串口 2 (干扰 1) 的时间与 ENU 坐标列表。
            t3, e3, n3, u3 (iterable, optional): 串口 3 (干扰 2) 的时间与 ENU 坐标列表。
        """
        self._enu1_times = list(t1)
        self._enu1_east = list(e1)
        self._enu1_north = list(n1)
        self._enu1_up = list(u1)

        self._enu2_times = list(t2)
        self._enu2_east = list(e2)
        self._enu2_north = list(n2)
        self._enu2_up = list(u2)

        if t3 is not None:
            self._enu3_times = list(t3)
            self._enu3_east = list(e3)
            self._enu3_north = list(n3)
            self._enu3_up = list(u3)
        else:
            self._enu3_times = []
            self._enu3_east = []
            self._enu3_north = []
            self._enu3_up = []

        self._enu_saved = True

    def clear_enu(self):
        """清除保存的全部 ENU 轨迹快照。"""
        self._enu1_times = []
        self._enu1_east = []
        self._enu1_north = []
        self._enu1_up = []

        self._enu2_times = []
        self._enu2_east = []
        self._enu2_north = []
        self._enu2_up = []

        self._enu3_times = []
        self._enu3_east = []
        self._enu3_north = []
        self._enu3_up = []

        self._enu_saved = False

    def get_enu1_snapshot(self):
        """获取串口 1 (基准) 的快照数据元组 (times, east, north, up)。"""
        return self._enu1_times, self._enu1_east, self._enu1_north, self._enu1_up

    def get_enu2_snapshot(self):
        """获取串口 2 (干扰 1) 的快照数据元组 (times, east, north, up)。"""
        return self._enu2_times, self._enu2_east, self._enu2_north, self._enu2_up

    def get_enu3_snapshot(self):
        """获取串口 3 (干扰 2) 的快照数据元组 (times, east, north, up)。"""
        return self._enu3_times, self._enu3_east, self._enu3_north, self._enu3_up

    def has_enu_data(self):
        """检查是否已成功保存基准口和至少一个测试口的有效快照数据。"""
        if not self._enu_saved or len(self._enu1_times) == 0:
            return False
        return len(self._enu2_times) > 0 or len(self._enu3_times) > 0

    def has_enu3_data(self):
        """检查是否存在有效的串口 3 快照数据。"""
        return self._enu_saved and len(self._enu1_times) > 0 and len(self._enu3_times) > 0

    def reset(self):
        """重置本方向的所有历元、误差极值、计时和快照数据。"""
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
        """启动本方向的测试计时。"""
        self._active = True
        self.start_time = datetime.now()

    def stop(self):
        """停止本方向的测试计时并结算总时长。"""
        if self._active:
            self.update_duration()
            self._active = False

    def update_duration(self):
        """若测试处于进行中，更新已耗费的持续秒数。"""
        if self._active and self.start_time:
            self.duration = (datetime.now() - self.start_time).total_seconds()

    def get_duration_str(self):
        """
        获取格式化的测试持续时间字符串 (HH:MM:SS)。

        返回:
            str: 格式如 "00:05:30" 的时长文本。
        """
        total_sec = int(self.duration)
        h = total_sec // 3600
        m = (total_sec % 3600) // 60
        s = total_sec % 60
        return f"{h:02d}:{m:02d}:{s:02d}"

    def success_rate(self):
        """
        计算定位成功率百分比。

        返回:
            float: 成功率 (0.0 ~ 100.0)。
        """
        if self.total_epochs == 0:
            return 0.0
        return self.successful_epochs / self.total_epochs * 100.0

    def get_stats(self):
        """
        汇总本方向的全部统计指标为字典。

        返回:
            dict: 包含 total_epochs, successful_epochs, success_rate,
                  h_mean, h_max, v_mean, v_max, duration, duration_seconds 的字典。
        """
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

