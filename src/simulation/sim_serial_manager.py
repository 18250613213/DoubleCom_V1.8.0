"""
虚拟串口仿真管理器 (SimulatedSerialManager)

继承自基类 SerialManager，通过回放预先生成的静态时间线数据 (Timeline) 来模拟真实串口。

架构设计要点:
  1. 接口与信号完全同构:
     发射完全相同的 Qt 业务信号 (data_received, ubx_received, connection_status, error_occurred)，
     上层业务函数 (如 MainWindow 中的 handle_data 与 handle_ubx) 无需区分当前连接的是物理串口还是仿真环境。
  2. 虚拟加速时钟驱动:
     基于系统单调时钟 time.monotonic() 乘以倍率因子 speed (如 10x 或 60x)，
     在保证真实时间调度非阻塞的前提下，实现 1 小时测试数据在 1~6 分钟内的高速完整回放。
"""

import time

from PyQt5.QtCore import pyqtSignal

from src.communication.serial_manager import SerialManager


class SimulatedSerialManager(SerialManager):
    """
    虚拟串口仿真管理器类。
    
    接管物理串口的读写轮询过程，通过加速时钟遍历预设数据时间线。
    """

    sim_finished = pyqtSignal(int)  # 仿真数据回放完毕信号: (端口标识 port_id)

    def __init__(self, port_id, timeline, speed=10.0):
        """
        初始化仿真管理器。

        参数:
            port_id (int): 端口逻辑编号 (1, 2, 3)。
            timeline (list): 由 generate_port_timeline 生成的按时间排序的三元组列表 [(time_sec, kind, payload), ...]。
            speed (float): 加速倍率 (例如 10.0 表示 10 倍速回放)。
        """
        super().__init__(port_id)
        self._timeline = timeline        # 预设的时间线数据
        self._speed = float(speed)       # 回放加速倍率
        self._cursor = 0                 # 当前回放到的时间线游标索引
        self._t0 = None                  # 仿真开始时的物理单调时间戳
        self._done = False               # 回放是否已完成

    def start_sim(self):
        """
        启动虚拟串口回放。
        
        记录初始时刻，启动 10ms 轮询定时器，并向系统广播连接成功状态。
        """
        self._t0 = time.monotonic()
        self._cursor = 0
        self._done = False
        self._read_timer.start(10)
        self.connection_status.emit(True, self.port_id)

    def _poll_serial(self):
        """
        重写基类的串口轮询方法。

        按虚拟加速时钟推进当前仿真时间 sim_t，将所有时间戳 <= sim_t 的
        NMEA 文本语句或 UBX 二进制帧批量弹出并通过信号发射。
        全部数据发送完毕后自动停止定时器并触发 sim_finished 信号。
        """
        if self._t0 is None:
            return

        # 计算当前仿真已流逝的虚拟时间 (秒)
        sim_t = (time.monotonic() - self._t0) * self._speed
        tl = self._timeline
        n = len(tl)

        # 批量弹出到达时间的数据项
        while self._cursor < n and tl[self._cursor][0] <= sim_t:
            _, kind, payload = tl[self._cursor]
            if kind == 'ubx':
                self.ubx_received.emit(payload, self.port_id)
            else:
                self.data_received.emit(payload, self.port_id)
            self._cursor += 1

        # 检查是否已播发完全部数据
        if self._cursor >= n and not self._done:
            self._done = True
            self._read_timer.stop()
            self.sim_finished.emit(self.port_id)

    def sim_time(self):
        """
        获取当前虚拟时钟的累计流逝时间 (秒)。

        返回:
            float: 虚拟秒数。
        """
        if self._t0 is None:
            return 0.0
        return (time.monotonic() - self._t0) * self._speed

    def progress(self):
        """
        获取当前端口数据的回放进度百分比 (0.0 ~ 1.0)。

        返回:
            float: 进度浮点数。
        """
        if not self._timeline:
            return 1.0
        return self._cursor / len(self._timeline)

    def is_done(self):
        """
        检查当前端口的仿真回放是否已彻底结束。

        返回:
            bool: 回放完毕返回 True，否则返回 False。
        """
        return self._done

    def connect(self, *args, **kwargs):
        """
        禁止在仿真模式下调用物理连接。
        
        抛出:
            RuntimeError: 提示调用者使用 start_sim() 代替。
        """
        raise RuntimeError("SimulatedSerialManager不支持真实串口连接, 请使用start_sim()")

