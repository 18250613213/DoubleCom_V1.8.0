import sys
import os
import math
import signal
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QComboBox, QPushButton, QLabel, QTextEdit,
    QMessageBox, QDoubleSpinBox, QSpinBox, QStatusBar,
    QGridLayout, QRadioButton, QButtonGroup, QCheckBox, QTabWidget,
    QLineEdit,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QTextCursor
from PyQt5.QtPrintSupport import QPrinter
from PyQt5.QtGui import QTextDocument
import re

import pyqtgraph as pg
import pyqtgraph.exporters
from pyqtgraph.exporters import ImageExporter

from src.communication.serial_manager import SerialManager
from src.nmea.nmea_parser import NMEAParser
from src.nmea.nmea_gga_parser import NMEAGGAParser
from src.positioning.error_calculator import ErrorCalculator
from src.positioning.statistics_accumulator import MultiDimensionStatistics
from src.positioning.direction_statistics import DirectionStatistics
from src.protocol.ubx_parser import parse_ubx_frame

from collections import deque


SIGNAL_NAMES = {
    'G': {0: 'ALL', 1: 'L1C', 2: 'L1P', 3: 'L1M', 4: 'L2P', 5: 'L2C', 6: 'L2C', 7: 'L5I', 8: 'L5Q'},
    'R': {0: 'ALL', 1: 'G1C', 2: 'G1P', 3: 'G2C', 4: 'G2P'},
    'E': {0: 'ALL', 1: 'E5A', 2: 'E5B', 3: 'E5AB', 4: 'E6A', 5: 'E6BC', 6: 'L1A', 7: 'L1BC'},
    'B': {0: 'ALL', 1: 'B1I', 2: 'B1Q', 3: 'B1C', 4: 'B1A', 5: 'B2A', 6: 'B2B', 7: 'B2AB', 8: 'B3', 9: 'B3Q', 10: 'B3A', 11: 'B2I', 12: 'B2Q'},
}

TALKER_TO_SYSTEM = {
    'GP': 'G', 'GL': 'R', 'GA': 'E', 'GB': 'B', 'BD': 'B',
}



# WGS84 ellipsoid constants (used in LLA-to-ECEF-to-ENU conversion)
WGS84_A = 6378137.0          # semi-major axis (meters)
WGS84_F = 1.0 / 298.257223563  # flattening
WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F  # first eccentricity squared
class SlidingWindowStd:
    def __init__(self, window_size=200):
        self._window = deque(maxlen=window_size)
        self._sum = 0.0
        self._sum_sq = 0.0

    def add(self, value):
        if len(self._window) == self._window.maxlen:
            old = self._window[0]
            self._sum -= old
            self._sum_sq -= old * old
        self._window.append(value)
        self._sum += value
        self._sum_sq += value * value

    def reset(self):
        self._window.clear()
        self._sum = 0.0
        self._sum_sq = 0.0

    @property
    def mean(self):
        n = len(self._window)
        if n == 0:
            return 0.0
        return self._sum / n

    @property
    def std(self):
        n = len(self._window)
        if n < 3:
            return 0.0
        mean = self._sum / n
        variance = self._sum_sq / n - mean * mean
        return math.sqrt(max(variance, 0.0))

    def __len__(self):
        return len(self._window)


class RunningAverage:
    def __init__(self):
        self._data = {}

    def add(self, key, value):
        if key not in self._data:
            self._data[key] = {'avg': float(value), 'n': 1}
        else:
            d = self._data[key]
            d['n'] += 1
            d['avg'] += (value - d['avg']) / d['n']

    def get(self, key):
        if key in self._data:
            return self._data[key]['avg']
        return None

    def reset(self):
        self._data.clear()


class NMEADataAnalyzer(QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NMEA 数据实时分析系统")
        self.setGeometry(100, 100, 1500, 1000)
        self.showMaximized()
        
        # 初始化独立模块
        self.gga_parser = NMEAGGAParser()
        self.nmea_parser = NMEAParser()
        self.error_calculator = ErrorCalculator()
        self.statistics = MultiDimensionStatistics()

        # 方向测试统计（4个方向，针对串口2干扰测试口）
        self.direction_stats = [DirectionStatistics() for _ in range(4)]
        self._dir_enu_active_index = -1
        self._dir_auto_stop_enabled = False
        self._dir_auto_stop_sec = 300
        self._dir_auto_stop_epochs = 300
        # [新增] 串口3方向测试统计
        self.direction_stats3 = [DirectionStatistics() for _ in range(4)]
        self._dir3_enu_active_index = -1
        self._dir3_auto_stop_enabled = False
        self._dir3_auto_stop_sec = 300
        self._dir3_auto_stop_epochs = 300
        self._latest_enu2_east = 0.0
        self._latest_enu2_north = 0.0
        self._latest_enu2_up = 0.0
        # [新增] 串口3最新ENU值
        self._latest_enu3_east = 0.0
        self._latest_enu3_north = 0.0
        self._latest_enu3_up = 0.0

        self._p2_gga_new_epoch = False
        self._last_enu1_pos = None
        self._last_enu2_pos = None
        self._latest_p2_quality = 0
        # [新增] 串口3相关GGA标记
        self._p3_gga_new_epoch = False
        self._latest_p3_quality = 0
        self._last_enu3_pos = None

        # GPS时间跟踪
        self.gps_week = 0
        self.gps_sow = 0.0  # seconds of week

        # 双串口卫星信噪比数据
        self.port1_satellites = {}  # {prn: {'snr': snr, 'system': sys}}
        self.port2_satellites = {}  # {prn: {'snr': snr, 'system': sys}}
        self._port1_snr_signals = {}
        self._port2_snr_signals = {}
        # [新增] 串口3卫星信噪比数据
        self.port3_satellites = {}
        self._port3_snr_signals = {}

        # ENU 误差基准点 - 串口1 (无干扰)
        self.enu1_ref_point = None
        self.enu1_buffer = []
        self.enu1_buffer_size = 100
        self.enu1_ref_ready = False
        self.enu_auto_mode = True
        self.enu_instant_mode = False
        self.enu1_times = []
        self.enu1_east_data = []
        self.enu1_north_data = []
        self.enu1_up_data = []

        # [新增] COM3 独立的 ENU1 副本（串口3 VS 串口1对比用）
        self.enu1_3_times = []
        self.enu1_3_east_data = []
        self.enu1_3_north_data = []
        self.enu1_3_up_data = []

        # ENU 误差基准点 - 串口2 (干扰测试)
        self.enu2_ref_point = None
        self.enu2_buffer = []
        self.enu2_buffer_size = 100
        self.enu2_ref_ready = False
        self.enu2_times = []
        self.enu2_east_data = []
        self.enu2_north_data = []
        self.enu2_up_data = []

        # [新增] ENU 误差基准点 - 串口3 (干扰测试2)
        self.enu3_ref_point = None
        self.enu3_buffer = []
        self.enu3_buffer_size = 100
        self.enu3_ref_ready = False
        self.enu3_times = []
        self.enu3_east_data = []
        self.enu3_north_data = []
        self.enu3_up_data = []

        self.ENU_STD_WINDOW = 200

        # ENU异常值剔除参数
        self.ENU_OUTLIER_SIGMA = 8.0
        self.ENU_OUTLIER_MIN_DELTA = 10000.0
        self.ENU_OUTLIER_MIN_SAMPLES = 30
        self._enu1_outlier_count = 0
        self._enu2_outlier_count = 0
        # [新增] 串口3异常值计数
        self._enu3_outlier_count = 0

        # ENU数据数组上限（4Hz × 1800s = 7200点，约30分钟）
        self.ENU_MAX_POINTS = 7200

        # 滑动窗口标准差计算器（O(1)递推，替代 _running_std 的O(n)遍历）
        self._std_enu1_east = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu1_north = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu1_up = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu2_east = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu2_north = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu2_up = SlidingWindowStd(self.ENU_STD_WINDOW)
        # [新增] 串口3滑动窗口标准差
        self._std_enu3_east = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu3_north = SlidingWindowStd(self.ENU_STD_WINDOW)

        # [新增] COM3 独立 ENU1 滑动标准差
        self._std_enu1_3_east = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu1_3_north = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu1_3_up = SlidingWindowStd(self.ENU_STD_WINDOW)
        self._std_enu3_up = SlidingWindowStd(self.ENU_STD_WINDOW)

        # 卫星信噪比递推平均值跟踪
        self._port1_snr_avg = RunningAverage()
        self._port2_snr_avg = RunningAverage()
        self._snr_diff_avg = RunningAverage()
        self._last_port1_snr = {}
        self._last_port2_snr = {}
        self._last_snr_diff = {}
        # [新增] 串口3信噪比递推均值和差异跟踪
        self._port3_snr_avg = RunningAverage()
        self._last_port3_snr = {}
        self._snr_diff2_avg = RunningAverage()  # Port3 vs Port1 差值平均值
        self._last_snr_diff2 = {}

        # 双串口GGA简明数据
        self.port1_gga = None
        self.port2_gga = None
        self.p1_gga_nsat = 0
        self.p2_gga_nsat = 0
        self.p1_utc_ts = ""
        self.p2_utc_ts = ""
        # [新增] 串口3 GGA简明数据
        self.port3_gga = None
        self.p3_gga_nsat = 0
        self.p3_utc_ts = ""

        self.p1_ttff_s = 0.0
        self.p2_ttff_s = 0.0
        # [新增] 串口3 TTFF
        self.p3_ttff_s = 0.0

        # 双串口NMEA解析器
        self.nmea_parser2 = NMEAParser()  # 串口2专用解析器
        # [新增] 串口3专用NMEA解析器
        self.nmea_parser3 = NMEAParser()

        # 保存最新GGA数据（用于改正数计算）
        self.last_gga_data = None
        self.last_smoothed_data = None

        # 其他组件
        self.current_parser = None
        self.current_data_source = None
        self.serial_port1 = SerialManager(port_id=1)
        self.serial_port2 = SerialManager(port_id=2)
        # [新增] 串口3管理器
        self.serial_port3 = SerialManager(port_id=3)

        # 串口数据缓存（设上限防止内存无限增长）
        self.serial_save_buffer = deque(maxlen=40000)
        self.serial2_save_buffer = deque(maxlen=40000)
        # [新增] 串口3数据缓存
        self.serial3_save_buffer = deque(maxlen=40000)
        self.MAX_SERIAL_BUFFER = 40000
        
        # 自动日志保存
        self.auto_log_enabled = True
        self.auto_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log")
        os.makedirs(self.auto_log_dir, exist_ok=True)
        self._log_file1 = None
        self._log_file2 = None
        # [新增] 串口3日志文件
        self._log_file3 = None
        self._log_flush_count = 0
        self._LOG_FLUSH_INTERVAL = 50
        self._data_count_update = 0
        self._DATA_COUNT_INTERVAL = 10
        
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 12px;
                padding-top: 12px;
                color: #2c3e50;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 2px 8px;
                background-color: #ecf0f1;
                border-radius: 3px;
                color: #2c3e50;
            }
            QPushButton {
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                padding: 4px 12px;
                background-color: #f8f9fa;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #e8f0fe;
                border-color: #4a90d9;
            }
            QPushButton:pressed {
                background-color: #d0e1fd;
            }
            QPushButton:disabled {
                background-color: #e0e0e0;
                color: #999;
            }
            QTextEdit {
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                background-color: #fafafa;
            }
            QComboBox {
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 2px 6px;
                background-color: white;
            }
            QComboBox:hover {
                border-color: #4a90d9;
            }
        """)

        # 初始化 UI
        self.init_ui()
        self.setup_connections()
        self.refresh_serial_ports(1)
        self.refresh_serial_ports(2)
        self.refresh_serial_ports(3)  # [新增]
        self._open_auto_log_files()
        
        # 定时器更新（4 Hz，降低UI刷新频率避免长期运行卡顿）
        self.update_timer = QTimer()
        self.update_timer.setInterval(250)
        self.update_timer.timeout.connect(self.update_plots)
        self._update_frame_count = 0
        self._last_snr_data = None
    
    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # 创建QTabWidget框架
        self.tab_widget = QTabWidget()
        
        # ========== 标签1：表格视图 ==========
        table_tab = QWidget()
        table_layout = QVBoxLayout(table_tab)
        
        # 卫星信噪比数据 (GSV实时)
        snr_box = QGroupBox("卫星信噪比数据 (GSV) — 实时更新")
        snr_layout = QVBoxLayout(snr_box)
        self.snr_text1 = QTextEdit()
        self.snr_text1.setReadOnly(True)
        self.snr_text1.setStyleSheet("font-family: Consolas; font-size: 13px; background-color: #1e1e2e; color: #cdd6f4;")
        self.snr_text1.setPlaceholderText("等待GSV数据...")
        self.snr_text1.setMinimumHeight(200)
        snr_layout.addWidget(self.snr_text1)
        table_layout.addWidget(snr_box, 1)

        # 数据预览
        self.data_preview = QTextEdit()
        self.data_preview.setReadOnly(True)
        self.data_preview.setStyleSheet("font-family: Consolas; font-size: 11px;")
        self.data_preview.setPlaceholderText("数据预览区域...")
        self.data_preview.setMinimumHeight(60)
        table_layout.addWidget(self.data_preview, 0)

        table_layout.setStretch(0, 2)  # GSV数据占2份
        table_layout.setStretch(1, 1)  # 数据预览占1份
        
        self.tab_widget.addTab(table_tab, "表格视图")
        
        # ========== 标签2：功能与控制面板 ==========
        control_tab = QWidget()
        control_layout = QVBoxLayout(control_tab)
        
        # 顶部布局：串口1 + 串口2 + 真值设置 + 阈值设置
        top_layout = QHBoxLayout()

        # === 串口1（无干扰/主接收机）===
        port1_panel = QGroupBox("串口1 (无干扰)")
        port1_panel.setMinimumWidth(170)
        port1_layout = QVBoxLayout(port1_panel)

        self.port1_combo = QComboBox()
        self.port1_baud = QComboBox()
        self.port1_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.port1_baud.setCurrentText("9600")

        port1_data_layout = QGridLayout()
        port1_data_layout.addWidget(QLabel("串口号:"), 0, 0)
        port1_data_layout.addWidget(self.port1_combo, 0, 1)
        port1_data_layout.addWidget(QLabel("波特率:"), 1, 0)
        port1_data_layout.addWidget(self.port1_baud, 1, 1)

        self.port1_refresh_btn = QPushButton("刷新")
        self.port1_connect_btn = QPushButton("连接")
        self.port1_disconnect_btn = QPushButton("断开")
        self.port1_disconnect_btn.setEnabled(False)

        port1_btn_layout = QHBoxLayout()
        port1_btn_layout.addWidget(self.port1_refresh_btn)
        port1_btn_layout.addWidget(self.port1_connect_btn)
        port1_btn_layout.addWidget(self.port1_disconnect_btn)

        self.port1_status_label = QLabel("未连接")
        self.port1_status_label.setStyleSheet("color: red; font-weight: bold;")

        port1_layout.addLayout(port1_data_layout)
        port1_layout.addLayout(port1_btn_layout)
        port1_layout.addWidget(self.port1_status_label)
        port1_layout.addStretch()

        top_layout.addWidget(port1_panel)

        # === 串口2（干扰测试/参考接收机）===
        port2_panel = QGroupBox("串口2 (干扰测试)")
        port2_panel.setMinimumWidth(170)
        port2_layout = QVBoxLayout(port2_panel)

        self.port2_combo = QComboBox()
        self.port2_baud = QComboBox()
        self.port2_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.port2_baud.setCurrentText("9600")

        port2_data_layout = QGridLayout()
        port2_data_layout.addWidget(QLabel("串口号:"), 0, 0)
        port2_data_layout.addWidget(self.port2_combo, 0, 1)
        port2_data_layout.addWidget(QLabel("波特率:"), 1, 0)
        port2_data_layout.addWidget(self.port2_baud, 1, 1)

        self.port2_refresh_btn = QPushButton("刷新")
        self.port2_connect_btn = QPushButton("连接")
        self.port2_disconnect_btn = QPushButton("断开")
        self.port2_disconnect_btn.setEnabled(False)

        port2_btn_layout = QHBoxLayout()
        port2_btn_layout.addWidget(self.port2_refresh_btn)
        port2_btn_layout.addWidget(self.port2_connect_btn)
        port2_btn_layout.addWidget(self.port2_disconnect_btn)

        self.port2_status_label = QLabel("未连接")
        self.port2_status_label.setStyleSheet("color: red; font-weight: bold;")

        port2_layout.addLayout(port2_data_layout)
        port2_layout.addLayout(port2_btn_layout)
        port2_layout.addWidget(self.port2_status_label)
        port2_layout.addStretch()

        top_layout.addWidget(port2_panel)

        # [新增] === 串口3（干扰测试2 / 第二干扰口）===
        port3_panel = QGroupBox("串口3 (干扰测试2)")
        port3_panel.setMinimumWidth(170)
        port3_layout = QVBoxLayout(port3_panel)

        self.port3_combo = QComboBox()
        self.port3_baud = QComboBox()
        self.port3_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.port3_baud.setCurrentText("9600")

        port3_data_layout = QGridLayout()
        port3_data_layout.addWidget(QLabel("串口号:"), 0, 0)
        port3_data_layout.addWidget(self.port3_combo, 0, 1)
        port3_data_layout.addWidget(QLabel("波特率:"), 1, 0)
        port3_data_layout.addWidget(self.port3_baud, 1, 1)

        self.port3_refresh_btn = QPushButton("刷新")
        self.port3_connect_btn = QPushButton("连接")
        self.port3_disconnect_btn = QPushButton("断开")
        self.port3_disconnect_btn.setEnabled(False)

        port3_btn_layout = QHBoxLayout()
        port3_btn_layout.addWidget(self.port3_refresh_btn)
        port3_btn_layout.addWidget(self.port3_connect_btn)
        port3_btn_layout.addWidget(self.port3_disconnect_btn)

        self.port3_status_label = QLabel("未连接")
        self.port3_status_label.setStyleSheet("color: red; font-weight: bold;")

        port3_layout.addLayout(port3_data_layout)
        port3_layout.addLayout(port3_btn_layout)
        port3_layout.addWidget(self.port3_status_label)
        port3_layout.addStretch()

        top_layout.addWidget(port3_panel)
        
        top_layout.setStretch(0, 1)
        top_layout.setStretch(1, 1)
        top_layout.setStretch(2, 1)  # [新增] 串口3
        
        control_layout.addLayout(top_layout)

        # 双串口GGA状态面板：串口1（左）+ 串口2（右）
        port_stats_layout = QHBoxLayout()

        port1_stats = QGroupBox("串口1 GGA 状态")
        p1_grid = QGridLayout(port1_stats)
        p1_grid.addWidget(QLabel("UTC 时间:"), 0, 0)
        self.p1_utc_label = QLabel("-")
        self.p1_utc_label.setStyleSheet("font-weight: bold; color: #2980b9;")
        p1_grid.addWidget(self.p1_utc_label, 0, 1)
        p1_grid.addWidget(QLabel("纬度 (°):"), 1, 0)
        self.p1_lat_label = QLabel("-")
        self.p1_lat_label.setStyleSheet("font-weight: bold;")
        p1_grid.addWidget(self.p1_lat_label, 1, 1)
        p1_grid.addWidget(QLabel("经度 (°):"), 2, 0)
        self.p1_lon_label = QLabel("-")
        self.p1_lon_label.setStyleSheet("font-weight: bold;")
        p1_grid.addWidget(self.p1_lon_label, 2, 1)
        p1_grid.addWidget(QLabel("海拔高程 (m):"), 3, 0)
        self.p1_alt_label = QLabel("-")
        self.p1_alt_label.setStyleSheet("font-weight: bold;")
        p1_grid.addWidget(self.p1_alt_label, 3, 1)
        p1_grid.addWidget(QLabel("定位状态:"), 4, 0)
        self.p1_quality_label = QLabel("无数据")
        self.p1_quality_label.setStyleSheet("font-weight: bold; color: #e74c3c;")
        p1_grid.addWidget(self.p1_quality_label, 4, 1)
        p1_grid.addWidget(QLabel("卫星数:"), 5, 0)
        self.p1_nsats_label = QLabel("0")
        self.p1_nsats_label.setStyleSheet("font-weight: bold; color: #2980b9;")
        p1_grid.addWidget(self.p1_nsats_label, 5, 1)
        p1_grid.addWidget(QLabel("UBX TTFF (s):"), 6, 0)
        self.p1_ttff_label = QLabel("-")
        self.p1_ttff_label.setStyleSheet("font-weight: bold; color: #8e44ad;")
        p1_grid.addWidget(self.p1_ttff_label, 6, 1)
        port_stats_layout.addWidget(port1_stats)

        port2_stats = QGroupBox("串口2 GGA 状态")
        p2_grid = QGridLayout(port2_stats)
        p2_grid.addWidget(QLabel("UTC 时间:"), 0, 0)
        self.p2_utc_label = QLabel("-")
        self.p2_utc_label.setStyleSheet("font-weight: bold; color: #2980b9;")
        p2_grid.addWidget(self.p2_utc_label, 0, 1)
        p2_grid.addWidget(QLabel("纬度 (°):"), 1, 0)
        self.p2_lat_label = QLabel("-")
        self.p2_lat_label.setStyleSheet("font-weight: bold;")
        p2_grid.addWidget(self.p2_lat_label, 1, 1)
        p2_grid.addWidget(QLabel("经度 (°):"), 2, 0)
        self.p2_lon_label = QLabel("-")
        self.p2_lon_label.setStyleSheet("font-weight: bold;")
        p2_grid.addWidget(self.p2_lon_label, 2, 1)
        p2_grid.addWidget(QLabel("海拔高程 (m):"), 3, 0)
        self.p2_alt_label = QLabel("-")
        self.p2_alt_label.setStyleSheet("font-weight: bold;")
        p2_grid.addWidget(self.p2_alt_label, 3, 1)
        p2_grid.addWidget(QLabel("定位状态:"), 4, 0)
        self.p2_quality_label = QLabel("无数据")
        self.p2_quality_label.setStyleSheet("font-weight: bold; color: #e74c3c;")
        p2_grid.addWidget(self.p2_quality_label, 4, 1)
        p2_grid.addWidget(QLabel("卫星数:"), 5, 0)
        self.p2_nsats_label = QLabel("0")
        self.p2_nsats_label.setStyleSheet("font-weight: bold; color: #2980b9;")
        p2_grid.addWidget(self.p2_nsats_label, 5, 1)
        p2_grid.addWidget(QLabel("UBX TTFF (s):"), 6, 0)
        self.p2_ttff_label = QLabel("-")
        self.p2_ttff_label.setStyleSheet("font-weight: bold; color: #8e44ad;")
        p2_grid.addWidget(self.p2_ttff_label, 6, 1)
        port_stats_layout.addWidget(port2_stats)

        # [新增] 串口3 GGA状态面板
        port3_stats = QGroupBox("串口3 GGA 状态")
        p3_grid = QGridLayout(port3_stats)
        p3_grid.addWidget(QLabel("UTC 时间:"), 0, 0)
        self.p3_utc_label = QLabel("-")
        self.p3_utc_label.setStyleSheet("font-weight: bold; color: #2980b9;")
        p3_grid.addWidget(self.p3_utc_label, 0, 1)
        p3_grid.addWidget(QLabel("纬度 (°):"), 1, 0)
        self.p3_lat_label = QLabel("-")
        self.p3_lat_label.setStyleSheet("font-weight: bold;")
        p3_grid.addWidget(self.p3_lat_label, 1, 1)
        p3_grid.addWidget(QLabel("经度 (°):"), 2, 0)
        self.p3_lon_label = QLabel("-")
        self.p3_lon_label.setStyleSheet("font-weight: bold;")
        p3_grid.addWidget(self.p3_lon_label, 2, 1)
        p3_grid.addWidget(QLabel("海拔高程 (m):"), 3, 0)
        self.p3_alt_label = QLabel("-")
        self.p3_alt_label.setStyleSheet("font-weight: bold;")
        p3_grid.addWidget(self.p3_alt_label, 3, 1)
        p3_grid.addWidget(QLabel("定位状态:"), 4, 0)
        self.p3_quality_label = QLabel("无数据")
        self.p3_quality_label.setStyleSheet("font-weight: bold; color: #e74c3c;")
        p3_grid.addWidget(self.p3_quality_label, 4, 1)
        p3_grid.addWidget(QLabel("卫星数:"), 5, 0)
        self.p3_nsats_label = QLabel("0")
        self.p3_nsats_label.setStyleSheet("font-weight: bold; color: #2980b9;")
        p3_grid.addWidget(self.p3_nsats_label, 5, 1)
        p3_grid.addWidget(QLabel("UBX TTFF (s):"), 6, 0)
        self.p3_ttff_label = QLabel("-")
        self.p3_ttff_label.setStyleSheet("font-weight: bold; color: #8e44ad;")
        p3_grid.addWidget(self.p3_ttff_label, 6, 1)
        port_stats_layout.addWidget(port3_stats)

        control_layout.addLayout(port_stats_layout)

        # ENU 误差实时显示 - 串口1 (无干扰) 和 串口2 (干扰测试)
        enu_val_box = QGroupBox("ENU 误差实时值")
        enu_val_layout = QVBoxLayout(enu_val_box)
        enu1_val_row = QHBoxLayout()
        enu1_val_row.addWidget(QLabel("ENU1 (无干扰):"))
        self.enu1_east_label = QLabel("东: -- m")
        self.enu1_east_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
        enu1_val_row.addWidget(self.enu1_east_label)
        self.enu1_north_label = QLabel("北: -- m")
        self.enu1_north_label.setStyleSheet("color: #2980b9; font-weight: bold;")
        enu1_val_row.addWidget(self.enu1_north_label)
        self.enu1_up_label = QLabel("天: -- m")
        self.enu1_up_label.setStyleSheet("color: #27ae60; font-weight: bold;")
        enu1_val_row.addWidget(self.enu1_up_label)
        enu1_val_row.addStretch()
        enu_val_layout.addLayout(enu1_val_row)
        enu2_val_row = QHBoxLayout()
        enu2_val_row.addWidget(QLabel("ENU2 (干扰测试):"))
        self.enu2_east_label = QLabel("东: -- m")
        self.enu2_east_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
        enu2_val_row.addWidget(self.enu2_east_label)
        self.enu2_north_label = QLabel("北: -- m")
        self.enu2_north_label.setStyleSheet("color: #2980b9; font-weight: bold;")
        enu2_val_row.addWidget(self.enu2_north_label)
        self.enu2_up_label = QLabel("天: -- m")
        self.enu2_up_label.setStyleSheet("color: #27ae60; font-weight: bold;")
        enu2_val_row.addWidget(self.enu2_up_label)
        enu2_val_row.addStretch()
        enu_val_layout.addLayout(enu2_val_row)
        # [新增] ENU3 实时值行
        enu3_val_row = QHBoxLayout()
        enu3_val_row.addWidget(QLabel("ENU3 (干扰测试2):"))
        self.enu3_east_label = QLabel("东: -- m")
        self.enu3_east_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
        enu3_val_row.addWidget(self.enu3_east_label)
        self.enu3_north_label = QLabel("北: -- m")
        self.enu3_north_label.setStyleSheet("color: #2980b9; font-weight: bold;")
        enu3_val_row.addWidget(self.enu3_north_label)
        self.enu3_up_label = QLabel("天: -- m")
        self.enu3_up_label.setStyleSheet("color: #27ae60; font-weight: bold;")
        enu3_val_row.addWidget(self.enu3_up_label)
        enu3_val_row.addStretch()
        enu_val_layout.addLayout(enu3_val_row)
        control_layout.addWidget(enu_val_box)

        # ENU 标准差统计
        enu_std_box = QGroupBox("ENU 标准差统计")
        enu_std_layout = QVBoxLayout(enu_std_box)
        std_row1 = QHBoxLayout()
        std_row1.addWidget(QLabel("ENU1 (无干扰):"))
        self.enu1_std_east = QLabel("E: -- m")
        self.enu1_std_east.setStyleSheet("color: #e74c3c; font-weight: bold;")
        std_row1.addWidget(self.enu1_std_east)
        self.enu1_std_north = QLabel("N: -- m")
        self.enu1_std_north.setStyleSheet("color: #2980b9; font-weight: bold;")
        std_row1.addWidget(self.enu1_std_north)
        self.enu1_std_up = QLabel("U: -- m")
        self.enu1_std_up.setStyleSheet("color: #27ae60; font-weight: bold;")
        std_row1.addWidget(self.enu1_std_up)
        std_row1.addStretch()
        enu_std_layout.addLayout(std_row1)
        std_row2 = QHBoxLayout()
        std_row2.addWidget(QLabel("ENU2 (干扰测试):"))
        self.enu2_std_east = QLabel("E: -- m")
        self.enu2_std_east.setStyleSheet("color: #e74c3c; font-weight: bold;")
        std_row2.addWidget(self.enu2_std_east)
        self.enu2_std_north = QLabel("N: -- m")
        self.enu2_std_north.setStyleSheet("color: #2980b9; font-weight: bold;")
        std_row2.addWidget(self.enu2_std_north)
        self.enu2_std_up = QLabel("U: -- m")
        self.enu2_std_up.setStyleSheet("color: #27ae60; font-weight: bold;")
        std_row2.addWidget(self.enu2_std_up)
        std_row2.addStretch()
        enu_std_layout.addLayout(std_row2)
        # [新增] ENU3标准差行
        std_row3 = QHBoxLayout()
        std_row3.addWidget(QLabel("ENU3 (干扰测试2):"))
        self.enu3_std_east = QLabel("E: -- m")
        self.enu3_std_east.setStyleSheet("color: #e74c3c; font-weight: bold;")
        std_row3.addWidget(self.enu3_std_east)
        self.enu3_std_north = QLabel("N: -- m")
        self.enu3_std_north.setStyleSheet("color: #2980b9; font-weight: bold;")
        std_row3.addWidget(self.enu3_std_north)
        self.enu3_std_up = QLabel("U: -- m")
        self.enu3_std_up.setStyleSheet("color: #27ae60; font-weight: bold;")
        std_row3.addWidget(self.enu3_std_up)
        std_row3.addStretch()
        enu_std_layout.addLayout(std_row3)
        control_layout.addWidget(enu_std_box)

        # 日志窗口
        self.log_window = QTextEdit()
        self.log_window.setReadOnly(True)
        self.log_window.setStyleSheet("font-family: Consolas; font-size: 11px;")
        self.log_window.setMaximumHeight(150)
        control_layout.addWidget(self.log_window)

        # 测试基本信息
        info_group = QGroupBox("测试基本信息")
        info_layout = QGridLayout(info_group)

        info_layout.addWidget(QLabel("测试地点:"), 0, 0)
        self.test_location_input = QLineEdit()
        self.test_location_input.setPlaceholderText("例如: 长沙")
        info_layout.addWidget(self.test_location_input, 0, 1)

        info_layout.addWidget(QLabel("待测设备型号:"), 0, 2)
        self.dut_model_input = QLineEdit()
        self.dut_model_input.setPlaceholderText("例如: 双频八通道抗干扰天线")
        info_layout.addWidget(self.dut_model_input, 0, 3)

        info_layout.addWidget(QLabel("阵列形式:"), 1, 0)
        self.array_form_combo = QComboBox()
        self.array_form_combo.setEditable(True)
        self.array_form_combo.addItems(["4 阵元", "7 阵元", "线阵", "面阵"])
        info_layout.addWidget(self.array_form_combo, 1, 1)

        info_layout.addWidget(QLabel("干扰类型:"), 1, 2)
        self.jam_type_combo = QComboBox()
        self.jam_type_combo.setEditable(True)
        self.jam_type_combo.addItems(["线性扫频干扰", "宽带干扰", "单音干扰", "脉冲干扰", "窄带干扰", "多音干扰"])
        info_layout.addWidget(self.jam_type_combo, 1, 3)

        info_layout.addWidget(QLabel("入射方位:"), 2, 0)
        self.azimuth_input = QLineEdit()
        self.azimuth_input.setPlaceholderText("例如: 正东 / 正南 / 正西 / 正北")
        info_layout.addWidget(self.azimuth_input, 2, 1)

        info_layout.addWidget(QLabel("收发天线间距:"), 2, 2)
        self.antenna_spacing_input = QLineEdit()
        self.antenna_spacing_input.setPlaceholderText("例如: 3 m")
        info_layout.addWidget(self.antenna_spacing_input, 2, 3)

        info_layout.addWidget(QLabel("天线架设高度:"), 3, 0)
        self.antenna_height_input = QLineEdit()
        self.antenna_height_input.setPlaceholderText("例如: 1.5 m")
        info_layout.addWidget(self.antenna_height_input, 3, 1)

        control_layout.addWidget(info_group)
        
        # 设置控制标签页的整体拉伸比例
        control_layout.setStretch(0, 1)  # 顶部面板占1份
        control_layout.setStretch(1, 1)  # 统计面板占1份
        control_layout.setStretch(2, 0)  # 日志窗口固定
        control_layout.setStretch(3, 0)  # 设备序号行固定
        control_layout.setStretch(4, 0)  # 测试基本信息固定
        control_layout.setStretch(5, 0)  # 底部按钮固定
        
        # 最底部：操作按钮
        bottom_layout = QHBoxLayout()
        self.clear_data_btn = QPushButton("清空数据")
        self.clear_data_btn.setMinimumWidth(100)
        self.reset_stats_btn = QPushButton("重置统计")
        self.reset_stats_btn.setMinimumWidth(100)
        self.clear_log_btn = QPushButton("清除日志")
        self.clear_log_btn.setMinimumWidth(100)
        self.save_all_log_btn = QPushButton("保存全部日志")
        self.save_all_log_btn.setMinimumWidth(110)
        # [新增] 串口2独立报告按钮
        self.export_report2_btn = QPushButton("导出串口2测试报告")
        self.export_report2_btn.setMinimumWidth(140)
        self.export_report2_btn.setStyleSheet("QPushButton { background-color: #27ae60; color: white; font-weight: bold; } QPushButton:hover { background-color: #2ecc71; }")
        self.export_pdf2_btn = QPushButton("导出串口2PDF报告")
        self.export_pdf2_btn.setMinimumWidth(140)
        self.export_pdf2_btn.setStyleSheet("QPushButton { background-color: #2980b9; color: white; font-weight: bold; } QPushButton:hover { background-color: #3498db; }")
        # [新增] 串口3独立报告按钮
        self.export_report3_btn = QPushButton("导出串口3测试报告")
        self.export_report3_btn.setMinimumWidth(140)
        self.export_report3_btn.setStyleSheet("QPushButton { background-color: #27ae60; color: white; font-weight: bold; } QPushButton:hover { background-color: #2ecc71; }")
        self.export_pdf3_btn = QPushButton("导出串口3PDF报告")
        self.export_pdf3_btn.setMinimumWidth(140)
        self.export_pdf3_btn.setStyleSheet("QPushButton { background-color: #2980b9; color: white; font-weight: bold; } QPushButton:hover { background-color: #3498db; }")

        bottom_layout.addWidget(self.clear_data_btn)
        bottom_layout.addWidget(self.reset_stats_btn)
        bottom_layout.addWidget(self.clear_log_btn)
        bottom_layout.addWidget(self.save_all_log_btn)
        bottom_layout.addWidget(self.export_report2_btn)
        bottom_layout.addWidget(self.export_pdf2_btn)
        bottom_layout.addWidget(self.export_report3_btn)
        bottom_layout.addWidget(self.export_pdf3_btn)

        self.auto_log_cb = QCheckBox("自动保存日志")
        self.auto_log_cb.setChecked(True)
        bottom_layout.addWidget(self.auto_log_cb)
        bottom_layout.addStretch()

        control_layout.addLayout(bottom_layout)
        
        self.tab_widget.addTab(control_tab, "功能与控制面板")
        
        # ========== 标签4：ENU 误差对比 ==========
        enu_comp_tab = QWidget()
        enu_comp_layout = QVBoxLayout(enu_comp_tab)

        # 标题行
        title_row = QHBoxLayout()
        title_label12 = QLabel("ENU1 (无干扰) vs ENU2 (干扰测试)")
        title_label12.setStyleSheet("font-weight: bold; font-size: 14px; color: #2c3e50;")
        title_label13 = QLabel("ENU1 (无干扰) vs ENU3 (干扰测试2)")
        title_label13.setStyleSheet("font-weight: bold; font-size: 14px; color: #2c3e50;")
        title_row.addWidget(title_label12)
        title_row.addSpacing(40)
        title_row.addWidget(title_label13)
        title_row.addStretch()
        enu_comp_layout.addLayout(title_row)

        # 双列布局
        enu_comp_cols = QHBoxLayout()

        # --- 左列: ENU1 vs ENU2 ---
        left_col = QVBoxLayout()
        left_col.setSpacing(2)

        self.enu_comp12_east_plot = pg.PlotWidget()
        self.enu_comp12_east_plot.setBackground('w')
        self.enu_comp12_east_plot.setLabel('left', '东向', units='m')
        self.enu_comp12_east_plot.showGrid(x=True, y=True)
        self.enu_comp12_east_plot.addLegend()
        self.enu1_east_curve = self.enu_comp12_east_plot.plot([], [], pen='r', name='ENU1 (无干扰)')
        self.enu2_east_curve = self.enu_comp12_east_plot.plot([], [], pen='b', name='ENU2 (干扰测试)')
        left_col.addWidget(self.enu_comp12_east_plot)

        self.enu_comp12_north_plot = pg.PlotWidget()
        self.enu_comp12_north_plot.setBackground('w')
        self.enu_comp12_north_plot.setLabel('left', '北向', units='m')
        self.enu_comp12_north_plot.showGrid(x=True, y=True)
        self.enu_comp12_north_plot.addLegend()
        self.enu1_north_curve = self.enu_comp12_north_plot.plot([], [], pen='r', name='ENU1 (无干扰)')
        self.enu2_north_curve = self.enu_comp12_north_plot.plot([], [], pen='b', name='ENU2 (干扰测试)')
        left_col.addWidget(self.enu_comp12_north_plot)

        self.enu_comp12_up_plot = pg.PlotWidget()
        self.enu_comp12_up_plot.setBackground('w')
        self.enu_comp12_up_plot.setLabel('left', '天向', units='m')
        self.enu_comp12_up_plot.setLabel('bottom', '时间', units='s')
        self.enu_comp12_up_plot.showGrid(x=True, y=True)
        self.enu_comp12_up_plot.addLegend()
        self.enu1_up_curve = self.enu_comp12_up_plot.plot([], [], pen='r', name='ENU1 (无干扰)')
        self.enu2_up_curve = self.enu_comp12_up_plot.plot([], [], pen='b', name='ENU2 (干扰测试)')
        left_col.addWidget(self.enu_comp12_up_plot)

        enu_comp_cols.addLayout(left_col)

        # --- 右列: ENU1 vs ENU3 ---
        right_col = QVBoxLayout()
        right_col.setSpacing(2)

        self.enu_comp13_east_plot = pg.PlotWidget()
        self.enu_comp13_east_plot.setBackground('w')
        self.enu_comp13_east_plot.setLabel('left', '东向', units='m')
        self.enu_comp13_east_plot.showGrid(x=True, y=True)
        self.enu_comp13_east_plot.addLegend()
        self.enu1_13_east_curve = self.enu_comp13_east_plot.plot([], [], pen='r', name='ENU1 (无干扰)')
        self.enu3_east_curve = self.enu_comp13_east_plot.plot([], [], pen={'color': '#f39c12', 'width': 2, 'style': Qt.DashLine}, name='ENU3 (干扰测试2)')
        right_col.addWidget(self.enu_comp13_east_plot)

        self.enu_comp13_north_plot = pg.PlotWidget()
        self.enu_comp13_north_plot.setBackground('w')
        self.enu_comp13_north_plot.setLabel('left', '北向', units='m')
        self.enu_comp13_north_plot.showGrid(x=True, y=True)
        self.enu_comp13_north_plot.addLegend()
        self.enu1_13_north_curve = self.enu_comp13_north_plot.plot([], [], pen='r', name='ENU1 (无干扰)')
        self.enu3_north_curve = self.enu_comp13_north_plot.plot([], [], pen={'color': '#f39c12', 'width': 2, 'style': Qt.DashLine}, name='ENU3 (干扰测试2)')
        right_col.addWidget(self.enu_comp13_north_plot)

        self.enu_comp13_up_plot = pg.PlotWidget()
        self.enu_comp13_up_plot.setBackground('w')
        self.enu_comp13_up_plot.setLabel('left', '天向', units='m')
        self.enu_comp13_up_plot.setLabel('bottom', '时间', units='s')
        self.enu_comp13_up_plot.showGrid(x=True, y=True)
        self.enu_comp13_up_plot.addLegend()
        self.enu1_13_up_curve = self.enu_comp13_up_plot.plot([], [], pen='r', name='ENU1 (无干扰)')
        self.enu3_up_curve = self.enu_comp13_up_plot.plot([], [], pen={'color': '#f39c12', 'width': 2, 'style': Qt.DashLine}, name='ENU3 (干扰测试2)')
        right_col.addWidget(self.enu_comp13_up_plot)

        enu_comp_cols.addLayout(right_col)

        enu_comp_layout.addLayout(enu_comp_cols)
        self.tab_widget.addTab(enu_comp_tab, "ENU 误差对比")
        
        # ========== 标签5：方向测试 ==========
        direction_tab = self._create_direction_tab()
        self.tab_widget.addTab(direction_tab, "方向测试")

        # ========== 标签6：各方向 ENU 误差对比 ==========
        dir_enu_tab = self._create_dir_enu_tab()
        self.tab_widget.addTab(dir_enu_tab, "方向ENU对比")
        
        # 将QTabWidget添加到主布局
        main_layout.addWidget(self.tab_widget)

        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        self.port1_indicator = QLabel("串口1: ● 未连接")
        self.port1_indicator.setStyleSheet("color: red; font-weight: bold; font-size: 13px; padding: 0 10px;")
        self.status_bar.addWidget(self.port1_indicator)

        sep1 = QLabel("|")
        sep1.setStyleSheet("color: #666;")
        self.status_bar.addWidget(sep1)

        self.port2_indicator = QLabel("串口2: ● 未连接")
        self.port2_indicator.setStyleSheet("color: red; font-weight: bold; font-size: 13px; padding: 0 10px;")
        self.status_bar.addWidget(self.port2_indicator)

        sep3 = QLabel("|")
        sep3.setStyleSheet("color: #666;")
        self.status_bar.addWidget(sep3)
        # [新增] 串口3状态指示器
        self.port3_indicator = QLabel("串口3: ● 未连接")
        self.port3_indicator.setStyleSheet("color: red; font-weight: bold; font-size: 13px; padding: 0 10px;")
        self.status_bar.addWidget(self.port3_indicator)

        sep4 = QLabel("|")
        sep4.setStyleSheet("color: #666;")
        self.status_bar.addWidget(sep4)

        self.data_count_label = QLabel("已读取: 0 行")
        self.data_count_label.setStyleSheet("font-size: 12px; color: #666;")
        self.status_bar.addPermanentWidget(self.data_count_label)
    
    def setup_connections(self):
        # 串口1
        self.port1_refresh_btn.clicked.connect(lambda: self.refresh_serial_ports(1))
        self.port1_connect_btn.clicked.connect(lambda: self.connect_serial(1))
        self.port1_disconnect_btn.clicked.connect(lambda: self.disconnect_serial(1))

        # 串口2
        self.port2_refresh_btn.clicked.connect(lambda: self.refresh_serial_ports(2))
        self.port2_connect_btn.clicked.connect(lambda: self.connect_serial(2))
        self.port2_disconnect_btn.clicked.connect(lambda: self.disconnect_serial(2))

        # [新增] 串口3按钮连接
        self.port3_refresh_btn.clicked.connect(lambda: self.refresh_serial_ports(3))
        self.port3_connect_btn.clicked.connect(lambda: self.connect_serial(3))
        self.port3_disconnect_btn.clicked.connect(lambda: self.disconnect_serial(3))

        self.clear_log_btn.clicked.connect(self.clear_log)
        self.save_all_log_btn.clicked.connect(self._save_all_serial_logs)

        self.export_report2_btn.clicked.connect(self.export_test_report2)
        self.export_pdf2_btn.clicked.connect(self.export_pdf_report2)
        self.export_report3_btn.clicked.connect(self.export_test_report3)
        self.export_pdf3_btn.clicked.connect(self.export_pdf_report3)

        self.auto_log_cb.stateChanged.connect(self._on_auto_log_toggled)

        # ENU基准模式切换（共享控制）
        self._enu_btn_group = QButtonGroup()
        self._enu_btn_group.addButton(self.enu_auto_radio)
        self._enu_btn_group.addButton(self.enu_instant_radio)
        self._enu_btn_group.addButton(self.enu_manual_radio)
        self._enu_btn_group.buttonClicked.connect(lambda btn: self._enu_mode_changed())
        self.enu_apply_btn.clicked.connect(self._apply_enu_manual_ref)

        # 统计操作
        self.reset_stats_btn.clicked.connect(self.reset_all_stats)
        self.clear_data_btn.clicked.connect(self.clear_all_data)

        # 模块信号
        self.serial_port1.data_received.connect(self.handle_data)
        self.serial_port1.connection_status.connect(self.update_connection_status)
        self.serial_port1.error_occurred.connect(self.handle_serial_error)

        self.serial_port2.data_received.connect(self.handle_data)
        self.serial_port2.connection_status.connect(self.update_connection_status)
        self.serial_port2.error_occurred.connect(self.handle_serial_error)

        self.serial_port1.ubx_received.connect(self.handle_ubx)
        self.serial_port2.ubx_received.connect(self.handle_ubx)

        # [新增] 串口3模块信号
        self.serial_port3.data_received.connect(self.handle_data)
        self.serial_port3.connection_status.connect(self.update_connection_status)
        self.serial_port3.error_occurred.connect(self.handle_serial_error)
        self.serial_port3.ubx_received.connect(self.handle_ubx)


    
    def refresh_serial_ports(self, port_id=1):
        if port_id == 1:
            combo = self.port1_combo
            manager = self.serial_port1
        elif port_id == 2:
            combo = self.port2_combo
            manager = self.serial_port2
        else:  # [新增] port_id == 3
            combo = self.port3_combo
            manager = self.serial_port3
        combo.clear()
        ports = manager.get_available_ports()
        if ports:
            combo.addItems(ports)
        else:
            self.log_info(f"串口{port_id}: 未检测到可用串口")
    
    def connect_serial(self, port_id=1):
        if port_id == 1:
            manager = self.serial_port1
            combo = self.port1_combo
            baud_widget = self.port1_baud
        elif port_id == 2:
            manager = self.serial_port2
            combo = self.port2_combo
            baud_widget = self.port2_baud
        else:  # [新增] port_id == 3
            manager = self.serial_port3
            combo = self.port3_combo
            baud_widget = self.port3_baud

        port = combo.currentText()
        baud = int(baud_widget.currentText())

        if not port:
            QMessageBox.warning(self, "警告", f"请选择串口{port_id}的串口号")
            return

        if port_id == 1:
            self.clear_data_preview()
            self.current_data_source = 'serial'
            self.current_parser = NMEAParser()

        manager.connect(port, baud)
        self.update_timer.start()

    def disconnect_serial(self, port_id=1):
        if port_id == 1:
            manager = self.serial_port1
        elif port_id == 2:
            manager = self.serial_port2
        else:  # [新增] port_id == 3
            manager = self.serial_port3
        manager.disconnect()
        # [新增] 三个串口均断开时停止定时器
        if not self.serial_port1.serial_port and not self.serial_port2.serial_port and not self.serial_port3.serial_port:
            self.update_timer.stop()

    def update_connection_status(self, connected, port_id):
        if port_id == 1:
            indicator = self.port1_indicator
            status_label = self.port1_status_label
            connect_btn = self.port1_connect_btn
            disconnect_btn = self.port1_disconnect_btn
            prefix = "串口1"
        elif port_id == 2:
            indicator = self.port2_indicator
            status_label = self.port2_status_label
            connect_btn = self.port2_connect_btn
            disconnect_btn = self.port2_disconnect_btn
            prefix = "串口2"
        else:  # [新增] port_id == 3
            indicator = self.port3_indicator
            status_label = self.port3_status_label
            connect_btn = self.port3_connect_btn
            disconnect_btn = self.port3_disconnect_btn
            prefix = "串口3"

        if connected:
            indicator.setText(f"{prefix}: ● 已连接")
            indicator.setStyleSheet("color: green; font-weight: bold; font-size: 12px; padding: 0 10px;")
            status_label.setText("已连接")
            status_label.setStyleSheet("color: green; font-weight: bold;")
            connect_btn.setEnabled(False)
            disconnect_btn.setEnabled(True)
            self.log_info(f"{prefix} 连接成功")
        else:
            indicator.setText(f"{prefix}: ● 未连接")
            indicator.setStyleSheet("color: red; font-weight: bold; font-size: 12px; padding: 0 10px;")
            status_label.setText("未连接")
            status_label.setStyleSheet("color: red; font-weight: bold;")
            connect_btn.setEnabled(True)
            disconnect_btn.setEnabled(False)
            self.log_info(f"{prefix} 已断开")

    def handle_serial_error(self, error_message, port_id):
        self.log_error(error_message)

    def handle_ubx(self, frame, port_id):
        msg_type, data = parse_ubx_frame(frame)
        if msg_type != 'NAV_STATUS' or data is None:
            return

        ttff_s = data['ttff_s']

        if port_id == 1:
            self.p1_ttff_s = ttff_s
            if ttff_s > 0:
                self.p1_ttff_label.setText(f"{ttff_s:.1f}")
            else:
                self.p1_ttff_label.setText("0.0 (冷启动中)")
        elif port_id == 2:
            self.p2_ttff_s = ttff_s
            if ttff_s > 0:
                self.p2_ttff_label.setText(f"{ttff_s:.1f}")
            else:
                self.p2_ttff_label.setText("0.0 (冷启动中)")
        else:  # [新增] port_id == 3
            self.p3_ttff_s = ttff_s
            if ttff_s > 0:
                self.p3_ttff_label.setText(f"{ttff_s:.1f}")
            else:
                self.p3_ttff_label.setText("0.0 (冷启动中)")
    
    @staticmethod
    def _dmm_to_dd(dmm, direction):
        degrees = int(dmm // 100)
        minutes = dmm % 100
        value = degrees + minutes / 60.0
        if direction in ('S', 'W'):
            value = -value
        return value

    def handle_data(self, data, port_id=1):
        if port_id == 2:
            self.serial2_save_buffer.append(data)
            self._write_auto_log(2, data)

            self._data_count_update += 1
            if self._data_count_update >= self._DATA_COUNT_INTERVAL:
                total = len(self.serial2_save_buffer) + len(self.serial_save_buffer) + len(self.serial3_save_buffer)
                self.data_count_label.setText(f"已读取: {total} 行")
                self._data_count_update = 0

            decoded = self.nmea_parser2.decode_line(data) if self.nmea_parser2 else None
            if decoded:
                self.data_preview.append(f"[串口2] {decoded.strip()}")

                if self.data_preview.document().blockCount() > 500:
                    cursor = self.data_preview.textCursor()
                    cursor.movePosition(QTextCursor.Start)
                    cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 200)
                    cursor.removeSelectedText()
                    cursor.movePosition(QTextCursor.End)

                # 解析GSV获取卫星信噪比（仅GSV行触发）
                is_gsv = False
                if decoded.startswith('$GP') and 'GSV' in decoded[3:6]:
                    self.nmea_parser2.parse(decoded)
                    is_gsv = True
                elif decoded.startswith('$BD') and 'GSV' in decoded[3:6]:
                    self.nmea_parser2.parse(decoded)
                    is_gsv = True
                elif decoded.startswith('$GL') and 'GSV' in decoded[3:6]:
                    self.nmea_parser2.parse(decoded)
                    is_gsv = True
                elif decoded.startswith('$GA') and 'GSV' in decoded[3:6]:
                    self.nmea_parser2.parse(decoded)
                    is_gsv = True
                elif decoded.startswith('$GB') and 'GSV' in decoded[3:6]:
                    self.nmea_parser2.parse(decoded)
                    is_gsv = True

                if is_gsv:
                    entry = self.nmea_parser2.gpgsv_data[-1]
                    talker = entry.get('talker_id', 'GP')
                    signal_id = entry.get('signal_id', 0)
                    system_key = TALKER_TO_SYSTEM.get(talker, 'G')
                    system_signals = SIGNAL_NAMES.get(system_key, SIGNAL_NAMES['G'])
                    signal_name = system_signals.get(signal_id, f'S{signal_id}')
                    for sat in entry.get('satellites', []):
                        prn = sat.get('prn', 0)
                        snr = sat.get('snr', 0)
                        if prn and 0 < snr <= 99:
                            key = f"{talker}{prn:02d}"
                            self.port2_satellites[key] = snr
                            self._port2_snr_signals[key] = signal_name

            for key, snr in self.port2_satellites.items():
                if self._last_port2_snr.get(key) != snr:
                    self._last_port2_snr[key] = snr
                    self._port2_snr_avg.add(key, snr)

            # 解析GGA获取定位状态
            if decoded.startswith('$GNGGA') or decoded.startswith('$GPGGA') or \
               decoded.startswith('$BDGGA') or decoded.startswith('$GLGGA') or \
               decoded.startswith('$GAGGA') or decoded.startswith('$GBGGA'):
                self.nmea_parser2.parse(decoded)
                if self.nmea_parser2.gpgga_data:
                    gga = self.nmea_parser2.gpgga_data[-1]
                    lat_str = gga.get('latitude', '')
                    lon_str = gga.get('longitude', '')
                    lat_dir = gga.get('lat_dir', 'N')
                    lon_dir = gga.get('lon_dir', 'E')
                    p2_lat = self._dmm_to_dd(float(lat_str) if lat_str else 0.0, lat_dir) or 0.0
                    p2_lon = self._dmm_to_dd(float(lon_str) if lon_str else 0.0, lon_dir) or 0.0

                    self.p2_lat_label.setText(f"{p2_lat:.8f}")
                    self.p2_lon_label.setText(f"{p2_lon:.8f}")
                    alt = float(gga.get('altitude', 0))
                    self.p2_alt_label.setText(f"{alt:.3f}")
                    q = int(gga.get('fix_quality', 0))
                    quality_text = {0: "无效", 1: "单点定位", 2: "单点定位", 3: "无效PPS", 4: "RTK固定", 5: "RTK浮点", 6: "正在估算"}
                    self.p2_quality_label.setText(quality_text.get(q, f"未知({q})"))
                    self.p2_quality_label.setStyleSheet(
                        "font-weight: bold; color: #e74c3c;" if q == 0 else
                        "font-weight: bold; color: #f39c12;" if q == 1 else
                        "font-weight: bold; color: #2980b9;" if q == 2 else
                        "font-weight: bold; color: #27ae60;"
                    )
                    ts = gga.get('timestamp', '')
                    if ts:
                        self.p2_utc_label.setText(ts)
                        self.p2_utc_ts = ts
                    gga_sats = int(gga.get('satellites_used', 0))
                    self.p2_nsats_label.setText(str(gga_sats))
                    self.p2_gga_nsat = gga_sats

                    self._latest_p2_quality = q
                    self._p2_gga_new_epoch = True

                    if q > 0:
                        self._feed_enu2_buffer(p2_lat, p2_lon, alt)
                        self._calc_and_update_enu2(p2_lat, p2_lon, alt,
                            self.enu2_ref_point, self.enu2_ref_ready,
                            self.enu2_east_label, self.enu2_north_label, self.enu2_up_label,
                            self.enu2_times, self.enu2_east_data, self.enu2_north_data, self.enu2_up_data,
                            self._std_enu2_east, self._std_enu2_north, self._std_enu2_up, "ENU2")
                        self._update_enu_std()

            if decoded.startswith('$BDRMC') or decoded.startswith('$GPRMC'):
                self.nmea_parser2.parse(decoded)
            if decoded.startswith('#OBSV'):
                self.nmea_parser2.parse(decoded)
            return

        # [新增] 串口3数据：完整处理（与串口2完全对称）
        if port_id == 3:
            self.serial3_save_buffer.append(data)
            self._write_auto_log(3, data)

            self._data_count_update += 1
            if self._data_count_update >= self._DATA_COUNT_INTERVAL:
                total = len(self.serial3_save_buffer) + len(self.serial2_save_buffer) + len(self.serial_save_buffer)
                self.data_count_label.setText(f"已读取: {total} 行")
                self._data_count_update = 0

            decoded = self.nmea_parser3.decode_line(data) if self.nmea_parser3 else None
            if decoded:
                self.data_preview.append(f"[串口3] {decoded.strip()}")

                if self.data_preview.document().blockCount() > 500:
                    cursor = self.data_preview.textCursor()
                    cursor.movePosition(QTextCursor.Start)
                    cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 200)
                    cursor.removeSelectedText()
                    cursor.movePosition(QTextCursor.End)

                # 解析GSV获取卫星信噪比
                is_gsv = False
                if decoded.startswith('$GP') and 'GSV' in decoded[3:6]:
                    self.nmea_parser3.parse(decoded)
                    is_gsv = True
                elif decoded.startswith('$BD') and 'GSV' in decoded[3:6]:
                    self.nmea_parser3.parse(decoded)
                    is_gsv = True
                elif decoded.startswith('$GL') and 'GSV' in decoded[3:6]:
                    self.nmea_parser3.parse(decoded)
                    is_gsv = True
                elif decoded.startswith('$GA') and 'GSV' in decoded[3:6]:
                    self.nmea_parser3.parse(decoded)
                    is_gsv = True
                elif decoded.startswith('$GB') and 'GSV' in decoded[3:6]:
                    self.nmea_parser3.parse(decoded)
                    is_gsv = True

                if is_gsv:
                    entry = self.nmea_parser3.gpgsv_data[-1]
                    talker = entry.get('talker_id', 'GP')
                    signal_id = entry.get('signal_id', 0)
                    system_key = TALKER_TO_SYSTEM.get(talker, 'G')
                    system_signals = SIGNAL_NAMES.get(system_key, SIGNAL_NAMES['G'])
                    signal_name = system_signals.get(signal_id, f'S{signal_id}')
                    for sat in entry.get('satellites', []):
                        prn = sat.get('prn', 0)
                        snr = sat.get('snr', 0)
                        if prn and 0 < snr <= 99:
                            key = f"{talker}{prn:02d}"
                            self.port3_satellites[key] = snr
                            self._port3_snr_signals[key] = signal_name

            for key, snr in self.port3_satellites.items():
                if self._last_port3_snr.get(key) != snr:
                    self._last_port3_snr[key] = snr
                    self._port3_snr_avg.add(key, snr)

            # 解析GGA获取定位状态
            if decoded.startswith('$GNGGA') or decoded.startswith('$GPGGA') or \
               decoded.startswith('$BDGGA') or decoded.startswith('$GLGGA') or \
               decoded.startswith('$GAGGA') or decoded.startswith('$GBGGA'):
                self.nmea_parser3.parse(decoded)
                if self.nmea_parser3.gpgga_data:
                    gga = self.nmea_parser3.gpgga_data[-1]
                    lat_str = gga.get('latitude', '')
                    lon_str = gga.get('longitude', '')
                    lat_dir = gga.get('lat_dir', 'N')
                    lon_dir = gga.get('lon_dir', 'E')
                    p3_lat = self._dmm_to_dd(float(lat_str) if lat_str else 0.0, lat_dir) or 0.0
                    p3_lon = self._dmm_to_dd(float(lon_str) if lon_str else 0.0, lon_dir) or 0.0

                    self.p3_lat_label.setText(f"{p3_lat:.8f}")
                    self.p3_lon_label.setText(f"{p3_lon:.8f}")
                    alt = float(gga.get('altitude', 0))
                    self.p3_alt_label.setText(f"{alt:.3f}")
                    q = int(gga.get('fix_quality', 0))
                    quality_text = {0: "无效", 1: "单点定位", 2: "单点定位", 3: "无效PPS", 4: "RTK固定", 5: "RTK浮点", 6: "正在估算"}
                    self.p3_quality_label.setText(quality_text.get(q, f"未知({q})"))
                    self.p3_quality_label.setStyleSheet(
                        "font-weight: bold; color: #e74c3c;" if q == 0 else
                        "font-weight: bold; color: #f39c12;" if q == 1 else
                        "font-weight: bold; color: #2980b9;" if q == 2 else
                        "font-weight: bold; color: #27ae60;"
                    )
                    ts = gga.get('timestamp', '')
                    if ts:
                        self.p3_utc_label.setText(ts)
                        self.p3_utc_ts = ts
                    gga_sats = int(gga.get('satellites_used', 0))
                    self.p3_nsats_label.setText(str(gga_sats))
                    self.p3_gga_nsat = gga_sats

                    self._latest_p3_quality = q
                    self._p3_gga_new_epoch = True

                    if q > 0:
                        self._feed_enu3_buffer(p3_lat, p3_lon, alt)
                        self._calc_and_update_enu2(p3_lat, p3_lon, alt,
                            self.enu3_ref_point, self.enu3_ref_ready,
                            self.enu3_east_label, self.enu3_north_label, self.enu3_up_label,
                            self.enu3_times, self.enu3_east_data, self.enu3_north_data, self.enu3_up_data,
                            self._std_enu3_east, self._std_enu3_north, self._std_enu3_up, "ENU3")
                        self._update_enu_std()

            if decoded.startswith('$BDRMC') or decoded.startswith('$GPRMC'):
                self.nmea_parser3.parse(decoded)
            if decoded.startswith('#OBSV'):
                self.nmea_parser3.parse(decoded)
            return

        # 串口1数据：完整处理

        # 串口数据缓存
        if self.current_data_source == 'serial':
            self.serial_save_buffer.append(data)
            self._write_auto_log(1, data)
            self._data_count_update += 1
            if self._data_count_update >= self._DATA_COUNT_INTERVAL:
                self.data_count_label.setText(f"已读取: {len(self.serial_save_buffer) + len(self.serial2_save_buffer) + len(self.serial3_save_buffer)} 行")
                self._data_count_update = 0

        if self.current_parser:
            decoded = self.current_parser.decode_line(data)
            if decoded:
                self.data_preview.append(decoded.strip())

                # 解析标准NMEA语句（提取GPS时间）
                if decoded.startswith('$BDRMC') or decoded.startswith('$GPRMC'):
                    self.nmea_parser.parse(decoded)
                    if self.nmea_parser.last_gps_time_valid:
                        self.gps_sow = self.nmea_parser.last_gps_tow
                        self.data_preview.append(f"【GPS时间】TOW: {self.gps_sow:.2f}秒")

                # 解析OBSVMA/OBSVHA数据（包含伪距和卫星信息）
                if decoded.startswith('#OBSV'):
                    self.nmea_parser.parse(decoded)
                    # 从OBSV数据更新GPS时间
                    if self.nmea_parser.obsvha_data:
                        obsv_data = self.nmea_parser.obsvha_data[-1]
                        self.gps_sow = obsv_data.get('rcv_tow', self.gps_sow) / 1000.0
                        self.gps_week = obsv_data.get('rcv_wkn', self.gps_week)
                    elif self.nmea_parser.obsvma_data:
                        obsv_data = self.nmea_parser.obsvma_data[-1]
                        self.gps_sow = obsv_data.get('rcv_tow', self.gps_sow) / 1000.0
                        self.gps_week = obsv_data.get('rcv_wkn', self.gps_week)

                # 解析GSV获取卫星信噪比
                if any(decoded.startswith(p) for p in ('$GPGSV', '$BDGSV', '$GLGSV', '$GAGSV', '$GBGSV')):
                    self.nmea_parser.parse(decoded)

                # GGA 解析
                gga_data = None
                if self.gga_parser.parse(decoded):
                    gga_data = self.gga_parser.get_data()

                    # 直接使用GGA原始数据
                    smoothed_lat = gga_data['lat']
                    smoothed_lon = gga_data['lon']
                    smoothed_alt = gga_data['alt']

                    # 没有真值，误差显示为0
                    errors = {'horizontal_error': 0.0, 'vertical_error': 0.0}
                    raw_errors = {'horizontal_error': 0.0, 'vertical_error': 0.0}

                    # 聚合数据预览显示
                    self.data_preview.append(
                        f"GGA → 纬度:{gga_data['lat']:.8f} 经度:{gga_data['lon']:.8f} "
                        f"海拔:{gga_data['alt']:.4f}m "
                        f"质量:{gga_data['quality']} 卫星:{gga_data['num_sat']}"
                    )

                    # 更新统计
                    self.statistics.add_errors(errors['horizontal_error'], errors['vertical_error'])

                    # 保存最新GGA数据（用于改正数计算）
                    self.last_gga_data = {
                        'lat': gga_data['lat'],
                        'lon': gga_data['lon'],
                        'alt': gga_data['alt'],
                        'quality': gga_data['quality'],
                        'num_sat': gga_data['num_sat'],
                        'smoothed_lat': smoothed_lat,
                        'smoothed_lon': smoothed_lon,
                        'smoothed_alt': smoothed_alt
                    }

                    # 获取当前统计值
                    stats = self.statistics.get_combined_stats()

                    # 更新显示
                    self.update_stats_display(
                        gga_data,
                        {'lat': smoothed_lat, 'lon': smoothed_lon, 'alt': smoothed_alt},
                        raw_errors,
                        errors,
                        stats
                    )

                if gga_data:
                    self._feed_enu1_buffer(gga_data['lat'], gga_data['lon'], gga_data['alt'])
                    self._calc_and_update_enu(gga_data['lat'], gga_data['lon'], gga_data['alt'],
                        self.enu1_ref_point, self.enu1_ref_ready,
                        self.enu1_east_label, self.enu1_north_label, self.enu1_up_label,
                        self.enu1_times, self.enu1_east_data, self.enu1_north_data, self.enu1_up_data,
                        self._std_enu1_east, self._std_enu1_north, self._std_enu1_up, "ENU1",
                        self.enu1_3_times, self.enu1_3_east_data, self.enu1_3_north_data, self.enu1_3_up_data,
                        self._std_enu1_3_east, self._std_enu1_3_north, self._std_enu1_3_up)
                    self._update_enu_std()

                if self.data_preview.document().blockCount() > 500:
                    cursor = self.data_preview.textCursor()
                    cursor.movePosition(QTextCursor.Start)
                    cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 200)
                    cursor.removeSelectedText()
                    cursor.movePosition(QTextCursor.End)
    
    def update_stats_display(self, gga_data, smoothed_coords, raw_errors, errors, stats):
        """更新串口1 GGA状态显示"""
        self.p1_lat_label.setText(f"{gga_data['lat']:.8f}")
        self.p1_lon_label.setText(f"{gga_data['lon']:.8f}")
        self.p1_alt_label.setText(f"{gga_data['alt']:.4f}")
        q = gga_data.get('quality', 0)
        quality_text = {0: "无效", 1: "单点定位", 2: "单点定位", 3: "无效PPS", 4: "RTK固定", 5: "RTK浮点", 6: "正在估算"}
        self.p1_quality_label.setText(quality_text.get(q, f"未知({q})"))
        self.p1_quality_label.setStyleSheet(
            "font-weight: bold; color: #e74c3c;" if q == 0 else
            "font-weight: bold; color: #f39c12;" if q == 1 else
            "font-weight: bold; color: #2980b9;" if q == 2 else
            "font-weight: bold; color: #27ae60;"
        )
        ts = gga_data.get('timestamp', '')
        if ts:
            self.p1_utc_label.setText(ts)
            self.p1_utc_ts = ts
        self.p1_nsats_label.setText(str(gga_data.get('num_sat', 0)))
        self.p1_gga_nsat = gga_data.get('num_sat', 0)

    def update_plots(self):
        self._update_frame_count += 1

        if self.nmea_parser and self.nmea_parser.gpgsv_data:
            self.port1_satellites = {}
            self._port1_snr_signals = {}
            for entry in self.nmea_parser.gpgsv_data:
                talker = entry.get('talker_id', '?')
                signal_id = entry.get('signal_id', 0)
                system_key = TALKER_TO_SYSTEM.get(talker, 'G')
                system_signals = SIGNAL_NAMES.get(system_key, SIGNAL_NAMES['G'])
                signal_name = system_signals.get(signal_id, f'S{signal_id}')
                for sat in entry.get('satellites', []):
                    prn = sat.get('prn', 0)
                    snr = sat.get('snr', 0)
                    if prn and 0 < snr <= 99:
                        key = f"{talker}{prn:02d}"
                        self.port1_satellites[key] = snr
                        self._port1_snr_signals[key] = signal_name

        for key, snr in self.port1_satellites.items():
            if self._last_port1_snr.get(key) != snr:
                self._last_port1_snr[key] = snr
                self._port1_snr_avg.add(key, snr)

        current_snr = (tuple(sorted(self.port1_satellites.items())),
                       tuple(sorted(self.port2_satellites.items())),
                       tuple(sorted(self.port3_satellites.items())))  # [新增] port3
        if current_snr != self._last_snr_data or self._update_frame_count % 2 == 0:
            self._refresh_snr_text()
            self._last_snr_data = current_snr

        self._update_enu_display()
        self._update_direction_stats_display()

    def _refresh_snr_text(self):
        p1_utc = getattr(self, 'p1_utc_ts', '') or '--'
        p2_utc = getattr(self, 'p2_utc_ts', '') or '--'
        p3_utc = getattr(self, 'p3_utc_ts', '') or '--'  # [新增]
        all_keys = sorted(
            set(self.port1_satellites.keys()) | set(self.port2_satellites.keys()) | set(self.port3_satellites.keys()),
            key=lambda x: (x[:2], int(x[2:])))
        html = ['<pre style="font-family: Consolas; font-size: 13px; color: #cdd6f4; margin:0;">']
        html.append(f"=== 卫星信噪比实时数据 (Port1 & Port2 & Port3) ===")
        html.append(f"UTC 串口1: {p1_utc}  |  串口2: {p2_utc}  |  串口3: {p3_utc}")

        # 更新差值平均值：Port2 vs Port1 (Delta)
        for key in all_keys:
            s1 = self.port1_satellites.get(key, None)
            s2 = self.port2_satellites.get(key, None)
            if s1 is not None and s2 is not None:
                diff = s2 - s1
                if self._last_snr_diff.get(key) != diff:
                    self._last_snr_diff[key] = diff
                    self._snr_diff_avg.add(key, diff)
        # [新增] 更新差值平均值：Port3 vs Port1 (Delta2)
        for key in all_keys:
            s1 = self.port1_satellites.get(key, None)
            s3 = self.port3_satellites.get(key, None)
            if s1 is not None and s3 is not None:
                diff2 = s3 - s1
                if self._last_snr_diff2.get(key) != diff2:
                    self._last_snr_diff2[key] = diff2
                    self._snr_diff2_avg.add(key, diff2)

        hdr_fmt = "  {:2}  {:>4}  {:>4}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}"
        html.append(hdr_fmt.format("Ty", "PRN", "Sig",
            "Port1", "Avg1", "Port2", "Avg2", "Delta", "AvgD",
            "Port3", "Avg3", "Delta2", "AvgD2"))
        html.append(hdr_fmt.format("--", "---", "---",
            "-----", "----", "-----", "----", "-----", "----",
            "-----", "----", "-----", "-----"))
        for key in all_keys:
            sys = key[:2]
            prn = int(key[2:])
            s1 = self.port1_satellites.get(key, None)
            s2 = self.port2_satellites.get(key, None)
            s3 = self.port3_satellites.get(key, None)  # [新增]
            a1 = self._port1_snr_avg.get(key)
            a2 = self._port2_snr_avg.get(key)
            a3 = self._port3_snr_avg.get(key)  # [新增]
            ad = self._snr_diff_avg.get(key)
            ad2 = self._snr_diff2_avg.get(key)  # [新增]
            signal = self._port1_snr_signals.get(key) or self._port2_snr_signals.get(key) or self._port3_snr_signals.get(key) or '---'

            s1_str = f"{s1:8.1f}" if s1 is not None else "      --"
            s2_str = f"{s2:8.1f}" if s2 is not None else "      --"
            s3_str = f"{s3:8.1f}" if s3 is not None else "      --"  # [新增]
            a1_str = f"{a1:8.1f}" if a1 is not None else "      --"
            a2_str = f"{a2:8.1f}" if a2 is not None else "      --"
            a3_str = f"{a3:8.1f}" if a3 is not None else "      --"  # [新增]
            ad_str = f"{ad:+8.1f}" if ad is not None else "      --"
            ad2_str = f"{ad2:+8.1f}" if ad2 is not None else "      --"  # [新增]

            if s1 is not None and s2 is not None:
                diff = s2 - s1
                diff_str = f"{diff:+8.1f}"
            else:
                diff_str = f'<span style="color:#888;">{"--":>8}</span>'
            # [新增] Delta2
            if s1 is not None and s3 is not None:
                diff2 = s3 - s1
                diff2_str = f"{diff2:+8.1f}"
            else:
                diff2_str = f'<span style="color:#888;">{"--":>8}</span>'

            if s1 is not None and s2 is not None:
                diff = s2 - s1
                if abs(diff) > 15:
                    row_color = "#ff4444"
                elif abs(diff) >= 10:
                    row_color = "#ff8844"
                else:
                    row_color = "#ffffff"
                row = f'<span style="color:{row_color};">  {sys}  {prn:4d}  {signal:>4s}  {s1_str}  {a1_str}  {s2_str}  {a2_str}  {diff_str}  {ad_str}  {s3_str}  {a3_str}  {diff2_str}  {ad2_str}</span>'
                html.append(row)
            else:
                html.append(f'  {sys}  {prn:4d}  {signal:>4s}  {s1_str}  {a1_str}  {s2_str}  {a2_str}  {diff_str}  {ad_str}  {s3_str}  {a3_str}  {diff2_str}  {ad2_str}')
        n1 = len(self.port1_satellites)
        n2 = len(self.port2_satellites)
        n3 = len(self.port3_satellites)  # [新增]
        common = sum(1 for k in all_keys if k in self.port1_satellites and k in self.port2_satellites)
        common3 = sum(1 for k in all_keys if k in self.port1_satellites and k in self.port3_satellites)  # [新增]
        html.append(f"--- 串口1: {n1} 颗 | 串口2: {n2} 颗 | 串口3: {n3} 颗 | 共同(1&2): {common} 颗 | 共同(1&3): {common3} 颗 ---")
        html.append("</pre>")
        self.snr_text1.setHtml("\n".join(html))

    def _feed_enu1_buffer(self, lat, lon, alt):
        if self.enu_instant_mode:
            if not self.enu1_ref_ready:
                self.enu1_ref_point = (lat, lon, alt)
                self.enu1_ref_ready = True
                self.enu1_ref_label.setText(f"ENU1 基准(瞬时): {lat:.8f}, {lon:.8f}, {alt:.3f}m")
                self.enu1_points_label.setText("(瞬时GGA)")
            return
        if not self.enu_auto_mode or self.enu1_ref_ready:
            return
        self.enu1_buffer.append((lat, lon, alt))
        self.enu1_points_label.setText(f"({len(self.enu1_buffer)}/{self.enu1_buffer_size})")
        if len(self.enu1_buffer) >= self.enu1_buffer_size:
            lats = [p[0] for p in self.enu1_buffer]
            lons = [p[1] for p in self.enu1_buffer]
            alts = [p[2] for p in self.enu1_buffer]
            self.enu1_ref_point = (sum(lats)/len(lats), sum(lons)/len(lons), sum(alts)/len(alts))
            self.enu1_ref_ready = True
            self.enu1_ref_label.setText(f"ENU1 基准: {self.enu1_ref_point[0]:.8f}, {self.enu1_ref_point[1]:.8f}, {self.enu1_ref_point[2]:.3f}m")
            self.enu1_points_label.setText("(已锁定)")
            self.log_info(f"ENU1基准点已锁定 (前{self.enu1_buffer_size}点均值)")

    def _feed_enu2_buffer(self, lat, lon, alt):
        if self.enu_instant_mode:
            if not self.enu2_ref_ready:
                self.enu2_ref_point = (lat, lon, alt)
                self.enu2_ref_ready = True
                self.enu2_ref_label.setText(f"ENU2 基准(瞬时): {lat:.8f}, {lon:.8f}, {alt:.3f}m")
                self.enu2_points_label.setText("(瞬时GGA)")
            return
        if not self.enu_auto_mode or self.enu2_ref_ready:
            return
        self.enu2_buffer.append((lat, lon, alt))
        self.enu2_points_label.setText(f"({len(self.enu2_buffer)}/{self.enu2_buffer_size})")
        if len(self.enu2_buffer) >= self.enu2_buffer_size:
            lats = [p[0] for p in self.enu2_buffer]
            lons = [p[1] for p in self.enu2_buffer]
            alts = [p[2] for p in self.enu2_buffer]
            self.enu2_ref_point = (sum(lats)/len(lats), sum(lons)/len(lons), sum(alts)/len(alts))
            self.enu2_ref_ready = True
            self.enu2_ref_label.setText(f"ENU2 基准: {self.enu2_ref_point[0]:.8f}, {self.enu2_ref_point[1]:.8f}, {self.enu2_ref_point[2]:.3f}m")
            self.enu2_points_label.setText("(已锁定)")
            self.log_info(f"ENU2基准点已锁定 (前{self.enu2_buffer_size}点均值)")

    # [新增] ENU3基准点采集
    def _feed_enu3_buffer(self, lat, lon, alt):
        if self.enu_instant_mode:
            if not self.enu3_ref_ready:
                self.enu3_ref_point = (lat, lon, alt)
                self.enu3_ref_ready = True
                self.enu3_ref_label.setText(f"ENU3 基准(瞬时): {lat:.8f}, {lon:.8f}, {alt:.3f}m")
                self.enu3_points_label.setText("(瞬时GGA)")
            return
        if not self.enu_auto_mode or self.enu3_ref_ready:
            return
        self.enu3_buffer.append((lat, lon, alt))
        self.enu3_points_label.setText(f"({len(self.enu3_buffer)}/{self.enu3_buffer_size})")
        if len(self.enu3_buffer) >= self.enu3_buffer_size:
            lats = [p[0] for p in self.enu3_buffer]
            lons = [p[1] for p in self.enu3_buffer]
            alts = [p[2] for p in self.enu3_buffer]
            self.enu3_ref_point = (sum(lats)/len(lats), sum(lons)/len(lons), sum(alts)/len(alts))
            self.enu3_ref_ready = True
            self.enu3_ref_label.setText(f"ENU3 基准: {self.enu3_ref_point[0]:.8f}, {self.enu3_ref_point[1]:.8f}, {self.enu3_ref_point[2]:.3f}m")
            self.enu3_points_label.setText("(已锁定)")
            self.log_info(f"ENU3基准点已锁定 (前{self.enu3_buffer_size}点均值)")

    def _enu_mode_changed(self):
        is_manual = self.enu_manual_radio.isChecked()
        is_instant = self.enu_instant_radio.isChecked()
        self.enu_auto_mode = self.enu_auto_radio.isChecked()
        self.enu_instant_mode = is_instant
        self.enu_manual_lat.setEnabled(is_manual)
        self.enu_manual_lon.setEnabled(is_manual)
        self.enu_manual_alt.setEnabled(is_manual)
        self.enu_apply_btn.setEnabled(is_manual)
        self.enu1_points_label.setVisible(not is_manual)
        self.enu2_points_label.setVisible(not is_manual)
        # 重置两个端口的基准状态
        self.enu1_ref_point = None
        self.enu1_ref_ready = False
        self.enu1_buffer = []
        self.enu1_times = []
        self.enu1_east_data = []
        self.enu1_north_data = []
        self.enu1_up_data = []
        self.enu1_3_times = []
        self.enu1_3_east_data = []
        self.enu1_3_north_data = []
        self.enu1_3_up_data = []
        self._std_enu1_east.reset()
        self._std_enu1_north.reset()
        self._std_enu1_up.reset()
        self._std_enu1_3_east.reset()
        self._std_enu1_3_north.reset()
        self._std_enu1_3_up.reset()
        self.enu1_east_curve.setData([], [])
        self.enu1_north_curve.setData([], [])
        self.enu1_up_curve.setData([], [])
        self.enu1_13_east_curve.setData([], [])
        self.enu1_13_north_curve.setData([], [])
        self.enu1_13_up_curve.setData([], [])
        self.enu2_ref_point = None
        self.enu2_ref_ready = False
        self.enu2_buffer = []
        self.enu2_times = []
        self.enu2_east_data = []
        self.enu2_north_data = []
        self.enu2_up_data = []
        self._std_enu2_east.reset()
        self._std_enu2_north.reset()
        self._std_enu2_up.reset()
        self.enu2_east_curve.setData([], [])
        self.enu2_north_curve.setData([], [])
        self.enu2_up_curve.setData([], [])
        self.enu3_ref_point = None  # [新增]
        self.enu3_ref_ready = False  # [新增]
        self.enu3_buffer = []  # [新增]
        self.enu3_times = []  # [新增]
        self.enu3_east_data = []  # [新增]
        self.enu3_north_data = []  # [新增]
        self.enu3_up_data = []  # [新增]
        self._std_enu3_east.reset()  # [新增]
        self._std_enu3_north.reset()  # [新增]
        self._std_enu3_up.reset()  # [新增]
        self.enu3_east_curve.setData([], [])  # [新增]
        self.enu3_north_curve.setData([], [])  # [新增]
        self.enu3_up_curve.setData([], [])  # [新增]
        label_text = "等待手动输入" if is_manual else "等待瞬时GGA..." if is_instant else "自动采集中..."
        pts_text = "(手动)" if is_manual else "(瞬时)" if is_instant else "(0/100)"
        self.enu1_ref_label.setText(f"ENU1 基准: {label_text}")
        self.enu2_ref_label.setText(f"ENU2 基准: {label_text}")
        self.enu3_ref_label.setText(f"ENU3 基准: {label_text}")  # [新增]
        self.enu1_points_label.setText(pts_text)
        self.enu2_points_label.setText(pts_text)
        self.enu3_points_label.setText(pts_text)  # [新增]
        self.enu1_east_label.setText("东: -- m")
        self.enu1_north_label.setText("北: -- m")
        self.enu1_up_label.setText("天: -- m")
        self.enu2_east_label.setText("东: -- m")
        self.enu2_north_label.setText("北: -- m")
        self.enu2_up_label.setText("天: -- m")
        self.enu3_east_label.setText("东: -- m")  # [新增]
        self.enu3_north_label.setText("北: -- m")  # [新增]
        self.enu3_up_label.setText("天: -- m")  # [新增]
        self.log_info(f"ENU基准模式切换: {'手动输入' if is_manual else '瞬时GGA' if is_instant else '自动均值'}")

    def _apply_enu_manual_ref(self):
        lat = self.enu_manual_lat.value()
        lon = self.enu_manual_lon.value()
        alt = self.enu_manual_alt.value()
        if lat == 0 and lon == 0:
            QMessageBox.warning(self, "警告", "请输入有效的经纬度坐标")
            return
        self.enu1_ref_point = (lat, lon, alt)
        self.enu1_ref_ready = True
        self.enu2_ref_point = (lat, lon, alt)
        self.enu2_ref_ready = True
        self.enu3_ref_point = (lat, lon, alt)  # [新增]
        self.enu3_ref_ready = True  # [新增]
        self.enu1_ref_label.setText(f"ENU1 基准: {lat:.8f}, {lon:.8f}, {alt:.3f}m")
        self.enu1_points_label.setText("(手动)")
        self.enu2_ref_label.setText(f"ENU2 基准: {lat:.8f}, {lon:.8f}, {alt:.3f}m")
        self.enu2_points_label.setText("(手动)")
        self.enu3_ref_label.setText(f"ENU3 基准: {lat:.8f}, {lon:.8f}, {alt:.3f}m")  # [新增]
        self.enu3_points_label.setText("(手动)")  # [新增]
        self.log_info(f"ENU手动基准已设置: {lat:.8f}°, {lon:.8f}°, {alt:.3f}m")

    def _update_enu_display(self):
        self._refresh_enu_charts()
        self._feed_direction_stats()
        self._feed_direction_stats3()
        self._update_enu_std()

    def _refresh_enu_charts(self):
        # 左列: ENU1 vs ENU2 (COM2)
        if self.enu1_times:
            self.enu1_east_curve.setData(self.enu1_times, self.enu1_east_data)
            self.enu1_north_curve.setData(self.enu1_times, self.enu1_north_data)
            self.enu1_up_curve.setData(self.enu1_times, self.enu1_up_data)
        if self.enu2_times:
            self.enu2_east_curve.setData(self.enu2_times, self.enu2_east_data)
            self.enu2_north_curve.setData(self.enu2_times, self.enu2_north_data)
            self.enu2_up_curve.setData(self.enu2_times, self.enu2_up_data)
        # ENU3曲线刷新
        if self.enu3_times:
            self.enu3_east_curve.setData(self.enu3_times, self.enu3_east_data)
            self.enu3_north_curve.setData(self.enu3_times, self.enu3_north_data)
            self.enu3_up_curve.setData(self.enu3_times, self.enu3_up_data)
        # 右列: ENU1 vs ENU3 (COM3，使用独立的 ENU1_3 副本)
        if self.enu1_3_times:
            self.enu1_13_east_curve.setData(self.enu1_3_times, self.enu1_3_east_data)
            self.enu1_13_north_curve.setData(self.enu1_3_times, self.enu1_3_north_data)
            self.enu1_13_up_curve.setData(self.enu1_3_times, self.enu1_3_up_data)

    def _update_enu_std(self):
        e1_std = self._std_enu1_east.std
        n1_std = self._std_enu1_north.std
        u1_std = self._std_enu1_up.std
        e2_std = self._std_enu2_east.std
        n2_std = self._std_enu2_north.std
        u2_std = self._std_enu2_up.std

        self.enu1_std_east.setText(f"E: {e1_std:.4f}m")
        self.enu1_std_north.setText(f"N: {n1_std:.4f}m")
        self.enu1_std_up.setText(f"U: {u1_std:.4f}m")
        self.enu2_std_east.setText(f"E: {e2_std:.4f}m")
        self.enu2_std_north.setText(f"N: {n2_std:.4f}m")
        self.enu2_std_up.setText(f"U: {u2_std:.4f}m")

        # [新增] ENU3标准差
        e3_std = self._std_enu3_east.std
        n3_std = self._std_enu3_north.std
        u3_std = self._std_enu3_up.std
        self.enu3_std_east.setText(f"E: {e3_std:.4f}m")
        self.enu3_std_north.setText(f"N: {n3_std:.4f}m")
        self.enu3_std_up.setText(f"U: {u3_std:.4f}m")

    def _is_enu_outlier(self, std_calc, value):
        if len(std_calc) < self.ENU_OUTLIER_MIN_SAMPLES:
            return False
        mean = std_calc.mean
        std = std_calc.std
        delta = abs(value - mean)
        if delta < self.ENU_OUTLIER_MIN_DELTA:
            return False
        return std > 0 and delta > self.ENU_OUTLIER_SIGMA * std

    def _calc_and_update_enu(self, lat, lon, alt, ref_point, ref_ready,
            east_label, north_label, up_label,
            times, east_data, north_data, up_data,
            std_east, std_north, std_up, name,
            times3=None, east_data3=None, north_data3=None, up_data3=None,
            std_east3=None, std_north3=None, std_up3=None):
        if not ref_ready:
            return
        if lat == 0 and lon == 0:
            return
        r_lat, r_lon, r_alt = ref_point

        a = WGS84_A
        f = WGS84_F
        e2 = WGS84_E2
        sin_lat = math.sin(math.radians(r_lat))
        cos_lat = math.cos(math.radians(r_lat))
        sin_lon = math.sin(math.radians(r_lon))
        cos_lon = math.cos(math.radians(r_lon))
        N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
        x0 = (N + r_alt) * cos_lat * cos_lon
        y0 = (N + r_alt) * cos_lat * sin_lon
        z0 = (N * (1 - e2) + r_alt) * sin_lat

        sin_lat2 = math.sin(math.radians(lat))
        cos_lat2 = math.cos(math.radians(lat))
        sin_lon2 = math.sin(math.radians(lon))
        cos_lon2 = math.cos(math.radians(lon))
        N2 = a / math.sqrt(1 - e2 * sin_lat2 * sin_lat2)
        x = (N2 + alt) * cos_lat2 * cos_lon2
        y = (N2 + alt) * cos_lat2 * sin_lon2
        z = (N2 * (1 - e2) + alt) * sin_lat2

        dx, dy, dz = x - x0, y - y0, z - z0
        east  = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up    =  cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz

        east_label.setText(f"东: {east:+.3f} m")
        north_label.setText(f"北: {north:+.3f} m")
        up_label.setText(f"天: {up:+.3f} m")

        east_bad = self._is_enu_outlier(std_east, east)
        north_bad = self._is_enu_outlier(std_north, north)
        up_bad = self._is_enu_outlier(std_up, up)

        if east_bad:
            self._enu1_outlier_count += 1
            self.log_info(f"{name} 东向异常值已剔除: {east:+.3f}m (σ: {std_east.std:.3f}m)")
        if north_bad:
            self._enu1_outlier_count += 1
            self.log_info(f"{name} 北向异常值已剔除: {north:+.3f}m (σ: {std_north.std:.3f}m)")
        if up_bad:
            self._enu1_outlier_count += 1
            self.log_info(f"{name} 天向异常值已剔除: {up:+.3f}m (σ: {std_up.std:.3f}m)")

        if east_bad or north_bad or up_bad:
            return

        std_east.add(east)
        std_north.add(north)
        std_up.add(up)
        if std_east3 is not None:
            std_east3.add(east)
            std_north3.add(north)
            std_up3.add(up)

        t = times[-1] + 1.0 if times else 0.0
        times.append(t)
        east_data.append(east)
        north_data.append(north)
        up_data.append(up)
        # [新增] 同步馈入 COM3 的独立 ENU1 副本
        if times3 is not None:
            times3.append(t)
            east_data3.append(east)
            north_data3.append(north)
            up_data3.append(up)

        if len(times) > self.ENU_MAX_POINTS:
            excess = len(times) - self.ENU_MAX_POINTS
            del times[:excess]
            del east_data[:excess]
            del north_data[:excess]
            del up_data[:excess]
            if times3 is not None:
                del times3[:excess]
                del east_data3[:excess]
                del north_data3[:excess]
                del up_data3[:excess]

    def _calc_and_update_enu2(self, lat, lon, alt, ref_point, ref_ready,
            east_label, north_label, up_label,
            times, east_data, north_data, up_data,
            std_east, std_north, std_up, name):
        if not ref_ready:
            return
        if lat == 0 and lon == 0:
            return
        r_lat, r_lon, r_alt = ref_point

        a = WGS84_A
        f = WGS84_F
        e2 = WGS84_E2
        sin_lat = math.sin(math.radians(r_lat))
        cos_lat = math.cos(math.radians(r_lat))
        sin_lon = math.sin(math.radians(r_lon))
        cos_lon = math.cos(math.radians(r_lon))
        N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
        x0 = (N + r_alt) * cos_lat * cos_lon
        y0 = (N + r_alt) * cos_lat * sin_lon
        z0 = (N * (1 - e2) + r_alt) * sin_lat

        sin_lat2 = math.sin(math.radians(lat))
        cos_lat2 = math.cos(math.radians(lat))
        sin_lon2 = math.sin(math.radians(lon))
        cos_lon2 = math.cos(math.radians(lon))
        N2 = a / math.sqrt(1 - e2 * sin_lat2 * sin_lat2)
        x = (N2 + alt) * cos_lat2 * cos_lon2
        y = (N2 + alt) * cos_lat2 * sin_lon2
        z = (N2 * (1 - e2) + alt) * sin_lat2

        dx, dy, dz = x - x0, y - y0, z - z0
        east  = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up    =  cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz

        if name != "ENU3":
            self._latest_enu2_east = east
            self._latest_enu2_north = north
            self._latest_enu2_up = up
        # [新增] ENU3最新值跟踪
        if name == "ENU3":
            self._latest_enu3_east = east
            self._latest_enu3_north = north
            self._latest_enu3_up = up

        east_label.setText(f"东: {east:+.3f} m")
        north_label.setText(f"北: {north:+.3f} m")
        up_label.setText(f"天: {up:+.3f} m")

        east_bad = self._is_enu_outlier(std_east, east)
        north_bad = self._is_enu_outlier(std_north, north)
        up_bad = self._is_enu_outlier(std_up, up)

        # [新增] ENU3异常值计数（同时避免双重计入_enu2_outlier_count）
        if name == "ENU3":
            if east_bad:
                self._enu3_outlier_count += 1
            if north_bad:
                self._enu3_outlier_count += 1
            if up_bad:
                self._enu3_outlier_count += 1
        else:
            if east_bad:
                self._enu2_outlier_count += 1
                self.log_info(f"{name} 东向异常值已剔除: {east:+.3f}m (σ: {std_east.std:.3f}m)")
            if north_bad:
                self._enu2_outlier_count += 1
                self.log_info(f"{name} 北向异常值已剔除: {north:+.3f}m (σ: {std_north.std:.3f}m)")
            if up_bad:
                self._enu2_outlier_count += 1
                self.log_info(f"{name} 天向异常值已剔除: {up:+.3f}m (σ: {std_up.std:.3f}m)")

        if east_bad or north_bad or up_bad:
            return

        std_east.add(east)
        std_north.add(north)
        std_up.add(up)

        t = times[-1] + 1.0 if times else 0.0
        times.append(t)
        east_data.append(east)
        north_data.append(north)
        up_data.append(up)

        if len(times) > self.ENU_MAX_POINTS:
            del times[:len(times) - self.ENU_MAX_POINTS]
            del east_data[:len(east_data) - self.ENU_MAX_POINTS]
            del north_data[:len(north_data) - self.ENU_MAX_POINTS]
            del up_data[:len(up_data) - self.ENU_MAX_POINTS]

    def _create_direction_tab(self):
        main_tab = QWidget()
        layout = QVBoxLayout(main_tab)

        # ENU基准值设置（共享一组控制）
        enu_ref_box = QGroupBox("ENU 基准值设置（启动时自动应用）")
        enu_ref_layout = QVBoxLayout(enu_ref_box)
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("基准模式:"))
        self.enu_auto_radio = QRadioButton("自动基准(前100点)")
        self.enu_auto_radio.setChecked(True)
        self.enu_instant_radio = QRadioButton("瞬时GGA定位值")
        self.enu_manual_radio = QRadioButton("手动输入")
        mode_row.addWidget(self.enu_auto_radio)
        mode_row.addWidget(self.enu_instant_radio)
        mode_row.addWidget(self.enu_manual_radio)
        mode_row.addStretch()
        enu_ref_layout.addLayout(mode_row)
        manual_row = QHBoxLayout()
        manual_row.addWidget(QLabel("纬度:"))
        self.enu_manual_lat = QDoubleSpinBox()
        self.enu_manual_lat.setRange(-90, 90)
        self.enu_manual_lat.setDecimals(8)
        self.enu_manual_lat.setEnabled(False)
        manual_row.addWidget(self.enu_manual_lat)
        manual_row.addWidget(QLabel("经度:"))
        self.enu_manual_lon = QDoubleSpinBox()
        self.enu_manual_lon.setRange(-180, 180)
        self.enu_manual_lon.setDecimals(8)
        self.enu_manual_lon.setEnabled(False)
        manual_row.addWidget(self.enu_manual_lon)
        manual_row.addWidget(QLabel("高程:"))
        self.enu_manual_alt = QDoubleSpinBox()
        self.enu_manual_alt.setRange(-1000, 10000)
        self.enu_manual_alt.setDecimals(3)
        self.enu_manual_alt.setEnabled(False)
        manual_row.addWidget(self.enu_manual_alt)
        self.enu_apply_btn = QPushButton("应用基准")
        self.enu_apply_btn.setEnabled(False)
        manual_row.addWidget(self.enu_apply_btn)
        manual_row.addStretch()
        enu_ref_layout.addLayout(manual_row)
        enu_ref_labels = QHBoxLayout()
        self.enu1_ref_label = QLabel("ENU1 基准: 未设置")
        self.enu1_ref_label.setStyleSheet("color: #888; font-size: 10px;")
        enu_ref_labels.addWidget(self.enu1_ref_label)
        self.enu1_points_label = QLabel("")
        self.enu1_points_label.setStyleSheet("color: #888; font-size: 10px;")
        enu_ref_labels.addWidget(self.enu1_points_label)
        enu_ref_labels.addStretch()
        self.enu2_ref_label = QLabel("ENU2 基准: 未设置")
        self.enu2_ref_label.setStyleSheet("color: #888; font-size: 10px;")
        enu_ref_labels.addWidget(self.enu2_ref_label)
        self.enu2_points_label = QLabel("")
        self.enu2_points_label.setStyleSheet("color: #888; font-size: 10px;")
        enu_ref_labels.addWidget(self.enu2_points_label)
        enu_ref_labels.addStretch()
        # [新增] ENU3基准点状态标签
        self.enu3_ref_label = QLabel("ENU3 基准: 未设置")
        self.enu3_ref_label.setStyleSheet("color: #888; font-size: 10px;")
        enu_ref_labels.addWidget(self.enu3_ref_label)
        self.enu3_points_label = QLabel("")
        self.enu3_points_label.setStyleSheet("color: #888; font-size: 10px;")
        enu_ref_labels.addWidget(self.enu3_points_label)
        enu_ref_labels.addStretch()
        enu_ref_layout.addLayout(enu_ref_labels)
        layout.addWidget(enu_ref_box)

        # 方向测试子标签：串口2和串口3分开
        dir_sub_tabs = QTabWidget()
        self._create_dir_port_widget(dir_sub_tabs, port_id=2)
        self._create_dir_port_widget(dir_sub_tabs, port_id=3)
        layout.addWidget(dir_sub_tabs)

        return main_tab

    def _create_dir_port_widget(self, parent_tab, port_id):
        """为指定串口创建方向测试UI"""
        tab = QWidget()
        tab_layout = QVBoxLayout(tab)

        is_com3 = (port_id == 3)

        # 设备序号输入行
        sn_row = QHBoxLayout()
        sn_row.addWidget(QLabel("设备序号:"))
        sn_input = QLineEdit()
        sn_input.setPlaceholderText("请输入设备序号")
        sn_input.setMinimumWidth(150)
        sn_row.addWidget(sn_input)
        sn_apply_btn = QPushButton("应用")
        sn_apply_btn.setMinimumWidth(60)
        sn_apply_btn.setStyleSheet("QPushButton { background-color: #27ae60; color: white; font-weight: bold; } QPushButton:hover { background-color: #2ecc71; }")
        if is_com3:
            self.device_sn3_input = sn_input
            sn_apply_btn.clicked.connect(lambda: self._apply_device_sn(3))
        else:
            self.device_sn2_input = sn_input
            sn_apply_btn.clicked.connect(lambda: self._apply_device_sn(2))
        sn_row.addWidget(sn_apply_btn)
        sn_row.addStretch()
        tab_layout.addLayout(sn_row)

        stats_grid = QGridLayout()
        stat_boxes = []

        for i in range(4):
            box = QGroupBox(f"方向{i + 1}")
            box.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #bdc3c7; border-radius: 6px; margin-top: 10px; padding-top: 14px; } QGroupBox::title { background-color: #ecf0f1; padding: 2px 10px; border-radius: 3px; }")
            box_layout = QVBoxLayout(box)

            duration_label = QLabel("测试时长: 00:00:00")
            duration_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #2980b9;")
            box_layout.addWidget(duration_label)

            epoch_label = QLabel("总历元: 0 | 成功: 0 | 成功率: 0.0%")
            epoch_label.setStyleSheet("font-size: 12px; color: #555;")
            box_layout.addWidget(epoch_label)

            h_label = QLabel("水平误差 均值: -- m | 最大: -- m")
            h_label.setStyleSheet("font-size: 12px; color: #e74c3c;")
            box_layout.addWidget(h_label)

            v_label = QLabel("垂直误差 均值: -- m | 最大: -- m")
            v_label.setStyleSheet("font-size: 12px; color: #27ae60;")
            box_layout.addWidget(v_label)

            start_btn = QPushButton("启动方向测试")
            start_btn.setMinimumWidth(100)
            start_btn.setStyleSheet("QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #2ecc71; }")
            idx = i
            if is_com3:
                start_btn.clicked.connect(lambda checked, d=idx: self._toggle_single_direction3(d))
            else:
                start_btn.clicked.connect(lambda checked, d=idx: self._toggle_single_direction(d))
            box_layout.addWidget(start_btn)

            stat_boxes.append({
                'box': box,
                'duration': duration_label,
                'epoch': epoch_label,
                'h': h_label,
                'v': v_label,
                'btn': start_btn,
            })
            stats_grid.addWidget(box, i // 2, i % 2)

        tab_layout.addLayout(stats_grid)

        # 自动停止控制
        auto_stop_row = QHBoxLayout()
        if is_com3:
            stop_cb = QCheckBox("启用自动停止")
            stop_cb.setStyleSheet("font-size: 13px; font-weight: bold;")
            stop_cb.stateChanged.connect(lambda state: setattr(self, '_dir3_auto_stop_enabled', state == Qt.Checked))
            auto_stop_row.addWidget(stop_cb)
            auto_stop_row.addWidget(QLabel("最大时长(秒):"))
            sec_spin = QSpinBox()
            sec_spin.setRange(0, 7200)
            sec_spin.setValue(self._dir3_auto_stop_sec)
            sec_spin.setSuffix(" s")
            sec_spin.valueChanged.connect(lambda v: setattr(self, '_dir3_auto_stop_sec', v))
            auto_stop_row.addWidget(sec_spin)
            auto_stop_row.addWidget(QLabel("最大历元数:"))
            epoch_spin = QSpinBox()
            epoch_spin.setRange(0, 100000)
            epoch_spin.setValue(self._dir3_auto_stop_epochs)
            epoch_spin.valueChanged.connect(lambda v: setattr(self, '_dir3_auto_stop_epochs', v))
            auto_stop_row.addWidget(epoch_spin)
        else:
            stop_cb = QCheckBox("启用自动停止")
            stop_cb.setStyleSheet("font-size: 13px; font-weight: bold;")
            stop_cb.stateChanged.connect(self._on_auto_stop_toggled)
            auto_stop_row.addWidget(stop_cb)
            auto_stop_row.addWidget(QLabel("最大时长(秒):"))
            sec_spin = QSpinBox()
            sec_spin.setRange(0, 7200)
            sec_spin.setValue(self._dir_auto_stop_sec)
            sec_spin.setSuffix(" s")
            sec_spin.valueChanged.connect(self._on_auto_stop_sec_changed)
            auto_stop_row.addWidget(sec_spin)
            auto_stop_row.addWidget(QLabel("最大历元数:"))
            epoch_spin = QSpinBox()
            epoch_spin.setRange(0, 100000)
            epoch_spin.setValue(self._dir_auto_stop_epochs)
            epoch_spin.valueChanged.connect(self._on_auto_stop_epoch_changed)
            auto_stop_row.addWidget(epoch_spin)
        auto_stop_row.addStretch()
        tab_layout.addLayout(auto_stop_row)

        # 统计柱状图
        charts_layout = QVBoxLayout()
        h_chart = pg.PlotWidget()
        h_chart.setBackground('w')
        h_chart.setLabel('left', '水平误差', units='m')
        h_chart.setLabel('bottom', '方向')
        h_chart.showGrid(y=True)
        h_chart.setMaximumHeight(220)
        charts_layout.addWidget(h_chart)

        v_chart = pg.PlotWidget()
        v_chart.setBackground('w')
        v_chart.setLabel('left', '垂直误差', units='m')
        v_chart.setLabel('bottom', '方向')
        v_chart.showGrid(y=True)
        v_chart.setMaximumHeight(220)
        charts_layout.addWidget(v_chart)
        tab_layout.addLayout(charts_layout)

        btn_row = QHBoxLayout()
        reset_btn = QPushButton("重置方向统计")
        reset_btn.setMinimumWidth(120)
        if is_com3:
            reset_btn.clicked.connect(self._reset_direction_stats3)
        else:
            reset_btn.clicked.connect(self._reset_direction_stats)
        btn_row.addWidget(reset_btn)
        btn_row.addStretch()
        tab_layout.addLayout(btn_row)

        # 保存引用
        if is_com3:
            self.dir_stat_boxes3 = stat_boxes
            self.dir_h_chart3 = h_chart
            self.dir_v_chart3 = v_chart
        else:
            self.dir_stat_boxes = stat_boxes
            self.dir_h_chart = h_chart
            self.dir_v_chart = v_chart

        title = f"串口3 (干扰测试2)" if is_com3 else f"串口2 (干扰测试)"
        parent_tab.addTab(tab, title)

    def _create_dir_enu_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        sub_tabs = QTabWidget()
        self._dir_enu_curves = []
        self._dir_enu_curves3 = []
        self._dir_enu_plots = []

        comp_names = ['东向', '北向', '天向']
        comp_units = ['m', 'm', 'm']

        for di in range(4):
            dir_tab = QWidget()
            dir_layout = QVBoxLayout(dir_tab)

            # 每个方向有两个子标签：ENU1vsENU2 和 ENU1vsENU3
            dir_comp_tabs = QTabWidget()

            # --- 子标签1: ENU1 vs ENU2 ---
            tab12 = QWidget()
            lay12 = QVBoxLayout(tab12)
            curves_for_dir = {}
            plots_for_dir = []
            for ci, (cname, cunit) in enumerate(zip(comp_names, comp_units)):
                plot = pg.PlotWidget()
                plot.setBackground('w')
                plot.setLabel('left', cname, units=cunit)
                if ci == 2:
                    plot.setLabel('bottom', '时间', units='历元')
                plot.showGrid(x=True, y=True)
                plot.addLegend()
                c1 = plot.plot([], [], pen={'color': '#e74c3c', 'width': 2}, name='ENU1 (无干扰)')
                c2 = plot.plot([], [], pen={'color': '#2980b9', 'width': 2}, name='ENU2 (干扰测试)')
                curves_for_dir[f'{["east","north","up"][ci]}1'] = c1
                curves_for_dir[f'{["east","north","up"][ci]}2'] = c2
                plots_for_dir.append(plot)
                lay12.addWidget(plot)
            dir_comp_tabs.addTab(tab12, "ENU1 vs ENU2")
            self._dir_enu_curves.append(curves_for_dir)
            self._dir_enu_plots.append(plots_for_dir)

            # --- 子标签2: ENU1 vs ENU3 ---
            tab13 = QWidget()
            lay13 = QVBoxLayout(tab13)
            curves_for_dir3 = {}
            for ci, (cname, cunit) in enumerate(zip(comp_names, comp_units)):
                plot = pg.PlotWidget()
                plot.setBackground('w')
                plot.setLabel('left', cname, units=cunit)
                if ci == 2:
                    plot.setLabel('bottom', '时间', units='历元')
                plot.showGrid(x=True, y=True)
                plot.addLegend()
                c1 = plot.plot([], [], pen={'color': '#e74c3c', 'width': 2}, name='ENU1 (无干扰)')
                c3 = plot.plot([], [], pen={'color': '#f39c12', 'width': 2, 'style': Qt.DashLine}, name='ENU3 (干扰测试2)')
                curves_for_dir3[f'{["east","north","up"][ci]}1'] = c1
                curves_for_dir3[f'{["east","north","up"][ci]}3'] = c3
                lay13.addWidget(plot)
            dir_comp_tabs.addTab(tab13, "ENU1 vs ENU3")
            self._dir_enu_curves3.append(curves_for_dir3)

            dir_layout.addWidget(dir_comp_tabs)
            sub_tabs.addTab(dir_tab, f"方向{di + 1}")

        layout.addWidget(sub_tabs)
        return tab

    def _toggle_single_direction(self, direction_index):
        ds = self.direction_stats[direction_index]
        boxes = self.dir_stat_boxes
        if ds._active:
            self._save_enu_for_direction(direction_index, self.direction_stats, boxes)
            ds.stop()
            boxes[direction_index]['btn'].setText("启动方向测试")
            boxes[direction_index]['btn'].setStyleSheet(
                "QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #2ecc71; }")
            self._dir_enu_active_index = -1
            self.log_info(f"方向{direction_index + 1} 测试已停止 (串口2)")
        else:
            self._save_enu_for_active_direction()
            self._reset_enu_chart_data(2)
            ds.start()
            ds.clear_enu()
            self._dir_enu_active_index = direction_index
            boxes[direction_index]['btn'].setText("停止方向测试")
            boxes[direction_index]['btn'].setStyleSheet(
                "QPushButton { background-color: #e74c3c; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #c0392b; }")
            self.log_info(f"方向{direction_index + 1} 测试已启动 (串口2)")

    def _toggle_single_direction3(self, direction_index):
        ds = self.direction_stats3[direction_index]
        boxes = self.dir_stat_boxes3
        if ds._active:
            self._save_enu_for_direction(direction_index, self.direction_stats3, boxes)
            ds.stop()
            boxes[direction_index]['btn'].setText("启动方向测试")
            boxes[direction_index]['btn'].setStyleSheet(
                "QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #2ecc71; }")
            self._dir3_enu_active_index = -1
            self.log_info(f"方向{direction_index + 1} 测试已停止 (串口3)")
        else:
            self._save_enu_for_active_direction3()
            self._reset_enu_chart_data(3)
            ds.start()
            ds.clear_enu()
            self._dir3_enu_active_index = direction_index
            boxes[direction_index]['btn'].setText("停止方向测试")
            boxes[direction_index]['btn'].setStyleSheet(
                "QPushButton { background-color: #e74c3c; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #c0392b; }")
            self.log_info(f"方向{direction_index + 1} 测试已启动 (串口3)")

    def _on_auto_stop_toggled(self, state):
        self._dir_auto_stop_enabled = (state == Qt.Checked)
        self.log_info(f"方向自动停止: {'启用' if self._dir_auto_stop_enabled else '禁用'}")

    def _on_auto_stop_sec_changed(self, val):
        self._dir_auto_stop_sec = val

    def _on_auto_stop_epoch_changed(self, val):
        self._dir_auto_stop_epochs = val

    def _save_enu_for_active_direction(self):
        for i in range(4):
            ds = self.direction_stats[i]
            if ds._active:
                self._save_enu_for_direction(i, self.direction_stats, self.dir_stat_boxes)
                ds.stop()
                self.dir_stat_boxes[i]['btn'].setText("启动方向测试")
                self.dir_stat_boxes[i]['btn'].setStyleSheet(
                    "QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #2ecc71; }")
                self.log_info(f"方向{i + 1} 测试已停止 (串口2)")

    def _save_enu_for_active_direction3(self):
        for i in range(4):
            ds = self.direction_stats3[i]
            if ds._active:
                self._save_enu_for_direction(i, self.direction_stats3, self.dir_stat_boxes3)
                ds.stop()
                self.dir_stat_boxes3[i]['btn'].setText("启动方向测试")
                self.dir_stat_boxes3[i]['btn'].setStyleSheet(
                    "QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #2ecc71; }")
                self.log_info(f"方向{i + 1} 测试已停止 (串口3)")

    def _save_enu_for_direction(self, direction_index, direction_stats=None, dir_stat_boxes=None):
        if direction_stats is None:
            direction_stats = self.direction_stats
        if dir_stat_boxes is None:
            dir_stat_boxes = self.dir_stat_boxes
        ds = direction_stats[direction_index]
        is_com3 = (direction_stats is self.direction_stats3)
        if is_com3:
            n_raw = len(self.enu3_times)
            n1_total = len(self.enu1_3_times)
            enu1_src = (self.enu1_3_times, self.enu1_3_east_data, self.enu1_3_north_data, self.enu1_3_up_data)
        else:
            n_raw = len(self.enu2_times)
            n1_total = len(self.enu1_times)
            enu1_src = (self.enu1_times, self.enu1_east_data, self.enu1_north_data, self.enu1_up_data)
        if n_raw > 0:
            n = min(n_raw, n1_total)
            enu1_t = enu1_src[0][-n:] if n1_total >= n else list(enu1_src[0])
            enu1_e = enu1_src[1][-n:] if n1_total >= n else list(enu1_src[1])
            enu1_n = enu1_src[2][-n:] if n1_total >= n else list(enu1_src[2])
            enu1_u = enu1_src[3][-n:] if n1_total >= n else list(enu1_src[3])
            if is_com3:
                enu3_t = self.enu3_times[-n:] if len(self.enu3_times) >= n else list(self.enu3_times)
                enu3_e = self.enu3_east_data[-n:] if len(self.enu3_east_data) >= n else list(self.enu3_east_data)
                enu3_n = self.enu3_north_data[-n:] if len(self.enu3_north_data) >= n else list(self.enu3_north_data)
                enu3_u = self.enu3_up_data[-n:] if len(self.enu3_up_data) >= n else list(self.enu3_up_data)
                ds.save_enu_snapshot(
                    enu1_t, enu1_e, enu1_n, enu1_u,
                    [], [], [], [],
                    enu3_t, enu3_e, enu3_n, enu3_u)
                self.log_info(f"方向{direction_index + 1} ENU3数据已保存 (ENU1: {len(enu1_e)}点, ENU3: {len(enu3_e)}点)")
            else:
                enu2_t = self.enu2_times[-n:] if len(self.enu2_times) >= n else list(self.enu2_times)
                enu2_e = self.enu2_east_data[-n:] if len(self.enu2_east_data) >= n else list(self.enu2_east_data)
                enu2_n = self.enu2_north_data[-n:] if len(self.enu2_north_data) >= n else list(self.enu2_north_data)
                enu2_u = self.enu2_up_data[-n:] if len(self.enu2_up_data) >= n else list(self.enu2_up_data)
                ds.save_enu_snapshot(
                    enu1_t, enu1_e, enu1_n, enu1_u,
                    enu2_t, enu2_e, enu2_n, enu2_u)
                self.log_info(f"方向{direction_index + 1} ENU数据已保存 (ENU1: {len(enu1_e)}点, ENU2: {len(enu2_e)}点)")

    def _reset_enu_chart_data(self, port=0):
        """重置ENU图表数据。port: 1=ENU1参考, 2=ENU2(串口2), 3=ENU3(串口3), 0=全部"""
        # ENU1 串口2侧（左列图表）
        if port in (0, 2):
            self.enu1_times = []
            self.enu1_east_data = []
            self.enu1_north_data = []
            self.enu1_up_data = []
            self._std_enu1_east.reset()
            self._std_enu1_north.reset()
            self._std_enu1_up.reset()
            self.enu1_east_curve.setData([], [])
            self.enu1_north_curve.setData([], [])
            self.enu1_up_curve.setData([], [])

        # ENU1 串口3侧（右列图表，独立副本）
        if port in (0, 3):
            self.enu1_3_times = []
            self.enu1_3_east_data = []
            self.enu1_3_north_data = []
            self.enu1_3_up_data = []
            self._std_enu1_3_east.reset()
            self._std_enu1_3_north.reset()
            self._std_enu1_3_up.reset()
            self.enu1_13_east_curve.setData([], [])
            self.enu1_13_north_curve.setData([], [])
            self.enu1_13_up_curve.setData([], [])

        if port in (0, 2):
            self.enu2_times = []
            self.enu2_east_data = []
            self.enu2_north_data = []
            self.enu2_up_data = []
            self._std_enu2_east.reset()
            self._std_enu2_north.reset()
            self._std_enu2_up.reset()
            self.enu2_east_curve.setData([], [])
            self.enu2_north_curve.setData([], [])
            self.enu2_up_curve.setData([], [])

        if port in (0, 3):
            self.enu3_times = []
            self.enu3_east_data = []
            self.enu3_north_data = []
            self.enu3_up_data = []
            self._std_enu3_east.reset()
            self._std_enu3_north.reset()
            self._std_enu3_up.reset()
            self.enu3_east_curve.setData([], [])
            self.enu3_north_curve.setData([], [])
            self.enu3_up_curve.setData([], [])

    def _feed_direction_stats(self):
        # 串口2方向测试数据馈入
        if self._p2_gga_new_epoch:
            if self.enu2_ref_ready:
                self._p2_gga_new_epoch = False
                self._do_feed_direction_stats(self._latest_enu2_east, self._latest_enu2_north, self._latest_enu2_up, self._latest_p2_quality,
                    self.direction_stats, self.dir_stat_boxes, self._dir_auto_stop_enabled, self._dir_auto_stop_sec, self._dir_auto_stop_epochs)

    def _feed_direction_stats3(self):
        # 串口3方向测试数据馈入
        if self._p3_gga_new_epoch:
            if self.enu3_ref_ready:
                self._p3_gga_new_epoch = False
                self._do_feed_direction_stats(self._latest_enu3_east, self._latest_enu3_north, self._latest_enu3_up, self._latest_p3_quality,
                    self.direction_stats3, self.dir_stat_boxes3, self._dir3_auto_stop_enabled, self._dir3_auto_stop_sec, self._dir3_auto_stop_epochs)

    def _do_feed_direction_stats(self, east, north, up, quality, direction_stats, dir_stat_boxes, auto_stop_enabled, auto_stop_sec, auto_stop_epochs):
        for i in range(4):
            ds = direction_stats[i]
            if ds._active:
                ds.add_epoch(east, north, up, quality)
                if auto_stop_enabled:
                    stopped = False
                    if auto_stop_sec > 0 and ds.duration >= auto_stop_sec:
                        stopped = True
                        reason = f"达到设定时长 {auto_stop_sec} 秒"
                    if not stopped and auto_stop_epochs > 0 and ds.total_epochs >= auto_stop_epochs:
                        stopped = True
                        reason = f"达到设定历元数 {auto_stop_epochs}"
                    if stopped:
                        self._save_enu_for_direction(i, direction_stats, dir_stat_boxes)
                        ds.stop()
                        dir_stat_boxes[i]['btn'].setText("启动方向测试")
                        dir_stat_boxes[i]['btn'].setStyleSheet(
                            "QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #2ecc71; }")
                        self.log_info(f"方向{i + 1} 自动停止 ({reason})")

    def _update_direction_stats_display(self):
        # 串口2方向测试显示
        for i in range(4):
            ds = self.direction_stats[i]
            if ds._active:
                ds.update_duration()
            stats = ds.get_stats()
            labels = self.dir_stat_boxes[i]
            labels['duration'].setText(f"测试时长: {stats['duration']}")
            labels['epoch'].setText(
                f"总历元: {stats['total_epochs']} | 成功: {stats['successful_epochs']} | 成功率: {stats['success_rate']:.1f}%")
            labels['h'].setText(
                f"水平误差 均值: {stats['h_mean']:.3f} m | 最大: {stats['h_max']:.3f} m")
            labels['v'].setText(
                f"垂直误差 均值: {stats['v_mean']:.3f} m | 最大: {stats['v_max']:.3f} m")
        self._update_direction_charts()
        # 串口3方向测试显示
        for i in range(4):
            ds = self.direction_stats3[i]
            if ds._active:
                ds.update_duration()
            stats = ds.get_stats()
            labels = self.dir_stat_boxes3[i]
            labels['duration'].setText(f"测试时长: {stats['duration']}")
            labels['epoch'].setText(
                f"总历元: {stats['total_epochs']} | 成功: {stats['successful_epochs']} | 成功率: {stats['success_rate']:.1f}%")
            labels['h'].setText(
                f"水平误差 均值: {stats['h_mean']:.3f} m | 最大: {stats['h_max']:.3f} m")
            labels['v'].setText(
                f"垂直误差 均值: {stats['v_mean']:.3f} m | 最大: {stats['v_max']:.3f} m")
        self._update_direction_charts3()
        self._update_direction_enu_charts()

    def _update_direction_charts(self):
        x = [1, 2, 3, 4]
        h_means = [self.direction_stats[i].h_mean for i in range(4)]
        h_maxs = [self.direction_stats[i].h_max for i in range(4)]
        v_means = [self.direction_stats[i].v_mean for i in range(4)]
        v_maxs = [self.direction_stats[i].v_max for i in range(4)]

        self.dir_h_chart.clear()
        bar_mean = pg.BarGraphItem(x=[v - 0.15 for v in x], height=h_means, width=0.3, brush='#e74c3c', name='均值')
        bar_max = pg.BarGraphItem(x=[v + 0.15 for v in x], height=h_maxs, width=0.3, brush='#c0392b', name='最大值')
        self.dir_h_chart.addItem(bar_mean)
        self.dir_h_chart.addItem(bar_max)
        self.dir_h_chart.addLegend()
        ax_h = self.dir_h_chart.getAxis('bottom')
        ax_h.setTicks([[(1, '方向1'), (2, '方向2'), (3, '方向3'), (4, '方向4')]])

        self.dir_v_chart.clear()
        bar_mean_v = pg.BarGraphItem(x=[v - 0.15 for v in x], height=v_means, width=0.3, brush='#27ae60', name='均值')
        bar_max_v = pg.BarGraphItem(x=[v + 0.15 for v in x], height=v_maxs, width=0.3, brush='#1e8449', name='最大值')
        self.dir_v_chart.addItem(bar_mean_v)
        self.dir_v_chart.addItem(bar_max_v)
        self.dir_v_chart.addLegend()
        ax_v = self.dir_v_chart.getAxis('bottom')
        ax_v.setTicks([[(1, '方向1'), (2, '方向2'), (3, '方向3'), (4, '方向4')]])

    def _update_direction_charts3(self):
        x = [1, 2, 3, 4]
        h_means = [self.direction_stats3[i].h_mean for i in range(4)]
        h_maxs = [self.direction_stats3[i].h_max for i in range(4)]
        v_means = [self.direction_stats3[i].v_mean for i in range(4)]
        v_maxs = [self.direction_stats3[i].v_max for i in range(4)]

        self.dir_h_chart3.clear()
        bar_mean = pg.BarGraphItem(x=[v - 0.15 for v in x], height=h_means, width=0.3, brush='#e74c3c', name='均值')
        bar_max = pg.BarGraphItem(x=[v + 0.15 for v in x], height=h_maxs, width=0.3, brush='#c0392b', name='最大值')
        self.dir_h_chart3.addItem(bar_mean)
        self.dir_h_chart3.addItem(bar_max)
        self.dir_h_chart3.addLegend()
        ax_h = self.dir_h_chart3.getAxis('bottom')
        ax_h.setTicks([[(1, '方向1'), (2, '方向2'), (3, '方向3'), (4, '方向4')]])

        self.dir_v_chart3.clear()
        bar_mean_v = pg.BarGraphItem(x=[v - 0.15 for v in x], height=v_means, width=0.3, brush='#27ae60', name='均值')
        bar_max_v = pg.BarGraphItem(x=[v + 0.15 for v in x], height=v_maxs, width=0.3, brush='#1e8449', name='最大值')
        self.dir_v_chart3.addItem(bar_mean_v)
        self.dir_v_chart3.addItem(bar_max_v)
        self.dir_v_chart3.addLegend()
        ax_v = self.dir_v_chart3.getAxis('bottom')
        ax_v.setTicks([[(1, '方向1'), (2, '方向2'), (3, '方向3'), (4, '方向4')]])

    def _update_direction_enu_charts(self):
        # 串口2方向ENU对比 (ENU1 vs ENU2)
        for i in range(4):
            ds = self.direction_stats[i]
            curves = self._dir_enu_curves[i]
            if ds.has_enu_data():
                t1, e1, n1, u1 = ds.get_enu1_snapshot()
                t2, e2, n2, u2 = ds.get_enu2_snapshot()
                curves['east1'].setData(t1, e1)
                curves['east2'].setData(t2, e2)
                curves['north1'].setData(t1, n1)
                curves['north2'].setData(t2, n2)
                curves['up1'].setData(t1, u1)
                curves['up2'].setData(t2, u2)
            else:
                for k in ('east1','east2','north1','north2','up1','up2'):
                    if k in curves:
                        curves[k].setData([], [])

        # 串口3方向ENU对比 (ENU1 vs ENU3)
        for i in range(4):
            ds = self.direction_stats3[i]
            curves = self._dir_enu_curves3[i]
            if ds.has_enu_data():
                t1, e1, n1, u1 = ds.get_enu1_snapshot()
                t3, e3, n3, u3 = ds.get_enu3_snapshot()
                curves['east1'].setData(t1, e1)
                curves['east3'].setData(t3, e3)
                curves['north1'].setData(t1, n1)
                curves['north3'].setData(t3, n3)
                curves['up1'].setData(t1, u1)
                curves['up3'].setData(t3, u3)
            else:
                for k in ('east1','east3','north1','north3','up1','up3'):
                    if k in curves:
                        curves[k].setData([], [])

    def _render_dir_enu_charts_for_report(self, report_dir, ts_str="", direction_stats=None, label2="ENU2 (干扰测试)", label3=None):
        """为报告渲染方向ENU对比图
        Args:
            direction_stats: 方向统计列表(COM2用self.direction_stats, COM3用self.direction_stats3)
            label2: 第二条曲线的标签(COM2用"ENU2 (干扰测试)", COM3用"ENU3 (干扰测试2)")
            label3: 第三条曲线标签(为None则只画两条曲线)
        """
        if direction_stats is None:
            direction_stats = self.direction_stats
        QApplication.processEvents()
        png_paths = []
        for i in range(4):
            ds = direction_stats[i]
            if not ds.has_enu_data():
                continue
            t1, e1, n1, u1 = ds.get_enu1_snapshot()
            if label3 is None:
                # 双曲线模式: ENU1 vs ENU2 或 ENU1 vs ENU3
                if direction_stats is self.direction_stats:
                    _, d2e, d2n, d2u = ds.get_enu2_snapshot()
                else:
                    _, d2e, d2n, d2u = ds.get_enu3_snapshot()
                components = [('东向 E', e1, d2e), ('北向 N', n1, d2n), ('天向 U', u1, d2u)]
            else:
                # 三曲线模式(保留兼容)
                t2, e2, n2, u2 = ds.get_enu2_snapshot()
                t3, e3, n3, u3 = ds.get_enu3_snapshot()
                components = [('东向 E', e1, e2), ('北向 N', n1, n2), ('天向 U', u1, u2)]
                components3 = [e3, n3, u3]

            grid = pg.GraphicsLayoutWidget()
            grid.resize(1000, 650)
            grid.setBackground('w')
            for r, (title, d1, d2) in enumerate(components):
                p = grid.addPlot(row=r, col=0)
                p.setLabel('left', title, units='m')
                if r == 2:
                    p.setLabel('bottom', '历元')
                p.showGrid(x=True, y=True)
                p.addLegend()
                p.plot(t1, d1, pen={'color': '#e74c3c', 'width': 2}, name='ENU1 (无干扰)')
                p.plot(t1, d2, pen={'color': '#2980b9', 'width': 2}, name=label2)
                if label3 is not None:
                    p.plot(t3, components3[r], pen={'color': '#f39c12', 'width': 2, 'style': Qt.DashLine}, name=label3)
            grid.show()
            # 多次 processEvents 确保 pyqtgraph GraphicsLayout 完成布局
            for _ in range(5):
                QApplication.processEvents()

            prefix = 'com2' if direction_stats is self.direction_stats else 'com3'
            png_name = f'dir{i + 1}_{prefix}_enu_{ts_str}.png' if ts_str else f'dir{i + 1}_{prefix}_enu.png'
            png_path = os.path.join(report_dir, png_name)
            grid.scene().setSceneRect(0, 0, 1000, 650)
            exporter = ImageExporter(grid.scene())
            exporter.parameters()['width'] = 1000
            exporter.parameters()['height'] = 650
            exporter.export(png_path)
            grid.hide()
            grid.deleteLater()
            QApplication.processEvents()
            png_paths.append((i + 1, png_name))

        return png_paths

    def _reset_direction_stats(self):
        previously_active = [i for i in range(4) if self.direction_stats[i]._active]
        for i in range(4):
            self.direction_stats[i].reset()
            btn = self.dir_stat_boxes[i]['btn']
            btn.setText("启动方向测试")
            btn.setStyleSheet(
                "QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #2ecc71; }")
        for i in previously_active:
            self.direction_stats[i].start()
            self._dir_enu_active_index = i
            self.dir_stat_boxes[i]['btn'].setText("停止方向测试")
            self.dir_stat_boxes[i]['btn'].setStyleSheet(
                "QPushButton { background-color: #e74c3c; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #c0392b; }")
        self._update_direction_charts()
        self._update_direction_stats_display()
        self.log_info("串口2方向统计已重置")

    def _reset_direction_stats3(self):
        previously_active = [i for i in range(4) if self.direction_stats3[i]._active]
        for i in range(4):
            self.direction_stats3[i].reset()
            btn = self.dir_stat_boxes3[i]['btn']
            btn.setText("启动方向测试")
            btn.setStyleSheet(
                "QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #2ecc71; }")
        for i in previously_active:
            self.direction_stats3[i].start()
            self._dir3_enu_active_index = i
            self.dir_stat_boxes3[i]['btn'].setText("停止方向测试")
            self.dir_stat_boxes3[i]['btn'].setStyleSheet(
                "QPushButton { background-color: #e74c3c; color: white; font-weight: bold; padding: 4px 12px; } QPushButton:hover { background-color: #c0392b; }")
        self._update_direction_charts3()
        self._update_direction_stats_display()
        self.log_info("串口3方向统计已重置")

    def reset_all_stats(self):
        """重置所有统计"""
        self.statistics.reset()
        self.port1_satellites = {}
        self.port2_satellites = {}
        self.port3_satellites = {}  # [新增]
        self._port1_snr_signals = {}
        self._port2_snr_signals = {}
        self._port3_snr_signals = {}  # [新增]
        self._port1_snr_avg.reset()
        self._port2_snr_avg.reset()
        self._port3_snr_avg.reset()  # [新增]
        self._snr_diff_avg.reset()
        self._snr_diff2_avg.reset()  # [新增]
        self._last_port1_snr.clear()
        self._last_port2_snr.clear()
        self._last_port3_snr.clear()  # [新增]
        self._last_snr_diff.clear()
        self._last_snr_diff2.clear()  # [新增]
        self._last_snr_data = None
        self.enu1_ref_point = None
        self.enu1_ref_ready = False
        self.enu1_buffer = []
        self.enu1_times = []
        self.enu1_east_data = []
        self.enu1_north_data = []
        self.enu1_up_data = []
        self.enu1_3_times = []
        self.enu1_3_east_data = []
        self.enu1_3_north_data = []
        self.enu1_3_up_data = []
        self.enu1_east_curve.setData([], [])
        self.enu1_north_curve.setData([], [])
        self.enu1_up_curve.setData([], [])
        self.enu1_13_east_curve.setData([], [])
        self.enu1_13_north_curve.setData([], [])
        self.enu1_13_up_curve.setData([], [])
        self.enu1_east_label.setText("东: -- m")
        self.enu1_north_label.setText("北: -- m")
        self.enu1_up_label.setText("天: -- m")
        if self.enu_auto_mode:
            self.enu1_ref_label.setText("ENU1 基准: 自动采集中...")
            self.enu2_ref_label.setText("ENU2 基准: 自动采集中...")
            self.enu3_ref_label.setText("ENU3 基准: 自动采集中...")  # [新增]
            self.enu1_points_label.setText("(0/100)")
            self.enu2_points_label.setText("(0/100)")
            self.enu3_points_label.setText("(0/100)")  # [新增]
        elif self.enu_instant_mode:
            self.enu1_ref_label.setText("ENU1 基准: 等待瞬时GGA...")
            self.enu2_ref_label.setText("ENU2 基准: 等待瞬时GGA...")
            self.enu3_ref_label.setText("ENU3 基准: 等待瞬时GGA...")  # [新增]
            self.enu1_points_label.setText("(瞬时)")
            self.enu2_points_label.setText("(瞬时)")
            self.enu3_points_label.setText("(瞬时)")  # [新增]
        else:
            self.enu1_ref_label.setText("ENU1 基准: 等待手动输入")
            self.enu2_ref_label.setText("ENU2 基准: 等待手动输入")
            self.enu3_ref_label.setText("ENU3 基准: 等待手动输入")  # [新增]
            self.enu1_points_label.setText("(手动)")
            self.enu2_points_label.setText("(手动)")
            self.enu3_points_label.setText("(手动)")  # [新增]

        self.enu2_ref_point = None
        self.enu2_ref_ready = False
        self.enu2_buffer = []
        self.enu2_times = []
        self.enu2_east_data = []
        self.enu2_north_data = []
        self.enu2_up_data = []
        self.enu2_east_curve.setData([], [])
        self.enu2_north_curve.setData([], [])
        self.enu2_up_curve.setData([], [])
        self.enu2_east_label.setText("东: -- m")
        self.enu2_north_label.setText("北: -- m")
        self.enu2_up_label.setText("天: -- m")

        # [新增] 重置串口3 ENU
        self.enu3_ref_point = None
        self.enu3_ref_ready = False
        self.enu3_buffer = []
        self.enu3_times = []
        self.enu3_east_data = []
        self.enu3_north_data = []
        self.enu3_up_data = []
        self.enu3_east_curve.setData([], [])
        self.enu3_north_curve.setData([], [])
        self.enu3_up_curve.setData([], [])
        self.enu3_east_label.setText("东: -- m")
        self.enu3_north_label.setText("北: -- m")
        self.enu3_up_label.setText("天: -- m")

        self.enu1_std_east.setText("E: -- m")
        self.enu1_std_north.setText("N: -- m")
        self.enu1_std_up.setText("U: -- m")
        self.enu2_std_east.setText("E: -- m")
        self.enu2_std_north.setText("N: -- m")
        self.enu2_std_up.setText("U: -- m")
        self.enu3_std_east.setText("E: -- m")  # [新增]
        self.enu3_std_north.setText("N: -- m")  # [新增]
        self.enu3_std_up.setText("U: -- m")  # [新增]

        self._std_enu1_east.reset()
        self._std_enu1_north.reset()
        self._std_enu1_up.reset()
        self._std_enu2_east.reset()
        self._std_enu2_north.reset()
        self._std_enu2_up.reset()
        self._std_enu3_east.reset()  # [新增]
        self._std_enu3_north.reset()  # [新增]
        self._std_enu3_up.reset()  # [新增]

        # 重置串口1 GGA显示
        self.p1_utc_label.setText("-")
        self.p1_lat_label.setText("-")
        self.p1_lon_label.setText("-")
        self.p1_alt_label.setText("-")
        self.p1_quality_label.setText("无数据")
        self.p1_nsats_label.setText("0")

        # 重置串口2 GGA显示
        self.p2_utc_label.setText("-")
        self.p2_lat_label.setText("-")
        self.p2_lon_label.setText("-")
        self.p2_alt_label.setText("-")
        self.p2_quality_label.setText("无数据")
        self.p2_nsats_label.setText("0")

        # [新增] 重置串口3 GGA显示
        self.p3_utc_label.setText("-")
        self.p3_lat_label.setText("-")
        self.p3_lon_label.setText("-")
        self.p3_alt_label.setText("-")
        self.p3_quality_label.setText("无数据")
        self.p3_nsats_label.setText("0")

        self.p1_gga_nsat = 0
        self.p2_gga_nsat = 0
        self.p3_gga_nsat = 0  # [新增]

        # 重置方向统计
        self._reset_direction_stats()
        self._reset_direction_stats3()
        self._latest_enu2_east = 0.0
        self._latest_enu2_north = 0.0
        self._latest_enu2_up = 0.0
        self._latest_enu3_east = 0.0  # [新增]
        self._latest_enu3_north = 0.0  # [新增]
        self._latest_enu3_up = 0.0  # [新增]
        self._p2_gga_new_epoch = False
        self._p3_gga_new_epoch = False  # [新增]
        self._last_enu1_pos = None
        self._last_enu2_pos = None
        self._last_enu3_pos = None  # [新增]
        self._latest_p2_quality = 0
        self._latest_p3_quality = 0  # [新增]

        self.update_plots()
        self.log_info("统计已重置")
    
    def clear_all_data(self):
        """清空所有数据"""
        self.data_preview.clear()
        self.log_window.clear()
        self.serial_save_buffer = deque(maxlen=40000)
        self.serial2_save_buffer = deque(maxlen=40000)
        self.serial3_save_buffer = deque(maxlen=40000)  # [新增]
        self.data_count_label.setText("已读取: 0 行")
        self.reset_all_stats()
        if self.auto_log_enabled:
            self._open_auto_log_files()

    def clear_data_preview(self):
        """清空数据预览"""
        self.data_preview.clear()

    def clear_log(self):
        """清除日志窗口"""
        self.log_window.clear()
        self.log_info("日志已清除")

    def save_serial_log(self, port_id):
        from PyQt5.QtWidgets import QFileDialog
        if port_id == 1:
            buf = self.serial_save_buffer
        elif port_id == 2:
            buf = self.serial2_save_buffer
        else:  # [新增] port_id == 3
            buf = self.serial3_save_buffer
        if not buf:
            QMessageBox.warning(self, "警告", f"串口{port_id}没有可保存的日志数据")
            return
        file_path, _ = QFileDialog.getSaveFileName(self, f"保存串口{port_id}日志", 
            f"serial_port{port_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log",
            "日志文件 (*.log);;文本文件 (*.txt);;所有文件 (*.*)")
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    for data in buf:
                        f.write(data.decode('utf-8', errors='replace') if isinstance(data, bytes) else str(data))
                self.log_info(f"串口{port_id}日志已保存到: {file_path}")
                QMessageBox.information(self, "成功", f"串口{port_id}日志已保存到:\n{file_path}")
            except Exception as e:
                self.log_error(f"保存串口{port_id}日志失败: {str(e)}")
                QMessageBox.critical(self, "错误", f"无法保存文件:\n{str(e)}")

    def _save_all_serial_logs(self):
        """保存所有串口的日志"""
        from PyQt5.QtWidgets import QFileDialog
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存全部日志",
            f"all_serial_logs_{ts}.txt",
            "文本文件 (*.txt);;所有文件 (*.*)")
        if not file_path:
            return
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                for port_id, buf in [(1, self.serial_save_buffer), (2, self.serial2_save_buffer), (3, self.serial3_save_buffer)]:
                    if buf:
                        f.write(f"=== 串口{port_id} 日志 ({len(buf)} 条) ===\n")
                        for data in buf:
                            f.write(data.decode('utf-8', errors='replace') if isinstance(data, bytes) else str(data))
                        f.write("\n")
                    else:
                        f.write(f"=== 串口{port_id} 日志: 无数据 ===\n\n")
            self.log_info(f"全部日志已保存到: {file_path}")
            QMessageBox.information(self, "成功", f"全部日志已保存到:\n{file_path}")
        except Exception as e:
            self.log_error(f"保存全部日志失败: {str(e)}")
            QMessageBox.critical(self, "错误", f"无法保存文件:\n{str(e)}")

    def _apply_device_sn(self, port_id):
        """应用设备序号并写入日志"""
        sn_input = self.device_sn3_input if port_id == 3 else self.device_sn2_input
        sn = sn_input.text().strip()
        if not sn:
            QMessageBox.warning(self, "警告", "请输入设备序号")
            return
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_msg = f"{sn} 设备序号开始测试"
        self.log_info(f"[串口{port_id}] {log_msg}")
        # 写入所有已开启的日志文件
        for f in (self._log_file1, self._log_file2, self._log_file3):
            if f and not f.closed:
                try:
                    f.write(f"[{ts}] {log_msg}\n")
                    f.flush()
                except Exception:
                    pass

    def _get_device_sn(self, direction_stats=None):
        """根据方向统计列表获取对应端口的设备序号"""
        if direction_stats is self.direction_stats3:
            if hasattr(self, 'device_sn3_input'):
                return self.device_sn3_input.text().strip() or "test"
        else:
            if hasattr(self, 'device_sn2_input'):
                return self.device_sn2_input.text().strip() or "test"
        return "test"

    def _build_report_lines(self, png_map=None, direction_stats=None, port_label="串口2", enu_label="ENU2"):
        """生成报告文本行
        Args:
            direction_stats: 方向统计列表(None=默认COM2, self.direction_stats3=COM3)
            port_label: 串口标签
            enu_label: ENU标签
        """
        if direction_stats is None:
            direction_stats = self.direction_stats
        is_com3_report = (direction_stats is self.direction_stats3)
        lines = []

        def w(line=""):
            lines.append(line)

        now = datetime.now()
        now_date = now.strftime('%Y-%m-%d')

        w(f"# 抗干扰天线测试报告")
        w()
        w(f"**生成时间:** {now.strftime('%Y-%m-%d %H:%M:%S')}")
        w()

        port1_connected = self.serial_port1.serial_port is not None and self.serial_port1.serial_port.is_open
        port2_connected = self.serial_port2.serial_port is not None and self.serial_port2.serial_port.is_open
        port3_connected = self.serial_port3.serial_port is not None and self.serial_port3.serial_port.is_open  # [新增]
        p1_port_name = self.port1_combo.currentText() if port1_connected else "-"
        p2_port_name = self.port2_combo.currentText() if port2_connected else "-"
        p3_port_name = self.port3_combo.currentText() if port3_connected else "-"  # [新增]
        p1_baud = self.port1_baud.currentText() if port1_connected else "-"
        p2_baud = self.port2_baud.currentText() if port2_connected else "-"
        p3_baud = self.port3_baud.currentText() if port3_connected else "-"  # [新增]
        if is_com3_report:
            w(f"**串口1 (无干扰):** {p1_port_name} @ {p1_baud} bps | **串口3 (干扰测试2):** {p3_port_name} @ {p3_baud} bps")
        else:
            w(f"**串口1 (无干扰):** {p1_port_name} @ {p1_baud} bps | **串口2 (干扰测试):** {p2_port_name} @ {p2_baud} bps")
        w()

        w("## 一、实验配置")
        w()
        w("### 1.1 基本信息")
        w()
        w("| 项目 | 内容 |")
        w("|------|------|")
        test_location = self.test_location_input.text().strip() or "长沙"
        dut_model = self.dut_model_input.text().strip() or "4 阵元 GNSS 抗干扰天线"
        array_form = self.array_form_combo.currentText().strip() or "4 阵元 GNSS 抗干扰天线"
        jam_type = self.jam_type_combo.currentText().strip() or "线性扫频干扰"
        azimuth = self.azimuth_input.text().strip() or "正东 / 正南 / 正西 / 正北"
        antenna_spacing = self.antenna_spacing_input.text().strip() or "3 m"
        antenna_height = self.antenna_height_input.text().strip() or "1.5 m"

        w(f"| 测试日期 | {now_date} |")
        w(f"| 测试地点 | {test_location} |")
        w(f"| 待测设备型号 | {dut_model} |")
        w(f"| 阵列形式 | {array_form} |")
        w(f"| 干扰类型 | {jam_type} |")
        w(f"| 入射方位 | {azimuth} |")
        w(f"| 收发天线间距 | {antenna_spacing} |")
        w(f"| 天线架设高度 | {antenna_height} |")
        w()

        w("### 1.2 UBX 首次定位时间")
        w()
        w("| 端口 | 首次定位时间 (s) |")
        w("|------|------------------|")
        p1_ttff_str = f"{self.p1_ttff_s:.1f}" if self.p1_ttff_s > 0 else "尚未定位"
        p2_ttff_str = f"{self.p2_ttff_s:.1f}" if self.p2_ttff_s > 0 else "尚未定位"
        p3_ttff_str = f"{self.p3_ttff_s:.1f}" if self.p3_ttff_s > 0 else "尚未定位"  # [新增]
        w(f"| 串口1 (无干扰) | {p1_ttff_str} |")
        w(f"| 串口2 (干扰测试) | {p2_ttff_str} |")
        w(f"| 串口3 (干扰测试2) | {p3_ttff_str} |")  # [新增]
        w()

        def calc_std(arr):
            if len(arr) <= 1:
                return 0.0
            m = sum(arr) / len(arr)
            variance = sum(v * v for v in arr) / len(arr) - m * m
            return math.sqrt(variance) if variance > 0 else 0.0

        all_snr_keys = sorted(
            set(self.port1_satellites.keys()) | set(self.port2_satellites.keys()) | set(self.port3_satellites.keys()),
            key=lambda x: (x[:2], int(x[2:])))
        system_names = {"GP": "GPS", "BD": "BDS", "GL": "GLONASS", "GA": "Galileo", "GB": "BDS-3"}

        section_names = ["二", "三", "四", "五"]
        tested = [(i, direction_stats[i]) for i in range(4) if direction_stats[i].total_epochs > 0]
        for idx, (i, ds) in enumerate(tested):
            sec_name = section_names[idx]
            sec_num = idx + 2
            ds.update_duration()
            s = ds.get_stats()

            w(f"## {sec_name}、方向{i + 1} 测试数据")
            w()

            w(f"### {sec_num}.1 GGA 数据分析")
            w()
            w("| 项目 | 数值 |")
            w("|------|------|")
            w(f"| 方向 | 方向{i + 1} |")
            w(f"| 测试时长 | {s['duration']} |")
            w(f"| 成功定位/总历元数 | {s['successful_epochs']}/{s['total_epochs']} |")

            if ds.has_enu_data():
                _, e1, n1, u1 = ds.get_enu1_snapshot()
                if is_com3_report:
                    _, e2, n2, u2 = ds.get_enu3_snapshot()
                else:
                    _, e2, n2, u2 = ds.get_enu2_snapshot()
                es = calc_std(e2); ns = calc_std(n2); us = calc_std(u2)
                e_max = max(abs(v) for v in e2) if e2 else 0.0
                n_max = max(abs(v) for v in n2) if n2 else 0.0
                u_max = max(abs(v) for v in u2) if u2 else 0.0
            else:
                es = ns = us = 0.0
                e_max = n_max = u_max = 0.0

            w(f"| 东向标准差 | {es:.4f} m |")
            w(f"| 东向最大值 | {e_max:.4f} m |")
            w(f"| 北向标准差 | {ns:.4f} m |")
            w(f"| 北向最大值 | {n_max:.4f} m |")
            w(f"| 天向标准差 | {us:.4f} m |")
            w(f"| 天向最大值 | {u_max:.4f} m |")
            w()

            w(f"### {sec_num}.2 GSV 卫星信噪比分析")
            w()
            w("| 星座 | PRN | 信号 | 串口1载噪比均值 | 串口2载噪比均值 | 恶化值1-2 | 串口3载噪比均值 | 恶化值1-3 |")
            w("|------|-----|------|---------------|---------------|-----------|---------------|-----------|")
            for key in all_snr_keys:
                sys = key[:2]
                prn = int(key[2:])
                sys_name = system_names.get(sys, sys)
                signal = self._port1_snr_signals.get(key) or self._port2_snr_signals.get(key) or self._port3_snr_signals.get(key) or '---'
                a1 = self._port1_snr_avg.get(key)
                a2 = self._port2_snr_avg.get(key)
                a3 = self._port3_snr_avg.get(key)  # [新增]
                ad = self._snr_diff_avg.get(key)
                ad2 = self._snr_diff2_avg.get(key)  # [新增]
                a1_str = f"{a1:.1f}" if a1 is not None else "--"
                a2_str = f"{a2:.1f}" if a2 is not None else "--"
                a3_str = f"{a3:.1f}" if a3 is not None else "--"  # [新增]
                ad_str = f"{ad:+.1f}" if ad is not None else "--"
                ad2_str = f"{ad2:+.1f}" if ad2 is not None else "--"  # [新增]
                w(f"| {sys_name} | {prn:02d} | {signal} | {a1_str} | {a2_str} | {ad_str} | {a3_str} | {ad2_str} |")
            w()

            if png_map and (i + 1) in png_map:
                w(f"### {sec_num}.3 ENU 误差对比图")
                w()
                w(f"![方向{i + 1} ENU误差对比]({png_map[i + 1]})")
                w()

        cnums = ["", "一", "二", "三", "四", "五", "六", "七"]
        next_sec = cnums[len(tested) + 2]
        w(f"## {next_sec}、抗干扰设备信息")
        w()
        w("| 项目 | 说明 |")
        w("|------|------|")
        sn = self._get_device_sn(direction_stats)
        w(f"| 设备序号 | {sn if sn else ''} |")
        w("| 备注 | |")
        w()

        w("---")
        w()
        w(f"*报告由 NMEA 数据实时分析系统自动生成 | {now.strftime('%Y-%m-%d %H:%M:%S')}*")

        return lines, now.strftime('%Y%m%d_%H%M%S')

    @staticmethod
    def _md_lines_to_html(lines, report_dir):
        html = ['<!DOCTYPE html><html><head><meta charset="utf-8"><style>',
                'body{font-family:SimSun,sans-serif;font-size:12pt;margin:20px;}',
                'h1{font-size:18pt;text-align:center;margin:10px 0;}',
                'h2{font-size:14pt;border-bottom:1px solid #000;padding-bottom:2px;margin:16px 0 8px 0;}',
                'h3{font-size:12pt;margin:12px 0 6px 0;}',
                'table{border-collapse:collapse;margin:8px 0;}',
                'th,td{border:1px solid #000;padding:3px 6px;font-size:10pt;}',
                'th{background-color:#ddd;font-weight:bold;}',
                'img{width:620px;}',
                'hr{border:none;border-top:1px solid #ccc;margin:15px 0;}',
                'ul{margin:5px 0;padding-left:20px;}',
                'li{font-size:11pt;margin:2px 0;}',
                'p{font-size:11pt;margin:4px 0;}',
                '</style></head><body>']

        i = 0
        while i < len(lines):
            line = lines[i]

            if line.startswith('|'):
                tbl_lines = []
                while i < len(lines) and lines[i].startswith('|'):
                    tbl_lines.append(lines[i])
                    i += 1

                ti = 0
                while ti < len(tbl_lines):
                    header_cells = [c.strip() for c in tbl_lines[ti].split('|')[1:-1]]
                    ti += 1
                    if ti < len(tbl_lines):
                        sep_cells = [c.strip() for c in tbl_lines[ti].split('|')[1:-1]]
                        if all(re.match(r'^[-:]+$', c) for c in sep_cells):
                            ti += 1

                    html.append('<table width="100%">')
                    html.append('<tr>')
                    for cell in header_cells:
                        cell = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', cell)
                        html.append(f'<th>{cell}</th>')
                    html.append('</tr>')

                    while ti < len(tbl_lines):
                        nc = [c.strip() for c in tbl_lines[ti].split('|')[1:-1]]
                        if all(re.match(r'^[-:]+$', c) for c in nc):
                            break
                        html.append('<tr>')
                        for cell in nc:
                            cell = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', cell)
                            html.append(f'<td>{cell}</td>')
                        html.append('</tr>')
                        ti += 1
                    html.append('</table>')
                continue

            m = re.match(r'^(#{1,3})\s+(.+)$', line)
            if m:
                level = len(m.group(1))
                text = m.group(2)
                text = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', text)
                text = re.sub(r'(?<!\*)\*(?!\*)(.+?)(?<!\*)\*(?!\*)', r'<i>\1</i>', text)
                html.append(f'<h{level}>{text}</h{level}>')
                i += 1
                continue

            m = re.match(r'^!\[(.+?)\]\((.+?)\)$', line)
            if m:
                alt = m.group(1)
                img_name = m.group(2)
                abs_path = os.path.join(report_dir, img_name).replace('\\', '/')
                html.append(f'<p><img src="file:///{abs_path}" width="620" alt="{alt}"></p>')
                i += 1
                continue

            if line.strip() == '---':
                html.append('<hr>')
                i += 1
                continue

            if re.match(r'^-\s+', line):
                html.append('<ul>')
                while i < len(lines) and re.match(r'^-\s+', lines[i]):
                    text = lines[i][2:]
                    text = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', text)
                    text = re.sub(r'(?<!\*)\*(?!\*)(.+?)(?<!\*)\*(?!\*)', r'<i>\1</i>', text)
                    html.append(f'<li>{text}</li>')
                    i += 1
                html.append('</ul>')
                continue

            if not line.strip():
                i += 1
                continue

            text = line
            text = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', text)
            text = re.sub(r'(?<!\*)\*(?!\*)(.+?)(?<!\*)\*(?!\*)', r'<i>\1</i>', text)
            html.append(f'<p>{text}</p>')
            i += 1

        html.append('</body></html>')
        return '\n'.join(html)

    # ========== 统一报告导出（合并原12个重复函数） ==========
    def _do_export_report(self, ext="md", direction_stats=None, port_label="串口2", enu_label="ENU2",
                          chart_label2="ENU2 (干扰测试)", file_prefix="抗干扰天线测试报告",
                          dialog_title="导出测试报告"):
        """统一报告导出核心实现"""
        ts_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        sn = self._get_device_sn(direction_stats)
        default_name = f"{file_prefix}_{sn}_{ts_str}.{ext}"
        reports_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'reports')
        os.makedirs(reports_dir, exist_ok=True)
        default_path = os.path.join(reports_dir, default_name)

        from PyQt5.QtWidgets import QFileDialog
        filt = "Markdown 文件 (*.md);;所有文件 (*.*)" if ext == "md" else "PDF 文件 (*.pdf);;所有文件 (*.*)"
        file_path, _ = QFileDialog.getSaveFileName(self, dialog_title, default_path, filt)
        if not file_path:
            return

        report_dir = os.path.dirname(file_path)
        if not report_dir:
            report_dir = os.path.dirname(os.path.abspath(__file__))

        png_paths = self._render_dir_enu_charts_for_report(report_dir, ts_str, direction_stats, chart_label2)
        png_map = {di: name for di, name in png_paths} if png_paths else None

        lines, _ = self._build_report_lines(png_map, direction_stats, port_label, enu_label)

        if ext == "md":
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines))
        else:
            html_text = self._md_lines_to_html(lines, report_dir)
            doc = QTextDocument()
            doc.setHtml(html_text)
            printer = QPrinter(QPrinter.HighResolution)
            printer.setOutputFormat(QPrinter.PdfFormat)
            printer.setOutputFileName(file_path)
            printer.setPageSize(QPrinter.A4)
            printer.setPageMargins(15, 15, 15, 15, QPrinter.Millimeter)
            doc.print_(printer)

        self.log_info(f"{dialog_title}已导出: {file_path}")
        if png_paths:
            self.log_info(f"共生成 {len(png_paths)} 张 ENU 对比图表")
        QMessageBox.information(self, "导出成功", f"{dialog_title}已保存到:\n{file_path}")

    # ========== 串口2独立报告导出 ==========
    def export_test_report2(self):
        try:
            self._do_export_report(direction_stats=self.direction_stats, file_prefix="串口2测试报告",
                                   dialog_title="导出串口2测试报告")
        except Exception as e:
            self.log_error(f"导出串口2报告失败: {str(e)}"); import traceback; traceback.print_exc()
            QMessageBox.critical(self, "导出失败", f"无法生成串口2报告:\n{str(e)}")

    def export_pdf_report2(self):
        try:
            self._do_export_report(ext="pdf", direction_stats=self.direction_stats, file_prefix="串口2测试报告",
                                   dialog_title="导出串口2PDF报告")
        except Exception as e:
            self.log_error(f"导出串口2PDF报告失败: {str(e)}"); import traceback; traceback.print_exc()
            QMessageBox.critical(self, "导出失败", f"无法生成串口2PDF报告:\n{str(e)}")

    # ========== 串口3独立报告导出 ==========
    def export_test_report3(self):
        try:
            self._do_export_report(ext="md", direction_stats=self.direction_stats3, port_label="串口3",
                                   enu_label="ENU3", chart_label2="ENU3 (干扰测试2)",
                                   file_prefix="串口3测试报告", dialog_title="导出串口3测试报告")
        except Exception as e:
            self.log_error(f"导出串口3报告失败: {str(e)}"); import traceback; traceback.print_exc()
            QMessageBox.critical(self, "导出失败", f"无法生成串口3报告:\n{str(e)}")

    def export_pdf_report3(self):
        try:
            self._do_export_report(ext="pdf", direction_stats=self.direction_stats3, port_label="串口3",
                                   enu_label="ENU3", chart_label2="ENU3 (干扰测试2)",
                                   file_prefix="串口3测试报告", dialog_title="导出串口3PDF报告")
        except Exception as e:
            self.log_error(f"导出串口3PDF报告失败: {str(e)}"); import traceback; traceback.print_exc()
            QMessageBox.critical(self, "导出失败", f"无法生成串口3PDF报告:\n{str(e)}")

    def _open_auto_log_files(self):
        self._close_auto_log_files()
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path1 = os.path.join(self.auto_log_dir, f"serial1_{ts}.log")
        path2 = os.path.join(self.auto_log_dir, f"serial2_{ts}.log")
        path3 = os.path.join(self.auto_log_dir, f"serial3_{ts}.log")  # [新增]
        try:
            self._log_file1 = open(path1, 'w', encoding='utf-8')
            self._log_file2 = open(path2, 'w', encoding='utf-8')
            self._log_file3 = open(path3, 'w', encoding='utf-8')  # [新增]
            self.log_info(f"自动日志已开启: {path1}, {path2}, {path3}")  # [修改]
        except Exception as e:
            self.log_error(f"无法创建日志文件: {str(e)}")
            self._log_file1 = None
            self._log_file2 = None
            self._log_file3 = None  # [新增]

    def _close_auto_log_files(self):
        for f in (self._log_file1, self._log_file2, self._log_file3):  # [修改] 添加_log_file3
            if f and not f.closed:
                try:
                    f.flush()
                    f.close()
                except:
                    pass
        self._log_file1 = None
        self._log_file2 = None
        self._log_file3 = None  # [新增]

    def _write_auto_log(self, port_id, data):
        if not self.auto_log_enabled:
            return
        if port_id == 1:
            f = self._log_file1
        elif port_id == 2:
            f = self._log_file2
        else:  # [新增] port_id == 3
            f = self._log_file3
        if f and not f.closed:
            try:
                line = data.decode('utf-8', errors='replace') if isinstance(data, bytes) else str(data)
                ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                line = f"[{ts}] {line.strip()}\n"
                f.write(line)
                self._log_flush_count += 1
                if self._log_flush_count >= self._LOG_FLUSH_INTERVAL:
                    f.flush()
                    self._log_flush_count = 0
            except:
                pass

    def _on_auto_log_toggled(self):
        self.auto_log_enabled = self.auto_log_cb.isChecked()
        if self.auto_log_enabled:
            self._open_auto_log_files()
            self.log_info("自动日志保存已开启")
        else:
            self._close_auto_log_files()
            self.log_info("自动日志保存已关闭")

    def _write_info_to_logs(self, prefix, message):
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        line = f"[{ts}] {prefix}: {message}\n"
        for f in (self._log_file1, self._log_file2, self._log_file3):  # [修改] 添加_log_file3
            if f and not f.closed:
                try:
                    f.write(line)
                    f.flush()
                except:
                    pass

    def log_info(self, message):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_window.append(f"[{timestamp}] INFO: {message}")
        self.log_window.moveCursor(QTextCursor.End)
        self._write_info_to_logs("INFO", message)

    def log_error(self, message):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_window.append(f"[{timestamp}] ERROR: {message}")
        self.log_window.moveCursor(QTextCursor.End)
        self._write_info_to_logs("ERROR", message)
        
    def closeEvent(self, event):
        self.serial_port1.disconnect()
        self.serial_port2.disconnect()
        self.serial_port3.disconnect()  # [新增]
        self.update_timer.stop()
        self._close_auto_log_files()
        event.accept()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda *args: QApplication.quit())
    app = QApplication(sys.argv)
    window = NMEADataAnalyzer()
    window.show()
    sys.exit(app.exec_())
