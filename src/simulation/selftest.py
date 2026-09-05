"""
一键自检运行器 (SelfTestRunner)

在无需连接真实硬件与物理串口的条件下，通过高仿真回放引擎走通全部业务功能闭环链路，
自动化校验定位解算、ENU 坐标变换、异常值剔除、SNR 衰减识别、方向测试以及报表生成，
并输出量化的自检验证清单与测试报告。

运行模式:
  1. GUI 模式: main.py 界面点击【一键自检】按钮启动交互式运行，实时展示进度条与日志。
  2. CLI 模式: 运行 python selftest_cli.py --minutes 60 --speed 10 --seed 42 进行静默回归测试。
"""

import glob
import os
import sys
import time
import traceback
from datetime import datetime

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from src.nmea.nmea_parser import NMEAParser
from src.simulation.nmea_generator import generate_port_timeline
from src.simulation.sim_serial_manager import SimulatedSerialManager

# 启动方向测试的仿真时间阈值 (秒):
# 前 100~150 秒用于基准串口累积足够的定位点以锁定 ENU 参考原点 (Ref Point)
DIR_START_EPOCH = 150


class SelfTestRunner(QObject):
    """
    一键自检运行器核心控制类。
    
    协调构建 3 路虚拟串口仿真器 -> 加速回放数据 -> 看门狗监控 -> 业务项指标检验 -> 导出评测报告。
    """

    # 信号定义: (是否全部通过 all_ok, 检验项明细列表 [(item_name, ok_bool, detail_str), ...])
    finished = pyqtSignal(bool, list)

    def __init__(self, window, headless=False):
        """
        初始化自检运行器。

        参数:
            window (NMEADataAnalyzer): 主窗口实例引用。
            headless (bool): 是否为无界面离屏命令行运行模式。
        """
        super().__init__()
        self.w = window                         # 主窗口引用
        self.headless = headless                # 是否静默无头模式
        self._running = False                   # 自检是否正在执行中
        self._finalized = False                 # 是否已完成结算与清理
        self._timeout = False                   # 是否发生看门狗超时
        self._cancelled = False                 # 是否被用户主动取消
        self._sims = []                         # 3 路 SimulatedSerialManager 实例列表
        self._finished_sims = 0                 # 已回放完毕的虚拟串口数
        self._dir_started = False               # 方向测试是否已自动触发
        self._minutes = 60                      # 仿真时长 (分钟)
        self._speed = 10                        # 回放加速倍率
        self._seed = 42                         # 随机数种子
        self._direction = True                  # 是否包含方向测试
        self._exc_count = 0                     # 未捕获异常计数
        self._exc_details = []                  # 异常堆栈信息详情
        self._orig_hook = None                  # 原始系统未捕获异常处理钩子
        self._orig_log_dir = None               # 启动前的原始日志保存目录
        self._orig_auto_log = None              # 启动前的自动保存日志开关状态
        self._wall0 = 0.0                       # 物理墙上时钟启动时刻
        self._last_quarter = 0                  # 进度百分比记录 (25%, 50%, 75%, 100%)
        
        # 状态监控与看门狗定时器 (500ms 周期)
        self._monitor = QTimer(self)
        self._monitor.setInterval(500)
        self._monitor.timeout.connect(self._on_monitor)

    def is_running(self):
        """检查自检当前是否正处于运行状态。"""
        return self._running

    def progress(self):
        """
        获取 3 路模拟串口回放的全局整体进度 (0.0 ~ 1.0)。
        
        取 3 路串口中的最小进度值作为全局保守进度。
        """
        if not self._running or not self._sims:
            return 0.0
        return min(s.progress() for s in self._sims)

    def cancel(self):
        """
        用户手动取消自检流程。
        
        停止数据回放与监控定时器，截断数据，并基于当前已接收的数据生成截断版评测报告。
        """
        if not self._running or self._finalized:
            return
        self._cancelled = True
        self._monitor.stop()
        for s in self._sims:
            try:
                s.disconnect()
            except Exception:
                pass
        # 若方向测试正在进行，则停止当前方向
        if self._dir_started and self.w.direction_stats[0]._active:
            self.w._toggle_single_direction(0)
        self.w.log_info("自检被用户取消, 提前结束")
        # 延迟 200ms 执行最终结算
        QTimer.singleShot(200, self._finalize)

    def _fail_start(self, message):
        """记录启动失败原因，并在 GUI 模式下弹出警告弹窗。"""
        self.w.log_error(f"一键自检启动被拒绝: {message}")
        if not self.headless:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self.w, "无法启动自检", message)
        return False

    def start(self, minutes=60, speed=10, seed=42, include_direction=True):
        """
        启动一键全自动自检流程。

        参数:
            minutes (int): 仿真时长 (分钟)，默认 60。
            speed (int 或 float): 回放加速倍率，默认 10。
            seed (int): 随机数种子，默认 42。
            include_direction (bool): 是否自动化测试方向测试环节，默认 True。

        返回:
            bool: 启动成功返回 True，若存在物理串口连接或正在运行则返回 False。
        """
        if self._running:
            return self._fail_start("自检正在进行中")
        w = self.w
        
        # 安全检查: 禁止在已有真实物理串口连接的情况下启动自检
        real = [m for m in (w.serial_port1, w.serial_port2, w.serial_port3)
                if m.serial_port is not None]
        if real:
            ports = ", ".join(str(m.port_id) for m in real)
            return self._fail_start(f"检测到真实串口已连接(串口{ports}), 禁止启动自检, 请先断开")

        self._minutes = int(minutes)
        self._speed = float(speed)
        self._seed = int(seed)
        self._direction = bool(include_direction)
        self._running = True
        self._finalized = False
        self._timeout = False
        self._cancelled = False
        self._dir_started = False
        self._finished_sims = 0
        self._last_quarter = 0
        self._exc_count = 0
        self._exc_details = []
        self._wall0 = time.monotonic()

        # 挂载异常拦截钩子，记录运行期间的任何未捕获异常
        self._orig_hook = sys.excepthook
        sys.excepthook = self._exc_hook

        # 创建本次自检专用的日志与报告目录 (带时间戳)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._ts = ts
        root = os.path.dirname(w.auto_log_dir)
        self._orig_log_dir = w.auto_log_dir
        self._orig_auto_log = w.auto_log_enabled
        self.log_dir = os.path.join(root, 'log', f'selftest_{ts}')
        self.report_dir = os.path.join(root, 'reports', f'selftest_{ts}')
        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.report_dir, exist_ok=True)

        # 重定向自动日志输出路径到自检专有目录
        w._close_auto_log_files()
        w.auto_log_dir = self.log_dir
        w.auto_log_enabled = True
        w._open_auto_log_files()

        # 清空界面数据预览与图表
        w.clear_data_preview()
        w.current_data_source = 'serial'
        w.current_parser = NMEAParser()

        # 实例化 3 路虚拟仿真串口并连接主窗口数据处理槽函数
        self._sims = []
        for pid in (1, 2, 3):
            tl = generate_port_timeline(pid, self._minutes, self._seed)
            sim = SimulatedSerialManager(pid, tl, self._speed)
            sim.data_received.connect(w.handle_data)
            sim.connection_status.connect(w.update_connection_status)
            sim.error_occurred.connect(w.handle_serial_error)
            sim.ubx_received.connect(w.handle_ubx)
            sim.sim_finished.connect(self._on_sim_finished)
            self._sims.append(sim)

        # 启动主界面图表定时更新与虚拟串口回放
        w.update_timer.start()
        for s in self._sims:
            s.start_sim()
        self._monitor.start()

        expected = self._minutes * 60 / self._speed
        w.log_info(f"一键自检已启动: {self._minutes}分钟@{self._speed}x seed={self._seed} "
                   f"方向测试={'开' if self._direction else '关'} 预计{expected:.0f}秒")
        return True

    def _exc_hook(self, etype, value, tb):
        """捕获运行期间未捕获异常的回调钩子。"""
        self._exc_count += 1
        detail = ''.join(traceback.format_exception(etype, value, tb))
        if len(self._exc_details) < 5:
            self._exc_details.append(detail)
        if self.headless:
            print(detail)

    def _on_sim_finished(self, port_id):
        """虚拟串口单路回放完成计数回调。"""
        self._finished_sims += 1

    def _on_monitor(self):
        """
        看门狗与自动化业务监控函数 (每 500ms 触发一次)。

        控制逻辑:
          1. 进度追踪: 每推进 25% 输出一次阶段进度日志。
          2. 方向测试自动激活: 虚拟时间流逝超过 150 秒且基准原点锁定后，自动启动方向 1 测试。
          3. 正常完成判断: 当 3 路串口全部回放结束，停止监控并调度最终结算。
          4. 看门狗超时保护: 若物理耗时超出预定时长达 60 秒以上，强制中断避免程序死锁挂起。
        """
        w = self.w
        elapsed = time.monotonic() - self._wall0
        expected = self._minutes * 60 / self._speed

        # 进度分段日志输出
        quarter = int(min(s.progress() for s in self._sims) * 4)
        if quarter > self._last_quarter:
            self._last_quarter = quarter
            w.log_info(f"自检进度约{quarter * 25}% ({elapsed:.0f}s/{expected:.0f}s)")

        # 自动触发方向测试
        if self._direction and not self._dir_started:
            if min(s.sim_time() for s in self._sims) >= DIR_START_EPOCH and w.enu2_ref_ready:
                w._toggle_single_direction(0)
                self._dir_started = True
                w.log_info("自检: 方向1测试已自动启动")

        # 判断三路是否全部回放完成
        if self._finished_sims >= len(self._sims):
            self._monitor.stop()
            if self._dir_started and w.direction_stats[0]._active:
                w._toggle_single_direction(0)
            QTimer.singleShot(800, self._finalize)
            return

        # 看门狗超时熔断机制 (预期时长 + 60秒冗余)
        if elapsed > expected + 60:
            self._monitor.stop()
            if self._dir_started and w.direction_stats[0]._active:
                w._toggle_single_direction(0)
            self._timeout = True
            w.log_error("自检超时(看门狗触发), 提前结束")
            QTimer.singleShot(200, self._finalize)

    def _restore(self):
        """恢复主窗口至自检前的原始状态与配置。"""
        w = self.w
        if self._orig_hook is not None:
            sys.excepthook = self._orig_hook
            self._orig_hook = None
        for s in self._sims:
            try:
                s.disconnect()
            except Exception:
                pass
        self._sims = []
        w._close_auto_log_files()
        w.auto_log_dir = self._orig_log_dir
        w.auto_log_enabled = self._orig_auto_log
        if w.auto_log_enabled:
            w._open_auto_log_files()
        w.current_data_source = None
        w.current_parser = None
        if not any(m.serial_port is not None for m in
                   (w.serial_port1, w.serial_port2, w.serial_port3)):
            w.update_timer.stop()

    def _finalize(self):
        """自检最终结算流程: 执行各项校验、写出自检报告文件、恢复环境并广播完成信号。"""
        if self._finalized:
            return
        self._finalized = True
        results = self._validate()
        self._write_report(results)
        self._restore()
        self._running = False
        all_ok = all(ok for _, ok, _ in results)
        self.w.log_info(f"一键自检结束: {'全部通过' if all_ok else '存在失败项'} "
                        f"报告目录: {self.report_dir}")
        self.finished.emit(all_ok, results)

    def _validate(self):
        """
        自检评测指标校验函数。

        逐一检验 11 项系统核心能力:
          1. 全程无未捕获异常
          2. 三口 GGA 定位与可用星刷新
          3. ENU 坐标数据量充足性
          4. ENU 标准差计算有效性
          5. 异常值剔除算法生效 (尖峰被成功拦截)
          6. SNR 卫星表正常且包含 GNGSV 多星
          7. 干扰口 SNR 恶化 (-15dB) 能被正确识别
          8. UBX NAV-STATUS 协议中 TTFF 首次定位时间提取
          9. 串口原始日志自动落盘
          10. 方向测试自动启停与 ENU 快照保存
          11. 报表完整链路 (图表渲染 PNG + Markdown 文档生成)

        返回:
            list[tuple]: [(item_name, ok_bool, detail_str), ...]
        """
        w = self.w
        epochs = self._minutes * 60
        results = []

        def item(name, ok, detail=""):
            results.append((name, bool(ok), str(detail)))

        # 1. 异常与超时核验
        timed_out = getattr(self, '_timeout', False)
        item("全程无未捕获异常(含看门狗)", self._exc_count == 0 and not timed_out,
             ("异常数=%d %s" % (self._exc_count, self._exc_details[0][:120]))
             if self._exc_details else ("超时中断" if timed_out else "无异常"))

        # 2. 三口 GGA 状态
        lat_ok = all(lbl.text() != '-' for lbl in
                     (w.p1_lat_label, w.p2_lat_label, w.p3_lat_label))
        nsat_txt = "/".join(lbl.text() for lbl in
                            (w.p1_nsats_label, w.p2_nsats_label, w.p3_nsats_label))
        item("三口GGA状态刷新", lat_ok and nsat_txt != "0/0/0", f"卫星数 {nsat_txt}")

        # 3. ENU 数据量
        expect = epochs - (DIR_START_EPOCH if self._direction else 0)
        min_pts = max(10, int(expect * 0.8))
        e2n, e3n = len(w.enu2_times), len(w.enu3_times)
        item("ENU数据量", e2n >= min_pts and e3n >= min_pts,
             f"ENU2={e2n} ENU3={e3n} 期望≥{min_pts}")

        # 4. ENU 标准差有效性
        s2e, s2n, s2u = w._std_enu2_east.std, w._std_enu2_north.std, w._std_enu2_up.std
        item("ENU标准差有效",
             s2e > 0 and s2n > 0 and s2u > 0 and w._std_enu3_east.std > 0,
             "σ2(E/N/U)=%.3f/%.3f/%.3f σ3E=%.3f" % (s2e, s2n, s2u, w._std_enu3_east.std))

        # 5. 尖峰异常值剔除
        item("异常值剔除生效(尖峰被拦截)",
             w._enu2_outlier_count > 0 and w._enu3_outlier_count > 0,
             f"剔除数 ENU2={w._enu2_outlier_count} ENU3={w._enu3_outlier_count}")

        # 6. SNR 卫星数据
        p1s, p2s, p3s = w.port1_satellites, w.port2_satellites, w.port3_satellites
        gn_cnt = sum(1 for d in (p1s, p2s, p3s) for k in d if k.startswith('GN'))
        item("SNR表三口有数据且含GNGSV卫星",
             len(p1s) > 0 and len(p2s) > 0 and len(p3s) > 0 and gn_cnt > 0,
             f"卫星数 {len(p1s)}/{len(p2s)}/{len(p3s)} GN键={gn_cnt}")

        # 7. 干扰口 SNR 恶化识别
        diff_vals = list(w._snr_diff_avg.values().values())
        degrade_ok = any(v <= -8 for v in diff_vals)
        item("干扰口SNR恶化可见",
             degrade_ok,
             ("最小Delta均值=%.1fdB" % min(diff_vals)) if diff_vals else "无共同卫星差值")

        # 8. UBX TTFF
        item("UBX TTFF解析",
             w.p1_ttff_s > 0 and w.p2_ttff_s > 0 and w.p3_ttff_s > 0,
             "%.1f/%.1f/%.1fs" % (w.p1_ttff_s, w.p2_ttff_s, w.p3_ttff_s))

        # 9. 日志持久化
        log_ok = True
        log_detail = []
        for pid in (1, 2, 3):
            files = glob.glob(os.path.join(self.log_dir, f'serial{pid}_*.log'))
            lines = 0
            for fp in files:
                with open(fp, 'r', encoding='utf-8', errors='ignore') as f:
                    lines += sum(1 for _ in f)
            if lines < epochs:
                log_ok = False
            log_detail.append("P%d:%d行" % (pid, lines))
        item("自检日志写入", log_ok, ", ".join(log_detail))

        # 10. 方向测试
        if self._direction:
            ds = w.direction_stats[0]
            dir_expect = int((epochs - DIR_START_EPOCH) / self._speed * 4 * 0.5)
            dir_min = max(5, min(dir_expect, epochs - DIR_START_EPOCH - 10))
            dir_ok = ds.total_epochs >= dir_min and ds.has_enu_data()
            item("方向测试自动启停",
                 dir_ok,
                 f"历元={ds.total_epochs}(≥{dir_min}) 成功={ds.successful_epochs} "
                 f"快照={'有' if ds.has_enu_data() else '无'}")

        # 11. 报告导出生成完整链路
        try:
            png_paths = w._render_dir_enu_charts_for_report(
                self.report_dir, self._ts, w.direction_stats, "ENU2 (干扰测试)")
            png_map = {d: n for d, n in png_paths} if png_paths else None
            app_lines, _ = w._build_report_lines(png_map, w.direction_stats, "串口2", "ENU2")
            app_md = os.path.join(self.report_dir, "app_report_com2.md")
            with open(app_md, 'w', encoding='utf-8') as f:
                f.write('\n'.join(app_lines))
            png_ok = all(os.path.exists(os.path.join(self.report_dir, n))
                         for n in (png_map or {}).values())
            md_ok = any('抗干扰天线测试报告' in l for l in app_lines)
            item("报告生成链路(PNG+MD)",
                 png_ok and md_ok and len(app_lines) > 30,
                 f"PNG={len(png_paths)}张 报告行={len(app_lines)}")
        except Exception as e:
            item("报告生成链路(PNG+MD)", False, f"异常: {e}")

        return results

    def _write_report(self, results):
        """格式化输出 Markdown 自检评测报告文件。"""
        npass = sum(1 for r in results if r[1])
        lines = ["# 一键自检报告", ""]
        lines.append(f"- 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"- 参数: {self._minutes}分钟 @ {self._speed}x, seed={self._seed}, "
                     f"方向测试={'开' if self._direction else '关'}")
        if self._cancelled:
            lines.append("- **注意: 自检被用户手动取消, 数据不完整, 失败项仅反映截断状态**")
        if getattr(self, '_timeout', False):
            lines.append("- **注意: 自检超时被看门狗中断, 数据不完整**")
        lines.append(f"- 结果: **{npass}/{len(results)} 项通过**")
        lines.append("")
        lines.append("| 检查项 | 结果 | 说明 |")
        lines.append("|--------|------|------|")
        for name, ok, detail in results:
            lines.append(f"| {name} | {'通过' if ok else '失败'} | {detail} |")
        lines.append("")
        path = os.path.join(self.report_dir, "selftest_report.md")
        with open(path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))

