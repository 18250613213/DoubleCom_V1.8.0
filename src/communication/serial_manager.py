"""
串口通信管理模块 (SerialManager)

本模块封装了对物理串口或虚拟串口的高性能非阻塞读写控制与健壮的容错机制。

核心设计特性:
  1. 定时器轮询读取机制:
     采用 PyQt 的 QTimer 定时器 (10ms 周期) 替代多线程阻塞读模式，避免跨线程死锁与
     复杂的全局解释器锁 (GIL) 同步开销，保证与 Qt 主事件循环的天然协同。
  2. 混合协议流式解包:
     通过内置字节缓冲区 (_read_buffer)，无损解耦抽取 u-blox 二进制 UBX 帧，
     同时按行 (以 \\r\\n 或 \\n 为分隔符) 切分出标准 NMEA ASCII 文本语句，分别发射信号。
  3. 智能指数退避重连机制 (Exponential Backoff):
     当物理串口异常断开或拔出时，自动启动重连定时器：
     初始间隔 3s，每次重连失败按 2.0 倍增长，上限 30s，最大重试 20 次，避免高频轮询耗尽系统句柄。
  4. 防溢出滑动截断:
     设置 64KB 缓冲区上限，当遭遇持续乱码或波特率不匹配导致的无分隔符数据堆积时，
     自动修剪旧数据，防止内存暴涨。
"""

import serial
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from src.protocol.ubx_parser import extract_ubx_frames_from_buffer

# --- 缓冲区与重连控制常量 ---
READ_BUFFER_MAX = 65536         # 接收缓冲区最大容量上限 (64 KB)，防止乱码无换行时内存无限制膨胀
RECONNECT_INITIAL_MS = 3000     # 首次重连初始延迟时间 (3 秒)
RECONNECT_BACKOFF = 2.0         # 指数退避倍率系数
RECONNECT_MAX_MS = 30000        # 最大重连重试间隔上限 (30 秒)
RECONNECT_MAX_RETRIES = 20      # 最大允许连续重试次数


class SerialManager(QObject):
    """
    单串口管理对象类。
    
    提供端口枚举、连接配置、定时数据轮询、NMEA/UBX 双协议帧分发与断线智能重连功能。
    所有公共方法均设计为单线程安全，通常直接运行在 Qt 主事件循环中。
    """

    # --- Qt 业务事件信号定义 ---
    connection_status = pyqtSignal(bool, int)  # 连接状态变更信号: (是否已连接 connected, 端口标识 port_id)
    data_received = pyqtSignal(bytes, int)     # NMEA 文本行数据到达信号: (行数据字节 line_bytes, 端口标识 port_id)
    ubx_received = pyqtSignal(bytes, int)      # UBX 二进制帧到达信号: (完整 UBX 帧字节 ubx_frame, 端口标识 port_id)
    error_occurred = pyqtSignal(str, int)      # 错误提示信号: (错误信息 error_msg, 端口标识 port_id)
    status_message = pyqtSignal(str, int)      # 运行状态文本信号: (状态描述 status_str, 端口标识 port_id)
    disconnected = pyqtSignal()                # 串口非预期异常断开信号

    def __init__(self, port_id=1):
        """
        初始化串口管理器实例。

        参数:
            port_id (int): 端口逻辑标识符 (1=串口1基准, 2=串口2干扰, 3=串口3干扰2)。
        """
        super().__init__()
        self.port_id = port_id                  # 端口序号
        self.serial_port = None                 # 底层 PySerial Serial 对象引用
        
        # 重连定时器配置
        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self.try_reconnect)
        self.reconnect_enabled = False          # 是否允许自动重连（用户手动断开时设为 False）
        self.last_port = None                   # 上次成功连接的串口名称 (如 'COM3')
        self.last_settings = None               # 上次成功连接的配置元组 (波特率、数据位、停止位、校验位)
        
        # 接收缓冲区与状态标记
        self._read_buffer = b''                 # 字节累积缓冲区
        self._disconnecting = False             # 正在主动执行断开操作标记（防竞争）
        self._reconnect_count = 0               # 当前已尝试的连续重连次数
        self._reconnect_interval_ms = RECONNECT_INITIAL_MS # 当前重连等待周期

        # 核心轮询定时器: 10ms 间隔查询串口是否到达新字节
        self._read_timer = QTimer()
        self._read_timer.timeout.connect(self._poll_serial)

    def get_available_ports(self):
        """
        枚举当前系统检测到的所有物理串口与虚拟串口设备名称。

        返回:
            list[str]: 可用串口设备名列表 (例如 ['COM1', 'COM3', 'COM7'])。
        """
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            return [port.device for port in ports]
        except Exception:
            return []

    def connect(self, port, baud_rate=9600, data_bits=8, stop_bits=1, parity='None'):
        """
        打开并配置指定的物理串口。

        参数:
            port (str): 串口名 (例如 'COM3')。
            baud_rate (int): 波特率 (例如 9600, 115200, 460800 等)。
            data_bits (int): 数据位 (5, 6, 7, 8)。
            stop_bits (float 或 int): 停止位 (1, 1.5, 2)。
            parity (str): 校验位 ('None', 'Odd', 'Even', 'Mark', 'Space')。

        返回:
            bool: 连接成功返回 True，发生异常或占用失败返回 False。
        """
        try:
            # 映射校验位枚举
            parity_map = {
                'None': serial.PARITY_NONE,
                'Odd': serial.PARITY_ODD,
                'Even': serial.PARITY_EVEN,
                'Mark': serial.PARITY_MARK,
                'Space': serial.PARITY_SPACE,
            }
            # 映射停止位枚举
            stop_bits_map = {
                1: serial.STOPBITS_ONE,
                1.5: serial.STOPBITS_ONE_POINT_FIVE,
                2: serial.STOPBITS_TWO,
            }

            # 创建并打开 PySerial 实例，设置超时为 0 实现纯非阻塞轮询
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                bytesize=data_bits,
                parity=parity_map[parity],
                stopbits=stop_bits_map[stop_bits],
                timeout=0,
            )

            # 记录连接参数以便断线后恢复
            self.last_port = port
            self.last_settings = (baud_rate, data_bits, stop_bits, parity)
            self._read_buffer = b''
            self._reconnect_count = 0
            self._reconnect_interval_ms = RECONNECT_INITIAL_MS

            # 启动 10ms 轮询定时器
            self._read_timer.start(10)

            # 发送连接成功信号并开启重连功能
            self.connection_status.emit(True, self.port_id)
            self.reconnect_enabled = True
            return True

        except Exception as e:
            self.error_occurred.emit(
                f"Port {self.port_id} connect failed: {str(e)}", self.port_id
            )
            return False

    def disconnect(self):
        """
        主动显式断开串口连接。
        
        停止轮询和重连定时器，安全关闭底层句柄，重置连接状态。
        由于是用户主动操作，将禁用后续自动重连。
        """
        self._disconnecting = True
        self.reconnect_enabled = False
        self.reconnect_timer.stop()
        self._read_timer.stop()

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass

        self.serial_port = None
        self._disconnecting = False
        self.connection_status.emit(False, self.port_id)

    def try_reconnect(self):
        """
        断线自动重连回调函数。

        执行逻辑:
          1. 检查是否具备上一次有效配置和重连权限。
          2. 若超出最大重试次数 (20次)，停止重试并上报错误。
          3. 尝试重新打开串口，成功则重置退避间隔；
             失败则按指数退避算法（乘 2.0，上限 30 秒）重新启动下一次定时。
        """
        if not self.last_port or not self.last_settings or not self.reconnect_enabled:
            return

        self._reconnect_count += 1
        if self._reconnect_count > RECONNECT_MAX_RETRIES:
            self.reconnect_timer.stop()
            self.error_occurred.emit(
                f"Port {self.port_id} max retries ({RECONNECT_MAX_RETRIES}) reached",
                self.port_id
            )
            return

        self.status_message.emit(
            f"Port {self.port_id} reconnect attempt {self._reconnect_count}/{RECONNECT_MAX_RETRIES}",
            self.port_id
        )

        if self.connect(self.last_port, *self.last_settings):
            self.status_message.emit(
                f"Port {self.port_id} reconnected successfully",
                self.port_id
            )
            self._reconnect_count = 0
            self._reconnect_interval_ms = RECONNECT_INITIAL_MS
        else:
            # 指数退避累乘，不超过上限 30 秒
            self._reconnect_interval_ms = min(
                int(self._reconnect_interval_ms * RECONNECT_BACKOFF),
                RECONNECT_MAX_MS
            )
            self.reconnect_timer.start(self._reconnect_interval_ms)

    def _poll_serial(self):
        """
        定时器触发的核心轮询函数 (每 10ms 执行一次)。

        处理流程:
          1. 读取串口驱动缓冲（最多 1024 字节）并追加至内部 `_read_buffer`。
          2. 溢出检查：若积压超过 64KB，截断前半段以保护内存。
          3. 优先抽离所有完整的 UBX 二进制协议帧并通过 `ubx_received` 信号发射。
          4. 循环按换行符 (\\r\\n 或 \\n) 提取所有完整的 NMEA 文本行，通过 `data_received` 发射。
          5. 若检测到串口底层断开或 I/O 错误，触发 `_on_disconnected` 流程。
        """
        # 防止并发竞争：若用户正在执行断开，则直接返回
        if self._disconnecting:
            return

        if not self.serial_port or not self.serial_port.is_open:
            self._read_timer.stop()
            self._on_disconnected()
            return

        try:
            data = self.serial_port.read(1024)
            if data:
                self._read_buffer += data

                # 缓冲区溢出保护截断
                if len(self._read_buffer) > READ_BUFFER_MAX:
                    overflow = len(self._read_buffer) - READ_BUFFER_MAX // 2
                    self._read_buffer = self._read_buffer[overflow:]
                    self.error_occurred.emit(
                        f"Port {self.port_id} read buffer overflow, trimmed {overflow} bytes",
                        self.port_id
                    )

                # 步骤一: 抽离所有完整的 UBX 二进制帧
                ubx_frames, self._read_buffer = extract_ubx_frames_from_buffer(
                    self._read_buffer
                )
                for frame in ubx_frames:
                    self.ubx_received.emit(frame, self.port_id)

                # 步骤二: 提取标准 NMEA-0183 文本行 (以 \\r\\n 结尾，兼容仅有 \\n)
                while True:
                    if b'\r\n' in self._read_buffer:
                        line, self._read_buffer = self._read_buffer.split(b'\r\n', 1)
                    elif b'\n' in self._read_buffer:
                        line, self._read_buffer = self._read_buffer.split(b'\n', 1)
                    else:
                        break
                    if line.strip():
                        # 发送带换行符的完整语句
                        self.data_received.emit(
                            line.strip() + b'\n', self.port_id
                        )
        except serial.SerialException as e:
            # 捕获串口拔出或驱动层故障
            self.error_occurred.emit(
                f"Port {self.port_id} error: {str(e)}", self.port_id
            )
            self._read_timer.stop()
            self._on_disconnected()
        except Exception as e:
            self.error_occurred.emit(
                f"Port {self.port_id} read error: {str(e)}", self.port_id
            )

    def _on_disconnected(self):
        """
        处理非预期的串口异常断开事件（如物理线缆脱落）。
        
        安全关闭残存句柄，通知上层 UI 状态变更，并触发自动指数退避重连。
        """
        # 关闭陈旧句柄防止句柄泄漏
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

        self.disconnected.emit()
        self.connection_status.emit(False, self.port_id)

        # 启动自动重连流程
        if self.reconnect_enabled:
            self.status_message.emit(
                f"Port {self.port_id} disconnected, auto-reconnecting...",
                self.port_id
            )
            self._reconnect_interval_ms = RECONNECT_INITIAL_MS
            self.reconnect_timer.start(self._reconnect_interval_ms)