import serial
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from src.protocol.ubx_parser import extract_ubx_frames_from_buffer

# Buffer cap to prevent unbounded growth on noise/garbled streams
READ_BUFFER_MAX = 65536  # 64 KB
# Reconnect: initial interval, backoff multiplier, max interval, max retries
RECONNECT_INITIAL_MS = 3000
RECONNECT_BACKOFF = 2.0
RECONNECT_MAX_MS = 30000
RECONNECT_MAX_RETRIES = 20


class SerialManager(QObject):
    """Manages a single serial port with timer-based polling, NMEA/UBX parsing,
    and automatic reconnection with exponential backoff.

    All public methods are designed to be called from a single thread
    (typically Qt's main event loop). Concurrent access from multiple
    threads requires external synchronization.
    """

    connection_status = pyqtSignal(bool, int)  # (connected, port_id)
    data_received = pyqtSignal(bytes, int)     # (data, port_id)
    ubx_received = pyqtSignal(bytes, int)      # (ubx_frame, port_id)
    error_occurred = pyqtSignal(str, int)      # (error_message, port_id)
    status_message = pyqtSignal(str, int)      # (status_text, port_id)
    disconnected = pyqtSignal()                # port unexpectedly disconnected

    def __init__(self, port_id=1):
        super().__init__()
        self.port_id = port_id
        self.serial_port = None
        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self.try_reconnect)
        self.reconnect_enabled = False
        self.last_port = None
        self.last_settings = None
        self._read_buffer = b''
        self._disconnecting = False
        self._reconnect_count = 0
        self._reconnect_interval_ms = RECONNECT_INITIAL_MS

        # Polling-based read via QTimer instead of QThread + blocking loop
        self._read_timer = QTimer()
        self._read_timer.timeout.connect(self._poll_serial)

    def get_available_ports(self):
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            return [port.device for port in ports]
        except Exception:
            return []

    def connect(self, port, baud_rate=9600, data_bits=8, stop_bits=1, parity='None'):
        try:
            parity_map = {
                'None': serial.PARITY_NONE,
                'Odd': serial.PARITY_ODD,
                'Even': serial.PARITY_EVEN,
                'Mark': serial.PARITY_MARK,
                'Space': serial.PARITY_SPACE,
            }
            stop_bits_map = {
                1: serial.STOPBITS_ONE,
                1.5: serial.STOPBITS_ONE_POINT_FIVE,
                2: serial.STOPBITS_TWO,
            }

            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                bytesize=data_bits,
                parity=parity_map[parity],
                stopbits=stop_bits_map[stop_bits],
                timeout=0,
            )

            self.last_port = port
            self.last_settings = (baud_rate, data_bits, stop_bits, parity)
            self._read_buffer = b''
            self._reconnect_count = 0
            self._reconnect_interval_ms = RECONNECT_INITIAL_MS

            self._read_timer.start(10)

            self.connection_status.emit(True, self.port_id)
            self.reconnect_enabled = True
            return True

        except Exception as e:
            self.error_occurred.emit(
                f"Port {self.port_id} connect failed: {str(e)}", self.port_id
            )
            return False

    def disconnect(self):
        """Explicit disconnect: stop timers, close port, prevent auto-reconnect."""
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
            # Exponential backoff, capped
            self._reconnect_interval_ms = min(
                int(self._reconnect_interval_ms * RECONNECT_BACKOFF),
                RECONNECT_MAX_MS
            )
            self.reconnect_timer.start(self._reconnect_interval_ms)

    def _poll_serial(self):
        # Guard against race: if user explicitly disconnected, don't re-enter
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

                # Cap buffer to prevent unbounded growth on noise
                if len(self._read_buffer) > READ_BUFFER_MAX:
                    overflow = len(self._read_buffer) - READ_BUFFER_MAX // 2
                    self._read_buffer = self._read_buffer[overflow:]
                    self.error_occurred.emit(
                        f"Port {self.port_id} read buffer overflow, trimmed {overflow} bytes",
                        self.port_id
                    )

                ubx_frames, self._read_buffer = extract_ubx_frames_from_buffer(
                    self._read_buffer
                )
                for frame in ubx_frames:
                    self.ubx_received.emit(frame, self.port_id)

                # NMEA-0183 sentences are \r\n terminated; handle \n-only as fallback
                while True:
                    if b'\r\n' in self._read_buffer:
                        line, self._read_buffer = self._read_buffer.split(b'\r\n', 1)
                    elif b'\n' in self._read_buffer:
                        line, self._read_buffer = self._read_buffer.split(b'\n', 1)
                    else:
                        break
                    if line.strip():
                        self.data_received.emit(
                            line.strip() + b'\n', self.port_id
                        )
        except serial.SerialException as e:
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
        """Handle unexpected serial port disconnection."""
        # Close the stale handle to prevent resource leak
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

        self.disconnected.emit()
        self.connection_status.emit(False, self.port_id)

        if self.reconnect_enabled:
            self.status_message.emit(
                f"Port {self.port_id} disconnected, auto-reconnecting...",
                self.port_id
            )
            self._reconnect_interval_ms = RECONNECT_INITIAL_MS
            self.reconnect_timer.start(self._reconnect_interval_ms)