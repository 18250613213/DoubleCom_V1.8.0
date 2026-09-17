"""
NMEA-0183 协议与原始观测数据综合解析器

本模块负责对 GNSS 接收机通过串口输出的标准 NMEA 文本语句以及专用原始观测数据帧进行解析与状态维护。

支持的标准 NMEA 语句类型:
  - GGA (Global Positioning System Fix Data): 三维坐标、高程、定位状态、有效星数、HDOP。
  - RMC (Recommended Minimum Specific GPS Data): 经纬度、地速、航向、UTC 日期与时间、GPS 星期秒 (TOW)。
  - GSV (GNSS Satellites in View): 可视卫星总数、星号 (PRN)、仰角 (Elevation)、方位角 (Azimuth)、载噪比 (SNR/CN0) 及信号频点 ID。
  - GSA (GNSS DOP and Active Satellites): 当前定位模式 (2D/3D)、参与解算的卫星列表及 PDOP/HDOP/VDOP。

支持的卫星导航系统前缀 (Talker ID):
  - GP: GPS (美国全球定位系统)
  - GL: GLONASS (俄罗斯格洛纳斯系统)
  - GA: Galileo (欧洲伽利略系统)
  - BD / GB: BDS (中国北斗卫星导航系统)
  - GN: 多星座联合解算 (Multi-GNSS)

支持的扩展/专用原始观测语句:
  - #OBSVHA / #OBSVMA: 包含双频/多频伪距、载波相位、多普勒频移、C/N0 及跟踪时长的原始测量报文。

线程安全性说明:
  本类内部使用无锁数据结构，设计为单线程专用（通常在 Qt 主事件循环或独立工作线程中运行）。
  若跨线程调用，必须在外部进行加锁或信号槽串行化调度。
"""

import re
from collections import deque


class NMEAParser:
    """
    单线程 NMEA 语句解析与导航状态管理类。
    
    维护最近历元的定位解、可视星天顶分布与信噪比快照、DOP 精度因子历史及异常错误统计。
    """

    def __init__(self):
        """初始化解析器并分配定长双端队列（deque）以防止内存无限增长。"""
        self.clear()

    def clear(self):
        """清空所有解析历史记录、卫星信噪比缓存及错误统计。"""
        self.total_sentences = 0            # 接收到的总语句行数
        self.unknown_sentences = 0          # 未知/未识别的语句行数
        self.gpgga_data = deque(maxlen=1)   # 最新一条 GGA 数据快照
        self.gprmc_data = deque(maxlen=1)   # 最新一条 RMC 数据快照
        self.gpgsv_data = deque(maxlen=50)  # 最近的 GSV 数据帧队列（支撑多星系多报文拼接）
        self.gpgsa_data = deque(maxlen=1)   # 最新一条 GSA 精度因子与可用星数据
        self.obsvha_data = deque(maxlen=1)  # 最新一条 OBSVHA 观测报文
        self.obsvma_data = deque(maxlen=1)  # 最新一条 OBSVMA 观测报文
        self.satellite_info = {}            # 当前所有可视卫星信息字典 {prn: {elevation, azimuth, snr, signal_id, talker_id}}
        self.position_fixes = deque(maxlen=1) # 最新有效定位解（转为十进制度）
        self.dop_values = deque(maxlen=1)   # 最新精度衰减因子 (PDOP, HDOP, VDOP)
        self.errors = deque(maxlen=100)     # 最近 100 条错误信息日志
        self.last_gps_tow = 0.0             # 最新推导出的 GPS 周内秒 (Time of Week, 秒)
        self.last_gps_time_valid = False    # GPS 时间是否已成功锁定有效

    def decode_line(self, data):
        """
        安全地将输入数据（bytes 或 str）解码并剥除空白换行符。

        参数:
            data (bytes 或 str): 原始接收数据行。

        返回:
            str 或 None: 成功解码后的字符串，若出现严重解码异常则返回 None。
        """
        try:
            if isinstance(data, bytes):
                return data.decode("utf-8", errors="ignore").strip()
            return str(data).strip()
        except Exception:
            return None

    @staticmethod
    def _compute_xor_checksum(data_part):
        """
        计算 NMEA 标准异或校验和。

        计算范围为从字符串首字符（紧跟在 '$' 之后）到结尾的所有 ASCII 字符。

        参数:
            data_part (str): 去除开头 '$' 和末尾 '*XX' 的有效语句体。

        返回:
            int: 8 位异或校验和整数。
        """
        calculated = 0
        for ch in data_part[1:]:  # 跳过开头的 '$'
            calculated ^= ord(ch)
        return calculated

    def parse(self, sentence):
        """
        解析单行 NMEA 字符串或扩展观测数据帧。

        执行流程:
          1. 校验语句非空。
          2. 若包含 '*XX' 校验和字段，则先核对异或校验，校验失败则记入错误日志并丢弃。
          3. 依据语句头部前缀将语句分发给对应的具体解析方法 (parse_gpgga, parse_gprmc, parse_gpgsv 等)。

        参数:
            sentence (str): 待解析的完整单行 NMEA 文本。
        """
        if not sentence:
            return

        self.total_sentences += 1

        try:
            # --- 分离并核对 NMEA 异或校验和 ---
            if "*" in sentence:
                data_part, checksum_hex = sentence.split("*", 1)
                calculated = self._compute_xor_checksum(data_part)
                try:
                    expected = int(checksum_hex[:2], 16)
                except ValueError:
                    expected = None
                if expected is None or calculated != expected:
                    self.errors.append(
                        f"Checksum mismatch: {sentence[:50]}..."
                    )
                    return
            else:
                data_part = sentence

            # --- 依据语句类型进行分发解析 (剥离末尾 *XX 后的干净文本) ---
            if (data_part.startswith("$GPGGA") or data_part.startswith("$BDGGA") or
                    data_part.startswith("$GLGGA") or data_part.startswith("$GNGGA") or
                    data_part.startswith("$GAGGA") or data_part.startswith("$GBGGA")):
                self.parse_gpgga(data_part)
            elif (data_part.startswith("$GPRMC") or data_part.startswith("$BDRMC") or
                    data_part.startswith("$GLRMC") or data_part.startswith("$GNRMC") or
                    data_part.startswith("$GARMC") or data_part.startswith("$GBRMC")):
                self.parse_gprmc(data_part)
            elif (data_part.startswith("$GPGSV") or data_part.startswith("$BDGSV") or
                    data_part.startswith("$GLGSV") or data_part.startswith("$GAGSV") or
                    data_part.startswith("$GBGSV") or data_part.startswith("$GNGSV")):
                self.parse_gpgsv(data_part)
            elif (data_part.startswith("$GPGSA") or data_part.startswith("$BDGSA") or
                    data_part.startswith("$GLGSA") or data_part.startswith("$GNGSA") or
                    data_part.startswith("$GAGSA") or data_part.startswith("$GBGSA")):
                self.parse_gpgsa(data_part)
            elif data_part.startswith("#OBSVHA"):
                self.parse_obsvha(data_part)
            elif data_part.startswith("#OBSVMA"):
                self.parse_obsvma(data_part)
            else:
                self.unknown_sentences += 1

        except Exception as e:
            self.errors.append(f"Parse error: {sentence} - {str(e)}")

    # 各 NMEA 语句类型的预期最少字段数量定义
    _EXPECTED_FIELDS = {"GGA": 15, "RMC": 13, "GSA": 18}

    @staticmethod
    def _normalize_fields(parts, expected_count):
        """
        原地补齐字段列表长度，避免因结尾空字段被 split 截断导致索引越界。

        参数:
            parts (list): 经 split(',') 分割的字段列表。
            expected_count (int): 预期的标准字段数量。

        返回:
            list: 补齐后的字段列表。
        """
        short = expected_count - len(parts)
        if short > 0:
            parts.extend([""] * short)
        return parts

    def parse_gpgga(self, sentence):
        """
        解析 GGA 定位信息语句。

        提取 UTC 时间、经纬度及半球标识、定位质量、卫星数、HDOP、海拔高程、水准面差距等。
        若定位质量大于 0 (有效定位)，则将坐标转换为十进制度并存入 position_fixes。

        参数:
            sentence (str): 干净的 GGA 字符串。
        """
        parts = sentence.split(",")
        if len(parts) < 2:
            return
        parts = self._normalize_fields(parts, 15)

        data = {
            "timestamp": parts[1],
            "latitude": parts[2],
            "lat_dir": parts[3],
            "longitude": parts[4],
            "lon_dir": parts[5],
            "fix_quality": int(parts[6]) if parts[6] else 0,
            "satellites_used": int(parts[7]) if parts[7] else 0,
            "hdop": float(parts[8]) if parts[8] else 0.0,
            "altitude": float(parts[9]) if parts[9] else 0.0,
            "altitude_units": parts[10],
            "geoidal_separation": float(parts[11]) if parts[11] else 0.0,
            "age_of_diff": parts[13],
            "diff_station": parts[14],
        }

        self.gpgga_data.append(data)

        # 有效定位时，执行坐标换算并记录最新有效位置
        if data["fix_quality"] > 0:
            lat = self.convert_coordinate(data["latitude"], data["lat_dir"])
            lon = self.convert_coordinate(data["longitude"], data["lon_dir"])
            self.position_fixes.append({
                "timestamp": data["timestamp"],
                "latitude": lat,
                "longitude": lon,
                "fix_quality": data["fix_quality"],
                "satellites": data["satellites_used"],
                "hdop": data["hdop"],
            })

    def parse_gprmc(self, sentence):
        """
        解析 RMC 推荐定位信息语句。

        提取定位状态 (A=有效/V=警告)、地速 (节)、航向角、UTC 日期与时间，
        并计算当日的秒数推导周内秒 (TOW)。

        参数:
            sentence (str): 干净的 RMC 字符串。
        """
        parts = sentence.split(",")
        if len(parts) < 2:
            return
        parts = self._normalize_fields(parts, 13)

        data = {
            "timestamp": parts[1],
            "status": parts[2],
            "latitude": parts[3],
            "lat_dir": parts[4],
            "longitude": parts[5],
            "lon_dir": parts[6],
            "speed": float(parts[7]) if parts[7] else 0.0,
            "course": float(parts[8]) if parts[8] else 0.0,
            "date": parts[9],
            "magnetic_variation": float(parts[10]) if parts[10] else 0.0,
            "variation_dir": parts[11],
            "mode": parts[12] if parts[12] else "",
        }

        self.gprmc_data.append(data)

        # 从 UTC 日期与时间戳推导 GPS 周内秒 (Time of Week)
        if data["date"] and data["timestamp"]:
            try:
                day = int(data["date"][:2])
                month = int(data["date"][2:4])
                year = 2000 + int(data["date"][4:6])

                hours = int(data["timestamp"][:2])
                minutes = int(data["timestamp"][2:4])
                seconds = float(data["timestamp"][4:])

                seconds_of_day = hours * 3600 + minutes * 60 + seconds

                # 计算本周内的秒数 (以 604800 秒为一周模长)
                self.last_gps_tow = seconds_of_day % 604800
                self.last_gps_time_valid = True
            except Exception:
                pass

    def parse_gpgsv(self, sentence):
        """
        解析 GSV 可见卫星状态语句。

        GSV 语句通常以多包序列形式发送（如 1/3, 2/3, 3/3 包）。每包含最多 4 颗卫星信息：
        - PRN 卫星编号
        - Elevation 卫星仰角 (0~90度)
        - Azimuth 卫星方位角 (0~359度)
        - SNR/CN0 载噪比 (0~99 dB-Hz)
        - Signal ID (可选，NMEA 4.10 标准字段，标识频点如 L1CA, L2C, L5 等)

        周期重置逻辑:
          当收到该 Talker ID 的第 1 包时 (message_number == 1)，清除该星座先前的卫星缓存，
          以确保旧历元已失效的卫星不会残留。

        参数:
            sentence (str): GSV 格式字符串。
        """
        # 防御性去除末尾校验和（若直接调用该函数）
        if "*" in sentence:
            sentence, _ = sentence.split("*", 1)

        parts = sentence.split(",")
        if len(parts) < 8:
            return

        talker_id = parts[0][1:3]  # 提取星座标识，如 "GP", "BD", "GL", "GA", "GB", "GN"
        total_messages = int(parts[1]) if parts[1] else 0
        message_number = int(parts[2]) if parts[2] else 0
        total_satellites = int(parts[3]) if parts[3] else 0

        # --- GSV 序列重置: 收到第 1 包时清空该星系旧缓存 ---
        if message_number == 1:
            self.satellite_info = {
                prn: info
                for prn, info in self.satellite_info.items()
                if info.get("talker_id") != talker_id
            }

        # 检查是否包含 NMEA 4.10+ 标准末尾的 Signal ID 字段
        extra_fields = len(parts) - 4
        signal_id = 0
        if extra_fields % 4 == 1:
            signal_id = int(parts[-1]) if parts[-1] else 0
            sat_count = extra_fields // 4
        else:
            sat_count = extra_fields // 4

        satellites = []

        # 循环提取包内的每颗卫星（每颗卫星占用 4 个连续字段）
        for i in range(sat_count):
            idx = 4 + i * 4
            if idx + 3 >= len(parts):
                break

            prn = int(parts[idx]) if parts[idx] else 0
            elevation = int(parts[idx + 1]) if parts[idx + 1] else 0
            azimuth = int(parts[idx + 2]) if parts[idx + 2] else 0
            raw_snr = int(parts[idx + 3]) if parts[idx + 3] else 0
            snr = raw_snr if 0 <= raw_snr <= 99 else 0

            satellites.append({
                "prn": prn,
                "elevation": elevation,
                "azimuth": azimuth,
                "snr": snr,
                "signal_id": signal_id,
            })

            # 更新全局卫星字典
            self.satellite_info[prn] = {
                "elevation": elevation,
                "azimuth": azimuth,
                "snr": snr,
                "signal_id": signal_id,
                "talker_id": talker_id,
            }

        self.gpgsv_data.append({
            "talker_id": talker_id,
            "total_messages": total_messages,
            "message_number": message_number,
            "total_satellites": total_satellites,
            "signal_id": signal_id,
            "satellites": satellites,
        })

    def parse_gpgsa(self, sentence):
        """
        解析 GSA 精度因子与当前使用卫星列表语句。

        字段说明:
          [1]: 模式 (M=手动, A=自动)
          [2]: 定位模式 (1=未定位, 2=2D定位, 3=3D定位)
          [3..14]: 参与当前定位解算的 12 颗主通道卫星 PRN
          [15]: PDOP 位置综合精度因子
          [16]: HDOP 水平精度因子
          [17]: VDOP 垂直高程精度因子

        参数:
            sentence (str): GSA 字符串。
        """
        parts = sentence.split(",")
        if len(parts) < 2:
            return
        parts = self._normalize_fields(parts, 18)

        data = {
            "mode": parts[1],
            "fix_type": int(parts[2]) if parts[2] else 0,
            "satellites": [int(p) for p in parts[3:15] if p],
            "pdop": float(parts[15]) if parts[15] else 0.0,
            "hdop": float(parts[16]) if parts[16] else 0.0,
            "vdop": float(parts[17]) if parts[17] else 0.0,
        }

        self.gpgsa_data.append(data)
        self.dop_values.append({
            "pdop": data["pdop"],
            "hdop": data["hdop"],
            "vdop": data["vdop"],
        })

    def parse_obsvha(self, sentence):
        """
        解析厂商扩展/自定义原始观测语句 #OBSVHA。

        提取接收机周内毫秒 (rcv_tow)、周数 (rcv_wkn)、卫星总数及
        每颗星的伪距、载波相位、多普勒和 C/N0 载噪比等底层观测量。

        参数:
            sentence (str): #OBSVHA 格式报文。
        """
        try:
            if "*" in sentence:
                main_part, checksum = sentence.split("*", 1)
            else:
                main_part = sentence
                checksum = None

            parts = main_part.split(",")
            if len(parts) < 10:
                return

            data = {
                "version": int(parts[1]) if parts[1] else 0,
                "gnss": parts[2],
                "mode": parts[3],
                "rcv_tow": int(parts[4]) if parts[4] else 0,
                "rcv_wkn": int(parts[5]) if parts[5] else 0,
                "num_sat": int(parts[8]) if parts[8] else 0,
                "satellites": [],
                "checksum": checksum,
            }

            prn_block = parts[9] if len(parts) > 9 else ""
            prn_list = [int(p) for p in prn_block.split(";") if p.strip().isdigit()]

            def safe_int(s, default=0):
                try:
                    return int(float(s))
                except Exception:
                    return default

            def safe_float(s, default=0.0):
                try:
                    return float(s)
                except Exception:
                    return default

            obs_data = parts[10:] if len(parts) > 10 else []

            # 每一个卫星观测块占用 11 个逗号字段
            num_satellites = len(obs_data) // 11
            for sat_idx in range(num_satellites):
                start = sat_idx * 11
                fields = obs_data[start:start + 11]

                sat = {
                    "prn": safe_int(fields[1]),
                    "gnss_id": safe_int(fields[0]),
                    "sv_id": safe_int(fields[1]),
                    "pseudorange": safe_float(fields[2]),
                    "carrier_phase": safe_float(fields[3]),
                    "doppler": safe_float(fields[4]),
                    "cn0": safe_int(fields[5]),
                    "locktime": safe_int(fields[6]),
                    "flags": safe_int(fields[7]),
                    "reserved": safe_int(fields[8]),
                    "pseudorange_rate": safe_float(fields[9]),
                    "timestamp": data["rcv_tow"] / 1000.0,
                }
                data["satellites"].append(sat)

            self.obsvha_data.append(data)

        except Exception as e:
            self.errors.append(
                f"Parse OBSVHA failed: {sentence[:50]}... - {str(e)}"
            )

    def parse_obsvma(self, sentence):
        """
        解析厂商扩展/自定义多频观测语句 #OBSVMA。

        与 OBSVHA 类似，但支持多频点（包含频点索引 freq_idx），伪距按厘米单位缩放 (除以 100 换算为米)。

        参数:
            sentence (str): #OBSVMA 格式报文。
        """
        try:
            if "*" in sentence:
                main_part, checksum = sentence.split("*", 1)
            else:
                main_part = sentence
                checksum = None

            parts = main_part.split(",")
            if len(parts) < 10:
                return

            data = {
                "version": int(parts[1]) if parts[1] else 0,
                "gnss": parts[2],
                "mode": parts[3],
                "rcv_tow": int(parts[4]) if parts[4] else 0,
                "rcv_wkn": int(parts[5]) if parts[5] else 0,
                "num_sat": int(parts[8]) if parts[8] else 0,
                "satellites": [],
                "checksum": checksum,
            }

            prn_list_str = parts[9] if len(parts) > 9 else ""
            prn_list = [
                int(p) for p in prn_list_str.split(";") if p.strip().isdigit()
            ]

            obs_data = parts[10:] if len(parts) > 10 else []

            def safe_int(s, default=0):
                try:
                    return int(float(s))
                except Exception:
                    return default

            def safe_float(s, default=0.0):
                try:
                    return float(s)
                except Exception:
                    return default

            num_satellites = len(obs_data) // 11

            for sat_idx in range(num_satellites):
                start = sat_idx * 11
                fields = obs_data[start:start + 11]

                prn = prn_list[sat_idx] if sat_idx < len(prn_list) else 0

                sat = {
                    "prn": prn,
                    "gnss_id": safe_int(fields[0]),
                    "sv_id": safe_int(fields[1]),
                    "pseudorange": safe_float(fields[2]) / 100.0,  # 厘米转为米
                    "carrier_phase": safe_float(fields[3]),
                    "doppler": safe_float(fields[4]),
                    "cn0": safe_int(fields[5]),
                    "locktime": safe_int(fields[6]),
                    "flags": safe_int(fields[7], 16),
                    "reserved": safe_int(fields[8]),
                    "freq_idx": safe_int(fields[9]),
                    "timestamp": data["rcv_tow"] / 1000.0,
                    "X": None,
                    "Y": None,
                    "Z": None,
                    "has_sat_pos": False,
                }

                data["satellites"].append(sat)

            self.obsvma_data.append(data)

        except Exception as e:
            self.errors.append(
                f"Parse OBSVMA failed: {sentence[:50]}... - {str(e)}"
            )

    def convert_coordinate(self, coord_str, direction):
        """
        将 NMEA 格式坐标转换为十进制度。

        转换逻辑:
          degrees = int(val // 100)
          minutes = val % 100
          deg = degrees + minutes / 60.0
          若半球为南纬 'S' 或西经 'W'，取相反数。

        参数:
            coord_str (str): 度分格式字符串 (如 "3112.3456")。
            direction (str): 方向字符 ('N', 'S', 'E', 'W')。

        返回:
            float: 保留 6 位小数的十进制度浮点数。
        """
        if not coord_str:
            return 0.0

        try:
            dmm = float(coord_str)
            degrees = int(dmm // 100)
            minutes = dmm % 100
            value = degrees + minutes / 60.0

            if direction in ["S", "W"]:
                value = -value

            return round(value, 6)
        except Exception:
            return 0.0

    def get_statistics(self):
        """
        汇总当前解析器统计指标，包括各语句计数、平均可用星数、DOP 极值与均值等。

        返回:
            dict: 统计指标汇总字典。
        """
        obsv_sat_info = {}
        for obsv in list(self.obsvha_data) + list(self.obsvma_data):
            for sat in obsv["satellites"]:
                prn = sat["prn"]
                if prn not in obsv_sat_info:
                    obsv_sat_info[prn] = []
                obsv_sat_info[prn].append({
                    "cn0": sat["cn0"] / 10.0,
                    "doppler": sat["doppler"],
                    "locktime": sat["locktime"],
                })

        stats = {
            "total_sentences": self.total_sentences,
            "unknown_sentences": self.unknown_sentences,
            "gpgga_count": len(self.gpgga_data),
            "gprmc_count": len(self.gprmc_data),
            "gpgsv_count": len(self.gpgsv_data),
            "gpgsa_count": len(self.gpgsa_data),
            "obsvha_count": len(self.obsvha_data),
            "obsvma_count": len(self.obsvma_data),
            "valid_fixes": len(self.position_fixes),
            "total_satellites": len(self.satellite_info),
            "obsv_satellites": len(obsv_sat_info),
            "obsv_sat_info": obsv_sat_info,
            "errors": len(self.errors),
        }

        if self.position_fixes:
            stats["avg_satellites"] = (
                sum(f["satellites"] for f in self.position_fixes)
                / len(self.position_fixes)
            )
            stats["avg_hdop"] = (
                sum(f["hdop"] for f in self.position_fixes)
                / len(self.position_fixes)
            )
        else:
            stats["avg_satellites"] = 0
            stats["avg_hdop"] = 0

        if self.dop_values:
            stats["min_pdop"] = min(d["pdop"] for d in self.dop_values)
            stats["max_pdop"] = max(d["pdop"] for d in self.dop_values)
            stats["avg_pdop"] = (
                sum(d["pdop"] for d in self.dop_values) / len(self.dop_values)
            )
            stats["min_hdop"] = min(d["hdop"] for d in self.dop_values)
            stats["max_hdop"] = max(d["hdop"] for d in self.dop_values)
            stats["min_vdop"] = min(d["vdop"] for d in self.dop_values)
            stats["max_vdop"] = max(d["vdop"] for d in self.dop_values)
        else:
            stats.update({
                "min_pdop": 0,
                "max_pdop": 0,
                "avg_pdop": 0,
                "min_hdop": 0,
                "max_hdop": 0,
                "min_vdop": 0,
                "max_vdop": 0,
            })

        return stats