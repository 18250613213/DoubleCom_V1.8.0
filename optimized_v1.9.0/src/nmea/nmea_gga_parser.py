"""
NMEA-0183 协议 GGA (Global Positioning System Fix Data) 语句专用解析模块

GGA 语句是 GNSS 接收机输出的最核心定位语句，提供了时间、三维位置坐标、定位解质量、
使用卫星数以及精度因子等核心指标。

典型 GGA 语句示例:
  $GPGGA,083559.00,4717.11437,N,00833.91522,E,1,08,1.01,499.6,M,48.0,M,,*5B
字段含义对照:
  [0]  $GPGGA: 语句标识 (Talker ID 为 GP, 语句为 GGA)
  [1]  083559.00: UTC 时间，格式 hhmmss.ss
  [2]  4717.11437: 纬度，格式 ddmm.mmmmm (度分)
  [3]  N: 纬度半球 (N=北纬, S=南纬)
  [4]  00833.91522: 经度，格式 dddmm.mmmmm (度分)
  [5]  E: 经度半球 (E=东经, W=西经)
  [6]  1: 定位质量指示 (0=未定位, 1=单点定位, 2=差分定位, 4=固定解 RTK, 5=浮点解 RTK 等)
  [7]  08: 参与定位的可用卫星总数 (00 ~ 99)
  [8]  1.01: HDOP 水平精度衰减因子 (越小精度越高)
  [9]  499.6: 海拔高程 (Antenna Altitude)，相对于平均海平面的高度
  [10] M: 高程单位 (Meters 米)
  [11] 48.0: 大地水准面差距 (Geoidal Separation)
  [12] M: 水准面差距单位 (Meters 米)
  [13] 差分数据龄期 (秒，单点定位时常为空)
  [14] 差分基准站 ID (单点定位时常为空)
  *5B: 十六进制异或校验和
"""

import re


class NMEAGGAParser:
    """
    NMEA GGA 语句解析器类。
    
    负责单条 GGA 语句的格式验证、异或校验和核对、度分到十进制度的坐标转换、
    以及关键定位指标（经纬度、高程、定位质量、卫星数、HDOP等）的提取与结构化输出。
    """

    def __init__(self):
        """初始化解析器内部状态与属性默认值。"""
        self.lat = 0.0           # 纬度 (十进制度，北纬正，南纬负)
        self.lon = 0.0           # 经度 (十进制度，东经正，西经负)
        self.alt = 0.0           # 海拔高程 (米)
        self.quality = 0         # 定位解状态标志 (0=无效, 1=SPS, 2=DGPS, 4=RTK Fix, 5=RTK Float)
        self.num_sat = 0         # 参与定位解算的卫星总数
        self.hdop = 0.0          # 水平精度因子 (HDOP)
        self.geoid_height = 0.0  # 大地水准面高 (米)
        self.timestamp = ""      # UTC 时间戳字符串 (hhmmss.ss)

    def parse(self, sentence):
        """
        解析单条原始 NMEA GGA 字符串。

        参数:
            sentence (str): 待解析的完整 NMEA 字符串 (例如 "$GPGGA,...*XX")。

        返回:
            bool: 解析成功返回 True；若格式不符、校验失败或字段异常则返回 False。
        """
        # 基础格式检查：必须以 '$' 开头
        if not sentence.startswith("$"):
            return False

        # 异或校验和核对
        if not self._validate_checksum(sentence):
            return False

        # 以逗号分割各字段
        parts = sentence.split(",")
        if len(parts) < 15:
            # 标准 GGA 至少包含 15 个逗号分隔部分（含末尾校验和）
            return False

        # 验证是否为 GGA 语句（支持各类 Talker，如 GPGGA, GNGGA, BDGGA, GLGGA 等）
        if not parts[0].endswith("GGA"):
            return False

        try:
            # 提取 UTC 时间
            self.timestamp = parts[1]

            # 提取并转换纬度 (度分 ddmm.mmmmm -> 十进制度)
            if parts[2]:
                self.lat = self._dmm_to_dd(float(parts[2]))
                if parts[3] == "S":
                    self.lat = -self.lat  # 南纬取负
            else:
                self.lat = 0.0

            # 提取并转换经度 (度分 dddmm.mmmmm -> 十进制度)
            if parts[4]:
                self.lon = self._dmm_to_dd(float(parts[4]))
                if parts[5] == "W":
                    self.lon = -self.lon  # 西经取负
            else:
                self.lon = 0.0

            # 定位质量与卫星数
            self.quality = int(parts[6]) if parts[6] else 0
            self.num_sat = int(parts[7]) if parts[7] else 0
            
            # HDOP 水平精度因子
            self.hdop = float(parts[8]) if parts[8] else 0.0
            
            # 天线海拔高度
            self.alt = float(parts[9]) if parts[9] else 0.0
            
            # 大地水准面异常值
            self.geoid_height = float(parts[11]) if parts[11] else 0.0

            return True
        except Exception:
            # 任何类型转换或下标越界异常均标记解析失败
            return False

    def _validate_checksum(self, sentence):
        """
        验证 NMEA 语句的异或（XOR）校验和。

        校验规则:
          对从 '$' 之后开始，到 '*' 之前的所有 ASCII 字符依次进行按位异或运算，
          所得 8 位整数转换为两位的十六进制大写字符串，并与 '*' 之后的校验字段对比。

        参数:
            sentence (str): 包含 '*XX' 的 NMEA 语句。

        返回:
            bool: 校验和匹配返回 True，不匹配或无校验字符返回 False。
        """
        if "*" not in sentence:
            return False
        try:
            data_part, checksum = sentence.split("*")
            calculated = 0
            for char in data_part[1:]:  # 跳过开头的 '$'
                calculated ^= ord(char)
            return calculated == int(checksum, 16)
        except Exception:
            return False

    def _dmm_to_dd(self, dmm):
        """
        将 NMEA 惯用的度分格式 (DDDMM.MMMMM / DDMM.MMMMM) 转换为十进制度 (Decimal Degrees)。

        转换公式:
          度数 degrees = int(dmm / 100)
          分数 minutes = dmm % 100
          十进制度 = degrees + minutes / 60.0

        例如:
          3112.3456 -> 31 度, 12.3456 分 -> 31 + 12.3456 / 60 = 31.20576 度

        参数:
            dmm (float): 度分数值。

        返回:
            float: 十进制度数值。
        """
        degrees = int(dmm // 100)
        minutes = dmm % 100
        return degrees + minutes / 60.0

    def get_data(self):
        """
        获取当前解析所得的所有定位数据字典。

        返回:
            dict: 包含经纬高、质量、星数、HDOP 等字段的结果字典。
        """
        return {
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "quality": self.quality,
            "num_sat": self.num_sat,
            "hdop": self.hdop,
            "geoid_height": self.geoid_height,
            "timestamp": self.timestamp,
        }

    def clear(self):
        """重置所有解析属性为初始值，便于下次复用或状态重置。"""
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.quality = 0
        self.num_sat = 0
        self.hdop = 0.0
        self.geoid_height = 0.0
        self.timestamp = ""