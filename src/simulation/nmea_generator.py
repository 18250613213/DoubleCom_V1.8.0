"""
导航仿真数据生成器 (NMEA & UBX Simulator)

纯 Python 实现，完全不依赖 PyQt 库，支持独立运行和跨平台自动化测试。
采用确定性伪随机数种子 (Seed)，相同种子下生成的时间线数据保证 100% 严格复现。

仿真场景与数据特征设计:
  1. 串口 1 (基准参考机):
     - GGA 采用 $GNGGA，无失锁间隙、无异常尖峰跳变，仅含高精度厘米级微小抖动 (σ_E/N = 0.02m, σ_U = 0.04m)。
     - 用于作为基准点计算基准均值，并与干扰机进行 ENU 差分比对。
  2. 串口 2 / 串口 3 (干扰测试机 1 / 2):
     - GGA 分别采用 $GPGGA 与 $BDGGA。
     - 注入米级高斯噪声抖动 (σ_E/N = 1.0m, σ_U = 1.5m)。
     - 注入周期性单历元异常尖峰跳变 (水平方向 +15m，天向 +18m，超过 8σ 且远超 5m 阈值)，
       用于检验系统核心的滑动窗口异常值动态剔除算法 (Outlier Rejection)。
     - 模拟中间阶段连续 5 个历元的失锁脱锁间隙 (Quality = 0)。
  3. 多星系统 GSV 报文:
     - 混合模拟 GPS(GP)、GLONASS(GL)、BDS(BD)、Multi-GNSS(GN) 四组星座。
     - 在串口 2 和串口 3 中针对指定卫星的人为信噪比衰减 (恶化 -15dB)，以检验抗干扰天线对不同频段的抑制特征。
  4. RMC 导航报文:
     - 输出 $GNRMC，包含 UTC 日期与时间，检验系统对多星 Talker 放行与 GPS 周内秒 (TOW) 提取功能。
  5. UBX 二进制报文:
     - 定期注入合法的 UBX-NAV-STATUS 帧 (符合 Fletcher-8 校验)，首次定位时间 (TTFF) 设为 35.0 秒。
"""

import math
import random
import struct

# --- 仿真基准地理坐标常量 (设定在测试场初始基准点) ---
BASE_LAT = 28.185273             # 仿真基准纬度 (十进制度)
BASE_LON = 112.938604            # 仿真基准经度 (十进制度)
BASE_ALT = 56.3                  # 仿真基准椭球/海拔高度 (米)
START_SOW = 2 * 3600             # 仿真起始时间 (当天秒数: UTC 02:00:00，对应 7200 秒)
RMC_DATE = "160826"              # RMC 日期格式 (日月年: 2026年08月16日)

# --- 预设的各导航星座卫星 PRN 号清单 ---
_GSV_SATS = {
    'GP': [2, 5, 6, 12, 17, 19, 25, 29, 31], # GPS 卫星列表
    'GL': [1, 2, 3, 8, 9, 11],                # GLONASS 卫星列表
    'BD': [1, 2, 3, 6, 9, 16],                # 北斗 BDS 卫星列表
    'GN': [7, 9, 13, 14, 21, 26],             # 混合/其他卫星列表
}

# --- 模拟干扰恶化的特定卫星列表 (降水、多径或压制干扰导致 SNR 下降 15dB) ---
_DEGRADED_SATS = {
    2: {('GP', 2), ('GP', 12), ('GP', 25), ('GL', 8)},  # 串口2中受干扰压制的卫星
    3: {('GP', 5), ('GP', 17), ('BD', 6), ('GN', 21)},  # 串口3中受干扰压制的卫星
}

# 尖峰脉冲注入轴与幅值
_SPIKE_AXES = ('east', 'north', 'up')
_SPIKE_AMPL_H = 15.0             # 水平尖峰扰动幅度 (米)
_SPIKE_AMPL_U = 18.0             # 天向尖峰扰动幅度 (米)


def _xor_checksum(body):
    """
    计算 NMEA 字符串异或校验和。

    参数:
        body (str): 包含 '$' 开头且不含 '*' 的 NMEA 语句内容。

    返回:
        int: 8 位校验和整数。
    """
    c = 0
    for ch in body[1:]:
        c ^= ord(ch)
    return c


def _nmea(body):
    """
    格式化并封装单条带校验和与换行符的 NMEA 字节流。

    参数:
        body (str): 不包含校验和的语句体 (如 "$GPGGA,...")。

    返回:
        bytes: 加上 "*XX\\n" 校验尾的 ASCII 字节串。
    """
    return (body + "*%02X" % _xor_checksum(body)).encode('ascii') + b'\n'


def _utc_str(sec):
    """
    将当天秒数转换为 NMEA 惯用的 UTC 时间字符串 (hhmmss.ss)。

    参数:
        sec (float 或 int): 当天秒数 (0 ~ 86399)。

    返回:
        str: 格式如 "020005.00" 的时间字符串。
    """
    sec = int(sec) % 86400
    return "%02d%02d%02d.00" % (sec // 3600, (sec % 3600) // 60, sec % 60)


def _dmm(value, is_lat):
    """
    将十进制度转换为 NMEA 度分格式字符串 (ddmm.mmmmm 或 dddmm.mmmmm)。

    参数:
        value (float): 十进制度数值。
        is_lat (bool): True 表示纬度 (2位度数)，False 表示经度 (3位度数)。

    返回:
        str: 格式化后的度分字符串。
    """
    d = int(value)
    m = abs(value - d) * 60
    return "%0*d%08.5f" % (2 if is_lat else 3, abs(d), m)


def _gga(talker, utc, lat, lon, alt, quality, nsat):
    """生成一条包含定位解的完整 GGA 语句。"""
    body = ("$%sGGA,%s,%s,N,%s,E,%d,%02d,0.9,%.2f,M,-8.7,M,," %
            (talker, utc, _dmm(lat, True), _dmm(lon, False), quality, nsat, alt))
    return _nmea(body)


def _gga_nofix(talker, utc, nsat):
    """生成一条质量为 0 (失锁/未定位) 的 GGA 语句。"""
    body = "$%sGGA,%s,,N,,E,0,%02d,,,M,,M,," % (talker, utc, nsat)
    return _nmea(body)


def _rmc(utc):
    """生成一条标准 GNRMC 推荐定位语句。"""
    body = ("$GNRMC,%s,A,%s,N,%s,E,0.4,54.7,%s,,,A" %
            (utc, _dmm(BASE_LAT + 0.0001, True), _dmm(BASE_LON + 0.0001, False), RMC_DATE))
    return _nmea(body)


def _gsv_sentences(talker, prns, snr_map, rng):
    """
    生成一组符合 NMEA 规范的 GSV 卫星可见性语句序列。
    
    每包最多封装 3 颗卫星，包含星号、仰角、方位角和载噪比。
    """
    total = (len(prns) + 2) // 3
    for mi in range(total):
        group = prns[mi * 3:(mi + 1) * 3]
        fields = []
        for prn in group:
            snr = max(5, min(55, snr_map[(talker, prn)] + rng.randint(-1, 1)))
            ele = 10 + (prn * 7) % 75
            azi = (prn * 47) % 360
            fields.append("%02d,%d,%d,%d" % (prn, ele, azi, snr))
        body = "$%sGSV,%d,%d,%d,%s,1" % (talker, total, mi + 1, len(prns), ",".join(fields))
        yield _nmea(body)


def _ubx_nav_status(tow_ms, ttff_ms):
    """
    打包生成符合 u-blox 规范的 UBX-NAV-STATUS 二进制帧。

    参数:
        tow_ms (int): GPS 周内毫秒。
        ttff_ms (int): 首次定位时间 (毫秒)。

    返回:
        bytes: 包含同步头、类别、长度、负载及 Fletcher 校验的完整帧。
    """
    payload = struct.pack('<IBBBBII', tow_ms, 2, 0x01, 0x00, 0x01, ttff_ms, 456789)
    body = b'\xb5\x62\x01\x03' + struct.pack('<H', len(payload)) + payload
    ck_a = ck_b = 0
    for byte in body[2:]:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return body + bytes([ck_a, ck_b])


def generate_port_timeline(port_id, minutes=60, seed=42):
    """
    生成指定端口的模拟数据时间线列表。

    参数:
        port_id (int): 端口号 (1=基准, 2=干扰1, 3=干扰2)。
        minutes (int): 仿真时长 (分钟)。
        seed (int): 随机种子，确保同种子下生成结果完全一致。

    返回:
        list[tuple]: 按时间升序排序的事件序列 [(sim_time_sec, kind, payload), ...]
                     其中 kind 为 'nmea' 或 'ubx'。
    """
    rng = random.Random(seed * 1000 + port_id)
    epochs = int(minutes * 60)
    timeline = []

    is_ref = (port_id == 1)
    # 基准机厘米级精度，测试机米级波动
    sigma_e = sigma_n = 0.02 if is_ref else 1.0
    sigma_u = 0.04 if is_ref else 1.5
    gga_talker = {1: 'GN', 2: 'GP', 3: 'BD'}[port_id]
    nsat = {1: 12, 2: 11, 3: 10}[port_id]

    # 尖峰异常值注入计划 (仅注入干扰端口)
    spike_start = 180
    spike_step = max(45, (epochs - 200) // 10)
    spikes = set() if is_ref else set(range(spike_start, epochs - 30, spike_step))

    # 失锁间隙注入计划 (仅注入干扰端口，持续 5 个历元)
    gap0 = epochs // 2 if port_id == 2 else epochs // 2 + 30
    gaps = set() if is_ref else set(range(gap0, gap0 + 5))

    # 初始化单星 SNR 基准值
    snr_map = {}
    degraded = _DEGRADED_SATS.get(port_id, set())
    for talker, prns in _GSV_SATS.items():
        for prn in prns:
            base = rng.randint(38, 52)
            if (talker, prn) in degraded:
                base -= 15  # 压制干扰导致衰减
            snr_map[(talker, prn)] = base

    # 局部大地坐标到经纬度的米制转换系数
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(BASE_LAT))

    # 1. 周期性生成 GSV 报文 (每 5 秒发送一次)
    for t in range(1, epochs, 5):
        for talker, prns in _GSV_SATS.items():
            for line in _gsv_sentences(talker, prns, snr_map, rng):
                timeline.append((float(t), 'nmea', line))

    # 2. 逐历元生成 GGA 与 RMC 定位解
    spike_idx = 0
    for k in range(epochs):
        utc = _utc_str(START_SOW + k)
        if k in gaps:
            # 注入失锁报文
            line = _gga_nofix(gga_talker, utc, nsat)
        else:
            # 高斯随机扰动
            de = rng.gauss(0, sigma_e)
            dn = rng.gauss(0, sigma_n)
            du = rng.gauss(0, sigma_u)
            
            # 注入大幅尖峰异常点
            if k in spikes:
                axis = _SPIKE_AXES[spike_idx % 3]
                spike_idx += 1
                if axis == 'east':
                    de += _SPIKE_AMPL_H
                elif axis == 'north':
                    dn += _SPIKE_AMPL_H
                else:
                    du += _SPIKE_AMPL_U
            
            # 转换为度分坐标并生成 GGA
            lat = BASE_LAT + dn / m_per_deg_lat
            lon = BASE_LON + de / m_per_deg_lon
            line = _gga(gga_talker, utc, lat, lon, BASE_ALT + du, 1, nsat)
        
        # 将语句错开微小时间差追加到时间轴
        timeline.append((float(k) + 0.2, 'nmea', line))
        timeline.append((float(k) + 0.5, 'nmea', _rmc(utc)))

    # 3. 定期生成 UBX-NAV-STATUS 帧 (第 35 秒首次输出，后续每 600 秒输出)
    for t in range(35, epochs, 600):
        frame = _ubx_nav_status((START_SOW + t) * 1000, 35000)
        timeline.append((float(t), 'ubx', frame))

    # 按仿真时间严格排序
    timeline.sort(key=lambda e: e[0])
    return timeline

