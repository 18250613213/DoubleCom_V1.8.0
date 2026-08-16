"""自检用导航数据生成器 - 纯Python无Qt依赖, 同一seed结果完全可复现。

数据设计:
- 串口1(基准): GGA含厘米级抖动($GNGGA), 无尖峰无失锁
- 串口2/3(干扰): GGA含米级抖动(σE/N=1.0m, σU=1.5m) + 周期性单历元尖峰
  (水平+15m/天向+18m, 稳超8σ且>5m门槛, 用于验证异常值剔除)
  + 少量q=0失锁间隙; 串口2用$GPGGA, 串口3用$BDGGA
- GSV: 混用GP/GL/BD/GN四种talker, 串口2/3部分卫星SNR恶化15dB
- RMC: $GNRMC(验证GN talker放行与TOW提取)
- UBX: NAV-STATUS帧(Fletcher校验), t=35s起TTFF=35.0s
"""
import math
import random
import struct

BASE_LAT = 28.185273
BASE_LON = 112.938604
BASE_ALT = 56.3
START_SOW = 2 * 3600  # UTC 02:00:00
RMC_DATE = "160826"

_GSV_SATS = {
    'GP': [2, 5, 6, 12, 17, 19, 25, 29, 31],
    'GL': [1, 2, 3, 8, 9, 11],
    'BD': [1, 2, 3, 6, 9, 16],
    'GN': [7, 9, 13, 14, 21, 26],
}

_DEGRADED_SATS = {
    2: {('GP', 2), ('GP', 12), ('GP', 25), ('GL', 8)},
    3: {('GP', 5), ('GP', 17), ('BD', 6), ('GN', 21)},
}

_SPIKE_AXES = ('east', 'north', 'up')
_SPIKE_AMPL_H = 15.0
_SPIKE_AMPL_U = 18.0


def _xor_checksum(body):
    c = 0
    for ch in body[1:]:
        c ^= ord(ch)
    return c


def _nmea(body):
    return (body + "*%02X" % _xor_checksum(body)).encode('ascii') + b'\n'


def _utc_str(sec):
    sec = int(sec) % 86400
    return "%02d%02d%02d.00" % (sec // 3600, (sec % 3600) // 60, sec % 60)


def _dmm(value, is_lat):
    d = int(value)
    m = abs(value - d) * 60
    return "%0*d%08.5f" % (2 if is_lat else 3, abs(d), m)


def _gga(talker, utc, lat, lon, alt, quality, nsat):
    body = ("$%sGGA,%s,%s,N,%s,E,%d,%02d,0.9,%.2f,M,-8.7,M,," %
            (talker, utc, _dmm(lat, True), _dmm(lon, False), quality, nsat, alt))
    return _nmea(body)


def _gga_nofix(talker, utc, nsat):
    body = "$%sGGA,%s,,N,,E,0,%02d,,,M,,M,," % (talker, utc, nsat)
    return _nmea(body)


def _rmc(utc):
    body = ("$GNRMC,%s,A,%s,N,%s,E,0.4,54.7,%s,,,A" %
            (utc, _dmm(BASE_LAT + 0.0001, True), _dmm(BASE_LON + 0.0001, False), RMC_DATE))
    return _nmea(body)


def _gsv_sentences(talker, prns, snr_map, rng):
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
    payload = struct.pack('<IBBBBII', tow_ms, 2, 0x01, 0x00, 0x01, ttff_ms, 456789)
    body = b'\xb5\x62\x01\x03' + struct.pack('<H', len(payload)) + payload
    ck_a = ck_b = 0
    for byte in body[2:]:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return body + bytes([ck_a, ck_b])


def generate_port_timeline(port_id, minutes=60, seed=42):
    """生成指定串口的模拟数据时间线。

    返回按时间升序的 (sim_time_sec, kind, payload) 列表:
    kind='nmea' 时 payload 为以\n结尾的语句字节串,
    kind='ubx' 时 payload 为完整UBX二进制帧。
    """
    rng = random.Random(seed * 1000 + port_id)
    epochs = int(minutes * 60)
    timeline = []

    is_ref = (port_id == 1)
    sigma_e = sigma_n = 0.02 if is_ref else 1.0
    sigma_u = 0.04 if is_ref else 1.5
    gga_talker = {1: 'GN', 2: 'GP', 3: 'BD'}[port_id]
    nsat = {1: 12, 2: 11, 3: 10}[port_id]

    spike_start = 180
    spike_step = max(45, (epochs - 200) // 10)
    spikes = set() if is_ref else set(range(spike_start, epochs - 30, spike_step))

    gap0 = epochs // 2 if port_id == 2 else epochs // 2 + 30
    gaps = set() if is_ref else set(range(gap0, gap0 + 5))

    snr_map = {}
    degraded = _DEGRADED_SATS.get(port_id, set())
    for talker, prns in _GSV_SATS.items():
        for prn in prns:
            base = rng.randint(38, 52)
            if (talker, prn) in degraded:
                base -= 15
            snr_map[(talker, prn)] = base

    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(BASE_LAT))

    for t in range(1, epochs, 5):
        for talker, prns in _GSV_SATS.items():
            for line in _gsv_sentences(talker, prns, snr_map, rng):
                timeline.append((float(t), 'nmea', line))

    spike_idx = 0
    for k in range(epochs):
        utc = _utc_str(START_SOW + k)
        if k in gaps:
            line = _gga_nofix(gga_talker, utc, nsat)
        else:
            de = rng.gauss(0, sigma_e)
            dn = rng.gauss(0, sigma_n)
            du = rng.gauss(0, sigma_u)
            if k in spikes:
                axis = _SPIKE_AXES[spike_idx % 3]
                spike_idx += 1
                if axis == 'east':
                    de += _SPIKE_AMPL_H
                elif axis == 'north':
                    dn += _SPIKE_AMPL_H
                else:
                    du += _SPIKE_AMPL_U
            lat = BASE_LAT + dn / m_per_deg_lat
            lon = BASE_LON + de / m_per_deg_lon
            line = _gga(gga_talker, utc, lat, lon, BASE_ALT + du, 1, nsat)
        timeline.append((float(k) + 0.2, 'nmea', line))
        timeline.append((float(k) + 0.5, 'nmea', _rmc(utc)))

    for t in range(35, epochs, 600):
        frame = _ubx_nav_status((START_SOW + t) * 1000, 35000)
        timeline.append((float(t), 'ubx', frame))

    timeline.sort(key=lambda e: e[0])
    return timeline
