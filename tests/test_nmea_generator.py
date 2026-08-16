"""nmea_generator 纯逻辑自测: python tests\\test_nmea_generator.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.nmea.nmea_parser import NMEAParser
from src.protocol.ubx_parser import parse_ubx_frame
from src.simulation.nmea_generator import generate_port_timeline, _xor_checksum

M_PER_DEG = 111320.0


def parse_all(timeline):
    p = NMEAParser()
    lats, lons, nofix = [], [], 0
    for _, kind, payload in timeline:
        if kind != 'nmea':
            continue
        s = payload.decode('ascii').strip()
        p.parse(s)
        parts = s.split(',')
        if parts[0][3:6] == 'GGA':
            if parts[2] == '':
                nofix += 1
            else:
                lats.append(float(parts[2][:2]) + float(parts[2][2:]) / 60)
                lons.append(float(parts[4][:3]) + float(parts[4][3:]) / 60)
    return p, lats, lons, nofix


def main():
    tl = generate_port_timeline(2, minutes=60, seed=42)
    assert all(tl[i][0] <= tl[i + 1][0] for i in range(len(tl) - 1)), 'timeline未排序'

    for port in (1, 2, 3):
        t = generate_port_timeline(port, minutes=60, seed=42)
        assert all(e[2].endswith(b'\n') for e in t if e[1] == 'nmea')
        assert any(e[1] == 'ubx' for e in t), '缺少UBX帧'
        assert any(e[2].startswith(b'$GNRMC') for e in t), '缺少$GNRMC'
        assert any(e[2].startswith(b'$GNGSV') for e in t), '缺少$GNGSV'
        p, lats, lons, nofix = parse_all(t)
        assert p.unknown_sentences == 0, f'port{port}有未知语句'
        assert not list(p.errors), f'port{port}解析错误: {list(p.errors)[:2]}'

    # 可复现性
    assert generate_port_timeline(3, 10, 7) == generate_port_timeline(3, 10, 7), '同seed结果不一致'

    # 串口1: 厘米级, 无失锁
    p1, lats1, lons1, nf1 = parse_all(generate_port_timeline(1, minutes=60, seed=42))
    assert nf1 == 0
    assert (max(lats1) - min(lats1)) * M_PER_DEG < 5, '串口1抖动应<5m'
    assert (max(lons1) - min(lons1)) * M_PER_DEG < 5

    # 串口2/3: 米级抖动+尖峰(纬/经跨度>14m)+失锁间隙存在
    for port in (2, 3):
        p, lats, lons, nf = parse_all(generate_port_timeline(port, minutes=60, seed=42))
        assert (max(lats) - min(lats)) * M_PER_DEG > 14, f'port{port}纬向尖峰缺失'
        assert (max(lons) - min(lons)) * M_PER_DEG > 14, f'port{port}经向尖峰缺失'
        assert nf >= 5, f'port{port}失锁间隙缺失'
        assert p.gpgsv_data, f'port{port}GSV未解析'

    # UBX帧可解析且TTFF正确
    ubx = [e[2] for e in tl if e[1] == 'ubx'][0]
    msg_type, data = parse_ubx_frame(ubx)
    assert msg_type == 'NAV_STATUS' and data['ttff_s'] == 35.0

    # 校验和与生产解析器往返一致
    from src.nmea.nmea_gga_parser import NMEAGGAParser
    s = '$GPGGA,062932.00,2249.37557,N,11305.29747,E,1,09,1.2,45.7,M,-1.5,M,,'
    stamped = s + '*%02X' % _xor_checksum(s)
    assert NMEAGGAParser()._validate_checksum(stamped), '校验和与生产解析器不一致'

    print('ALL PASS: 生成器时间线/校验和/尖峰/间隙/GN语句/UBX/可复现性均正常')


if __name__ == '__main__':
    main()
