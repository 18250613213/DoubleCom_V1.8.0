"""
NMEA 数据生成器与底层协议解析器纯逻辑单元测试

运行命令:
    python tests\\test_nmea_generator.py

测试验证项:
  1. 时间线升序单调性校验。
  2. NMEA 换行符完整性、UBX 帧存在性、GNRMC 与 GNGSV 语句输出。
  3. 伪随机数种子 (Seed) 跨次运行的严格确定性与可复现性。
  4. 串口 1 (基准机) 厘米级微抖动与零失锁约束 (<5m 极差)。
  5. 串口 2/3 (干扰机) 米级抖动、>14m 大幅尖峰跳变以及连续失锁脱锁间隙。
  6. UBX-NAV-STATUS 二进制帧解析与 TTFF 首次定位时间准确性。
  7. 异或校验和在生成器与生产解析器间的往返一致性。
"""

import os
import sys

# 将工程根目录追加到模块搜索路径中
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.nmea.nmea_parser import NMEAParser
from src.protocol.ubx_parser import parse_ubx_frame
from src.simulation.nmea_generator import generate_port_timeline, _xor_checksum

# 纬度 1 度对应地面近似米数 (WGS84 椭球平均值)
M_PER_DEG = 111320.0


def parse_all(timeline):
    """
    辅助测试函数: 遍历回放时间线并使用 NMEAParser 全量解析所有 NMEA 语句。

    参数:
        timeline (list): 时间线事件序列。

    返回:
        tuple: (parser_instance, lats_list, lons_list, nofix_count)
    """
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
                nofix += 1  # 记录失锁/无有效坐标的 GGA 历元
            else:
                lats.append(float(parts[2][:2]) + float(parts[2][2:]) / 60)
                lons.append(float(parts[4][:3]) + float(parts[4][3:]) / 60)
    return p, lats, lons, nofix


def main():
    """单元测试主断言流程。"""
    # 1. 验证时间线时间戳单调递增
    tl = generate_port_timeline(2, minutes=60, seed=42)
    assert all(tl[i][0] <= tl[i + 1][0] for i in range(len(tl) - 1)), 'timeline未排序'

    # 2. 验证各串口输出内容合法性与无未知/错误语句
    for port in (1, 2, 3):
        t = generate_port_timeline(port, minutes=60, seed=42)
        assert all(e[2].endswith(b'\n') for e in t if e[1] == 'nmea'), f'port{port} NMEA末尾缺少换行'
        assert any(e[1] == 'ubx' for e in t), '缺少UBX帧'
        assert any(e[2].startswith(b'$GNRMC') for e in t), '缺少$GNRMC'
        assert any(e[2].startswith(b'$GNGSV') for e in t), '缺少$GNGSV'
        p, lats, lons, nofix = parse_all(t)
        assert p.unknown_sentences == 0, f'port{port}有未知语句'
        assert not list(p.errors), f'port{port}解析错误: {list(p.errors)[:2]}'

    # 3. 验证确定性可复现 (同一种子两次生成必须完全一致)
    assert generate_port_timeline(3, 10, 7) == generate_port_timeline(3, 10, 7), '同seed结果不一致'

    # 4. 验证基准机串口 1 行为特征: 抖动极小且全程不失锁
    p1, lats1, lons1, nf1 = parse_all(generate_port_timeline(1, minutes=60, seed=42))
    assert nf1 == 0, '基准口不应出现失锁'
    assert (max(lats1) - min(lats1)) * M_PER_DEG < 5, '串口1抖动应<5m'
    assert (max(lons1) - min(lons1)) * M_PER_DEG < 5, '串口1抖动应<5m'

    # 5. 验证干扰机串口 2/3 行为特征: 米级抖动 + 大幅尖峰跳变 (>14m) + 存在失锁间隙
    for port in (2, 3):
        p, lats, lons, nf = parse_all(generate_port_timeline(port, minutes=60, seed=42))
        assert (max(lats) - min(lats)) * M_PER_DEG > 14, f'port{port}纬向尖峰缺失'
        assert (max(lons) - min(lons)) * M_PER_DEG > 14, f'port{port}经向尖峰缺失'
        assert nf >= 5, f'port{port}失锁间隙缺失'
        assert p.gpgsv_data, f'port{port}GSV未解析'

    # 6. 验证 UBX 帧解析与 TTFF 数值
    ubx = [e[2] for e in tl if e[1] == 'ubx'][0]
    msg_type, data = parse_ubx_frame(ubx)
    assert msg_type == 'NAV_STATUS' and data['ttff_s'] == 35.0, 'UBX NAV-STATUS或TTFF解析异常'

    # 7. 验证校验和在生成器与生产 NMEAGGAParser 之间的一致性
    from src.nmea.nmea_gga_parser import NMEAGGAParser
    s = '$GPGGA,062932.00,2249.37557,N,11305.29747,E,1,09,1.2,45.7,M,-1.5,M,,'
    stamped = s + '*%02X' % _xor_checksum(s)
    assert NMEAGGAParser()._validate_checksum(stamped), '校验和与生产解析器不一致'

    print('ALL PASS: 生成器时间线/校验和/尖峰/间隙/GN语句/UBX/可复现性均正常')


if __name__ == '__main__':
    main()

