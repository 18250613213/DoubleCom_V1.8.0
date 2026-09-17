"""
u-blox GNSS 接收机专有 UBX 二进制协议解析器

UBX 协议帧结构格式说明:
  同步头 (Sync):   0xB5 0x62  (2 字节，固定同步字符，标识帧开始)
  类别 (Class):    1 字节     (消息类别，如 NAV=0x01, RXM=0x02 等)
  标识 (ID):       1 字节     (具体消息标识，如 NAV-STATUS=0x03)
  长度 (Length):   2 字节     (小端格式 unsigned short，仅表示负载 Payload 的字节数)
  负载 (Payload):  N 字节     (实际消息数据内容，由具体的 Class/ID 决定)
  校验A (CK_A):    1 字节     (8 位 Fletcher 校验和第一个累加器)
  校验B (CK_B):    1 字节     (8 位 Fletcher 校验和第二个累加器)

校验和计算范围:
  从 Class 开始，经过 ID、Length 到 Payload 结束（即同步头之后、校验字节之前的所有字节）。
  算法采用 8 位 Fletcher 算法（以 256 为模）。
"""

import struct

# --- UBX 协议常量定义 ---
UBX_SYNC1 = 0xB5          # 同步字节 1 (ASCII: µ)
UBX_SYNC2 = 0x62          # 同步字节 2 (ASCII: b)

UBX_HEADER_SIZE = 6       # 头部固定大小: 同步头(2) + Class(1) + ID(1) + Length(2)
UBX_OVERHEAD = 8          # 帧总开销: 头部(6) + 校验和(2)
UBX_SYNC_LEN = 2          # 同步字节长度

# --- 消息类别 (Class) 与标识 (ID) ---
CLASS_NAV = 0x01          # 导航类消息 (Navigation Results)
ID_NAV_STATUS = 0x03      # 接收机导航状态信息 (Receiver Navigation Status)

# NAV-STATUS 消息负载的标准最小长度 (字节数: iTOW(4) + gpsFix(1) + flags(1) + fixStat(1) + flags2(1) + ttff(4) + msss(4) = 16)
NAV_STATUS_PAYLOAD_LEN = 16


def ubx_checksum(data):
    """
    计算 UBX 专用的 8 位 Fletcher 校验和。

    算法规则:
      遍历待校验数据字节序列，维护两个 8 位累加器 (ck_a, ck_b):
      ck_a = (ck_a + byte) mod 256
      ck_b = (ck_b + ck_a) mod 256

    参数:
        data (bytes 或 bytearray): 待计算校验和的字节序列（包含 Class, ID, Length, Payload）。

    返回:
        tuple (int, int): (ck_a, ck_b) 校验和元组。
    """
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def find_ubx_frame(buffer_data):
    """
    在字节缓冲区中搜索首个合法的完整 UBX 帧。

    处理逻辑:
      1. 查找 0xB5 0x62 同步字，若未找到则返回无帧。
      2. 检查剩余长度是否达到最小头部开销 (8 字节)，不足则等待后续数据。
      3. 解析负载长度 length，判断整帧是否接收完整 (8 + length 字节)。
      4. 计算并核对 Fletcher 校验和:
         - 校验失败: 说明匹配到了虚假同步字（如恰好在 NMEA 文本或随机噪声中出现 0xB5 0x62），
           标记校验失败并返回前驱数据和后续数据，供上层跳过该位置继续寻找。
         - 校验成功: 提取出完整帧，返回前驱数据、有效帧及尾部剩余数据。

    参数:
        buffer_data (bytes): 待搜索的原始字节流。

    返回:
        tuple (idx, result):
            - 若未找到同步字或数据不足: (idx, None)
            - 若找到有效帧: (idx, (pre_data, frame, rest))
            - 若校验和不匹配(伪同步字): (-1, (pre_data, None, rest))
    """
    # 查找同步字位置
    idx = buffer_data.find(bytes([UBX_SYNC1, UBX_SYNC2]))
    if idx < 0:
        return -1, None

    # 同步字之前的数据（可能是 NMEA 语句或其他非 UBX 数据）
    if idx > 0:
        pre_data = buffer_data[:idx]
    else:
        pre_data = b''

    # 从同步头开始的剩余数据
    remaining = buffer_data[idx:]
    if len(remaining) < UBX_OVERHEAD:
        # 数据长度不足以解析头部和校验和，等待更多字节流入
        return idx, None

    cls = remaining[2]
    msg_id = remaining[3]
    # 小端字节序解析负载长度 (2 字节无符号整数)
    length = struct.unpack_from('<H', remaining, 4)[0]

    total_len = UBX_OVERHEAD + length
    if len(remaining) < total_len:
        # 负载数据尚未接收完整，继续等待
        return idx, None

    frame = remaining[:total_len]
    rest = remaining[total_len:]

    # 提取 Class + ID + Length + Payload 参与校验和计算
    class_id_payload = frame[2:UBX_HEADER_SIZE] + frame[UBX_HEADER_SIZE:UBX_HEADER_SIZE + length]
    ck_a, ck_b = ubx_checksum(class_id_payload)

    # 验证末尾两字节校验和
    if ck_a != frame[UBX_HEADER_SIZE + length] or ck_b != frame[UBX_HEADER_SIZE + length + 1]:
        # 校验和不匹配：为虚假同步字（False Sync），通知调用者跳过该错误帧并继续检索
        return -1, (pre_data, None, rest)

    return idx, (pre_data, frame, rest)


def extract_ubx_frames_from_buffer(buffer_data):
    """
    从字节缓冲区中循环提取出所有完整的有效 UBX 帧，同时保留未消费的数据。

    该设计保证了在 NMEA 与 UBX 混合传输的串口流中，UBX 帧能被无损抽离，
    而夹杂在其中的 NMEA 文本数据仍能被保留并送往 NMEA 解析器。

    参数:
        buffer_data (bytes): 包含混合协议数据的串口接收缓冲区。

    返回:
        tuple (list, bytes):
            - frames: 解析出的合法 UBX 帧列表（每个元素为完整的 bytes 帧）。
            - remaining: 剩余未消费的字节（包括非 UBX 数据及未接收完整的半包）。
    """
    result_frames = []
    remaining = buffer_data

    while True:
        idx, result = find_ubx_frame(remaining)
        if result is None:
            # 没有找到同步字或数据长度不足，退出循环
            break

        pre_data, frame, rest = result
        if frame is None:
            # 校验和失败（虚假同步字）：丢弃错误的头部，并将前后数据拼接继续扫描
            remaining = pre_data + rest
            continue

        # 将同步头之前的数据保留在缓冲区中，避免截断夹杂的 NMEA 文本
        if pre_data:
            remaining_after_pre = pre_data + rest
        else:
            remaining_after_pre = rest

        result_frames.append(frame)
        remaining = remaining_after_pre

    return result_frames, remaining


def parse_nav_status(payload):
    """
    解析 NAV-STATUS (Class 0x01, ID 0x03) 消息负载。

    NAV-STATUS 包含接收机的当前导航定位状态与 TTFF（首次定位时间）等关键指标。

    负载二进制布局 (共 16 字节，小端格式):
      U4  iTOW:    GPS 毫秒时间戳 (Time of Week)
      U1  gpsFix:  定位类型 (0=无定位, 1=航位推算, 2=2D定位, 3=3D定位, 4=GPS+航位推算, 5=时间同步)
      X1  flags:   导航状态标志位:
                   - bit 0: gpsFixOk (定位有效标记, 1=有效且在公差范围内)
                   - bit 1: diffSoln (差分定位标记, 1=已应用 DGPS 差分修正)
      X1  fixStat: 定位状态辅助信息
      X1  flags2:  高级标志位:
                   - bits 6..7: carrSoln (载波相位解算状态: 0=无, 1=浮点解 Float, 2=固定解 Fix)
      U4  ttff:    首次定位时间 (Time To First Fix)，单位: 毫秒 (ms)
      U4  msss:    开机运行时间 (Milliseconds since Startup)，单位: 毫秒 (ms)

    参数:
        payload (bytes): NAV-STATUS 消息的有效载荷（至少 16 字节）。

    返回:
        dict: 结构化解析结果字典。
    """
    iTOW, gpsFix, flags, fixStat, flags2, ttff, msss = struct.unpack(
        '<I B B B B I I', payload[:NAV_STATUS_PAYLOAD_LEN]
    )

    # 提取位字段标志
    gpsFixOk = (flags >> 0) & 0x01      # 定位有效标志
    diffSoln = (flags >> 1) & 0x01      # 是否差分解
    carrSoln = (flags2 >> 6) & 0x03     # 载波相位解状态 (RTK Float / Fix)

    return {
        'iTOW': iTOW,
        'gpsFix': gpsFix,
        'gpsFixOk': gpsFixOk,
        'diffSoln': diffSoln,
        'fixStat': fixStat,
        'carrSoln': carrSoln,
        'ttff_ms': ttff,
        'ttff_s': ttff / 1000.0 if ttff > 0 else 0.0,  # 转换为秒
        # msss: 开机至今毫秒数
        'msss_ms': msss,
        'msss_s': msss / 1000.0,
    }


def parse_ubx_frame(frame):
    """
    分发并解析已完成校验的完整 UBX 帧。

    参数:
        frame (bytes): 包含同步头、长度及校验和在内的完整 UBX 字节帧。

    返回:
        tuple (str or None, dict or None):
            - 若为受支持的 NAV-STATUS 消息: ('NAV_STATUS', parse_nav_status(payload))
            - 若为未知或暂不支持的消息类型: (None, None)
    """
    cls = frame[2]
    msg_id = frame[3]
    length = struct.unpack_from('<H', frame, 4)[0]
    payload = frame[UBX_HEADER_SIZE:UBX_HEADER_SIZE + length]

    # 分发 NAV-STATUS 消息
    if cls == CLASS_NAV and msg_id == ID_NAV_STATUS:
        if length < NAV_STATUS_PAYLOAD_LEN:
            return (None, None)  # 载荷长度不足，截断异常
        return ('NAV_STATUS', parse_nav_status(payload[:NAV_STATUS_PAYLOAD_LEN]))

    # 其他类型消息在此处扩展（如 NAV-PVT, NAV-TIMEGPS 等）
    return (None, None)