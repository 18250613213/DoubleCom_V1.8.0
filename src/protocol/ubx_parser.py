"""UBX binary protocol parser for u-blox GNSS receivers.

UBX frame format:
  Sync:     0xB5 0x62  (2 bytes)
  Class:    1 byte
  ID:       1 byte
  Length:   2 bytes (little-endian, payload length)
  Payload:  N bytes
  CK_A:     1 byte  (Fletcher-8 first accumulator)
  CK_B:     1 byte  (Fletcher-8 second accumulator)

Checksum is computed over Class + ID + Length + Payload (everything between
sync bytes and checksum). See u-blox protocol specification for details.
"""

import struct

# --- UBX Protocol Constants ---
UBX_SYNC1 = 0xB5
UBX_SYNC2 = 0x62

UBX_HEADER_SIZE = 6       # Class(1) + ID(1) + Length(2) + Sync(2)
UBX_OVERHEAD = 8          # Header + 2 checksum bytes
UBX_SYNC_LEN = 2          # Sync bytes length

# --- Message Class / ID ---
CLASS_NAV = 0x01
ID_NAV_STATUS = 0x03

# NAV-STATUS payload layout (16 bytes)
NAV_STATUS_PAYLOAD_LEN = 16


def ubx_checksum(data):
    """Compute UBX Fletcher-8 checksum over data bytes.

    Returns (ck_a, ck_b) tuple.
    """
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def find_ubx_frame(buffer_data):
    """Scan buffer_data for a valid UBX frame.

    Returns:
        (idx, None)         - No frame found (idx: sync position or -1)
        (idx, (pre, frm, rest)) - Valid frame found
        (-1, (pre, None, rest)) - Checksum failure (false sync detected)
          where pre = data before sync, frm = complete frame, rest = after frame
    """
    idx = buffer_data.find(bytes([UBX_SYNC1, UBX_SYNC2]))
    if idx < 0:
        return -1, None

    if idx > 0:
        pre_data = buffer_data[:idx]
    else:
        pre_data = b''

    remaining = buffer_data[idx:]
    if len(remaining) < UBX_OVERHEAD:
        return idx, None

    cls = remaining[2]
    msg_id = remaining[3]
    length = struct.unpack_from('<H', remaining, 4)[0]

    total_len = UBX_OVERHEAD + length
    if len(remaining) < total_len:
        return idx, None

    frame = remaining[:total_len]
    rest = remaining[total_len:]

    class_id_payload = frame[2:UBX_HEADER_SIZE] + frame[UBX_HEADER_SIZE:UBX_HEADER_SIZE + length]
    ck_a, ck_b = ubx_checksum(class_id_payload)
    if ck_a != frame[UBX_HEADER_SIZE + length] or ck_b != frame[UBX_HEADER_SIZE + length + 1]:
        # Checksum mismatch: false sync, return signal for caller to skip and retry
        return -1, (pre_data, None, rest)

    return idx, (pre_data, frame, rest)


def extract_ubx_frames_from_buffer(buffer_data):
    """Extract all valid UBX frames from a byte buffer.

    Returns:
        (frames, remaining) - list of extracted frames and unconsumed tail
    """
    result_frames = []
    remaining = buffer_data

    while True:
        idx, result = find_ubx_frame(remaining)
        if result is None:
            break

        pre_data, frame, rest = result
        if frame is None:
            # Checksum failure: discard the entire invalid frame and continue
            remaining = pre_data + rest
            continue

        if pre_data:
            remaining_after_pre = pre_data + rest
        else:
            remaining_after_pre = rest

        result_frames.append(frame)
        remaining = remaining_after_pre

    return result_frames, remaining


def parse_nav_status(payload):
    """Parse NAV-STATUS (0x01 0x03) payload into a dictionary.

    Expects exactly NAV_STATUS_PAYLOAD_LEN bytes.
    """
    iTOW, gpsFix, flags, fixStat, flags2, ttff, msss = struct.unpack(
        '<I B B B B I I', payload[:NAV_STATUS_PAYLOAD_LEN]
    )

    gpsFixOk = (flags >> 0) & 0x01
    diffSoln = (flags >> 1) & 0x01
    carrSoln = (flags2 >> 6) & 0x03

    return {
        'iTOW': iTOW,
        'gpsFix': gpsFix,
        'gpsFixOk': gpsFixOk,
        'diffSoln': diffSoln,
        'fixStat': fixStat,
        'carrSoln': carrSoln,
        'ttff_ms': ttff,
        'ttff_s': ttff / 1000.0 if ttff > 0 else 0.0,
        # msss retained for future use (milliseconds since startup)
        'msss_ms': msss,
        'msss_s': msss / 1000.0,
    }


def parse_ubx_frame(frame):
    """Parse a validated UBX frame into (message_type, data_dict).

    Returns:
        ('NAV_STATUS', {...}) for recognized NAV-STATUS messages
        (None, None) for unsupported or unrecognized frames
    """
    cls = frame[2]
    msg_id = frame[3]
    length = struct.unpack_from('<H', frame, 4)[0]
    payload = frame[UBX_HEADER_SIZE:UBX_HEADER_SIZE + length]

    if cls == CLASS_NAV and msg_id == ID_NAV_STATUS:
        if length < NAV_STATUS_PAYLOAD_LEN:
            return (None, None)  # truncated payload
        return ('NAV_STATUS', parse_nav_status(payload[:NAV_STATUS_PAYLOAD_LEN]))

    return (None, None)