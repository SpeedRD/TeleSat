from crc16 import compute_crc16

CMD_MAP = {
    'PING':      0x01,
    'MODE_SAFE': 0x02,
    'MODE_NOM':  0x03,
    'MODE_DIAG': 0x04,
    'RESET':     0x05,
    'CLR_FAULT': 0x06,
}


def build_tc_packet(cmd_name: str, arg: int = 0x00) -> bytes:
    """Build a 6-byte TC packet with CRC16-CCITT over the first 4 bytes."""
    payload = bytes([0xAB, 0x12, CMD_MAP[cmd_name], arg & 0xFF])
    crc = compute_crc16(payload)
    return payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF])
