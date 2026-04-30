def compute_crc16(data: bytes) -> int:
    """CRC16-CCITT: polynomial 0x1021, init 0xFFFF. Matches ESP32 firmware."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc
