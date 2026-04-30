import struct
import threading
import time
import serial
from crc16 import compute_crc16

# TM packet: 32 bytes, little-endian
# sync(H) apid(H) seq(H) dlen(H) ts(I) mode(B) temp(h) hum(h)
# ax(h) ay(h) az(h) gx(h) gy(h) gz(h) fault(B) crc(H)
_TM_FMT = '<HHHHIBhhhhhhhhBH'
_TM_SIZE = 32

_MODES = ['BOOT', 'SAFE', 'NOMINAL', 'DIAGNOSTIC']


class SerialReader:
    def __init__(self, port, baud, on_packet_callback):
        self.port = port
        self.baud = baud
        self.on_packet_callback = on_packet_callback
        self.ser = None
        self.running = False
        self._thread = None
        self._buf = bytearray()

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False

    def send(self, data: bytes):
        if self.ser and self.ser.is_open:
            self.ser.write(data)

    def _run(self):
        SYNC = bytes([0x90, 0xEB])  # 0xEB90 little-endian on wire
        while self.running:
            try:
                if self.ser is None or not self.ser.is_open:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                    time.sleep(0.5)
                    continue
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if chunk:
                    self._buf.extend(chunk)
                    while len(self._buf) >= 2:
                        idx = self._buf.find(SYNC)
                        if idx == -1:
                            self._buf = self._buf[-1:]
                            break
                        if idx > 0:
                            self._buf = self._buf[idx:]
                        if len(self._buf) < _TM_SIZE:
                            break
                        raw = bytes(self._buf[:_TM_SIZE])
                        self._buf = self._buf[_TM_SIZE:]
                        self._try_parse(raw)
            except serial.SerialException:
                self.ser = None
                time.sleep(2)
            except Exception:
                time.sleep(0.01)

    def _try_parse(self, raw: bytes):
        if len(raw) != _TM_SIZE:
            return
        fields = struct.unpack(_TM_FMT, raw)
        sync, apid, seq, dlen, ts, mode, tmp, hum, ax, ay, az, gx, gy, gz, fault, crc = fields
        if sync != 0xEB90:
            return
        if compute_crc16(raw[:30]) != crc:
            return
        pkt = {
            'seq':        seq,
            'timestamp':  ts,
            'mode':       _MODES[mode] if mode < 4 else 'UNKNOWN',
            'mode_raw':   mode,
            'temperature': round(tmp / 100.0, 1),
            'humidity':    round(hum / 100.0, 1),
            'accel_x':    round(ax / 100.0, 2),
            'accel_y':    round(ay / 100.0, 2),
            'accel_z':    round(az / 100.0, 2),
            'gyro_x':     round(gx / 100.0, 2),
            'gyro_y':     round(gy / 100.0, 2),
            'gyro_z':     round(gz / 100.0, 2),
            'fault_flags': fault,
            'fault_dht':  bool(fault & 0x01),
            'fault_imu':  bool(fault & 0x02),
            'fault_wdg':  bool(fault & 0x04),
            'crc':        f'0x{crc:04X}',
            'raw_hex':    ' '.join(f'{b:02X}' for b in raw),
        }
        self.on_packet_callback(pkt)
