#include "ccsds.h"

// CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF)
// Used in space comms, HDLC, and many embedded protocols
uint16_t computeCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}

void buildTMPacket(TM_Packet* pkt, const SensorData* sensor, uint16_t seq, uint32_t timestamp) {
    pkt->sync_word   = 0xEB90;
    pkt->apid        = 0x0001;
    pkt->seq_count   = seq;
    pkt->data_len    = 24;
    pkt->timestamp   = timestamp;
    pkt->sat_mode    = (uint8_t)sensor->mode;
    pkt->temperature = (int16_t)(sensor->temperature * 100);
    pkt->humidity    = (int16_t)(sensor->humidity    * 100);
    pkt->accel_x     = (int16_t)(sensor->accel_x     * 100);
    pkt->accel_y     = (int16_t)(sensor->accel_y     * 100);
    pkt->accel_z     = (int16_t)(sensor->accel_z     * 100);
    pkt->gyro_x      = (int16_t)(sensor->gyro_x      * 100);
    pkt->gyro_y      = (int16_t)(sensor->gyro_y      * 100);
    pkt->gyro_z      = (int16_t)(sensor->gyro_z      * 100);
    pkt->fault_flags = sensor->fault_flags;
    pkt->crc16       = computeCRC16((uint8_t*)pkt, sizeof(TM_Packet) - 2);
}

void sendPacketBinary(const TM_Packet* pkt) {
    Serial.write((const uint8_t*)pkt, sizeof(TM_Packet));
}

void printPacketHex(const TM_Packet* pkt) {
    const uint8_t* bytes = (const uint8_t*)pkt;
    Serial.print("[PKT] ");
    for (size_t i = 0; i < sizeof(TM_Packet); i++) {
        if (bytes[i] < 0x10) Serial.print("0");
        Serial.print(bytes[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
