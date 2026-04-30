#pragma once
#include <Arduino.h>

// Satellite operating modes
typedef enum {
    MODE_BOOT       = 0,
    MODE_SAFE       = 1,
    MODE_NOMINAL    = 2,
    MODE_DIAGNOSTIC = 3
} SatMode;

// Fault flag bitmask values (stored in SensorData.fault_flags and TM packet byte 29)
#define FAULT_DHT_ERR   0x01   // bit 0 — DHT11 read failure
#define FAULT_IMU_ERR   0x02   // bit 1 — MPU6050 read failure
#define FAULT_WDG_HIT   0x04   // bit 2 — watchdog timeout fired

// Shared sensor data struct — written by sensor tasks, read by packet builder
// Always access under xSensorMutex
typedef struct {
    float temperature;
    float humidity;
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    uint8_t fault_flags;  // bit0=DHT_ERR, bit1=IMU_ERR, bit2=WDG_HIT
    SatMode mode;
} SensorData;

// TM packet — 32 bytes, packed (no padding)
#pragma pack(push, 1)
typedef struct {
    uint16_t sync_word;    // 0xEB90
    uint16_t apid;         // 0x0001 = housekeeping
    uint16_t seq_count;    // increments each packet
    uint16_t data_len;     // payload length in bytes (24)
    uint32_t timestamp;    // seconds since boot
    uint8_t  sat_mode;     // SatMode value
    int16_t  temperature;  // DHT11 temp × 100
    int16_t  humidity;     // DHT11 humidity × 100
    int16_t  accel_x;      // MPU6050 raw × 100
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;       // MPU6050 raw × 100
    int16_t  gyro_y;
    int16_t  gyro_z;
    uint8_t  fault_flags;
    uint16_t crc16;        // CRC16-CCITT of bytes 0-29
} TM_Packet;
#pragma pack(pop)

// TC packet — 6 bytes, packed (no padding)
#pragma pack(push, 1)
typedef struct {
    uint16_t sync_word;
    uint8_t  cmd_id;
    uint8_t  cmd_arg;
    uint16_t crc16;
} TC_Packet;
#pragma pack(pop)

// Function declarations
uint16_t computeCRC16(const uint8_t* data, size_t length);
void buildTMPacket(TM_Packet* pkt, const SensorData* sensor, uint16_t seq, uint32_t timestamp);
void sendPacketBinary(const TM_Packet* pkt);
void printPacketHex(const TM_Packet* pkt);
