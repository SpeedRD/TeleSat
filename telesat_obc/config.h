#pragma once

// TelemetrySat OBC — ESP32 firmware v0.1.0
// Configuration header — pin assignments, baud rate, and compile-time constants.
// All hardware-specific values live here so .ino files never hard-code numbers.
// GPIOs use both columns; see CLAUDE.md hardware table for per-pin breadboard locations.

// Sensor pins
#define PIN_DHT11       4   // GPIO4 (D4), a5

// LED pin assignments (active HIGH)
#define PIN_LED_GREEN  15   // GPIO15, a3
#define PIN_LED_YELLOW 16   // GPIO16, a6
#define PIN_LED_RED    17   // GPIO17, a7

// I2C bus (shared by MPU6050 and DS1307)
#define PIN_I2C_SCL    22   // GPIO22, j9
#define PIN_I2C_SDA    21   // GPIO21, j10

// I2C device addresses
// DS1307 requires 5V VCC — connect to VIN (right red rail), not 3V3
#define DS1307_ADDR  0x68
#define MPU6050_ADDR 0x69  // AD0 pulled to 3V3

// UART
#define SERIAL_BAUD 115200

// Firmware identity
#define FIRMWARE_VERSION "0.1.0"

// FreeRTOS task stack size in words — ESP32 tasks need more stack than ESP8266
#define STACK_SIZE     4096
#define STACK_SIZE_DHT  4096  // DHT Adafruit library needs 4096 words minimum — stack canary crash at 2048
#define STACK_SIZE_UART 4096  // Serial.printf with floats needs extra stack

// Watchdog timing
#define WDG_TIMEOUT_MS  10000  // task must check in within 10s
#define WDG_CHECK_MS     5000  // watchdog checks every 5s

// TC packet sync word bytes and command IDs (see CLAUDE.md telecommand packet spec)
#define TC_SYNC_0       0xAB
#define TC_SYNC_1       0x12
#define CMD_PING        0x01
#define CMD_MODE_SAFE   0x02
#define CMD_MODE_NOM    0x03
#define CMD_MODE_DIAG   0x04
#define CMD_RESET       0x05
#define CMD_CLEAR_FAULT 0x06
