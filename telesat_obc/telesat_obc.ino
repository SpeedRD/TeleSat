    #include <DHT.h>
    #include <Wire.h>
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_Sensor.h>
    #include <RTClib.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "config.h"
    #include "ccsds.h"

    #define DHT_PIN  PIN_DHT11
    #define DHT_TYPE DHT11
    DHT dht(DHT_PIN, DHT_TYPE);
    Adafruit_MPU6050 mpu;
    RTC_DS1307 rtc;

    SensorData g_sensorData = {0};          // shared sensor struct
    SemaphoreHandle_t xSensorMutex = NULL;  // protects g_sensorData
    static uint16_t g_seqCount = 0;         // packet sequence counter
    QueueHandle_t xTxQueue = NULL;          // TM packet TX queue, depth 4
    QueueHandle_t xModeQueue = NULL;        // mode change requests → taskModeManager
    volatile SatMode g_currentMode = MODE_BOOT;

    typedef struct {
        TickType_t sensorRead;
        TickType_t packetBuilder;
        TickType_t commandRx;
    } WatchdogHeartbeats;

    volatile WatchdogHeartbeats g_heartbeats = {0, 0, 0};

    // ---------------------------------------------------------------------------
    // [I2C] I2C bus scanner task
    // Runs once at boot (priority 2 so it fires before the priority-1 tasks), scans
    // all 127 addresses, prints any devices found, then self-terminates via vTaskDelete.
    // Lets us confirm MPU6050 (0x68) and DS1307 (0x68/0x69) are alive before the
    // real sensor tasks start.
    // ---------------------------------------------------------------------------
    void taskI2CScanner(void *pvParameters) {
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

        bool found = false;
        for (uint8_t addr = 1; addr < 128; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                Serial.printf("[I2C] Device found at address 0x%02X\n", addr);
                found = true;
            }
        }
        if (!found) {
            Serial.println("[I2C] No devices found — check wiring");
        }
        Serial.println("[I2C] Scan complete");

        vTaskDelete(NULL);
    }

    // ---------------------------------------------------------------------------
    // [SENSOR] DHT11 sensor task
    // Reads temperature and humidity every 2s, writes to g_sensorData under mutex,
    // and prints to Serial. Sets FAULT_DHT_ERR on read error, clears it on success.
    // ---------------------------------------------------------------------------
    void taskSensorRead(void *pvParameters) {
        for (;;) {
            float t = dht.readTemperature();
            float h = dht.readHumidity();

            xSemaphoreTake(xSensorMutex, portMAX_DELAY);
            if (isnan(t) || isnan(h)) {
                g_sensorData.fault_flags |= FAULT_DHT_ERR;
                xSemaphoreGive(xSensorMutex);
                Serial.println("[DHT] ERROR — check wiring");
            } else {
                g_sensorData.temperature  = t;
                g_sensorData.humidity     = h;
                g_sensorData.fault_flags &= ~FAULT_DHT_ERR;
                xSemaphoreGive(xSensorMutex);
                Serial.printf("[DHT] Temp: %.1f C  Humidity: %.1f %%\n", t, h);
            }

            vTaskDelay(pdMS_TO_TICKS(2000));
            g_heartbeats.sensorRead = xTaskGetTickCount();
        }
    }

    // ---------------------------------------------------------------------------
    // [IMU] MPU6050 accelerometer and gyroscope task
    // Reads all six axes every 1s, writes to g_sensorData under mutex,
    // and prints to Serial.
    // ---------------------------------------------------------------------------
    void taskMPU6050Test(void *pvParameters) {
        for (;;) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            xSemaphoreTake(xSensorMutex, portMAX_DELAY);
            g_sensorData.accel_x = a.acceleration.x;
            g_sensorData.accel_y = a.acceleration.y;
            g_sensorData.accel_z = a.acceleration.z;
            g_sensorData.gyro_x  = g.gyro.x;
            g_sensorData.gyro_y  = g.gyro.y;
            g_sensorData.gyro_z  = g.gyro.z;
            xSemaphoreGive(xSensorMutex);

            Serial.printf("[IMU] Accel X:%.2f Y:%.2f Z:%.2f m/s2\n",
                        a.acceleration.x, a.acceleration.y, a.acceleration.z);
            Serial.printf("[IMU] Gyro  X:%.2f Y:%.2f Z:%.2f rad/s\n",
                        g.gyro.x, g.gyro.y, g.gyro.z);

            vTaskDelay(pdMS_TO_TICKS(1000));
            g_heartbeats.packetBuilder = xTaskGetTickCount();
        }
    }

    // ---------------------------------------------------------------------------
    // [PKT] Packet builder task
    // Wakes every 1s, snapshots g_sensorData under mutex, builds a 32-byte
    // CCSDS-inspired TM packet, and pushes it to xTxQueue for taskUARTTx to send.
    // Priority 2 ensures it runs before sensor tasks when all wake together.
    // ---------------------------------------------------------------------------
    void taskPacketBuilder(void *pvParameters) {
        TM_Packet pkt;
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));

            SensorData localCopy;
            xSemaphoreTake(xSensorMutex, portMAX_DELAY);
            localCopy = g_sensorData;
            xSemaphoreGive(xSensorMutex);

            g_seqCount++;
            DateTime now = rtc.now();
            uint32_t ts = now.unixtime();
            buildTMPacket(&pkt, &localCopy, g_seqCount, ts);

            if (xQueueSend(xTxQueue, &pkt, pdMS_TO_TICKS(100)) != pdTRUE) {
                Serial.println("[PKT] WARNING — TX queue full, packet dropped");
            }
        }
    }

    // ---------------------------------------------------------------------------
    // [UART] TX task
    // Blocks on xTxQueue and transmits each TM packet to Serial as it arrives.
    // Decouples packet building from transmission — builder never stalls on Serial.
    // ---------------------------------------------------------------------------
    void taskUARTTx(void *pvParameters) {
        TM_Packet pkt;
        for (;;) {
            if (xQueueReceive(xTxQueue, &pkt, portMAX_DELAY) == pdTRUE) {
                sendPacketBinary(&pkt);
            }
        }
    }

    // ---------------------------------------------------------------------------
    // [CMD] TC command receiver task
    // Reads incoming bytes from Serial, assembles 6-byte TC packets using a state
    // machine parser (sync bytes first), validates CRC16, and dispatches commands.
    // Mode change commands are forwarded to taskModeManager via xModeQueue.
    // ---------------------------------------------------------------------------
    void taskCommandRx(void *pvParameters) {
        uint8_t buf[6];
        uint8_t idx = 0;
        for (;;) {
            if (Serial.available()) {
                uint8_t b = Serial.read();
                if (idx == 0 && b != TC_SYNC_0) continue;
                if (idx == 1 && b != TC_SYNC_1) { idx = 0; continue; }
                buf[idx++] = b;
                if (idx == 6) {
                    idx = 0;
                    uint16_t rxCRC = (buf[4] << 8) | buf[5];
                    uint16_t calcCRC = computeCRC16(buf, 4);
                    if (rxCRC != calcCRC) {
                        Serial.println("[CMD] Bad CRC — discarding");
                        continue;
                    }
                    uint8_t cmd = buf[2];
                    uint8_t arg = buf[3];
                    Serial.printf("[CMD] Received cmd=0x%02X arg=0x%02X\n", cmd, arg);
                    SatMode newMode = g_currentMode;
                    switch (cmd) {
                        case CMD_PING:
                            Serial.println("[CMD] PING — ACK");
                            break;
                        case CMD_MODE_SAFE:   newMode = MODE_SAFE;       break;
                        case CMD_MODE_NOM:    newMode = MODE_NOMINAL;    break;
                        case CMD_MODE_DIAG:   newMode = MODE_DIAGNOSTIC; break;
                        case CMD_CLEAR_FAULT:
                            xSemaphoreTake(xSensorMutex, portMAX_DELAY);
                            g_sensorData.fault_flags = 0;
                            xSemaphoreGive(xSensorMutex);
                            Serial.println("[CMD] Fault flags cleared");
                            break;
                        case CMD_RESET:
                            Serial.println("[CMD] RESET commanded — rebooting");
                            vTaskDelay(pdMS_TO_TICKS(100));
                            esp_restart();
                            break;
                    }
                    if (newMode != g_currentMode) {
                        xQueueSend(xModeQueue, &newMode, pdMS_TO_TICKS(100));
                    }
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
                g_heartbeats.commandRx = xTaskGetTickCount();
            }
        }
    }

    // ---------------------------------------------------------------------------
    // [MODE] Mode manager task
    // Owns the LED outputs. Waits on xModeQueue for mode change requests,
    // updates g_currentMode and g_sensorData.mode, and drives LEDs accordingly.
    // Transitions BOOT → SAFE once at startup before entering the queue loop.
    // ---------------------------------------------------------------------------
    void taskModeManager(void *pvParameters) {
        pinMode(PIN_LED_GREEN,  OUTPUT);
        pinMode(PIN_LED_YELLOW, OUTPUT);
        pinMode(PIN_LED_RED,    OUTPUT);
        digitalWrite(PIN_LED_GREEN,  LOW);
        digitalWrite(PIN_LED_YELLOW, LOW);
        digitalWrite(PIN_LED_RED,    LOW);

        g_currentMode = MODE_SAFE;
        digitalWrite(PIN_LED_YELLOW, HIGH);
        Serial.println("[MODE] BOOT complete → SAFE");

        SatMode requested;
        for (;;) {
            if (xQueueReceive(xModeQueue, &requested, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_currentMode = requested;
                digitalWrite(PIN_LED_GREEN,  LOW);
                digitalWrite(PIN_LED_YELLOW, LOW);
                digitalWrite(PIN_LED_RED,    LOW);
                switch (g_currentMode) {
                    case MODE_SAFE:
                        digitalWrite(PIN_LED_YELLOW, HIGH);
                        Serial.println("[MODE] → SAFE");
                        break;
                    case MODE_NOMINAL:
                        digitalWrite(PIN_LED_GREEN, HIGH);
                        Serial.println("[MODE] → NOMINAL");
                        break;
                    case MODE_DIAGNOSTIC:
                        digitalWrite(PIN_LED_GREEN, HIGH);
                        digitalWrite(PIN_LED_YELLOW, HIGH);
                        Serial.println("[MODE] → DIAGNOSTIC");
                        break;
                    default: break;
                }
                xSemaphoreTake(xSensorMutex, portMAX_DELAY);
                g_sensorData.mode = g_currentMode;
                xSemaphoreGive(xSensorMutex);
            }
        }
    }

    // ---------------------------------------------------------------------------
    // [WDG] Watchdog task
    // Waits one full timeout period at startup to let all tasks initialise and
    // post their first heartbeat. Then checks every WDG_CHECK_MS that each
    // monitored task has updated its heartbeat within WDG_TIMEOUT_MS. Sets
    // FAULT_WDG_HIT and forces SAFE mode on any timeout.
    // ---------------------------------------------------------------------------
    void taskWatchdog(void *pvParameters) {
        vTaskDelay(pdMS_TO_TICKS(WDG_TIMEOUT_MS));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(WDG_CHECK_MS));
            TickType_t now = xTaskGetTickCount();
            TickType_t timeout = pdMS_TO_TICKS(WDG_TIMEOUT_MS);
            bool fault = false;

            if ((now - g_heartbeats.sensorRead) > timeout) {
                Serial.println("[WDG] FAULT — taskSensorRead timeout");
                xSemaphoreTake(xSensorMutex, portMAX_DELAY);
                g_sensorData.fault_flags |= FAULT_WDG_HIT;
                xSemaphoreGive(xSensorMutex);
                fault = true;
            }
            if ((now - g_heartbeats.packetBuilder) > timeout) {
                Serial.println("[WDG] FAULT — taskPacketBuilder timeout");
                xSemaphoreTake(xSensorMutex, portMAX_DELAY);
                g_sensorData.fault_flags |= FAULT_WDG_HIT;
                xSemaphoreGive(xSensorMutex);
                fault = true;
            }
            if ((now - g_heartbeats.commandRx) > timeout) {
                Serial.println("[WDG] FAULT — taskCommandRx timeout");
                xSemaphoreTake(xSensorMutex, portMAX_DELAY);
                g_sensorData.fault_flags |= FAULT_WDG_HIT;
                xSemaphoreGive(xSensorMutex);
                fault = true;
            }
            if (fault) {
                SatMode safe = MODE_SAFE;
                xQueueSend(xModeQueue, &safe, pdMS_TO_TICKS(100));
                Serial.println("[WDG] Forcing SAFE mode");
            } else {
                Serial.println("[WDG] All tasks healthy");
            }
        }
    }

    // ---------------------------------------------------------------------------
    // [BOOT] Hardware initialisation and task creation
    // setup() runs once before tasks start. Arduino delay() is acceptable here
    // because we are still in single-threaded boot context, not inside a task.
    // After xTaskCreatePinnedToCore() returns, the scheduler owns all execution.
    // ---------------------------------------------------------------------------
    void setup() {
        // Start Serial and allow the USB-UART bridge to settle.
        Serial.begin(SERIAL_BAUD);
        delay(1000);

        Serial.println("[BOOT] TelemetrySat OBC " FIRMWARE_VERSION " — FreeRTOS starting...");

        // Initialise DHT11 before tasks start so the sensor is ready on first read.
        dht.begin();

        // Initialise I2C bus, MPU6050, and DS1307.
        // 1000ms lets I2C devices fully stabilise on cold boot — prevents watchdog reset loop.
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        delay(1000);

        if (!mpu.begin(0x69, &Wire)) {
            Serial.println("[IMU] MPU6050 not found — check wiring");
        } else {
            Serial.println("[IMU] MPU6050 initialised");
            mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
            mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        }

        if (!rtc.begin(&Wire)) {
            Serial.println("[RTC] DS1307 not found");
        } else {
            Serial.println("[RTC] DS1307 initialised");
            if (!rtc.isrunning()) {
                Serial.println("[RTC] RTC not running — setting time to compile time");
                rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
            }
        }

        // Create shared mutex and TX queue before any task that uses them.
        xSensorMutex = xSemaphoreCreateMutex();
        xTxQueue = xQueueCreate(4, sizeof(TM_Packet));
        if (xTxQueue == NULL) {
            Serial.println("[BOOT] ERROR — failed to create TX queue");
        }
        xModeQueue = xQueueCreate(4, sizeof(SatMode));

        // Create tasks pinned to core 1. Core 0 is left for the ESP32 WiFi/BT stack.
        // taskPacketBuilder runs at priority 2; all other tasks at priority 1.
        xTaskCreatePinnedToCore(taskPacketBuilder, "PktBuilder",  4096,       NULL, 2, NULL, 1);
        xTaskCreatePinnedToCore(taskUARTTx,        "UARTTx",      STACK_SIZE_UART, NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(taskSensorRead,     "SensorRead",  STACK_SIZE_DHT, NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(taskMPU6050Test,   "MPU6050Test", 4096,       NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(taskCommandRx,     "CommandRx",   4096,       NULL, 3, NULL, 1);
        xTaskCreatePinnedToCore(taskModeManager,   "ModeManager", 4096,       NULL, 2, NULL, 1);
        xTaskCreatePinnedToCore(taskWatchdog,      "Watchdog",    4096,       NULL, 1, NULL, 1);

        Serial.println("[BOOT] Tasks created. Scheduler running.");
    }

    // FreeRTOS tasks own all execution. loop() must remain empty.
    void loop() {
    }
