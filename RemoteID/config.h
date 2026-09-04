#pragma once

// Fill these before flight. UAS_SERIAL must be the unmanned aircraft serial
// number in ANSI/CTA-2063-A form: 4-char manufacturer code, 1-char length,
// then the serial (max 20 characters total).
// Example shape: "1681FEXAMPLESERIAL01"
#define UAS_SERIAL                "1681FCHANGEME00001"
#define OPERATOR_ID               "FA1234567"   // FAA registration (optional)
#define SELF_ID_TEXT              "Remote ID beacon"

// 2 = helicopter / multirotor (ASTM F3411 UA type)
#define UA_TYPE                   2

// XIAO ESP32S3: use D1/D2 so GPS UART does not collide with USB-CDC UART0
// (GPIO43/D6 and GPIO44/D7).
#define GPS_RX_PIN                2    // D1  GPS TX -> ESP RX
#define GPS_TX_PIN                3    // D2  GPS RX <- ESP TX
#define GPS_BAUD                  115200
#define GPS_UART_NUM              1
#define GPS_UART_BUF              1024

// Betaflight GPS UART (UBX, same baud). Byte-for-byte bridge so the FC can
// auto-configure the M10 while this board sniffs NAV-PVT for Remote ID.
#define ENABLE_FC_PASSTHROUGH     1
#define FC_RX_PIN                 4    // D3  FC TX -> ESP RX
#define FC_TX_PIN                 5    // D4  ESP TX -> FC RX
#define FC_UART_NUM               2
#define FC_UART_BUF               1024

#define USER_LED_PIN              21   // XIAO user LED, active low
#define LED_ACTIVE_LOW            1

#define BLE_ROTATE_MS             150
#define WIFI_BEACON_MS            200
#define GPS_STALE_MS              2000

// ASTM F3411 transports. Wi-Fi must start before BLE on ESP32-S3 coexistence.
#define ENABLE_BLE_LEGACY         1    // Bluetooth 4 advertising, UUID 0xFFFA
#define ENABLE_BLE_LONG_RANGE     1    // Bluetooth 5 coded PHY + message pack
#define ENABLE_WIFI_BEACON        1    // Wi-Fi Beacon vendor IE + injected frames
#define WIFI_SSID                 "OpenDroneID"
#define WIFI_CHANNEL              6

// BOOT on the XIAO (GPIO0) opens the config site. Optional extra button: pin to GND.
#define CONFIG_BTN_PIN            0
#define CONFIG_BTN_ACTIVE_LOW     1
#define CONFIG_PORTAL_MS          300000

#define TAKEOFF_MIN_SATS          6
#define AIRBORNE_AGL_M            1.5f
#define AIRBORNE_SPEED_MPS        1.0f
