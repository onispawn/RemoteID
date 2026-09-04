#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>

#include "config.h"
#include "odid.h"

// ============================================================================
// XIAO ESP32-S3
// Arduino-ESP32 3.3.11
// ============================================================================

static bool bleReady = false;
static bool ble5Ready = false;

static BLEAdvertising *bleAdvertising = nullptr;


// ============================================================================
// Legacy Remote ID advertisement
// ============================================================================

static uint8_t bleAdv[31];


// ============================================================================
// Build legacy Remote ID advertisement
// ============================================================================

static void bleBuildLegacy(
    uint8_t counter,
    const uint8_t msg[25])
{
    bleAdv[0] = 0x1E;
    bleAdv[1] = 0x16;

    // Open Drone ID UUID = 0xFFFA, little endian
    bleAdv[2] = 0xFA;
    bleAdv[3] = 0xFF;

    // Remote ID message type
    bleAdv[4] = 0x0D;

    // Counter
    bleAdv[5] = counter;

    memcpy(
        bleAdv + 6,
        msg,
        25
    );
}


// ============================================================================
// Start BLE
// ============================================================================

static bool bleStart()
{
#if !ENABLE_BLE_LEGACY && !ENABLE_BLE_LONG_RANGE

    Serial.println("BLE disabled in config.h");
    return false;

#endif

    // Initialize BLE.
    BLEDevice::init("RemoteID");

    // Get advertising object.
    bleAdvertising =
        BLEDevice::getAdvertising();

    if (bleAdvertising == nullptr) {

        Serial.println(
            "ERROR: BLE advertising object unavailable"
        );

        return false;
    }

    // Advertising interval.
    //
    // 0xA0 = 100 ms
    // 0xC8 = 125 ms
    bleAdvertising->setMinInterval(0xA0);
    bleAdvertising->setMaxInterval(0xC8);

    // Remote ID is broadcast only.
    bleAdvertising->setScanResponse(false);

    // Non-connectable advertising.
    bleAdvertising->setAdvertisementType(
        BLE_GAP_CONN_MODE_NON
    );

    bleReady = true;

    Serial.println("BLE initialized");

#if ENABLE_BLE_LEGACY
    Serial.println("BLE 4 Remote ID enabled");
#endif

#if ENABLE_BLE_LONG_RANGE
    Serial.println(
        "BLE 5 Long Range requested"
    );

    Serial.println(
        "NOTE: Arduino BLEAdvertising does not expose "
        "LE Coded PHY"
    );
#endif

    return true;
}


// ============================================================================
// Send legacy Remote ID
// ============================================================================

static void bleSendLegacy(
    uint8_t counter,
    const uint8_t msg[25])
{
#if ENABLE_BLE_LEGACY

    if (!bleReady || bleAdvertising == nullptr) {

        Serial.println(
            "BLE not initialized"
        );

        return;
    }

    // Build the 31-byte legacy packet.
    bleBuildLegacy(
        counter,
        msg
    );

    // Create advertising data.
    BLEAdvertisementData advertisementData;

    // Arduino-ESP32 3.3.11 provides:
    //
    //   addData(char *data, size_t length)
    //
    // Use that directly for binary data.
    advertisementData.addData(
        reinterpret_cast<char *>(bleAdv),
        sizeof(bleAdv)
    );

    // Replace existing advertising data.
    bleAdvertising->setAdvertisementData(
        advertisementData
    );

    // Start only once.
    static bool started = false;

    if (!started) {

        if (bleAdvertising->start()) {

            started = true;
            ble5Ready = false;

            Serial.println(
                "BLE 4 Remote ID ON"
            );

        } else {

            Serial.println(
                "BLE 4 Remote ID FAILED"
            );
        }
    }

#else

    (void)counter;
    (void)msg;

#endif
}


// ============================================================================
// Send packed Remote ID data
// ============================================================================
//
// This uses ordinary BLE advertising.
//
// IMPORTANT:
// The Arduino BLEAdvertising API does NOT expose the ESP32-S3 LE Coded PHY
// configuration. Therefore this is NOT true BLE 5 Long Range advertising.
//
// ============================================================================

static void bleSendPacked(
    uint8_t counter,
    const uint8_t *pack,
    size_t packLen)
{
#if ENABLE_BLE_LONG_RANGE

    if (!bleReady || bleAdvertising == nullptr) {

        Serial.println(
            "BLE not initialized"
        );

        return;
    }

    // ------------------------------------------------------------------------
    // Legacy advertising payload is limited to 31 bytes.
    //
    // Flags:
    //   3 bytes
    //
    // Service Data:
    //   1 length
    //   1 AD type
    //   2 UUID
    //   1 message type
    //   1 counter
    //
    // Available for pack:
    //
    //   31 - 3 - 1 - 1 - 2 - 1 - 1 = 22
    // ------------------------------------------------------------------------

    constexpr size_t MAX_PACK = 22;

    if (packLen > MAX_PACK) {

        Serial.printf(
            "Packed BLE payload too large: %u bytes "
            "(maximum %u bytes)\n",
            static_cast<unsigned>(packLen),
            static_cast<unsigned>(MAX_PACK)
        );

        return;
    }


    uint8_t adv[31];

    size_t n = 0;


    // ------------------------------------------------------------------------
    // Flags
    // ------------------------------------------------------------------------

    adv[n++] = 0x02;
    adv[n++] = 0x01;
    adv[n++] = 0x06;


    // ------------------------------------------------------------------------
    // Service Data length
    // ------------------------------------------------------------------------

    adv[n++] =
        static_cast<uint8_t>(
            1 +       // AD type
            2 +       // UUID
            1 +       // message type
            1 +       // counter
            packLen
        );


    // Service Data AD type
    adv[n++] = 0x16;


    // Open Drone ID UUID 0xFFFA
    adv[n++] = 0xFA;
    adv[n++] = 0xFF;


    // Remote ID message type
    adv[n++] = 0x0D;


    // Counter
    adv[n++] = counter;


    // Packed payload
    memcpy(
        adv + n,
        pack,
        packLen
    );

    n += packLen;


    // ------------------------------------------------------------------------
    // Configure advertising data
    // ------------------------------------------------------------------------

    BLEAdvertisementData advertisementData;

    advertisementData.addData(
        reinterpret_cast<char *>(adv),
        n
    );


    bleAdvertising->setAdvertisementData(
        advertisementData
    );


    // ------------------------------------------------------------------------
    // Start advertising
    // ------------------------------------------------------------------------

    static bool started = false;

    if (!started) {

        if (bleAdvertising->start()) {

            started = true;
            ble5Ready = true;

            Serial.println(
                "BLE Remote ID advertising ON"
            );

        } else {

            Serial.println(
                "BLE Remote ID advertising FAILED"
            );
        }
    }


#else

    (void)counter;
    (void)pack;
    (void)packLen;

#endif
}


// ============================================================================
// Status
// ============================================================================

static bool bleIsReady()
{
    return bleReady;
}


static bool ble5IsReady()
{
    return ble5Ready;
}
