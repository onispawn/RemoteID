# XIAO ESP32S3 + HGLRC M100 Mini — ASTM F3411 Remote ID

Arduino firmware that broadcasts **ASTM F3411-22a (OpenDroneID)** over:

- **Bluetooth 4** legacy advertising (service UUID `0xFFFA`)
- **Bluetooth 5** Long Range (LE Coded PHY) message packs, when the Arduino core supports it
- **Wi-Fi Beacon** vendor IEs (OUI `FA:0B:BC`) plus injected beacon frames

That is the radio protocol used for FAA Broadcast Remote ID. Wi-Fi is started before Bluetooth so both can share the ESP32-S3 radio.

## Wiring

| HGLRC M100 Mini | XIAO ESP32S3 | Notes |
| --- | --- | --- |
| VCC | 5V | Module is 3.3–5 V. 5V from USB-C is preferred. |
| GND | GND | Common ground required |
| TX | D1 (GPIO2) | GPS data out → ESP UART RX |
| RX | D2 (GPIO3) | ESP UART TX → GPS config in |

| Betaflight GPS UART | XIAO ESP32S3 | Notes |
| --- | --- | --- |
| GND | GND | Common ground with the FC |
| TX | D3 (GPIO4) | FC UBX config → GPS (via ESP) |
| RX | D4 (GPIO5) | GPS UBX stream → FC (via ESP) |
| 5V | — | Do not also feed USB 5V into the XIAO if the FC already powers the 5V pad |

UART: **115200 8N1**, 3.3 V logic. Do **not** use D6/D7 (GPIO43/44); those pins fight USB-CDC on the S3.

The ESP32 is a full-duplex **u-blox (UBX) passthrough**: every byte from the M100 Mini is forwarded to Betaflight, and every byte from Betaflight (auto-config, polls, ACKs) is forwarded to the GPS. Remote ID sniffs `NAV-PVT` on the GPS→FC path and does not rewrite the protocol.

```
M100 Mini TX ──► XIAO D1 ──┬──► parse NAV-PVT (Remote ID)
                           └──► XIAO D4 ──► Betaflight RX
M100 Mini RX ◄── XIAO D2 ◄────── XIAO D3 ◄── Betaflight TX
```

Mount the GPS ceramic antenna facing the sky, away from ESCs and the ESP32 antenna.

Power the XIAO from USB-C (bench) or a 5 V BEC / FC 5V into the 5V pad (aircraft). The GPS draws about 40 mA. Use one 5 V source at a time.

## Betaflight

Use firmware **4.3 or later** (M10). On the GPS UART:

1. **Ports**: set that UART to **GPS**, baud **115200**.
2. **Configuration**: enable **GPS**, provider **UBLOX**, **Auto config** on. Ground assistance / Galileo as you prefer.
3. Save and reboot. The GPS icon should light once the M100 has a lock; CLI `status` should show a fix.

If the icon never comes on: confirm TX/RX are not swapped, common GND, and that the UART is not also assigned to Serial RX or MSP.

## Arduino IDE setup

1. Install **esp32** by Espressif (`https://espressif.github.io/arduino-esp32/package_esp32_index.json`).
2. Board: **XIAO_ESP32S3** (Seeed) or **ESP32S3 Dev Module**.
3. Tools: **USB CDC On Boot = Enabled**, **Bluetooth = Enabled**, **PSRAM = Disabled** (or OPI on Sense), **Flash Size = 8MB**.
4. Open `RemoteID/RemoteID.ino`.
5. Edit `config.h`:
   - `UAS_SERIAL` — ANSI/CTA-2063-A serial of the **aircraft** (required for FAA identity).
   - `OPERATOR_ID` — your FAA registration number (e.g. `FA1234567`).
6. Upload, then open Serial Monitor at 115200. You should see `Wi-Fi Beacon RID ON`, `Bluetooth controller ON`, then `BLE 4 Remote ID ON`. BLE 5 Long Range is also started when the Arduino core supports it.

## Set serial / FAA ID (config button)

Press **BOOT** on the XIAO after it has already started (do not hold BOOT while resetting — that is flash mode). An extra momentary button from **GPIO0** (or `CONFIG_BTN_PIN`) to GND works the same.

The LED blinks slowly while the setup site is up (5 minutes). On your phone:

1. Join Wi-Fi **OpenDroneID** (no password).
2. The captive-portal page should open. If it does not, go to **http://192.168.4.1**.
3. Enter aircraft **serial ID** (CTA-2063-A) and **FAA operator / pilot ID**, then Save.

Values are stored in flash and survive reboot. `config.h` is only the factory default until you save from the page.

On-board LED: fast blink = no GPS, slow blink = 2D, solid = 3D fix and broadcasting.

## Check the broadcast

Outdoors with a GPS lock, use **OpenDroneID OSM** or **Drone Scanner** on Android. Enable both Bluetooth and Wi-Fi scanning. You should see Basic ID, Location, System, and Operator ID. The board also raises a Wi-Fi AP named `OpenDroneID` (the RID payload is in the beacon IE, not the SSID).

## What this implements (14 CFR 89.305)

| Required field | Source |
| --- | --- |
| UA unique ID | `UAS_SERIAL` (Basic ID, serial number type) |
| UA lat / lon / geometric altitude | u-blox NAV-PVT |
| UA velocity | NAV-PVT ground speed + vertical speed |
| Control station location | First solid 3D fix (takeoff location) |
| Time mark | GPS UTC, tenths of a second past the hour |
| Emergency status | Encoded; remains normal unless you extend the sketch |

Location is advertised several times per second on BLE 4; Basic ID / System / Self ID / Operator ID rotate around it.

## Compliance (read this)

This sketch speaks the **same OpenDroneID encoding** FAA Broadcast Remote ID uses. It is **not** by itself an FAA-listed production Remote ID module.

- Legal US operations generally need a **standard Remote ID drone** or an **FAA-listed broadcast module**, plus aircraft registration.
- A homemade ESP32 is appropriate for development, ground tests, and learning the protocol. Filing a manufacturer declaration (and meeting F3586 / performance rules, including range) is a separate process.
- `UAS_SERIAL` must be the real aircraft serial, not a placeholder, if you rely on this identity in the field.
- This board does not include a second GNSS for a live operator/control-station position. FAA allows broadcasting **takeoff location** when a live CS position is not available; that is what the System message uses after the first good 3D fix.

This is not legal advice.

## Hardware notes

- **HGLRC M100 Mini**: u-blox M10, default **UBX @ 115200**, 10 Hz. This firmware requests `NAV-PVT` for Remote ID and otherwise leaves rate/config to Betaflight auto-config through the UART bridge.
- **XIAO ESP32S3**: onboard 2.4 GHz antenna. Range is typically hundreds of meters, not the multi-kilometer figures of listed RID modules with an external antenna. Keep the USB-C / antenna end unobstructed.
