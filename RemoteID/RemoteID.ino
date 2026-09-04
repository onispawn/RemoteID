#include <Arduino.h>

#include "config.h"
#include "odid.h"
#include "gps_ubx.h"
#include "radio_wifi.h"
#include "radio_ble.h"
static void encodeStaticMessages();
#include "config_web.h"

static HardwareSerial GpsSerial(GPS_UART_NUM);
static UbxGps gps(GpsSerial);
#if ENABLE_FC_PASSTHROUGH
static HardwareSerial FcSerial(FC_UART_NUM);
#endif

static uint8_t msgBasic[25];
static uint8_t msgLocation[25];
static uint8_t msgSelf[25];
static uint8_t msgSystem[25];
static uint8_t msgOperator[25];

static uint8_t ctrBasic = 0;
static uint8_t ctrLocation = 0;
static uint8_t ctrSelf = 0;
static uint8_t ctrSystem = 0;
static uint8_t ctrOperator = 0;
static uint8_t ctrPack = 0;

static void copyMsgSet(uint8_t msgs[5][25]) {
  memcpy(msgs[0], msgBasic, 25);
  memcpy(msgs[1], msgLocation, 25);
  memcpy(msgs[2], msgSystem, 25);
  memcpy(msgs[3], msgSelf, 25);
  memcpy(msgs[4], msgOperator, 25);
}

static bool takeoffSet = false;
static double takeoffLat = 0;
static double takeoffLon = 0;
static float takeoffAltGeo = 0;
static uint32_t unix2019 = 0;

static const uint32_t UNIX_2019_01_01 = 1546300800UL;

static void ledWrite(bool on) {
  digitalWrite(USER_LED_PIN, LED_ACTIVE_LOW ? !on : on);
}

static uint32_t utcUnix(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t s) {
  static const uint16_t mdays[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  if (y < 1970 || mo < 1 || mo > 12 || d < 1) return 0;
  uint32_t days = (uint32_t)(y - 1970) * 365u + (uint32_t)((y - 1969) / 4);
  days -= (uint32_t)((y - 1901) / 100);
  days += (uint32_t)((y - 1601) / 400);
  days += mdays[mo - 1];
  bool leap = ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
  if (mo > 2 && leap) days++;
  days += (uint8_t)(d - 1);
  return days * 86400u + h * 3600u + mi * 60u + s;
}

static void encodeStaticMessages() {
  odidEncodeBasicId(msgBasic, UA_TYPE, ODID_ID_SERIAL, gUasSerial);
  odidEncodeSelfId(msgSelf, 0, SELF_ID_TEXT);
  odidEncodeOperatorId(msgOperator, 0, gOperatorId);
}

static void encodeDynamic(const GpsFix &f, bool fresh) {
  OdidLocation loc{};
  loc.altBaro = ODID_INV_ALT;
  loc.baroAcc = 0;
  loc.heightType = ODID_HEIGHT_OVER_TAKEOFF;
  loc.tsAcc = 2;

  if (!fresh || !f.valid) {
    loc.status = ODID_STATUS_UNDECLARED;
    loc.direction = ODID_INV_DIR;
    loc.speedH = ODID_INV_HSPEED;
    loc.speedV = ODID_INV_VSPEED;
    loc.lat = 0;
    loc.lon = 0;
    loc.altGeo = ODID_INV_ALT;
    loc.height = ODID_INV_ALT;
    loc.horizAcc = 0;
    loc.vertAcc = 0;
    loc.speedAcc = 0;
    loc.timestamp = ODID_INV_TS;
  } else {
    if (!takeoffSet && f.fixType >= 3 && f.numSV >= TAKEOFF_MIN_SATS) {
      takeoffLat = f.lat;
      takeoffLon = f.lon;
      takeoffAltGeo = f.altEllipsoidM;
      takeoffSet = true;
    }

    float agl = takeoffSet ? (f.altEllipsoidM - takeoffAltGeo) : 0;
    bool airborne = takeoffSet && (agl > AIRBORNE_AGL_M || f.speedMps > AIRBORNE_SPEED_MPS);
    loc.status = airborne ? ODID_STATUS_AIRBORNE : ODID_STATUS_GROUND;
    loc.direction = (f.speedMps > 0.3f) ? f.headingDeg : ODID_INV_DIR;
    loc.speedH = f.speedMps;
    loc.speedV = f.vertMps;
    loc.lat = f.lat;
    loc.lon = f.lon;
    loc.altGeo = f.altEllipsoidM;
    loc.height = takeoffSet ? agl : ODID_INV_ALT;
    loc.horizAcc = odidHorizAcc(f.hAccM);
    loc.vertAcc = odidVertAcc(f.vAccM);
    loc.speedAcc = odidSpeedAcc(f.sAccMps);
    loc.timestamp = f.minute * 60.0f + f.second;
    uint32_t unix = utcUnix(f.year, f.month, f.day, f.hour, f.minute, f.second);
    if (unix > UNIX_2019_01_01) unix2019 = unix - UNIX_2019_01_01;
  }
  odidEncodeLocation(msgLocation, loc);

  OdidSystem sys{};
  sys.opLocType = ODID_OPLOC_TAKEOFF;
  sys.classType = 0;
  sys.areaCount = 1;
  sys.areaRadiusM = 0;
  sys.areaCeil = ODID_INV_ALT;
  sys.areaFloor = ODID_INV_ALT;
  sys.catEU = 0;
  sys.classEU = 0;
  sys.timestamp2019 = unix2019;
  if (takeoffSet) {
    sys.opLat = takeoffLat;
    sys.opLon = takeoffLon;
    sys.opAltGeo = takeoffAltGeo;
  } else {
    sys.opLat = 0;
    sys.opLon = 0;
    sys.opAltGeo = ODID_INV_ALT;
  }
  odidEncodeSystem(msgSystem, sys);
}

// BLE 4/5 and Wi-Fi Beacon are in radio_ble.h / radio_wifi.h

static void updateLed(const GpsFix &f, bool fresh) {
  uint32_t t = millis();
  if (configModeActive()) {
    ledWrite((t / 200) % 2);
    return;
  }
  if (!fresh || !f.valid) {
    ledWrite((t / 120) % 2);
  } else if (f.fixType < 3) {
    ledWrite((t / 400) % 2);
  } else {
    ledWrite(true);
  }
}

void setup() {
  pinMode(USER_LED_PIN, OUTPUT);
  pinMode(CONFIG_BTN_PIN, INPUT_PULLUP);
  ledWrite(false);

  Serial.begin(115200);
  delay(200);
  identityLoad();
  Serial.println("XIAO ESP32S3 ASTM F3411-22a Remote ID");
  Serial.printf("UAS serial: %s  operator: %s\n", gUasSerial, gOperatorId);
  Serial.println("Press BOOT (or config button) to open the ID setup page");

  encodeStaticMessages();
  encodeDynamic(GpsFix{}, false);

  gps.begin(GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN, true);
#if ENABLE_FC_PASSTHROUGH
  FcSerial.setRxBufferSize(FC_UART_BUF);
  FcSerial.setTxBufferSize(FC_UART_BUF);
  FcSerial.begin(GPS_BAUD, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  Serial.println("UBX passthrough: GPS UART1 <-> FC UART2 @ 115200");
#endif

  // Wi-Fi first, then BLE — required for ESP32-S3 coexistence.
#if ENABLE_WIFI_BEACON
  wifiStart();
  uint8_t bootMsgs[5][25];
  copyMsgSet(bootMsgs);
  wifiBroadcast(bootMsgs, 5);
#endif
  bleStart();
  delay(100);
  bleSendLegacy(++ctrBasic, msgBasic);
  {
    uint8_t pack[3 + 5 * 25];
    uint8_t bootMsgs[5][25];
    copyMsgSet(bootMsgs);
    size_t n = odidEncodePack(pack, bootMsgs, 5);
    bleSendPacked(++ctrPack, pack, n);
  }
}

#if ENABLE_FC_PASSTHROUGH
static void drainSerial(HardwareSerial &from, HardwareSerial &to, bool sniffGps) {
  uint8_t buf[128];
  size_t n = 0;
  while (from.available() && n < sizeof(buf)) {
    int c = from.read();
    if (c < 0) break;
    buf[n++] = (uint8_t)c;
  }
  if (!n) return;
  to.write(buf, n);
  if (sniffGps) {
    for (size_t i = 0; i < n; i++) {
      gps.feed(buf[i]);
    }
  }
}

static void bridgeUbx() {
  drainSerial(GpsSerial, FcSerial, true);
  drainSerial(FcSerial, GpsSerial, false);
}
#endif

void loop() {
  configPoll();
#if ENABLE_FC_PASSTHROUGH
  bridgeUbx();
#else
  gps.poll();
#endif
  const GpsFix &f = gps.fix();
  bool fresh = f.lastMs && (millis() - f.lastMs) < GPS_STALE_MS;

  static uint32_t lastEnc = 0;
  if (millis() - lastEnc >= 100) {
    lastEnc = millis();
    encodeDynamic(f, fresh);
  }

  static uint32_t lastBle = 0;
  static uint8_t slot = 0;
  if (millis() - lastBle >= BLE_ROTATE_MS) {
    lastBle = millis();
    // Location at least ~3 Hz; other types rotate around it (ASTM BT4).
    switch (slot++ % 8) {
      case 0:
      case 2:
      case 4:
      case 6:
        bleSendLegacy(++ctrLocation, msgLocation);
        break;
      case 1:
        bleSendLegacy(++ctrBasic, msgBasic);
        break;
      case 3:
        bleSendLegacy(++ctrSystem, msgSystem);
        break;
      case 5:
        bleSendLegacy(++ctrSelf, msgSelf);
        break;
      default:
        bleSendLegacy(++ctrOperator, msgOperator);
        break;
    }
    uint8_t pack[3 + 5 * 25];
    uint8_t msgs[5][25];
    copyMsgSet(msgs);
    size_t n = odidEncodePack(pack, msgs, 5);
    bleSendPacked(++ctrPack, pack, n);
  }

#if ENABLE_WIFI_BEACON
  static uint32_t lastWifi = 0;
  if (millis() - lastWifi >= WIFI_BEACON_MS) {
    lastWifi = millis();
    uint8_t msgs[5][25];
    copyMsgSet(msgs);
    wifiBroadcast(msgs, 5);
  }
#endif

  static uint32_t lastLog = 0;
  if (millis() - lastLog >= 2000) {
    lastLog = millis();
    if (!fresh || !f.valid) {
      Serial.println("GPS: no fix (place antenna skyward, outdoors)");
    } else {
      Serial.printf("GPS 3D=%u sats=%u lat=%.6f lon=%.6f alt=%.1fm spd=%.1fm/s takeoff=%u\n",
                    f.fixType >= 3, f.numSV, f.lat, f.lon, f.altEllipsoidM, f.speedMps, takeoffSet);
    }
  }

  updateLed(f, fresh);
#if ENABLE_FC_PASSTHROUGH
  bridgeUbx();
#endif
}
