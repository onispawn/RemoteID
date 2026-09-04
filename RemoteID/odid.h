#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>

// ASTM F3411-22a / OpenDroneID packed 25-byte messages (protocol version 2).
// Byte layout matches opendroneid-core-c (Apache-2.0, Intel).

static const uint8_t ODID_MSG_SIZE = 25;
static const uint8_t ODID_PROTO = 2;
static const float ODID_INV_DIR = 361.0f;
static const float ODID_INV_HSPEED = 255.0f;
static const float ODID_INV_VSPEED = 63.0f;
static const float ODID_INV_ALT = -1000.0f;
static const uint16_t ODID_INV_TS = 0xFFFF;

enum {
  ODID_BASIC_ID = 0,
  ODID_LOCATION = 1,
  ODID_SELF_ID = 3,
  ODID_SYSTEM = 4,
  ODID_OPERATOR_ID = 5,
  ODID_PACKED = 0xF
};

enum {
  ODID_ID_SERIAL = 1,
  ODID_STATUS_UNDECLARED = 0,
  ODID_STATUS_GROUND = 1,
  ODID_STATUS_AIRBORNE = 2,
  ODID_STATUS_EMERGENCY = 3,
  ODID_HEIGHT_OVER_TAKEOFF = 0,
  ODID_OPLOC_TAKEOFF = 0
};

static inline void odidPutI32(uint8_t *p, int32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static inline void odidPutU16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static inline void odidPutU32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static inline int32_t odidLatLon(double deg) {
  long v = lround(deg * 10000000.0);
  if (v > 1800000000L) v = 1800000000L;
  if (v < -1800000000L) v = -1800000000L;
  return (int32_t)v;
}

static inline uint16_t odidAlt(float m) {
  long v = lround((m + 1000.0f) / 0.5f);
  if (v < 0) v = 0;
  if (v > 65535) v = 65535;
  return (uint16_t)v;
}

static inline uint8_t odidDirection(float deg, uint8_t *ew) {
  unsigned d = (unsigned)lroundf(deg);
  if (d == 360) d = 0;
  if (d < 180) {
    *ew = 0;
    return (uint8_t)d;
  }
  *ew = 1;
  return (uint8_t)(d - 180);
}

static inline uint8_t odidHSpeed(float mps, uint8_t *mult) {
  if (mps <= 255.0f * 0.25f) {
    *mult = 0;
    return (uint8_t)lroundf(mps / 0.25f);
  }
  *mult = 1;
  long v = lroundf((mps - 255.0f * 0.25f) / 0.75f);
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

static inline int8_t odidVSpeed(float mps) {
  long v = lroundf(mps / 0.5f);
  if (v > 127) v = 127;
  if (v < -128) v = -128;
  return (int8_t)v;
}

static inline uint16_t odidTimestamp(float secPastHour) {
  if (secPastHour < 0) return ODID_INV_TS;
  long v = lroundf(secPastHour * 10.0f);
  if (v > 36000) return ODID_INV_TS;
  if (v < 0) v = 0;
  return (uint16_t)v;
}

static inline uint8_t odidHorizAcc(float meters) {
  if (meters <= 0) return 0;
  if (meters <= 1.0f) return 12;
  if (meters <= 3.0f) return 11;
  if (meters <= 10.0f) return 10;
  if (meters <= 30.0f) return 9;
  if (meters <= 92.6f) return 8;
  if (meters <= 185.2f) return 7;
  return 6;
}

static inline uint8_t odidVertAcc(float meters) {
  if (meters <= 0) return 0;
  if (meters <= 1.0f) return 6;
  if (meters <= 3.0f) return 5;
  if (meters <= 10.0f) return 4;
  if (meters <= 25.0f) return 3;
  if (meters <= 45.0f) return 2;
  if (meters <= 150.0f) return 1;
  return 0;
}

static inline uint8_t odidSpeedAcc(float mps) {
  if (mps <= 0) return 0;
  if (mps <= 0.3f) return 4;
  if (mps <= 1.0f) return 3;
  if (mps <= 3.0f) return 2;
  if (mps <= 10.0f) return 1;
  return 0;
}

static inline void odidEncodeBasicId(uint8_t out[25], uint8_t uaType, uint8_t idType, const char *uasId) {
  memset(out, 0, 25);
  out[0] = (uint8_t)((ODID_BASIC_ID << 4) | ODID_PROTO);
  out[1] = (uint8_t)((idType << 4) | (uaType & 0x0F));
  strncpy((char *)(out + 2), uasId, 20);
}

static inline void odidEncodeSelfId(uint8_t out[25], uint8_t descType, const char *text) {
  memset(out, 0, 25);
  out[0] = (uint8_t)((ODID_SELF_ID << 4) | ODID_PROTO);
  out[1] = descType;
  strncpy((char *)(out + 2), text, 23);
}

static inline void odidEncodeOperatorId(uint8_t out[25], uint8_t opType, const char *opId) {
  memset(out, 0, 25);
  out[0] = (uint8_t)((ODID_OPERATOR_ID << 4) | ODID_PROTO);
  out[1] = opType;
  strncpy((char *)(out + 2), opId, 20);
}

struct OdidLocation {
  uint8_t status;
  float direction;
  float speedH;
  float speedV;
  double lat;
  double lon;
  float altBaro;
  float altGeo;
  uint8_t heightType;
  float height;
  uint8_t horizAcc;
  uint8_t vertAcc;
  uint8_t baroAcc;
  uint8_t speedAcc;
  uint8_t tsAcc;
  float timestamp;
};

static inline void odidEncodeLocation(uint8_t out[25], const OdidLocation &d) {
  memset(out, 0, 25);
  uint8_t ew = 0, mult = 0;
  float dirIn = d.direction;
  if (dirIn < 0 || dirIn > 361.0f) dirIn = 361.0f;
  uint8_t dir = odidDirection(dirIn, &ew);
  uint8_t hs = odidHSpeed(d.speedH, &mult);
  int8_t vs = odidVSpeed(d.speedV);

  out[0] = (uint8_t)((ODID_LOCATION << 4) | ODID_PROTO);
  out[1] = (uint8_t)((d.status << 4) | ((d.heightType & 1) << 2) | (ew << 1) | (mult & 1));
  out[2] = dir;
  out[3] = hs;
  out[4] = (uint8_t)vs;
  odidPutI32(out + 5, odidLatLon(d.lat));
  odidPutI32(out + 9, odidLatLon(d.lon));
  odidPutU16(out + 13, odidAlt(d.altBaro));
  odidPutU16(out + 15, odidAlt(d.altGeo));
  odidPutU16(out + 17, odidAlt(d.height));
  out[19] = (uint8_t)((d.vertAcc << 4) | (d.horizAcc & 0x0F));
  out[20] = (uint8_t)((d.baroAcc << 4) | (d.speedAcc & 0x0F));
  odidPutU16(out + 21, odidTimestamp(d.timestamp));
  out[23] = (uint8_t)(d.tsAcc & 0x0F);
  out[24] = 0;
}

struct OdidSystem {
  uint8_t opLocType;
  uint8_t classType;
  double opLat;
  double opLon;
  uint16_t areaCount;
  uint16_t areaRadiusM;
  float areaCeil;
  float areaFloor;
  uint8_t catEU;
  uint8_t classEU;
  float opAltGeo;
  uint32_t timestamp2019;
};

static inline void odidEncodeSystem(uint8_t out[25], const OdidSystem &d) {
  memset(out, 0, 25);
  out[0] = (uint8_t)((ODID_SYSTEM << 4) | ODID_PROTO);
  out[1] = (uint8_t)(((d.classType & 7) << 2) | (d.opLocType & 3));
  odidPutI32(out + 2, odidLatLon(d.opLat));
  odidPutI32(out + 6, odidLatLon(d.opLon));
  odidPutU16(out + 10, d.areaCount);
  uint16_t r = d.areaRadiusM / 10;
  if (r > 255) r = 255;
  out[12] = (uint8_t)r;
  odidPutU16(out + 13, odidAlt(d.areaCeil));
  odidPutU16(out + 15, odidAlt(d.areaFloor));
  out[17] = (uint8_t)((d.catEU << 4) | (d.classEU & 0x0F));
  odidPutU16(out + 18, odidAlt(d.opAltGeo));
  odidPutU32(out + 20, d.timestamp2019);
  out[24] = 0;
}

// Message pack used by Wi-Fi Beacon: header (3) + N * 25
static inline size_t odidEncodePack(uint8_t *out, const uint8_t msgs[][25], uint8_t count) {
  out[0] = (uint8_t)((ODID_PACKED << 4) | ODID_PROTO);
  out[1] = ODID_MSG_SIZE;
  out[2] = count;
  for (uint8_t i = 0; i < count; i++) {
    memcpy(out + 3 + (size_t)i * 25, msgs[i], 25);
  }
  return 3 + (size_t)count * 25;
}
