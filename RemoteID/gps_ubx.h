#pragma once
#include <Arduino.h>
#include "config.h"
#include "odid.h"

// u-blox UBX-NAV-PVT (class 0x01, id 0x07), 92-byte payload. HGLRC M100 Mini
// (M10) defaults to UBX at 115200.

struct GpsFix {
  bool valid = false;
  uint8_t fixType = 0;
  uint8_t numSV = 0;
  double lat = 0;
  double lon = 0;
  float altEllipsoidM = ODID_INV_ALT;
  float altMslM = ODID_INV_ALT;
  float speedMps = 0;
  float vertMps = 0;
  float headingDeg = ODID_INV_DIR;
  float hAccM = 0;
  float vAccM = 0;
  float sAccMps = 0;
  uint16_t year = 0;
  uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
  uint32_t lastMs = 0;
};

class UbxGps {
 public:
  explicit UbxGps(HardwareSerial &s) : serial_(s) {}

  void begin(uint32_t baud, int rx, int tx, bool configure) {
    serial_.setRxBufferSize(GPS_UART_BUF);
    serial_.setTxBufferSize(GPS_UART_BUF);
    serial_.begin(baud, SERIAL_8N1, rx, tx);
    delay(50);
    if (configure) {
      enableNavPvt();
    }
  }

  void feed(uint8_t b) { parseByte(b); }

  void poll() {
    while (serial_.available()) {
      parseByte((uint8_t)serial_.read());
    }
  }

  const GpsFix &fix() const { return fix_; }

 private:
  HardwareSerial &serial_;
  GpsFix fix_;
  enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAY, S_SKIP, S_CKA, S_CKB } st_ = S_SYNC1;
  uint8_t cls_ = 0, id_ = 0, cka_ = 0, ckb_ = 0;
  uint16_t len_ = 0, idx_ = 0, skip_ = 0;
  uint8_t pay_[128];

  static void checksum(const uint8_t *p, uint16_t n, uint8_t *cka, uint8_t *ckb) {
    uint8_t a = 0, b = 0;
    for (uint16_t i = 0; i < n; i++) {
      a = (uint8_t)(a + p[i]);
      b = (uint8_t)(b + a);
    }
    *cka = a;
    *ckb = b;
  }

  void sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t n) {
    uint8_t hdr[4] = {cls, id, (uint8_t)n, (uint8_t)(n >> 8)};
    uint8_t a, b;
    uint8_t tmp[8 + 32];
    memcpy(tmp, hdr, 4);
    if (n) memcpy(tmp + 4, payload, n);
    checksum(tmp, 4 + n, &a, &b);
    serial_.write((uint8_t)0xB5);
    serial_.write((uint8_t)0x62);
    serial_.write(tmp, 4 + n);
    serial_.write(a);
    serial_.write(b);
  }

  void enableNavPvt() {
    // Ask for NAV-PVT on UART1. Do not change the navigation rate; Betaflight
    // auto-config will set UBX output and 10 Hz on the M10.
    const uint8_t msg[] = {0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    sendUbx(0x06, 0x01, msg, sizeof(msg));
  }

  static int32_t i32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
  }
  static uint32_t u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }
  static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

  void parseByte(uint8_t b) {
    switch (st_) {
      case S_SYNC1:
        if (b == 0xB5) st_ = S_SYNC2;
        break;
      case S_SYNC2:
        st_ = (b == 0x62) ? S_CLASS : S_SYNC1;
        break;
      case S_CLASS:
        cls_ = b;
        cka_ = b;
        ckb_ = b;
        st_ = S_ID;
        break;
      case S_ID:
        id_ = b;
        cka_ = (uint8_t)(cka_ + b);
        ckb_ = (uint8_t)(ckb_ + cka_);
        st_ = S_LEN1;
        break;
      case S_LEN1:
        len_ = b;
        cka_ = (uint8_t)(cka_ + b);
        ckb_ = (uint8_t)(ckb_ + cka_);
        st_ = S_LEN2;
        break;
      case S_LEN2:
        len_ |= (uint16_t)b << 8;
        cka_ = (uint8_t)(cka_ + b);
        ckb_ = (uint8_t)(ckb_ + cka_);
        idx_ = 0;
        if (len_ > sizeof(pay_)) {
          skip_ = len_;
          st_ = S_SKIP;
        } else if (len_ == 0) {
          st_ = S_CKA;
        } else {
          st_ = S_PAY;
        }
        break;
      case S_SKIP:
        if (skip_ > 0) skip_--;
        if (skip_ == 0) st_ = S_CKA;
        break;
      case S_PAY:
        pay_[idx_++] = b;
        cka_ = (uint8_t)(cka_ + b);
        ckb_ = (uint8_t)(ckb_ + cka_);
        if (idx_ >= len_) st_ = S_CKA;
        break;
      case S_CKA:
        st_ = (b == cka_) ? S_CKB : S_SYNC1;
        break;
      case S_CKB:
        if (b == ckb_) handleMsg();
        st_ = S_SYNC1;
        break;
    }
  }

  void handleMsg() {
    if (cls_ != 0x01 || id_ != 0x07 || len_ < 92) return;
    const uint8_t *p = pay_;
    GpsFix f;
    f.year = u16(p + 4);
    f.month = p[6];
    f.day = p[7];
    f.hour = p[8];
    f.minute = p[9];
    f.second = p[10];
    f.fixType = p[20];
    uint8_t flags = p[21];
    f.numSV = p[23];
    f.lon = i32(p + 24) * 1e-7;
    f.lat = i32(p + 28) * 1e-7;
    f.altEllipsoidM = i32(p + 32) * 0.001f;
    f.altMslM = i32(p + 36) * 0.001f;
    f.hAccM = u32(p + 40) * 0.001f;
    f.vAccM = u32(p + 44) * 0.001f;
    f.vertMps = -i32(p + 56) * 0.001f;  // NED down -> up
    f.speedMps = i32(p + 60) * 0.001f;
    f.headingDeg = i32(p + 64) * 1e-5f;
    if (f.headingDeg < 0) f.headingDeg += 360.0f;
    f.sAccMps = u32(p + 68) * 0.001f;
    f.valid = (f.fixType >= 2) && (flags & 0x01) && (fabs(f.lat) > 1e-8 || fabs(f.lon) > 1e-8);
    f.lastMs = millis();
    fix_ = f;
  }
};
