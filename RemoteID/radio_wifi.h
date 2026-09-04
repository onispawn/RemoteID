#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "config.h"
#include "odid.h"

static uint8_t wifiMac[6];
static uint8_t wifiCtr = 0;
static bool wifiReady = false;

static size_t wifiFillPack(uint8_t *pack, const uint8_t msgs[][25], uint8_t count) {
  return odidEncodePack(pack, msgs, count);
}

static size_t wifiBuildBeaconFrame(uint8_t *buf, size_t cap, const uint8_t mac[6],
                                   const char *ssid, uint8_t counter,
                                   const uint8_t *pack, size_t packLen) {
  if (cap < 80 + packLen) return 0;
  size_t n = 0;
  buf[n++] = 0x80;
  buf[n++] = 0x00;
  buf[n++] = 0x00;
  buf[n++] = 0x00;
  memset(buf + n, 0xFF, 6);
  n += 6;
  memcpy(buf + n, mac, 6);
  n += 6;
  memcpy(buf + n, mac, 6);
  n += 6;
  buf[n++] = 0x00;
  buf[n++] = 0x00;

  uint64_t ts = micros();
  memcpy(buf + n, &ts, 8);
  n += 8;
  buf[n++] = 100;
  buf[n++] = 0x00;
  buf[n++] = 0x20;
  buf[n++] = 0x04;

  uint8_t slen = (uint8_t)strlen(ssid);
  if (slen > 32) slen = 32;
  buf[n++] = 0x00;
  buf[n++] = slen;
  memcpy(buf + n, ssid, slen);
  n += slen;

  buf[n++] = 0x01;
  buf[n++] = 0x01;
  buf[n++] = 0x8C;

  size_t vend = n;
  buf[n++] = 0xDD;
  buf[n++] = 0;
  buf[n++] = 0xFA;
  buf[n++] = 0x0B;
  buf[n++] = 0xBC;
  buf[n++] = 0x0D;
  buf[n++] = counter;
  memcpy(buf + n, pack, packLen);
  n += packLen;
  buf[vend + 1] = (uint8_t)(4 + 1 + packLen);
  return n;
}

static void wifiSetVendorIe(const uint8_t *payload, uint8_t payloadLen) {
  uint8_t raw[8 + 160];
  memset(raw, 0, sizeof(raw));
  vendor_ie_data_t *ie = (vendor_ie_data_t *)raw;
  ie->element_id = WIFI_VENDOR_IE_ELEMENT_ID;
  ie->length = (uint8_t)(4 + payloadLen);
  ie->vendor_oui[0] = 0xFA;
  ie->vendor_oui[1] = 0x0B;
  ie->vendor_oui[2] = 0xBC;
  ie->vendor_oui_type = 0x0D;
  memcpy(ie->payload, payload, payloadLen);

  esp_wifi_set_vendor_ie(false, WIFI_VND_IE_TYPE_BEACON, WIFI_VND_IE_ID_0, nullptr);
  esp_wifi_set_vendor_ie(true, WIFI_VND_IE_TYPE_BEACON, WIFI_VND_IE_ID_0, ie);
  esp_wifi_set_vendor_ie(false, WIFI_VND_IE_TYPE_PROBE_RESP, WIFI_VND_IE_ID_0, nullptr);
  esp_wifi_set_vendor_ie(true, WIFI_VND_IE_TYPE_PROBE_RESP, WIFI_VND_IE_ID_0, ie);
}

static bool wifiStart() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(WIFI_SSID, nullptr, WIFI_CHANNEL, 0, 4);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(84);
  WiFi.softAPmacAddress(wifiMac);
  wifiReady = ok;
  Serial.printf("Wi-Fi Beacon RID %s  SSID=%s ch=%u mac=%02X:%02X:%02X:%02X:%02X:%02X\n",
                ok ? "ON" : "FAIL", WIFI_SSID, WIFI_CHANNEL,
                wifiMac[0], wifiMac[1], wifiMac[2], wifiMac[3], wifiMac[4], wifiMac[5]);
  return ok;
}

static void wifiBroadcast(const uint8_t msgs[][25], uint8_t count) {
  if (!wifiReady) return;
  uint8_t pack[3 + 5 * 25];
  size_t packLen = wifiFillPack(pack, msgs, count);
  uint8_t payload[1 + 3 + 5 * 25];
  payload[0] = wifiCtr;
  memcpy(payload + 1, pack, packLen);
  wifiSetVendorIe(payload, (uint8_t)(1 + packLen));

  uint8_t frame[256];
  size_t flen = wifiBuildBeaconFrame(frame, sizeof(frame), wifiMac, WIFI_SSID, wifiCtr, pack, packLen);
  if (flen) {
    esp_wifi_80211_tx(WIFI_IF_AP, frame, (int)flen, true);
  }
  wifiCtr++;
}
