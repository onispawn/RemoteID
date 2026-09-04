#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WiFi.h>
#include "config.h"

static char gUasSerial[21];
static char gOperatorId[21];

static Preferences ridPrefs;
static WebServer ridWeb(80);
static DNSServer ridDns;
static bool configOpen = false;
static uint32_t configUntil = 0;
static uint32_t btnLastMs = 0;
static bool btnWasDown = false;

static void identityLoad() {
  strncpy(gUasSerial, UAS_SERIAL, 20);
  gUasSerial[20] = 0;
  strncpy(gOperatorId, OPERATOR_ID, 20);
  gOperatorId[20] = 0;
  if (!ridPrefs.begin("rid", true)) return;
  String uas = ridPrefs.getString("uas", "");
  String op = ridPrefs.getString("op", "");
  ridPrefs.end();
  if (uas.length() > 0) {
    strncpy(gUasSerial, uas.c_str(), 20);
    gUasSerial[20] = 0;
  }
  if (op.length() > 0) {
    strncpy(gOperatorId, op.c_str(), 20);
    gOperatorId[20] = 0;
  }
}

static void identitySave() {
  if (!ridPrefs.begin("rid", false)) return;
  ridPrefs.putString("uas", gUasSerial);
  ridPrefs.putString("op", gOperatorId);
  ridPrefs.end();
}

static void clipId(char *dst, size_t dstLen, const String &src) {
  size_t j = 0;
  for (size_t i = 0; i < src.length() && j + 1 < dstLen; i++) {
    char c = src[i];
    if (c >= 33 && c <= 126 && c != '<' && c != '>' && c != '"' && c != '&') {
      dst[j++] = c;
    }
  }
  dst[j] = 0;
}

static String htmlEscape(const char *s) {
  String o;
  for (; *s; s++) {
    if (*s == '&') o += F("&amp;");
    else if (*s == '<') o += F("&lt;");
    else if (*s == '>') o += F("&gt;");
    else if (*s == '"') o += F("&quot;");
    else o += *s;
  }
  return o;
}

static void sendConfigPage(bool saved) {
  String html;
  html.reserve(1800);
  html += F(
      "<!DOCTYPE html><html><head><meta charset=utf-8>"
      "<meta name=viewport content='width=device-width,initial-scale=1'>"
      "<title>Remote ID setup</title><style>"
      "body{font-family:sans-serif;max-width:28rem;margin:1.5rem auto;padding:0 1rem;background:#111;color:#eee}"
      "label{display:block;margin:.8rem 0 .2rem}"
      "input{width:100%;box-sizing:border-box;padding:.6rem;font-size:1rem;border-radius:.4rem;border:1px solid #555;background:#222;color:#eee}"
      "button{margin-top:1rem;width:100%;padding:.8rem;font-size:1.05rem;border:0;border-radius:.4rem;background:#3b82f6;color:#fff}"
      "p.note{color:#aaa;font-size:.9rem;line-height:1.4}"
      ".ok{background:#14532d;padding:.7rem;border-radius:.4rem}"
      "</style></head><body><h1>Remote ID</h1>");
  if (saved) html += F("<p class=ok>Saved. Broadcasting the new IDs now.</p>");
  html += F("<p class=note>Aircraft serial is ANSI/CTA-2063-A (max 20 characters). "
            "FAA pilot / registration number goes in Operator ID (for example FA1234567).</p>"
            "<form method=POST action=/save>");
  html += F("<label>Aircraft serial ID</label><input name=uas maxlength=20 value=\"");
  html += htmlEscape(gUasSerial);
  html += F("\"><label>FAA operator / pilot ID</label><input name=op maxlength=20 value=\"");
  html += htmlEscape(gOperatorId);
  html += F("\"><button type=submit>Save</button></form>"
            "<p class=note>This page: http://192.168.4.1 &nbsp; Wi-Fi SSID: ");
  html += WIFI_SSID;
  html += F("</p></body></html>");
  ridWeb.send(200, "text/html", html);
}

static void handleRoot() { sendConfigPage(false); }

static void handleSave() {
  char prevUas[21], prevOp[21];
  strncpy(prevUas, gUasSerial, 21);
  strncpy(prevOp, gOperatorId, 21);
  if (ridWeb.hasArg("uas")) clipId(gUasSerial, sizeof(gUasSerial), ridWeb.arg("uas"));
  if (ridWeb.hasArg("op")) clipId(gOperatorId, sizeof(gOperatorId), ridWeb.arg("op"));
  if (gUasSerial[0] == 0) strncpy(gUasSerial, prevUas, 21);
  if (gOperatorId[0] == 0) strncpy(gOperatorId, prevOp, 21);
  gUasSerial[20] = 0;
  gOperatorId[20] = 0;
  identitySave();
  encodeStaticMessages();
  Serial.printf("Saved IDs  uas=%s  op=%s\n", gUasSerial, gOperatorId);
  sendConfigPage(true);
}

static void configStart() {
  if (!wifiReady) wifiStart();
  if (!configOpen) {
    ridDns.start(53, "*", WiFi.softAPIP());
    ridWeb.on("/", HTTP_GET, handleRoot);
    ridWeb.on("/save", HTTP_POST, handleSave);
    ridWeb.on("/generate_204", HTTP_GET, handleRoot);
    ridWeb.on("/hotspot-detect.html", HTTP_GET, handleRoot);
    ridWeb.on("/ncsi.txt", HTTP_GET, handleRoot);
    ridWeb.on("/connecttest.txt", HTTP_GET, handleRoot);
    ridWeb.on("/canonical.html", HTTP_GET, handleRoot);
    ridWeb.onNotFound(handleRoot);
    ridWeb.begin();
    configOpen = true;
  }
  configUntil = millis() + (uint32_t)CONFIG_PORTAL_MS;
  Serial.printf("Config site open: connect to Wi-Fi '%s' then http://%s\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str());
}

static void configStop() {
  if (!configOpen) return;
  ridWeb.stop();
  ridDns.stop();
  configOpen = false;
  Serial.println("Config site closed");
}

static bool configModeActive() { return configOpen; }

static void configPoll() {
  bool down = digitalRead(CONFIG_BTN_PIN) == (CONFIG_BTN_ACTIVE_LOW ? LOW : HIGH);
  uint32_t now = millis();
  if (down && !btnWasDown && (now - btnLastMs) > 50) {
    btnLastMs = now;
    configStart();
  }
  if (!down && btnWasDown) btnLastMs = now;
  btnWasDown = down;

  if (!configOpen) return;
  ridDns.processNextRequest();
  ridWeb.handleClient();
  if ((int32_t)(now - configUntil) >= 0) configStop();
}
