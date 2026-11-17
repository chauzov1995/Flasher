// Green2 registration-only bench firmware. ESP32-C6, Arduino-ESP32 3.3.12.
// Intentionally standalone like testteplica: never starts the working controller.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <Wire.h>
#include <esp_mac.h>
#include <time.h>
#include "registration.h"
#include "console.h"

#if !CONFIG_IDF_TARGET_ESP32C6
#error Select ESP32C6 Dev Module, flash 8MB, USB CDC On Boot Disabled.
#endif
#if ARDUINO_USB_CDC_ON_BOOT
#error Disable USB CDC On Boot to use the CP210x service UART on GPIO16/17.
#endif

#if __has_include("green2_wifi.local.h")
#include "green2_wifi.local.h"
#else
inline void configureBenchWiFi(WiFiMulti&) {}
#endif

namespace {
constexpr char version[] = "green2-test-1.0.2";
constexpr char api[] = "https://smart.giulianovars.ru/api/app/v1.0/";
constexpr int motorPins[] = {18, 19, 20, 21};
constexpr int manualPin = 3, syncPin = 7;
WiFiMulti networks;
String parentId, childId;
float temperature = NAN, humidity = NAN;
unsigned vinMv = 0;
bool registered = false, errorState = false;
String lastError;
uint32_t lastStatus = 0;
green2test::ConsoleLine consoleLine;

void motorsOff() {
  for (int pin : motorPins) digitalWrite(pin, LOW);
}

void fail(const char* reason) {
  errorState = true;
  lastError = reason;
  Serial.printf("GREEN2_TEST ERROR %s\n", reason);
}

uint8_t crc(const uint8_t* word) {
  uint8_t value = 0xff;
  for (unsigned i = 0; i < 2; ++i) {
    value ^= word[i];
    for (unsigned b = 0; b < 8; ++b)
      value = (value & 0x80) ? (value << 1) ^ 0x31 : value << 1;
  }
  return value;
}

bool measure() {
  temperature = humidity = NAN;
  Wire.beginTransmission(0x44);
  Wire.write(0x24); Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    delay(20);
    uint8_t data[6];
    if (Wire.requestFrom(uint8_t(0x44), sizeof(data)) == sizeof(data)) {
      for (auto& byte : data) byte = Wire.read();
      if (crc(data) == data[2] && crc(data + 3) == data[5]) {
        temperature = -45.0f + 175.0f * ((uint16_t(data[0]) << 8) | data[1]) / 65535.0f;
        humidity = 100.0f * ((uint16_t(data[3]) << 8) | data[4]) / 65535.0f;
      }
    }
  }
  // Informational only: the VIN divider is not a calibrated acceptance test.
  unsigned sum = 0;
  analogReadMilliVolts(1);  // Discard first ADC conversion after channel setup.
  for (int i = 0; i < 16; ++i) sum += analogReadMilliVolts(1);
  vinMv = (uint64_t(sum) * 415 + 120) / (16 * 15);
  // GPIO0/current sense is deliberately never read.
  return isfinite(temperature) && temperature >= -40 && temperature <= 125 &&
         isfinite(humidity) && humidity >= 0 && humidity <= 100;
}

void status() {
  const bool sensorOk = measure();
  Serial.printf("GREEN2_TEST STATUS id=%s version=%s flash_MB=%u sht31=%s temp_C=%.2f hum_pct=%.2f vin_mV=%u button=%u sync=%u nfault1=%u nfault2=%u wifi=%s motors=off current=not_measured ds18b20=not_tested\n",
      parentId.c_str(), version, unsigned(ESP.getFlashChipSize() / (1024 * 1024)),
      sensorOk ? "ok" : "error", temperature, humidity, vinMv,
      digitalRead(manualPin) == LOW, digitalRead(syncPin) == LOW,
      digitalRead(2), digitalRead(22), WiFi.status() == WL_CONNECTED ? "connected" : "offline");
  if (errorState) Serial.printf("GREEN2_TEST ERROR %s\n", lastError.c_str());
  else if (registered) Serial.printf("GREEN2_TEST REGISTERED id=%s child=%s\n", parentId.c_str(), childId.c_str());
  else if (sensorOk && ESP.getFlashChipSize() == 8 * 1024 * 1024)
    Serial.println("GREEN2_TEST READY send @g register or hold SYNC 2s; network/server not checked yet");
  else Serial.println("GREEN2_TEST WAIT check SHT31 and 8MB flash");
  lastStatus = millis();
}

bool networkReady() {
  Serial.println("GREEN2_TEST WAIT wifi");
  if (networks.run(15000) != WL_CONNECTED) { fail("wifi_connection"); return false; }
  Serial.println("GREEN2_TEST WAIT ntp");
  configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
  const uint32_t start = millis();
  while (time(nullptr) < 1767225600 && millis() - start < 15000) delay(50);
  if (time(nullptr) < 1767225600) { fail("ntp_time_required_for_tls"); return false; }
  return true;
}

int request(const char* route, const String* payload, String& response) {
  NetworkClientSecure tls;
  tls.useBuiltinCACertBundle();
  tls.setHandshakeTimeout(15);
  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  if (!http.begin(tls, String(api) + route)) return -100;
  int code;
  if (payload) {
    http.addHeader("Content-Type", "application/json");
    code = http.POST(*payload);
  } else code = http.GET();
  response = code > 0 ? http.getString() : String();
  http.end();
  Serial.printf("GREEN2_TEST HTTP %s code=%d\n", payload ? "POST" : "GET", code);
  return code;
}

bool registrationConfirmed() {
  String response;
  const String route = "device/getConfig?id=" + parentId;
  if (request(route.c_str(), nullptr, response) != 200) return false;
  JsonDocument doc;
  return !deserializeJson(doc, response) &&
      green2test::registrationMatches(doc.as<JsonVariantConst>(), childId.c_str());
}

bool reportParent() {
  // create_conf stores only the parent's owner. The established device hello
  // endpoint persists its model/version, so both parent and child become green2.
  JsonDocument doc;
  doc["id"] = parentId;
  doc["type"] = "green2";
  doc["ver"] = version;
  doc["localip"] = WiFi.localIP().toString();
  doc["macaddress"] = WiFi.macAddress();
  doc["rssi"] = WiFi.RSSI();
  doc["wifiBSSID"] = WiFi.BSSIDstr();
  doc["wifiName"] = WiFi.SSID();
  String payload, response;
  serializeJson(doc, payload);
  if (request("device/upd_wifi2", &payload, response) != 200) return false;
  doc.clear();
  return !deserializeJson(doc, response) && doc["devices"].is<JsonObject>() &&
         doc["devices"].as<JsonObject>().containsKey(childId);
}

void registerDevice() {
  motorsOff();
  errorState = false;
  registered = false;
  if (ESP.getFlashChipSize() != 8 * 1024 * 1024) { fail("expected_8MB_flash"); return; }
  if (!measure()) { fail("sht31_not_ready"); return; }
  if (!networkReady()) return;
  if (!measure()) { fail("sht31_not_ready"); return; }
  // Factory workflow: every explicit action replaces the server configuration,
  // including an existing Green2 and a retry after an uncertain response.
  // No preflight GET, persistent attempt marker or already-registered shortcut.
  JsonDocument doc;
  green2test::makeRegistration(doc, parentId.c_str(), childId.c_str(), temperature, humidity);
  String payload, response;
  serializeJson(doc, payload);
  Serial.println("GREEN2_TEST WAIT overwriting_registration");
  const int code = request("user/create_conf", &payload, response);
  response.trim();
  if (code != 200 || response != "OK") {
    fail("registration_response_unconfirmed_retry_command_overwrites"); return;
  }
  // Verify the result AFTER writing; this never gates or skips the POST.
  if (!registrationConfirmed()) { fail("registration_readback_failed"); return; }
  if (!reportParent()) { fail("child_registered_parent_report_failed_retry_command"); return; }
  registered = true;
  status();
}

void command(const char* line) {
  if (!strcmp(line, "@g status")) status();
  else if (!strcmp(line, "@g register")) registerDevice();
  else if (!strcmp(line, "@g payload")) {
    measure();
    JsonDocument doc;
    green2test::makeRegistration(doc, parentId.c_str(), childId.c_str(), temperature, humidity);
    serializeJson(doc, Serial); Serial.println();
  } else if (!strcmp(line, "@g help"))
    Serial.println("GREEN2_TEST HELP @g status | @g payload (preview only) | @g register; SYNC held 2s registers; motors never run");
  else {
    static uint32_t lastUnknown = 0;
    if (millis() - lastUnknown >= 1000) {
      Serial.println("GREEN2_TEST INFO unknown_command_send_@g_help");
      lastUnknown = millis();
    }
  }
}
}  // namespace

void setup() {
  for (int pin : motorPins) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
  pinMode(10, OUTPUT); digitalWrite(10, LOW);
  pinMode(manualPin, INPUT_PULLUP); pinMode(syncPin, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP); pinMode(22, INPUT_PULLUP);
  // HardwareSerial::begin takes RX first, TX second. J3 uses the native C6 UART.
  static_assert(RX == 17 && TX == 16, "Green2 service UART requires RX=17, TX=16");
  Serial0.begin(115200, SERIAL_8N1, RX, TX);
  Wire.begin(4, 5); Wire.setTimeOut(25);
  analogReadResolution(12); analogSetPinAttenuation(1, ADC_6db);
  uint8_t mac[6];
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
    Serial0.println("GREEN2_TEST ERROR mac_unavailable");
    for (;;) delay(1000);
  }
  char id[16];
  snprintf(id, sizeof(id), "GN_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  parentId = id; childId = parentId + "_1";
  WiFi.persistent(false); WiFi.mode(WIFI_STA);
  configureBenchWiFi(networks);
  Serial.println("GREEN2_TEST BOOT registration-only; no automatic POST, no OTA, no motor test");
  command("@g help");
  status();
}

void loop() {
  motorsOff();
  // A noisy/echoing RX must not starve status updates and button servicing.
  for (unsigned bytes = 0; bytes < 128 && Serial.available(); ++bytes) {
    if (const char* line = consoleLine.push(static_cast<char>(Serial.read()))) command(line);
  }
  // Require a release after boot and after each action. A stuck button cannot
  // automatically register at power-up or repeatedly call the server.
  static bool released = false, held = false;
  static uint32_t heldSince = 0;
  if (digitalRead(syncPin) == HIGH) { released = true; held = false; }
  else if (released) {
    if (!held) { held = true; heldSince = millis(); }
    if (millis() - heldSince >= 2000) {
      released = false; held = false;
      registerDevice();
    }
  }
  if (millis() - lastStatus >= 3000) status();
  delay(5);
}
