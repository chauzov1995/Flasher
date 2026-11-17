#pragma once
#include <ArduinoJson.h>

namespace green2test {

// Readback only, after create_conf has replaced the configuration.
inline bool registrationMatches(JsonVariantConst root, const char* childId) {
  if (!root.is<JsonObjectConst>() || root.size() != 1) return false;
  JsonObjectConst child = root[childId];
  return !child.isNull() && child["id"] == childId && child["type"] == "green2";
}

inline void makeRegistration(JsonDocument& doc, const char* parentId,
                             const char* childId, float temperature, float humidity) {
  doc.clear();
  doc["parrent"] = parentId;  // Spelling is part of the existing API.
  doc["owner"] = 0;
  doc["type"] = "green2";
  doc["chekedprosh"] = false;  // Never ask the legacy server to trigger an OTA.
  JsonObject child = doc["devices"].to<JsonArray>().add<JsonObject>();
  child["id"] = childId;
  child["type"] = "green2";
  child["name"] = "Теплица Green2";
  child["room"] = "Комната по умолчанию";
  child["retain"] = false;
  JsonObject state = child["state"]["value"].to<JsonObject>();
  state["on"] = nullptr;  // The factory test has not homed or moved the motors.
  state["position_known"] = false;
  state["position_estimated"] = false;
  state["settemperature"] = 27;
  state["gisterezistemp"] = 1;
  JsonObject values = child["float"]["value"].to<JsonObject>();
  values["temperature"] = temperature;
  values["hum"] = humidity;
  values["temp_sht"] = temperature;
  values["temp_ds"] = nullptr;  // DS18B20 is not tested by this registration sketch.
  values["temp_source"] = "sht31";
  values["sensor_mismatch"] = false;
  values["motor_armed"] = false;
  values["current_feedback"] = false;
  child["event"] = nullptr;
  child["parameters"].to<JsonObject>();
}

}  // namespace green2test
