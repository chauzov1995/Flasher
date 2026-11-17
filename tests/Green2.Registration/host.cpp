#include "registration.h"
#include "console.h"
#include <string.h>
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

extern "C" int registrationTests() {
  using namespace green2test;
  ConsoleLine console;
  unsigned commands = 0;
  auto feed = [&](const char* text) {
    while (*text) if (console.push(*text++)) ++commands;
  };
  feed("GREEN2_TEST INFO unknown_command_send_@g_help\r\n");
  feed("GREEN2_TEST READY send @g register or hold SYNC 2s\r\n");
  feed("garbage\n\r\n");
  CHECK(commands == 0);
  feed("@g sta");
  CHECK(commands == 0);
  feed("tus\r\n@g help\n");
  CHECK(commands == 2);
  feed("@g ");
  for (unsigned i = 0; i < 200; ++i) console.push('x');
  feed("@g register\n");
  CHECK(commands == 2);  // Oversized line cannot execute its command-like tail.
  feed("@g sta"); console.push('\0'); feed("tus\n");
  CHECK(commands == 2);
  feed("@g register\n@g register\n");
  CHECK(commands == 4);  // Two explicit registration commands remain two actions.
  const char* parent = "GN_206ef1184354";
  const char* child = "GN_206ef1184354_1";
  JsonDocument doc;
  auto parse = [&](const char* text) {
    if (deserializeJson(doc, text)) return false;
    return registrationMatches(doc.as<JsonVariantConst>(), child);
  };
  CHECK(!parse("[]"));
  CHECK(!parse("{}"));
  CHECK(!parse("null"));
  CHECK(!parse("\"OK\""));
  CHECK(!parse("[{}]"));
  CHECK(!parse("<html>503</html>"));
  CHECK(!parse("{\"error\":\"unauthorized\"}"));
  CHECK(parse("{\"GN_206ef1184354_1\":{\"id\":\"GN_206ef1184354_1\",\"type\":\"green2\",\"parameters\":{}}}"));
  doc[child]["type"] = "teplica";
  CHECK(!registrationMatches(doc.as<JsonVariantConst>(), child));
  doc[child]["type"] = "green2";
  doc[child]["id"] = "GN_other_1";
  CHECK(!registrationMatches(doc.as<JsonVariantConst>(), child));
  doc[child]["id"] = child;
  doc["GN_206ef1184354_2"]["type"] = "green2";
  CHECK(!registrationMatches(doc.as<JsonVariantConst>(), child));

  makeRegistration(doc, parent, child, 23.5f, 48.25f);
  CHECK(doc["parrent"] == parent && doc["type"] == "green2");
  CHECK(doc["owner"].is<int>() && doc["owner"] == 0);
  CHECK(doc["chekedprosh"].is<bool>() && !doc["chekedprosh"].as<bool>());
  CHECK(doc["devices"].size() == 1);
  JsonObject d = doc["devices"][0];
  CHECK(d["id"] == child && d["type"] == "green2");
  CHECK(d["state"]["value"]["on"].isNull());
  CHECK(d["state"]["value"]["settemperature"] == 27);
  CHECK(!d["state"]["value"]["position_known"].as<bool>());
  CHECK(d["float"]["value"]["temperature"] == 23.5f);
  CHECK(d["float"]["value"]["hum"] == 48.25f);
  CHECK(d["float"]["value"]["temp_ds"].isNull());
  CHECK(d["event"].isNull() && d["parameters"].is<JsonObject>());
  char wire[2048];
  size_t length = serializeJson(doc, wire, sizeof(wire));
  CHECK(length > 0 && length < sizeof(wire) - 1);
  doc.clear();
  CHECK(!deserializeJson(doc, wire));
  CHECK(doc["devices"][0]["id"] == child);
  CHECK(doc["devices"][0]["event"].isNull());
  // Reusing a document must also replace its old owner, model and settings.
  doc["owner"] = 123;
  doc["type"] = "teplica2";
  doc["devices"][0]["state"]["value"]["settemperature"] = 35;
  doc["devices"].as<JsonArray>().add<JsonObject>()["id"] = "old_child";
  makeRegistration(doc, parent, child, 24.0f, 50.0f);
  CHECK(doc["owner"] == 0 && doc["type"] == "green2");
  CHECK(doc["devices"].size() == 1 && doc["devices"][0]["type"] == "green2");
  CHECK(doc["devices"][0]["state"]["value"]["settemperature"] == 27);
  CHECK(doc["devices"][0]["float"]["value"]["temperature"] == 24.0f);
  return 0;
}
