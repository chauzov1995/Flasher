#pragma once

// Copy to green2_wifi.local.h. This local file is excluded from Git.
// Credentials are compiled into the test image; keep that image on the bench.
inline void configureBenchWiFi(WiFiMulti& wifi) {
  wifi.addAP("CHAUZOV HOME", "User1234");
}
