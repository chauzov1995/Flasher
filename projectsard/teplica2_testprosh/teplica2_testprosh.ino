#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <INA219.h>
#include <NetworkClientSecure.h>

#define TYPE_VER "teplica2"
#define PREFIX "GN_"  // если используешь другой прект

WiFiMulti WiFiMulti;
const char* serverName = "https://smart.giulianovars.ru/api/app/v1.0/user/create_conf";

int teplica_pin_d1_open = 18;
int teplica_pin_d1_close = 19;
int teplica_pin_d2_open = 16;
int teplica_pin_d2_close = 17;


int btnupr1 = 33;
int btnupr2 = 32;

int ledpib = 4;
int btnpin = 0;



INA219 INA(0x40);
OneWire oneWire(26);
DallasTemperature sensors(&oneWire);
float temp = 0;

bool stat1 = false, stat2 = false, stat3 = false, stat4 = false, stat5 = false, stat6 = false, stat7 = false, stat8 = false, stat9 = false, stat10 = false;
bool ledsost = false;



void setClock() {
  configTime(0, 0, "pool.ntp.org");

  Serial.print(F("Waiting for NTP time sync: "));
  time_t nowSecs = time(nullptr);
  while (nowSecs < 8 * 3600 * 2) {
    delay(500);
    Serial.print(F("."));
    yield();
    nowSecs = time(nullptr);
  }

  Serial.println();
  struct tm timeinfo;
  gmtime_r(&nowSecs, &timeinfo);
  Serial.print(F("Current time: "));
  Serial.print(asctime(&timeinfo));
}


void remoteUPD(String typever = TYPE_VER) {
  Serial.println("Start remoteUPD");
  WiFiClient client;
  String pathupd = "http://4k.teplogico.ru/";
  pathupd += typever;
  pathupd += ".bin";
  t_httpUpdate_return ret = httpUpdate.update(client, pathupd);
  // Or:
  //t_httpUpdate_return ret = httpUpdate.update(client, "server", 80, "file.bin");

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}
String mac_adress;

bool readTemperature() {
  for (int i = 0; i < 10; i++) {
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);

    if (tempC != DEVICE_DISCONNECTED_C && tempC > -40 && tempC < 85 && tempC != 0.0) {
      temp = tempC;
      return true;
    }

    delay(500);
  }

  return false;
}


void setup() {







  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(ledpib, OUTPUT);
  pinMode(teplica_pin_d1_open, OUTPUT);
  pinMode(teplica_pin_d1_close, OUTPUT);
  pinMode(teplica_pin_d2_open, OUTPUT);
  pinMode(teplica_pin_d2_close, OUTPUT);
  pinMode(btnupr1, INPUT);
  pinMode(btnupr2, INPUT);
  pinMode(btnpin, INPUT);

  sensors.begin();
  delay(1000);

  if (readTemperature()) {
    Serial.println("temp");
    Serial.println(temp);

    if (temp > 10 && temp < 40) {
      stat5 = true;
    }
  } else {
    Serial.println("Error: Could not read temperature data");
  }


  //ina226
  Wire.begin();
  if (!INA.begin()) {
    Serial.println("could not connect. Fix and Reboot");
  }
 
 // INA.setMaxCurrentShunt(2, 0.010);
 // INA.setAverage(0);                   //1
 // INA.setBusVoltageConversionTime(7);  //5
 INA.setMaxCurrentShunt(4.0, 0.010);


  float cu = INA.getBusVoltage();
  Serial.print("BUS volt - ");
  Serial.println(cu, 3);
  if (cu > 20 && cu < 28) {
    stat8 = true;
  }

  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("CHAUZOV HOME", "User1234");
}


void loop() {

  if (digitalRead(btnupr1) == LOW) {
    stat1 = true;
    analogWrite(teplica_pin_d1_open, 255);
    analogWrite(teplica_pin_d1_close, 0);
  } else {
    stat2 = true;
    analogWrite(teplica_pin_d1_open, 0);
    analogWrite(teplica_pin_d1_close, 255);
  }

  if (digitalRead(btnupr2) == LOW) {
    stat3 = true;
    analogWrite(teplica_pin_d2_open, 255);
    analogWrite(teplica_pin_d2_close, 0);
  } else {
    stat4 = true;
    analogWrite(teplica_pin_d2_open, 0);
    analogWrite(teplica_pin_d2_close, 255);
  }


  if (digitalRead(btnpin) == LOW) {
    stat6 = true;
  } else {
    stat7 = true;
  }

  float cu = INA.getCurrent_mA();
  if (cu > 0 && cu <= 20) {
    stat9 = true;
  }
  if (cu > 20) {
    stat10 = true;
  }

  ledsost = !ledsost;
  digitalWrite(ledpib, ledsost);

  delay(100);
  if (!stat1) {
    Serial.print(" btn_dlin_off");
  }
  if (!stat2) {
    Serial.print(" btn_dlin_on");
  }
  if (!stat3) {
    Serial.print(" btn_kor_off");
  }
  if (!stat4) {
    Serial.print(" btn_kor_on");
  }
  if (!stat5) {
    Serial.print(" temp_");
    Serial.print(temp);
  }
  if (!stat6) {
    Serial.print(" btn_conf_off");
  }
  if (!stat7) {
    Serial.print(" btn_conf_on");
  }
  if (!stat8) {
    Serial.print(" ina_napr");
    float volt = INA.getBusVoltage();
    Serial.print(volt);
  }
  if (!stat9) {
    Serial.print(" mintok");

    Serial.print(cu, 3);
  }
  if (!stat10) {
    Serial.print(" maxtok");

    Serial.print(cu, 3);
  }
  Serial.println("");

  // put your main code here, to run repeatedly:

  if (stat1 && stat2 && stat3 && stat4 && stat5 && stat6 && stat7 && stat8 && stat9 && stat10) {
    if ((WiFiMulti.run() == WL_CONNECTED)) {

      //получим мак адрес платы тобишь id
      String mac_adress_v = WiFi.macAddress();
      mac_adress_v.replace(":", "");
      mac_adress_v.toLowerCase();
      mac_adress = PREFIX + mac_adress_v;
      Serial.println(mac_adress);


      setClock();

      NetworkClientSecure* client = new NetworkClientSecure;
      if (client) {

        client->setInsecure();      // Отключает проверку сертификата (используй только для тестов!)
        client->setTimeout(15000);  // Увеличь таймаут до 15 секунд

        {
          // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
          HTTPClient https;

          Serial.print("[HTTPS] begin...\n");
          https.begin(*client, serverName);
          Serial.print("[HTTPS] GET...\n");
          // start connection and send HTTP header
          https.addHeader("Content-Type", "application/json");



          String httpRequestData = "{\"parrent\": \"" + mac_adress + "\", \"owner\": 0, \"devices\": [{\"id\": \"" + mac_adress + "_1\", \"name\": \"Теплица\", \"room\": \"Комната по умолчанию\", \"state\": {\"value\": {\"on\": false, \"gisterezistemp\": 1, \"settemperature\": 27}}, \"float\": {\"value\": {\"temperature\": 0}}, \"event\": {}, \"parameters\": {}, \"type\": \"teplica\", \"retain\": false}], \"chekedprosh\": false, \"type\": \"" + TYPE_VER + "\"}";
          httpRequestData.replace("\"temperature\": 0", "\"temperature\": " + String(temp, 1));
          Serial.println(httpRequestData);
          int httpCode = https.POST(httpRequestData);


          // httpCode will be negative on error
          if (httpCode > 0) {
            // HTTP header has been send and Server response header has been handled
            Serial.printf("[HTTPS] GET... code: %d\n", httpCode);

            // file found at server
            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
              String payload = https.getString();
              Serial.println(payload);
            }
          } else {
            Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
          }

          https.end();


          if (httpCode == 200) {
            while(true){
              Serial.println("OK");
              delay(200);
            }
            //remoteUPD();
          } else {
            digitalWrite(ledpib, HIGH);
          }
        }

        delete client;
      } else {
        Serial.println("Unable to create client");
      }

    } else {
      Serial.println("not connect");
    }
    delay(2000);
  }
}
