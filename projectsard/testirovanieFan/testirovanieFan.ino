#define LED_BUILTIN 23//  //индикатор  23
#define BUTT_CONF 19//19 //кнопка синх 19
#define BUTT_CONF_STATE HIGH //реверс данных



#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>
#include <HTTPUpdate.h>

WiFiMulti WiFiMulti;
#include <Wire.h>
#include "DFRobot_SHT20.h"
#include "Adafruit_CCS811.h"



unsigned long timing, timing2; // Переменная для хранения точки отсчета
DFRobot_SHT20 sht20;
Adafruit_CCS811 ccs;
void IRAM_ATTR ZCDetectorChange();
#define ZC_PIN 26
#define OUT_PIN 25
volatile uint8_t pwr = 0;
TaskHandle_t Task1;
int buttonState = 0;

void setup() {
  pinMode(BUTT_CONF, INPUT);

  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);
  pinMode(ZC_PIN, INPUT_PULLUP);

  pinMode(18, OUTPUT); //дадим питание на i2c
  digitalWrite(18, LOW);
  pinMode(5, OUTPUT); //дадим питание illum
  digitalWrite(5, LOW);
  delay(100);

  sht20.initSHT20(); // Init SHT20 Sensor
  delay(100);
  sht20.checkSHT20(); // Check SHT20 Sensor



  if (!ccs.begin()) {
    //  Serial.println("Failed to start sensor! Please check your wiring.");
    // while (1);
  }

  //calibrate temperature sensor
  while (!ccs.available());
  //  float temp = ccs.calculateTemperature();
  // ccs.setTempOffset(temp - 25.0);

  disableCore0WDT();
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(500);



  digitalWrite(OUT_PIN, 1);



  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("Giulia Novars", "Giulia49");
  WiFiMulti.addAP("AIP", "AIP43211234");
  WiFiMulti.addAP("MikroTik", "Qw8675309Qw");
  WiFiMulti.addAP("Keenetic-8138", "3kEMmPdz");

}


void Task1code( void * pvParameters ) {

  attachInterrupt(ZC_PIN, ZCDetectorChange, CHANGE);
  for (;;) {

  }
}

void IRAM_ATTR ZCDetectorChange() {

  //  pwr = 100;

  if (digitalRead(ZC_PIN) == 1 || pwr == 0) {
    digitalWrite(OUT_PIN, 0);
  } else {
    if (pwr > 99)
      pwr = 99;
    for (int cnt = 99; cnt > pwr && digitalRead(ZC_PIN) == 0; cnt--) {
      delayMicroseconds(100);
    }
    if (digitalRead(ZC_PIN) == 0)
      digitalWrite(OUT_PIN, 1);
  }
}
bool error = false;
bool firsasda = true;


void loop() {
  if (millis() - timing2 > 2000) { // Вместо 10000 подставьте нужное вам значение паузы
    timing2 = millis();
    float humd = sht20.readHumidity(); // Read Humidity
    float temp = sht20.readTemperature(); // Read Temperature
    float illuminat =  analogRead(34) ;
    float co2 = ccs.geteCO2();
    Serial.print("hum=");
    Serial.println(humd);
    Serial.print("temp=");
    Serial.println(temp);
    Serial.print("illuminat=");
    Serial.println(illuminat);
    Serial.print("co2=");
    Serial.println(co2);

  }
  if (millis() - timing > 100) { // Вместо 10000 подставьте нужное вам значение паузы
    timing = millis();


    if (error) {
      digitalWrite(LED_BUILTIN, pwr % 2 == 1 ? LOW : HIGH);
    }
    pwr++;
    //  Serial.println(pwr);
    if (pwr > 99) {
      pwr = 0;
    }
    delay(100);
    buttonState = digitalRead(BUTT_CONF);
    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == HIGH) {


      if ((WiFiMulti.run() == WL_CONNECTED)) {
        float humd = sht20.readHumidity(); // Read Humidity
        if (humd < 10 || humd > 90) {
          error = true;
          Serial.print("sht20 does not show hum hum=");
          Serial.println(humd);
        }
        float temp = sht20.readTemperature(); // Read Temperature
        if (temp < 10 || temp > 90) {
          error = true;
          Serial.print("sht20 does not show temp temp=");
          Serial.println(temp);
        }
        float illuminat = 100 - analogRead(34) / 40.96;
        if (illuminat < 1 || illuminat > 90) {
          error = true;
          Serial.print("illuminat does not show illuminat=");
          Serial.println(illuminat);
        }
        if (ccs.available()) {
          //  float temp = ccs.calculateTemperature();
          if (!ccs.readData()) {
            float co2 = ccs.geteCO2();
            if (co2 < 400 || co2 > 6000) {
              error = true;
              Serial.print("ccs co2 levie dannie co2=");
              Serial.println(co2);
            }
          } else {
            error = true;
            Serial.println("ccs co2 net dannih");
          }
        } else {
          error = true;
          Serial.println("ccs co2 ne viden na shine");
        }
      } else {
        error = true;
        Serial.println("no wifi");
      }


      if (error) {


        Serial.println("error");
        //   ESP.restart();
      } else {
        digitalWrite(LED_BUILTIN,  HIGH);
        Serial.println("succes");
        WiFiClient client;
        String  pathupd = "http://4k.teplogico.ru/";
        pathupd += "fan";
        pathupd += ".bin";
        t_httpUpdate_return ret = httpUpdate.update(client, pathupd);
        delay(100000);
      }
    }

  }
}
