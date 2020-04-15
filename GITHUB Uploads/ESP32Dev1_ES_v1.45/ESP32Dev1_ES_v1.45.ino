/* ************************************************************************
   EnviroSense for Arduino boards and compatible systems
   (C) 2019 by Wayne Sowka  https://github.com/wwitesp/ESP32-EnviroCorder

   EnviroSense for Arduino boards and compatible systems is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   EnviroSense for Arduino boards and compatible systems is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with EnviroCorder for Arduino boards and compatible systems.  If not, see <http://www.gnu.org/licenses/>.
 * ***********************************************************************/
//#include "src/yourLib/yourLib.h" - Enables the ability to keep library files within the project folder (use on final build)
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WiFiClient.h>
#include "Settings.h"
#include <Wire.h>
#include <time.h>
#include <WiFiUdp.h>
#include <ThingSpeak.h>
#include <ArduinoUniqueID.h>  // https://github.com/ricaun/ArduinoUniqueID
#include <ArduinoOTA.h>  // https://github.com/jandrassy/ArduinoOTA
#include "uptime_formatter.h"  // https://github.com/YiannisBourkelis/Uptime-Library
#include <Adafruit_Sensor.h>  // Sensor lib
#include <Adafruit_BMP280.h>  // GY-21p - Pressure, Temp & Alt sensor
#include <Adafruit_Si7021.h>  // GY-21p - Humidity & Temp sensor
#include "Adafruit_BME680.h"  // Air Quality sensor
#include "ccs811.h"  // ccs811 - Co2 & VoC sensor  https://github.com/maarten-pennings/CCS811
#include "VEML6075.h"  // https://github.com/schizobovine/VEML6075
#include <SSD1306.h>  // https://github.com/ThingPulse/esp8266-oled-ssd1306
//#define devicename "EnviroSense"  // Set your server's logical name here e.g. if myserver then address is http://myserver.local/
#define pressure_offset 6.5  // Air pressure calibration, adjust for your altitude
CCS811 ccs811(23);  // nWAKE on 23
SSD1306 display(0x3c, SDA, SCL);   // OLED display object definition (address, SDA, SCL)
Adafruit_BMP280 bmp280;
Adafruit_BME680 bme680;
Adafruit_Si7021 si7021 = Adafruit_Si7021();
VEML6075 veml6075 = VEML6075();
bool bme680VocValid = false;
AsyncWebServer server(80);
WiFiMulti wifiMulti;
WiFiClient  client;
//*****************************************************************************************
void setup() {
  chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  //  setCpuFrequencyMhz(240);
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  SerialNo = (uint32_t)chipid;
  powerSaving();
  config_wire();
  Start_OLED_Setup();
  StartWiFi();
  MDNS.begin(device);
  Start_Time_Services();
  Setup_Interrupts_and_Initialise_Clock(); // Setup a timer interrupt to occur every 1-second, to keep seconds accurate
  ccs811.set_i2cdelay(50);
  ccs811.begin();
  Serial.begin(115200);
  if (DebugMode == 1) {
    Serial.print("setup: hardware    version: "); Serial.println(ccs811.hardware_version(), HEX); // Print CCS811 versions
    Serial.print("setup: bootloader  version: "); Serial.println(ccs811.bootloader_version(), HEX);
    Serial.print("setup: application version: "); Serial.println(ccs811.application_version(), HEX);
  }
  if (!bmp280.begin(0x76)) Serial.println("Could not find a valid Pressure sensor - error code BMP");
  if (!bme680.begin()) Serial.println("Could not find a valid IAQ Sensor - error code BME");
  if (!veml6075.begin()) Serial.println("Could not find a valid UVa UVb Sensor - error code VEML");
  if (!ccs811.begin()) Serial.println("Could not find a valid CO2 VoC sensor - error code CCS");
  // Set up oversampling and filter initialization
  //  bme680.setTemperatureOversampling(BME680_OS_8X);
  //  bme680.setHumidityOversampling(BME680_OS_2X);
  //  bme680.setPressureOversampling(BME680_OS_4X);
  bme680.setTemperatureOversampling(BME680_OS_NONE);
  bme680.setHumidityOversampling(BME680_OS_NONE);
  bme680.setPressureOversampling(BME680_OS_NONE);
  bme680.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme680.setGasHeater(320, 150); // 320°C for 150 ms
  GetGasReference();
  ccs811.start(CCS811_MODE_10SEC);
  TScount = ThingSpeak.readFloatField(986281, 1);  // Read TS update count and restore local counter
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.name());
    file = root.openNextFile();
  }
  Serial.print("Used bytes after write:");
  Serial.println(SPIFFS.usedBytes());
  server.serveStatic("/config", SPIFFS, "/config.html");
  server.on("/dashboard.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/dashboard.css", "text/css");
  });
  server.on("/config.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/config.css", "text/css");
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/dashboard.html", String(), false, processor);
  });
  server.on("/bmp280_temp", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(bmp280_temp, 1).c_str());
  });
  server.on("/dewpoint_c", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(dewpoint_c, 0).c_str());
  });
  server.on("/current_humidity", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(current_humidity, 0).c_str());
  });
  server.on("/ab_humi", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(ab_humi, 1).c_str());
  });
  server.on("/bmp280_pres", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(bmp280_pres, 0).c_str());
  });
  server.on("/eco2", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(eco2).c_str());
  });
  server.on("/etvoc", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(etvoc).c_str());
  });
  server.on("/CO2Health", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String (CO2Health).c_str());
  });
  server.on("/final_air_quality_score", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String((final_air_quality_score), 0).c_str());
  });
  server.on("/iaq", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String (IAQ).c_str());
  });
  server.on("/UVa", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(UVA, 0).c_str());
  });
  server.on("/UVb", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(UVB, 0).c_str());
  });
  server.on("/UVi", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(UVI, 0).c_str());
  });
  server.on("/UVIExp_text", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String (UVIExp_text).c_str());
  });
  server.on("/UVIScore", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String (UVIScore_text).c_str());
  });
  server.on("/fw", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(fw).c_str());
  });
  server.on("/chipid", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String((uint32_t)chipid).c_str());
  });
  server.on("/getUptime", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", uptime_formatter::getUptime().c_str());
  });
  server.on("/screen", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(screen).c_str());
  });
  server.on("/screenweb", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(screenweb).c_str());
  });
  server.on("/cycleweb", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(cycleweb).c_str());
  });
  server.on("/son", HTTP_GET, [](AsyncWebServerRequest * request) {
    screenstate = 1; screenweb = "On";
    if (cycle == 1) {
      cycleweb = "On";
    }
    else {
      cycleweb = "Off";
    }
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/soff", HTTP_GET, [](AsyncWebServerRequest * request) {
    screenstate = 0; screenweb = "Off"; cycleweb = "Disabled";
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/con", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (screenstate == 1) {
      cycle = 1;
      cycleweb = "On";
    }
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/coff", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (screenstate == 1) {
      cycle = 0;
      cycleweb = "Off";
    }
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/s1", HTTP_GET, [](AsyncWebServerRequest * request) {
    cycle = 0; cycleweb = "Off"; screen = 1;
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/s2", HTTP_GET, [](AsyncWebServerRequest * request) {
    cycle = 0; cycleweb = "Off"; screen = 2;
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/s3", HTTP_GET, [](AsyncWebServerRequest * request) {
    cycle = 0; cycleweb = "Off"; screen = 3;
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/s4", HTTP_GET, [](AsyncWebServerRequest * request) {
    cycle = 0; cycleweb = "Off"; screen = 4;
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/wificycle", HTTP_GET, [](AsyncWebServerRequest * request) {
    StartWiFi();
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest * request) {
    ESP.restart();
    request->send(SPIFFS, "/config.html", String(), false, processor);
  });
  server.begin();
  MDNS.addService("http", "tcp", 80);
}
//*****************************************************************************************
void loop()
{
  Uni_Flash();
  ArduinoOTA.handle();
  Check_WiFi_ConnectStatus(); // Checks for loss of WiFi network cinnection & auto reconnect once WiFi network is available again.
  UpdateLocalTime(); // Variables 'Date_str' and 'Time_str' now have current date-time values
  Screen_State();
  Read_SensorPack(); // Reading of sensor pack ex. bme680
  BME680_Sensor();  //IAQ Score, IAQ Text
  Screen_State();
  TSUpload();
  //  LEDs();
  if (DebugMode == 1) {
    ccs811_error_status();
    Serial.println(s);
  }
}
//*****************************************************************************************
void IRAM_ATTR Timer_TImeout_ISR() {
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL_ISR(&timerMux);
  local_Unix_time++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
//*****************************************************************************************
void config_wire() {
  Wire.begin(SDA, SCL); // Start the Wire service for the OLED display - ESP32 Dev1 board.
}
//*****************************************************************************************
void Esp32Reset() {
  int trig = 0;
  display.clear();
  display.drawString(0, 0, "WiFi Connect..");
  display.display();
  unsigned long ResetcurrentMillis = millis();
  if (DebugMode == 1) {
    Serial.println("ESP Restart counter initiated");
    Serial.println(millis());
  }
  if (ResetcurrentMillis >= 10000) display.drawString(0, 10, "Restart in 5secs..");
  if (trig == 0) {
    display.display();
    trig = 1;
  }
  if (ResetcurrentMillis - ResetpreviousMillis >= StartTimeout)
    resetcounter = 1;
  trig = 0;
}
//*****************************************************************************************
void StartWiFi()
{
  WiFi.mode(WIFI_AP_STA); WiFi.setAutoConnect(true); WiFi.setAutoReconnect(true); WiFi.softAP(device, APpassword);
  wifiMulti.addAP(ssid1, password1);
  wifiMulti.addAP(ssid2, password2);
  if (wifiMulti.run() == WL_DISCONNECTED) {
    Esp32Reset();
    if (resetcounter == 1) ESP.restart();
  }
  if (DebugMode == 1) {
    Serial.print("WiFi connected to address: "); Serial.println(WiFi.localIP());
  }
  resetcounter = 2;  // Stops the ESP.restart() after inital boot & WiFi connect fail
  // Serial.println("\nConnected to " + WiFi.SSID() + " Use IP address: " + WiFi.localIP().toString()); // Report which SSID and IP is in use
  // ArduinoOTA.setPort(3232);  // Port defaults to 3232
  ArduinoOTA.setHostname(model);  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setPassword("ESP32Dev1"); // No authentication by default
  // Password can be set with it's md5 value as well  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]() {
    digitalWrite(ledCO2r, LOW);
    digitalWrite(ledCO2g, LOW);
    digitalWrite(ledCO2b, LOW);
    digitalWrite(ledIAQr, LOW);
    digitalWrite(ledIAQg, LOW);
    digitalWrite(ledIAQb, LOW);
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 10, "OTA Update");
    display.drawProgressBar(2, 28, 124, 10, progress / (total / 100));
    display.display();
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  ArduinoOTA.handle();
}
//*****************************************************************************************
void Start_Time_Services() {
  // Now configure time services
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", Timezone, 1);  // See below for other time zones
  delay(1000);  // Wait for time services
}
//*****************************************************************************************
void Setup_Interrupts_and_Initialise_Clock() {
  hw_timer_t * timer = NULL;
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &Timer_TImeout_ISR, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  //Now get current Unix time and assign the value to local Unix time counter and start the clock.
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println(F("Failed to obtain time"));
  }
  time_t now;
  time(&now);
  local_Unix_time = now + 1; // The addition of 1 counters the NTP setup time delay
  next_update_due = local_Unix_time + update_duration;
}
//*****************************************************************************************
void UpdateLocalTime() {
  time_t now;
  if (local_Unix_time > next_update_due) { // only get a time synchronisation from the NTP server at the update-time delay set
    time(&now);
    //   Serial.println("Synchronising local time, time error was: " + String(now - local_Unix_time));
    // If this displays a negative result the interrupt clock is running fast or positive running slow
    local_Unix_time = now;
    next_update_due = local_Unix_time + update_duration;
  } else now = local_Unix_time;
  //See http://www.cplusplus.com/reference/ctime/strftime/
  char hour_output[30], day_output[30];
  if (Format == "M" || Format == "X") {
    strftime(day_output, 30, "%d-%m-%y", localtime(&now)); // Formats date as: 24-05-17
    strftime(hour_output, 30, "%R", localtime(&now));      // Formats time as: 14:05:49
  }
  else {
    strftime(day_output, 30, "%m-%d-%y", localtime(&now)); // Formats date as: 05-24-17
    strftime(hour_output, 30, "%r", localtime(&now));      // Formats time as: 2:05:49pm
  }
  Date_str = day_output;
  Time_str = hour_output;
}
//**************************************************************************************
void TSUpload() {
  unsigned long TScurrentMillis = millis();
  if (TScurrentMillis - TSpreviousMillis >= TSinterval) {
    TSpreviousMillis = TScurrentMillis;
    if (wifiMulti.run() == WL_CONNECTED ) Write_TS(); // Writes data values to ThingsSpeak server only if WiFi connected. Prevents lock up.
  }
}
//**************************************************************************************
void Read_SensorPack() {
  if (Format == "M" || Format == "X") bmp280_temp = bmp280.readTemperature();
  else bmp280_temp = bmp280.readTemperature() * 9.00F / 5.00F + 32;
  if (Format == "M" || Format == "X") bmp280_pres = bmp280.readPressure() / 100.0F + pressure_offset;
  else bmp280_pres = (bmp280.readPressure() / 100.0F + pressure_offset) / 33.863886666667; // For inches
  temp_f = bmp280_temp * 9 / 5 + 32;
  current_humidity = si7021.readHumidity();  // Humidity Sensor
  ab_humi = (6.112 * pow(2.71828, ((17.67 * bmp280_temp) / (243.5 + bmp280_temp))) * current_humidity * 2.1674) / (273.15 + bmp280_temp);
  dewpoint_c = 243.5 * (log(current_humidity / 100) + ((17.67 * bmp280_temp) / (243.5 + bmp280_temp))) / (17.67 - log(current_humidity / 100) - ((17.67 * bmp280_temp) / (243.5 + bmp280_temp)));
  //  dewpoint_f = (243.5 * (log(current_humidity / 100) + ((17.67 * bmp280_temp) / (243.5 + bmp280_temp))) / (17.67 - log(current_humidity / 100) - ((17.67 * bmp280_temp) / (243.5 + bmp280_temp)))) * 9 / 5 + 32;
  ccs811.set_envdata(bmp280_temp, current_humidity);
  ccs811.read(&eco2, &etvoc, &errstat, &raw);  // Co2, VoC Sensor
  CO2Health = CalculateCO2Health(CO2Health_Score);
  veml6075.poll();  // UVa UVb UVi
  UVA = veml6075.getUVA(), UVB = veml6075.getUVB(), UVI = veml6075.getUVIndex();
  UVIScore = CalculateUVIScore(UVI_Score);
}
//*****************************************************************************************
void BME680_Sensor() {
  unsigned long BME680currentMillis = millis();
  if (BME680currentMillis - BME680previousMillis >= 5000) {
    BME680previousMillis = BME680currentMillis;
    humidity_score = GetHumidityScore();
    gas_score = GetGasScore();
    air_quality_score = humidity_score + gas_score;
    final_air_quality_score = (100 - air_quality_score) * 5;
    if ((getgasreference_count++) % 5 == 0) GetGasReference();
    IAQ = CalculateIAQ(air_quality_score);
    if (DebugMode == 1) {
      Serial.println(" comprised of " + String(humidity_score) + "% Humidity and " + String(gas_score) + "% Gas");
      Serial.println(CalculateIAQ(air_quality_score));
      Serial.println("--------------------------------------------------------------");
    }
  }
}
//*****************************************************************************************
void Start_OLED_Setup() {
  display.init();  // Initialise the display
  display.flipScreenVertically();  // Flip the screen around by 180°
  display.setContrast(255);  // Display contrast, 255 is maxium and 0 in minimum, around 128 is fine
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(15, 0, device);
  display.drawString(20, 25, SerialNo);
  display.drawString(14, 48, fw);
  display.display();
}
//*****************************************************************************************
void Screen_Cycle() {
  unsigned long SCRNcurrentMillis = millis();
  if (SCRNcurrentMillis - SCRNpreviousMillis > SCRNinterval) {
    SCRNpreviousMillis = SCRNcurrentMillis;
    if (screen == 1)Screen_1();
    if (screen == 2)Screen_2();
    if (screen == 3)Screen_3();
    if (screen == 4)Screen_4();
    screen ++;
    if (screen > 4)screen = 1;
  }
}
//*****************************************************************************************
void Screen_State() {
  if (screenstate == 1) {
    display.clear();
    display.setFont(ArialMT_Plain_10); display.drawString(85, 0, Date_str);
    display.setFont(ArialMT_Plain_10); display.drawString((Format == "I" ? 0 : 4), 0, Time_str);  // Adjust position for addition of AM/PM indicator if required

    if (wifiMulti.run() == WL_CONNECTED) {
      display.setFont(ArialMT_Plain_10);
      display.drawString(50, 0, "WiFi");
    }
    display.drawLine(0, 12, 128, 12);
    switch (cycle) {
      case 0:
        switch (screen) {
          case 1:
            Screen_1(); break;
          case 2:
            Screen_2(); break;
          case 3:
            Screen_3(); break;
          case 4:
            Screen_4(); break;
        } break;
      case 1:
        Screen_Cycle(); break;
    }
  }
  else if (screenstate == 0) {
    display.clear(); display.display(); screenstate = 2;
  }
}
//*****************************************************************************************
void Screen_1() {
  display.setFont(ArialMT_Plain_10);
  display.drawString(62, 29, "CO2"); display.drawString(62, 35, "ppm");
  display.drawString(0, 29, "VOC"); display.drawString(0, 35, "ppb");
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 13, String(bmp280_temp, 1) + "°" + (Format == "M" || Format == "X" ? "c" : "f"));  // Display temperature in °C (M) or °F (I)
  display.drawString((Format == "I" ? 70 : 62), 13, String(bmp280_pres, (Format == "I" ? 1 : 0)) + (Format == "M" || Format == "X" ? "hPa" : " in"));  // Display air pressure in hecto Pascals or inches
  display.drawString(88, 30, String(eco2));
  display.drawString(26, 30, String(etvoc));
  display.drawString(0, 46, CO2Health);
  display.display();
}
void Screen_2() {
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 13, "RH");
  display.drawString(55, 13, "Abs"); display.drawString(55, 20, "Hum");
  display.drawString(55, 29, "Dew"); display.drawString(55, 36, "Pnt");
  display.drawString(0, 29, "Air"); display.drawString(0, 35, "Qua");
  display.setFont(ArialMT_Plain_16);
  display.drawString(15, 13, String(current_humidity, 0) + "%");
  display.drawString(77, 13, (String (ab_humi, 1)));
  display.drawString(80, 30, (String (dewpoint_c, 0) + "°" + "c"));
  display.drawString(22, 30, String((final_air_quality_score), 0));
  display.drawString(0, 47, String (IAQ));
  display.display();
}
void Screen_3() {
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 13, "UVA " + (String (UVA, 0)));
  display.drawString(55, 13, "UVB " + (String (UVB, 0)));
  display.drawString(0, 30, "UVI " + (String (UVI, 0)));
  display.drawString(47, 30, "Exp " + (UVIExp_text));// display.drawString(77, 30, UVIExp_text);
  display.drawString(0, 47, (UVIScore_text) + " risk");
  display.display();
}
void Screen_4() {
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 13, "CO2:" + (CO2Health));
  display.drawString(0, 30, "AQ:" + (String (IAQ)));
  display.drawString(0, 47, "UV:" + (UVIScore_text) + " risk");
  display.display();
}
//*****************************************************************************************
void Check_WiFi_ConnectStatus() {
  if (wifiMulti.run() == WL_DISCONNECTED) {
    WiFi_Lost();
    wifiMulti.addAP(ssid1, password1);
    wifiMulti.addAP(ssid2, password2);
  }
  else if (wifiMulti.run() == WL_CONNECTED) digitalWrite(ONBOARD_LED, LOW);
  if (DebugMode == 1) {
    Serial.println ("WiFi Start (Check_WiFi_ConnectStatus()");
  }
}
//*****************************************************************************************
void WiFi_Lost() {
  digitalWrite(ONBOARD_LED, LEDState);
}
//*****************************************************************************************
void Uni_Flash() {
  unsigned long Uni_FlashLEDcurrent = millis();
  if (Uni_FlashLEDcurrent - Uni_FlashLEDprevious > Uni_FlashLEDinterval) {
    Uni_FlashLEDprevious = Uni_FlashLEDcurrent;
    LEDState = not(LEDState);
  }
}
//*****************************************************************************************
void Write_TS() {
  TScount++;
  ThingSpeak.setField(1, String(bmp280_temp, 1));
  ThingSpeak.setField(2, String(bmp280_pres, 1));
  ThingSpeak.setField(3, String(current_humidity, 1));
  ThingSpeak.setField(4, eco2);
  ThingSpeak.setField(5, etvoc);
  ThingSpeak.setField(6, String (dewpoint_c, 1));
  ThingSpeak.setField(7, String(final_air_quality_score, 0));
  ThingSpeak.setField(8, String (ab_humi, 1));
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(ETCChannelNumber, ETCwriteAPIKey);
  if (DebugMode == 1) {
    if (x == 200) {
      Serial.println("Channel update successful.");
    }
    else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
  }
  ThingSpeak.setField(1, TScount);
  ThingSpeak.setField(2, String(UVA, 0));
  ThingSpeak.setField(3, String(UVB, 0));
  ThingSpeak.setField(4, String(UVI, 0));
  //  ThingSpeak.setField(5, value);
  //  ThingSpeak.setField(6, value);
  //  ThingSpeak.setField(7, value);
  //  ThingSpeak.setField(8, value);
  // write to the ThingSpeak channel
  int x2 = ThingSpeak.writeFields(ESChannelNumber, ESwriteAPIKey);
  if (DebugMode == 1) {
    if (x2 == 200) {
      Serial.println("Channel update successful.");
    }
    else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
  }
}
//*****************************************************************************************
void GetGasReference() {
  int readings = 10;
  for (int i = 1; i <= readings; i++) {
    digitalWrite(ledIAQb, HIGH);
    gas_reference += bme680.readGas();
    digitalWrite(ledIAQb, LOW);
  }
  gas_reference = gas_reference / readings;
  if (DebugMode == 1) {
    Serial.println("Gas Reference = " + String(gas_reference, 3));
  }
}
//*****************************************************************************************
String processor(const String& var) {
  if (var == "BPM280_TEMP") {
    return String(bmp280_temp, 1);
  }
  else if (var == "DEWPOINT_C") {
    return String(dewpoint_c, 0);
  }
  else if (var == "CURRENT_HUMIDITY") {
    return String(current_humidity, 0);
  }
  else if (var == "AB_HUMI") {
    return String(ab_humi, 1);
  }
  else if (var == "BPM280_PRES") {
    return String(bmp280_pres, 0);
  }
  else if (var == "ECO2") {
    return String(eco2);
  }
  else if (var == "ETVOC") {
    return String(etvoc);
  }
  else if (var == "CO2HEALTH") {
    return CO2Health;
  }
  else if (var == "IAQ") {
    return String (IAQ);
  }
  else if (var == "FINAL_AIR_QUALITY_SCORE") {
    return String((final_air_quality_score), 0);
  }
  else if (var == "UVA") {
    return String(UVA, 0);
  }
  else if (var == "UVB") {
    return String(UVB, 0);
  }
  else if (var == "UVI") {
    return String(UVI, 0);
  }
  else if (var == "UVIEXP_TEXT") {
    return String(UVIExp_text);
  }
  else if (var == "UVISCORE") {
    return String(UVIScore_text);
  }
  else if (var == "FW") {
    return String(fw);
  }
  else if (var == "CHIPID") {
    return String((uint32_t)chipid);
  }
  else if (var == "GETUPTIME") {
    return uptime_formatter::getUptime();
  }
  else if (var == "SCREENWEB") {
    return String(screenweb);
  }
  else if (var == "SCREEN") {
    return String(screen);
  }
  else if (var == "CYCLEWEB") {
    return String(cycleweb);
  }
  return String();
}
//*****************************************************************************************
String CalculateCO2Health(int CO2Health_Score) {
  String CO2Health_text = "";
  if (eco2 >= 40000) {
    CO2Health_text += "O2 Deprived";
    digitalWrite(ledCO2b, LOW);
    digitalWrite(ledCO2g, LOW);
    digitalWrite(ledCO2r, LEDState);
  }
  else if (eco2 >= 1801 && eco2 <= 40000) {
    CO2Health_text += "Inadequate";
    digitalWrite(ledCO2b, LOW);
    digitalWrite(ledCO2g, LOW);
    digitalWrite(ledCO2r, LEDState);
  }
  else if (eco2 >= 1501 && eco2 <= 1800) {
    CO2Health_text += "Poor";
    digitalWrite(ledCO2b, LOW);
    digitalWrite(ledCO2g, LOW);
    digitalWrite(ledCO2r, HIGH);
  }
  else if (eco2 >= 801 && eco2 <= 1500) {
    CO2Health_text += "Fair";
    digitalWrite(ledCO2b, LOW);
    digitalWrite(ledCO2g, LOW);
    digitalWrite(ledCO2r, HIGH);
  }
  else if (eco2 >=  601 && eco2 <= 800) {
    CO2Health_text += "Good";
    digitalWrite(ledCO2b, LOW);
    digitalWrite(ledCO2g, HIGH);
    digitalWrite(ledCO2r, HIGH);
  }
  else if (eco2 >=  01 && eco2 <=  600) {
    CO2Health_text += "Excellent";
    digitalWrite(ledCO2b, LOW);
    digitalWrite(ledCO2g, HIGH);
    digitalWrite(ledCO2r, LOW);
  }
  else if (eco2 <=  01) {
    CO2Health_text += "Sensing..";
    digitalWrite(ledCO2b, LEDState);
    digitalWrite(ledCO2g, LOW);
    digitalWrite(ledCO2r, LOW);
  }
  if (IAQscore >= 301) {
    digitalWrite(ledIAQb, LOW);
    digitalWrite(ledIAQg, LOW);
    digitalWrite(ledIAQr, LEDState);
  }
  if (IAQscore <= 01) {
    digitalWrite(ledIAQb, LEDState);
    digitalWrite(ledIAQg, LOW);
    digitalWrite(ledIAQr, LOW);
  }
  if (DebugMode == 1) {
    Serial.print("Co2: " + String(CO2Health_Score) + ", ");
  }
  return CO2Health_text;
}
//*****************************************************************************************
String CalculateIAQ(int score) {
  String IAQ_text = "";
  score = (100 - score) * 5;
  IAQscore = score;
  if (score >= 301) {
    IAQ_text += "Hazardous";
  }
  else if (score >= 201 && score <= 300) {
    IAQ_text += "Ext. Bad";
    digitalWrite(ledIAQb, LOW);
    digitalWrite(ledIAQg, LOW);
    digitalWrite(ledIAQr, HIGH);
  }
  else if (score >= 176 && score <= 200) {
    IAQ_text += "Very Bad";
    digitalWrite(ledIAQb, LOW);
    digitalWrite(ledIAQg, LOW);
    digitalWrite(ledIAQr, HIGH);
  }
  else if (score >= 151 && score <= 175) {
    IAQ_text += "Unhealthy";
    digitalWrite(ledIAQb, LOW);
    digitalWrite(ledIAQg, HIGH);
    digitalWrite(ledIAQr, HIGH);
  }
  else if (score >= 51 && score <= 150) {
    IAQ_text += "Moderate";
    digitalWrite(ledIAQb, LOW);
    digitalWrite(ledIAQg, HIGH);
    digitalWrite(ledIAQr, HIGH);
  }
  else if (score >  01 && score <=  50) {
    IAQ_text += "Good";
    digitalWrite(ledIAQb, LOW);
    digitalWrite(ledIAQg, HIGH);
    digitalWrite(ledIAQr, LOW);
  }
  else if (score <=  01) {
    IAQ_text += "Sensing AQ...";
    // digitalWrite(ledIAQb, LEDState);
    // digitalWrite(ledIAQg, LOW);
    // digitalWrite(ledIAQr, LOW);
  }
  if (DebugMode == 1) {
    Serial.print("AQ Score: " + String(score) + ", ");
  }
  return IAQ_text;
}
//*****************************************************************************************
/*void LEDs()
  {
  if (bmp280_temp >= 25.0) {
  
  }
  else if (bmp280_temp >= 17.0 && bmp280_temp <= 24.9) {
  
  }
  else if (bmp280_temp <=  16.9) {
  
  }
  if      (current_humidity >= 80.0) HumiLED = 2;
  else if (current_humidity >= 60.0 && current_humidity <= 79.9) HumiLED = 1;
  else if (current_humidity <=  59.9) HumiLED = 0;
  }*/
//*****************************************************************************************
String CalculateUVIScore(int UVI_Score) {
  //  String UVIScore_text = "";
  if      (UVI >= 11) {
    UVIScore_text = "Extreme";
    UVIExp_text = "15mins";
  }
  else if (UVI >= 8 && UVI <= 10 ) {
    UVIScore_text = "Very high";
    UVIExp_text = "20mins";
  }
  else if (UVI >= 6 && UVI <= 7 ) {
    UVIScore_text = "High";
    UVIExp_text = "30mins";
  }
  else if (UVI >=  3 && UVI <= 5 ) {
    UVIScore_text = "Low";
    UVIExp_text = "40mins";
  }
  else if (UVI >=  1 && UVI <=  2 ) {
    UVIScore_text = "No danger";
    UVIExp_text = "1Hr+";
  }
  else if (UVI <=  1) {
    UVIScore_text = "None";
    UVIExp_text = "No limit";
  }
  if (DebugMode == 1) {
    Serial.print("UV Index: " + String(UVI_Score) + ", ");
  }
  return UVIScore_text, UVIExp_text;
}
//*****************************************************************************************
int GetHumidityScore() {  //Calculate humidity contribution to IAQ index
  if (current_humidity >= 38 && current_humidity <= 42) // Humidity +/-5% around optimum
    humidity_score = 0.25 * 100;
  else
  { // Humidity is sub-optimal
    if (current_humidity < 38)
      humidity_score = 0.25 / hum_reference * current_humidity * 100;
    else
    {
      humidity_score = ((-0.25 / (100 - hum_reference) * current_humidity) + 0.416666) * 100;
    }
  }
  return humidity_score;
}
//*****************************************************************************************
int GetGasScore() {
  //Calculate gas contribution to IAQ index
  gas_score = (0.75 / (gas_upper_limit - gas_lower_limit) * gas_reference - (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) * 100.00;
  if (gas_score > 75) gas_score = 75; // Sometimes gas readings can go outside of expected scale maximum
  if (gas_score <  0) gas_score = 0;  // Sometimes gas readings can go outside of expected scale minimum
  return gas_score;
}
//*****************************************************************************************
void powerSaving() {
  btStop();
  for (byte i = 13; i <= 33; i++) {
    pinMode(i, INPUT_PULLUP);
    digitalWrite(i, LOW);
  }
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(wake, OUTPUT);// CO2 Sensor
  pinMode(ledCO2b, OUTPUT);// Blue - Co2
  pinMode(ledCO2g, OUTPUT);// Green - Co2
  pinMode(ledCO2r, OUTPUT);// Red - Co2
  pinMode(ledIAQb, OUTPUT);// Blue - IAQ
  pinMode(ledIAQg, OUTPUT);// Green - IAQ
  pinMode(ledIAQr, OUTPUT);// Red - IAQ
  digitalWrite(ledCO2r, LOW);
  digitalWrite(ledCO2g, LOW);
  digitalWrite(ledCO2b, LOW);
  digitalWrite(ledIAQr, LOW);
  digitalWrite(ledIAQg, LOW);
  digitalWrite(ledIAQb, LOW);
}
//*****************************************************************************************
void ccs811_error_status()
{
  // First the ERROR_ID flags
  s[ 0] = '-';
  s[ 1] = '-';
  if ( errstat & CCS811_ERRSTAT_HEATER_SUPPLY    ) s[ 2] = 'V'; else s[2] = 'v';
  if ( errstat & CCS811_ERRSTAT_HEATER_FAULT     ) s[ 3] = 'H'; else s[3] = 'h';
  if ( errstat & CCS811_ERRSTAT_MAX_RESISTANCE   ) s[ 4] = 'X'; else s[4] = 'x';
  if ( errstat & CCS811_ERRSTAT_MEASMODE_INVALID ) s[ 5] = 'M'; else s[5] = 'm';
  if ( errstat & CCS811_ERRSTAT_READ_REG_INVALID ) s[ 6] = 'R'; else s[6] = 'r';
  if ( errstat & CCS811_ERRSTAT_WRITE_REG_INVALID) s[ 7] = 'W'; else s[7] = 'w';
  // Then the STATUS flags
  if ( errstat & CCS811_ERRSTAT_FW_MODE          ) s[ 8] = 'F'; else s[8] = 'f';
  s[ 9] = '-';
  s[10] = '-';
  if ( errstat & CCS811_ERRSTAT_APP_VALID        ) s[11] = 'A'; else s[11] = 'a';
  if ( errstat & CCS811_ERRSTAT_DATA_READY       ) s[12] = 'D'; else s[12] = 'd';
  s[13] = '-';
  // Next bit is used by SW to signal I2C transfer error
  if ( errstat & CCS811_ERRSTAT_I2CFAIL          ) s[14] = 'I'; else s[14] = 'i';
  if ( errstat & CCS811_ERRSTAT_ERROR            ) s[15] = 'E'; else s[15] = 'e';
  s[16] = '\0';
  //  return s;
}
//*****************************************************************************************
/*
  WiFi event status:
  Event: 0 - WIFI_READY
  Event: 2 - STA_START
  Event: 3 - Normal network connection
  Event: 4 - STA_CONNECTED
  Event: 5 - STA_DISCONNECTED & Reason: 3 - AUTH_LEAVE / Reason: 202 - AUTH_FAIL / Reason: 201 - NO_AP_FOUND
  Event: 6 - No network connection
  Event: 7 - STA_GOT_IP
  Event: 8 - STA_LOST_IP
  Example time zones see: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
  //const char* Timezone = "GMT0BST,M3.5.0/01,M10.5.0/02";     // UK
  //const char* Timezone = "MET-2METDST,M3.5.0/01,M10.5.0/02"; // Most of Europe
  //const char* Timezone = "CET-1CEST,M3.5.0,M10.5.0/3";       // Central Europe
  //const char* Timezone = "EST-2METDST,M3.5.0/01,M10.5.0/02"; // Most of Europe
  //const char* Timezone = "EST5EDT,M3.2.0,M11.1.0";           // EST USA
  //const char* Timezone = "CST6CDT,M3.2.0,M11.1.0";           // CST USA
  //const char* Timezone = "MST7MDT,M4.1.0,M10.5.0";           // MST USA
  //const char* Timezone = "NZST-12NZDT,M9.5.0,M4.1.0/3";      // Auckland
  //const char* Timezone = "EET-2EEST,M3.5.5/0,M10.5.5/0";     // Asia
  //const char* Timezone = "ACST-9:30ACDT,M10.1.0,M4.1.0/3":   // Australia

  FUTURE HEAT INDEX CALC CODE TO BE INTERGRATED

  float computeHeatIndex(float temp, float humi, bool metricUnit) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml

  float hi;

  if (metricUnit){
    temp = convertCtoF(temp);
  }

  hi = 0.5 * (temp + 61.0 + ((temp - 68.0) * 1.2) + (humi * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temp +
            10.14333127 * humi +
            -0.22475541 * temp*humi +
            -0.00683783 * pow(temp, 2) +
            -0.05481717 * pow(humi, 2) +
             0.00122874 * pow(temp, 2) * humi +
             0.00085282 * temp*pow(humi, 2) +
            -0.00000199 * pow(temp, 2) * pow(humi, 2);

    if ((humi < 13) && (temp >= 80.0) && (temp <= 112.0)){
      hi -= ((13.0 - humi) * 0.25) * sqrt((17.0 - abs(temp - 95.0)) * 0.05882);
    } else if ((humi > 85.0) && (temp >= 80.0) && (temp <= 87.0)){
      hi += ((humi - 85.0) * 0.1) * ((87.0 - temp) * 0.2);
    }
  }

  return metricUnit ? convertFtoC(hi) : hi ;
  }

  float convertCtoF(float c) {
  return c * 1.8 + 32;
  }

  float convertFtoC(float f) {
  return (f - 32) * 0.55555;
  }
*/
