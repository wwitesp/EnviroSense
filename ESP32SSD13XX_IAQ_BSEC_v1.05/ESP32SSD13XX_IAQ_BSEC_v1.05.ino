#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "bsec.h"
#include <WiFiClient.h>
#include "Settings.h"
#include <SSD_13XX.h>
#include <Adafruit_Sensor.h>
#include "_fonts/Terminal_9.c"
#include "_fonts/square_small.c"
#include "_fonts/unborn_small.c";
#include "_fonts/akasha_cap.c";
#include <SPI.h>
#include <time.h>
//#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoUniqueID.h>
#include <ArduinoOTA.h>
#include "uptime_formatter.h"
#define pressure_offset 6.5  // Air pressure calibration, adjust for your altitude
#define cs   17
#define dc   16
#define rst  5
#define mosi 23
#define sclk 14
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
//Adafruit_BME680 bme680; // I2C
SSD_13XX display = SSD_13XX(cs, dc, rst);
AsyncWebServer server(80);
String processor(const String& var) {
  Serial.println(var);
  if (var == "CURRENT_TEMP") {
    return String(current_temp,1);
  }
  else if (var == "CURRENT_HUMIDITY") {
    return String(current_humidity,1);
  }
  else if (var == "CURRENT_PRESSURE") {
    return String(current_pressure,0);
  }
  else if (var == "AIR_QUALITY_SCORE") {
    return String(iaq_score,2);
  }
  else if (var == "AIR_QUALITY_ACCURACY") {
    return String(iaq_acc,0);
  }
  else if (var == "CO2") {
    return String(current_co2,0);
  }
  else if (var == "VOC") {
    return String(current_voc,0);
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
  return String();
}
void checkIaqSensorStatus(void);
void errLeds(void);
WiFiClient  client;
//*****************************************************************************************
void setup(void) {

  Serial.begin(115200);
  Wire.begin();
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  SerialNo = (uint32_t)chipid;
  display.begin();
  Start_SSD1306_Setup();
  StartWiFi();
  Start_Time_Services();
  Setup_Interrupts_and_Initialise_Clock();       // Now setup a timer interrupt to occur every 1-second, to keep seconds accurate
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/current_temp", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(current_temp, 1).c_str());
  });
  server.on("/current_humidity", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(current_humidity, 1).c_str());
  });
  server.on("/current_pressure", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(current_pressure, 0).c_str());
  });
  server.on("/air_quality_score", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(iaq_score,2).c_str());
  });
  server.on("/air_quality_accuracy", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(iaq_acc,0).c_str());
  });
  server.on("/co2", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(current_co2, 0).c_str());
  });
  server.on("/voc", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(current_voc, 0).c_str());
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
  // Start server
  server.begin();
}
//*****************************************************************************************
void loop(void)
{
  checkIaqSensorStatus();
//  unsigned long time_trigger = millis();
//  if (iaqSensor.run()) { // If new data is available
    output = String();
    // output += ", " + String(iaqSensor.rawTemperature);
    output += "Pressure: " + String(iaqSensor.pressure);
    // output += ", " + String(iaqSensor.rawHumidity);
    // output += ", " + String(iaqSensor.gasResistance);
    output += "  IAQ: " + String(iaqSensor.iaq);
    output += "  IAQ acc: " + String(iaqSensor.iaqAccuracy);
    output += "  T: " + String(iaqSensor.temperature);
    output += "  RH: " + String(iaqSensor.humidity);
    output += "  Static IAQ: " + String(iaqSensor.staticIaq);
    output += "  CO2: " + String(iaqSensor.co2Equivalent);
    output += "  VOC: " + String(iaqSensor.breathVocEquivalent);
    output += "  Static IAQ acc: " + String(iaqSensor.staticIaqAccuracy);
    output += "  CO2 acc: " + String(iaqSensor.co2Accuracy);
    output += "  VOC acc: " + String(iaqSensor.breathVocAccuracy);
    output += "  Gas R acc: " + String(iaqSensor.compGasAccuracy);
    output += "  Gas % acc: " + String(iaqSensor.gasPercentageAcccuracy);
      Serial.println(output);

  }
  ArduinoOTA.handle();
  //  if (WiFi.status() == WL_DISCONNECTED) WiFi_Lost();
  if (WiFi.status() == WL_CONNECTED)digitalWrite(ONBOARD_LED, LOW);
  Check_WiFi_ConnectStatus();  // Checks for loss of WiFi network cinnection & auto reconnect once WiFi network is available again.
  UpdateLocalTime();     // The variables 'Date_str' and 'Time_str' now have current date-time values
  BME680_Sensor();
  display.setTextScale(1); display.setTextColor(WHITE); display.setFont(&Terminal_9);
  display.setCursor(0, 0); display.print(WkDay_str); display.setCursor(20, 0); display.print(Day_str); display.setCursor(35, 0); display.print(Mth_str); display.setCursor(65, 0); display.print(Time_str);
  display.setTextScale(1); display.setTextColor(MAGENTA); display.setCursor(0, 8); display.print(String(current_temp, 1) + "°c");
  display.setTextScale(1); display.setTextColor(GREEN); display.setCursor(52, 8); display.print(String(current_humidity, 1) + "%");
  display.setTextScale(1); display.setTextColor(CYAN); display.setCursor(0, 18); display.print(String(current_pressure, 0) + "hPa");
  display.setTextScale(1); display.setTextColor(YELLOW); display.setCursor(0, 28); display.print(String(current_co2,0) + "ppm");
  display.setTextScale(1); display.setTextColor(GREEN); display.setCursor(0, 38); display.print(String(current_voc,0) + "ppm");
  display.setTextScale(1); display.setTextColor(YELLOW); display.setCursor(0, 47); display.print(String(iaq_score,2));
  display.setTextScale(1); display.setTextColor(YELLOW); display.setCursor(50, 47); display.print("Acc: " + String(iaq_acc,0));

  if (PrevTime_str != Time_str)
  {
    display.fillRect(65, 0, 36, 7, random(0x0000, 0x0000));  // x,y,w,h
    PrevTime_str = Time_str;
  }
  if (PrevTemp != String(current_temp, 1))
  {
    display.fillRect(0, 8, 36, 7, random(0x0000, 0x0000));  // x,y,w,h
    PrevTemp = String(current_temp, 1);
  }
  if (PrevHum != String(current_humidity, 1))
  {
    display.fillRect(52, 8, 36, 7, random(0x0000, 0x0000));  // x,y,w,h
    PrevHum = String(current_humidity, 1);
  }
  if (PrevPress != String(current_pressure, 0))
  {
    display.fillRect(0, 18, 125, 7, random(0x0000, 0x0000));  // x,y,w,h
    PrevPress = String(current_pressure, 0);
  }
  if (Prevco2 != String(current_co2,0))
  {
    display.fillRect(0, 28, 36, 7, random(0x0000, 0x0000));  // x,y,w,h
    Prevco2 = String(current_co2,0);
  }
  if (Prevvoc != String(current_voc,0))
  {
    display.fillRect(0, 38, 125, 7, random(0x0000, 0x0000));  // x,y,w,h
    Prevvoc = String(current_voc,0);
  }
  if (PrevIAQ != String(iaq_score,2))
  {
    display.fillRect(0, 47, 36, 7, random(0x0000, 0x0000));  // x,y,w,h
    PrevIAQ = String(iaq_score,2);
  }
  if (PrevIAQacc != String(iaq_acc,0))
  {
    display.fillRect(50, 47, 125, 7, random(0x0000, 0x0000));  // x,y,w,h
    PrevIAQacc = String(iaq_acc,0);
  }
}

//*****************************************************************************************
void IRAM_ATTR Timer_TImeout_ISR(void) {
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL_ISR(&timerMux);
  local_Unix_time++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
//*****************************************************************************************
void StartWiFi(void)
{
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  if (DebugMode == 1) {
    Serial.print(F("\r\nConnecting to: ")); Serial.println(ssid);
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    if (WiFi.status() == WL_CONNECTED)
    {
      digitalWrite(ONBOARD_LED, LOW);
    }
    if (WiFi.status() == WL_DISCONNECTED)
    {
      Esp32Reset();
    }
    if (resetcounter == 1) ESP.restart();
    if (DebugMode == 1) {
      Serial.print(F("."));
    }
  }
  if (DebugMode == 1) {
    Serial.print("WiFi connected to address: "); Serial.println(WiFi.localIP());
  }
  resetcounter = 2;

  // Port defaults to 3232                            // ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(model); // Hostname defaults to esp3232-[MAC]               // ArduinoOTA.setHostname("myesp32");
  // ArduinoOTA.setPassword("ESP32Dev1"); // No authentication by default                     // ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    if (DebugMode == 1) {
      Serial.println("Start updating " + type);
    }
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (DebugMode == 1) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    }
    display.fillScreen(BLACK);
    display.setFont(&unborn_small);
    display.setRotation(0);
    display.setTextScale(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 20); display.print("OTA Update");
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
void Setup_Interrupts_and_Initialise_Clock(void) {
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
void UpdateLocalTime(void) {
  time_t now;
  if (local_Unix_time > next_update_due) { // only get a time synchronisation from the NTP server at the update-time delay set
    time(&now);
    //   Serial.println("Synchronising local time, time error was: " + String(now - local_Unix_time));
    // If this displays a negative result the interrupt clock is running fast or positive running slow
    local_Unix_time = now;
    next_update_due = local_Unix_time + update_duration;
  } else now = local_Unix_time;
  //See http://www.cplusplus.com/reference/ctime/strftime/
  char time_output[30], day_output[30], wkday_output[30], mth_output[30];
  if (Format == "M" || Format == "X") {
    strftime(day_output, 30, "%d", localtime(&now)); // Formats date as: 24-05-17
    strftime(time_output, 30, "%R", localtime(&now));      // Formats time as: 14:05
    strftime(wkday_output, 30, "%a", localtime(&now)); // Formats date as: 24-05-17
    strftime(mth_output, 30, "%b", localtime(&now)); // Formats date as: 24-05-17
  }
  else {
    strftime(day_output, 30, "%m-%d-%y", localtime(&now)); // Formats date as: 05-24-17
    strftime(time_output, 30, "%r", localtime(&now));      // Formats time as: 2:05:49pm
  }
  Day_str = day_output;
  Time_str = time_output;
  WkDay_str = wkday_output;
  Mth_str = mth_output;
}
//*****************************************************************************************
void Start_Time_Services(void) {
  // Now configure time services
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", Timezone, 1);  // See below for other time zones
  delay(1000);  // Wait for time services
}
//*****************************************************************************************
void Esp32Reset(void) {
  display.setCursor(0, 0);
  display.print("WiFi Connect..");
  unsigned long currentMillis = millis();
  if (DebugMode == 1) {
    Serial.println("ESP Restart counter initiated");
    Serial.println(millis());
  }
  if (currentMillis - previousMillis >= StartTimeout)
    resetcounter = 1;
}
//*****************************************************************************************
void Start_SSD1306_Setup(void) {
  display.fillScreen(BLACK);
  display.setFont(&unborn_small);
  display.setRotation(0);
  display.setTextScale(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0); display.print(device);
  display.setCursor(0, 25); display.print(SerialNo);
  display.setCursor(0, 48); display.print(fw);
  delay(2000);
  display.fillScreen(BLACK);
}
//*****************************************************************************************
void Check_WiFi_ConnectStatus(void) {
  if (WiFi.status() == WL_DISCONNECTED)
  { WiFi_Lost();
    wifbegincycle();
    if (DebugMode == 1) {
      Serial.println ("WiFi Start (Check_WiFi_ConnectStatus()");
    }
  }
}
//*****************************************************************************************
void wifbegincycle(void) {
  unsigned long WIFIcurrentMillis = millis();
  if (WIFIcurrentMillis - WIFIpreviousMillis >= 30000) {
    WIFIpreviousMillis = WIFIcurrentMillis;
    WiFi.begin(ssid, password);
  }
}
//*****************************************************************************************
void WiFi_Lost(void) {
  unsigned long LEDcurrentMillis = millis();
  if (LEDcurrentMillis - LEDpreviousMillis > LEDinterval) {
    LEDpreviousMillis = LEDcurrentMillis;
    if (count % 2 == 0)
      digitalWrite(ONBOARD_LED, HIGH);
    else
      digitalWrite(ONBOARD_LED, LOW);
    count ++;
  }
}
//*****************************************************************************************
void BME680_Sensor(void) {
  current_temp =  iaqSensor.temperature;
  current_humidity = iaqSensor.humidity;
  current_pressure = iaqSensor.pressure / 100.0F + pressure_offset;
  current_co2 = iaqSensor.co2Equivalent;
  current_voc = iaqSensor.breathVocEquivalent;
  iaq_acc = iaqSensor.iaqAccuracy;
  iaq_score = iaqSensor.iaq;
}
//*****************************************************************************************

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
  iaqSensor.status = BSEC_OK;
}
//*****************************************************************************************

void errLeds(void)
{
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
  delay(100);
}
//*****************************************************************************************
