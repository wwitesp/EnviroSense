#include <WiFi.h>
#include <WebServer.h>  // added v1.18
WebServer server(80);  // added v1.18
#include <WiFiClient.h>  // added v1.18  
#include <Wire.h>  // i2c bus
#include <time.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ThingSpeak.h>
#include <ArduinoUniqueID.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>  // Sensor lib
#include <Adafruit_SI1145.h>  // UV, IR & Vis sensor
Adafruit_SI1145 uv = Adafruit_SI1145();
#include <Adafruit_BMP280.h>  // GY-21p - Pressure, Temp & Alt sensor
#include <Adafruit_Si7021.h>  // GY-21p - Humidity & Temp sensor
#include <ccs811.h>  // ccs811 - Co2 & VoC sensor
CCS811 ccs811(23);  // nWAKE on 23
#include <SSD1306.h>  // OLED display
SSD1306 display(0x3c, SDA, SCL);   // OLED display object definition (address, SDA, SCL)
Adafruit_BMP280 bme;
Adafruit_Si7021 si7021 = Adafruit_Si7021();
int ONBOARD_LED = 2; // onboard blue LED
int WiFiLED = 0;  // Set WiFi disconnect status LED
char* device = "EnviTriCorder";
char* model = "ETCESP32Dev1";
char* ver = "v1.0";
char* fw = "v1.18.080919";
char* fwname = "ESP32Dev1_OTA_ETC_AJAX_811_280_1302_7021_1145_TS_v1.19";
char* bmp280status = "Active";  // Sensor status
char* si7021status = "Active";  // Sensor status
char* gy1145status = "Active";  // Sensor status
char* ccs811status = "Active";  // Sensor status
char* criticalWiFi = "N";  // If Y, system reset performed after WiFi reconect fail followed by user defined timeout (criticalWiFiTimeOut) loop.
char* criticalWiFiTimeOut = "1000";  // default 1 second
char* screen_select = "Main_Screen";  // Call screen to display on base unit
const char* ssid = "ssid";
const char* password = "password";
int count = 0;
const long StartTimeout = 30000;
int resetcounter = 0;
int DebugMode = 0;
unsigned long LEDpreviousMillis = 0;
const long LEDinterval = 300;  // 3sec interval at which to flash LED (milliseconds)
String SerialNo;
char thingSpeakAddress[] = "api.thingspeak.com";
unsigned long myChannelNumber = xxxxxx;  // Your ThingSpeak Channel ID
const char* writeAPIKey = "XXXXXXXXXXXXXX";   // Get the key for your channel to approve writing
const int UpdateThingSpeakInterval = 10 * 60; // e.g. 10 * 60 for a 10-Min update interval (10-mins x 60-secs)
// Change to your WiFi credentials and select your time zone
const char* Timezone = "GMT0BST,M3.5.0/01,M10.5.0/02";  // UK, see below for others and link to database
String Format = "X";  // Time format M for dd-mm-yy and 23:59:59, "I" for mm-dd-yy and 12:59:59 PM, "X" for Metric units.
#define pressure_offset 6.5  // Air pressure calibration, adjust for your altitude
static String         Date_str, Time_str;
volatile unsigned int local_Unix_time = 0, next_update_due = 0;
volatile unsigned int update_duration = 60 * 60;  // Time duration in seconds, so synchronise every hour
static float          bme_temp, bme_humi, bme_pres;
static unsigned int   Last_Event_Time;
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time TS was updated
const long interval = 1800000;           // 30min interval at which to run Write_TS()(milliseconds)
//const long interval = 60000;
int TScount = 1;
//*****************************************************************************************
void IRAM_ATTR Timer_TImeout_ISR() {
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL_ISR(&timerMux);
  local_Unix_time++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
//*****************************************************************************************
// Future dev
/*void TSSpiffsR() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  File TSR = SPIFFS.open("/spiffs.txt");
  if (!TSR) {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.println("File Content:");
  while (TSR.available()) {
    TScount = (TSR.read());
    //  Serial.print ("Read TScount: "); Serial.println (TScount);
  }
  TSR.close();
  }*/
//*****************************************************************************************
// Future dev
/*void TSSpiffsW() {
  SPIFFS.begin(true);
  File TSW = SPIFFS.open("/spiffs.txt", FILE_WRITE);
  if (!TSW) {
    Serial.println("There was an error opening the file for writing");
    return;
  }
  if (TSW.print(TScount)) {
    Serial.print ("Write TScount: "); Serial.println (TScount);
  } else {
    Serial.println("TScount data write failed");
  }
  TSW.close();
  }*/
//*****************************************************************************************
WiFiClient  client;
String readString;
// v1.17  WiFiServer server(80);
String header;
void setup() {
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);
  Serial.begin(115200);
  SerialNo = (UniqueID, HEX);
  config_wire();
  ccs811.set_i2cdelay(50);
  ccs811.begin();
  if (DebugMode == 1) {
    Serial.print("setup: hardware    version: "); Serial.println(ccs811.hardware_version(), HEX);   // Print CCS811 versions
    Serial.print("setup: bootloader  version: "); Serial.println(ccs811.bootloader_version(), HEX);
    Serial.print("setup: application version: "); Serial.println(ccs811.application_version(), HEX);
  }
  UniqueIDdump(Serial);  // Get device serial number
  bool status = bme.begin(0x76);                 // Initialise CCS811 - address 0x76
  if (!status) Serial.println("Could not find a valid BME280 sensor, check wiring!"); // Check for a sensor
  uv.begin();
  Start_SSD1306_Setup();
  StartWiFi();
  Start_Time_Services();
  Setup_Interrupts_and_Initialise_Clock();       // Now setup a timer interrupt to occur every 1-second, to keep seconds accurate
  //  TSSpiffsR();  // Future dev
  ccs811.start(CCS811_MODE_1SEC);
  float TSData = ThingSpeak.readFloatField(830856, 8); TScount = TSData;  // Read TS update count and restore local counter
  server.on("/", handleRoot);         // This displays the main webpage, it is called when you open a client connection on the IP address using a browser
  server.on("/TEMPread", handleTEMP); // To update Temperature called by the function getSensorData
  server.on("/HUMIread", handleHUMI); // To update Humidity called by the function getSensorData
  server.on("/PRESread", handlePRES); // To update Pressure called by the function getSensorData
  server.on("/CO2read", handleCO2); // To update Pressure called by the function getSensorData
  server.on("/VOCread", handleVOC); // To update Pressure called by the function getSensorData
  server.on("/UVread", handleUV); // To update Pressure called by the function getSensorData
  server.on("/FWread", handleFW); // To update Pressure called by the function getSensorData
  server.on("/SNread", handleSN); // To update Pressure called by the function getSensorData
  server.begin();
}
//*****************************************************************************************
void loop()
{ ArduinoOTA.handle();
  // yield();
  if (WiFi.status() == WL_DISCONNECTED) WiFi_Lost();
  if (WiFi.status() == WL_CONNECTED) digitalWrite(ONBOARD_LED, LOW);
  Check_WiFi_ConnectStatus();  // Checks for loss of WiFi network cinnection & auto reconnect once WiFi network is available again.
  UpdateLocalTime();     // The variables 'Date_str' and 'Time_str' now have current date-time values
  BMP280_Read_Sensor();  // The variables 'bme_temp', 'bme_humi', 'bme_pres' now have current values
  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2, &etvoc, &errstat, &raw);
  float UVindex = uv.readUV(); UVindex /= 100.0; // the index is multiplied by 100 so to get the integer index, divide by 100!
  display.clear();
  display.drawString(1, 0, Date_str);  // Display current date
  display.drawString((Format == "I" ? 68 : 85), 0, Time_str);  // Adjust position for addition of AM/PM indicator if required
  display.drawLine(0, 12, 128, 12);  // Draw a line to seperate date and time section
  display.setFont(ArialMT_Plain_16);  // Set the Font size larger
  display.drawString(0, 13, String(bme_temp, 1) + "°" + (Format == "M" || Format == "X" ? "C" : "F"));  // Display temperature in °C (M) or °F (I)
  display.drawString((Format == "I" ? 70 : 62), 13, String(bme_pres, (Format == "I" ? 1 : 0)) + (Format == "M" || Format == "X" ? "hPa" : "in"));  // Display air pressure in hecto Pascals or inches
  display.setFont(ArialMT_Plain_10); display.drawString(0, 29, "RH"); display.setFont(ArialMT_Plain_16); display.drawString(18, 29, String(si7021.readHumidity(), 0) + "%"); // Display temperature and relative humidity in %
  display.setFont(ArialMT_Plain_10); display.drawString(0, 45, "UV"); display.drawString(0, 54, "10 /"); display.setFont(ArialMT_Plain_16); display.drawString(22, 47, String(UVindex, 0));
  display.setFont(ArialMT_Plain_10); display.drawString(62, 29, "CO2"); display.drawString(62, 35, "ppm"); display.setFont(ArialMT_Plain_16); display.drawString(88, 30, String(eco2));
  display.setFont(ArialMT_Plain_10); display.drawString(62, 45, "VOC"); display.drawString(62, 52, "ppb"); display.setFont(ArialMT_Plain_16); display.drawString(88, 47, String(etvoc));
  display.setFont(ArialMT_Plain_10); display.drawString(48, 0, "TS:"); display.drawString(65, 0, String(TScount));
  display.setFont(ArialMT_Plain_10);
  display.display();
  // Timer delay for writing TS values
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you wrote data to TS
    previousMillis = currentMillis;
    { if (WiFi.status() == WL_CONNECTED ) Write_TS(); // Writes data values to ThingsSpeak server only if WiFi connected. Prevents lock up.
    }
  }
  server.handleClient();
}
//*****************************************************************************************
void config_wire() {
  Wire.begin(SDA, SCL);               // (sda,scl) Start the Wire service for the OLED display using pin=22 for SCL and Pin-21 for SDA on ESP32 Dev1 board.
  // Wire.begin(SDA, SCL, 100000);       // (sda,scl,bus_speed) Start the Wire service for the OLED display using pin=22 for SCL and Pin-21 for SDA on ESP32 Dev1 board.
}
//*****************************************************************************************
void Esp32Reset() {
  display.clear();
  display.drawString(0, 0, "WiFi Connect..");
  display.display();
  unsigned long currentMillis = millis();
  if (DebugMode == 1) {
    Serial.println("ESP Restart counter initiated");
    Serial.println(millis());
  }
  if (currentMillis >= 20000) display.drawString(0, 10, "Restart in 10secs..");
  display.display();
  if (currentMillis - previousMillis >= StartTimeout)
    resetcounter = 1;
}
//*****************************************************************************************
void StartWiFi()
{
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("EnviCorder", "password");
  if (DebugMode == 1) {
    Serial.print(F("\r\nConnecting to: ")); Serial.println(ssid);
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    if (WiFi.status() == WL_DISCONNECTED)
    {
      WiFi_Lost();
      Esp32Reset();
    }
    if (WiFi.status() == WL_CONNECTED)digitalWrite(ONBOARD_LED, LOW);
    if (resetcounter == 1) ESP.restart();
    Serial.print(F("."));
  }
  if (DebugMode == 1) {
    Serial.print("WiFi connected to address: "); Serial.println(WiFi.localIP());
  }
  resetcounter = 2;  // Stops the ESP.restart() after inital boot & WiFi connect fail
  // ArduinoOTA.setPort(3232);  // Port defaults to 3232
  ArduinoOTA.setHostname(model);  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setPassword("ESP32Dev1"); // No authentication by default
  // Password can be set with it's md5 value as well  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
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
void ccs811_Sensor() {
  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2, &etvoc, &errstat, &raw);
}
//*****************************************************************************************
void Start_SSD1306_Setup() {
  display.init();  // Initialise the display
  display.flipScreenVertically();  // Flip the screen around by 180°
  display.setContrast(255);  // Display contrast, 255 is maxium and 0 in minimum, around 128 is fine
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, device);
  display.drawString(0, 10, model);
  display.drawString(0, 20, fwname);
  display.drawString(0, 30, fw);
  display.display();
}
//*****************************************************************************************
void Check_WiFi_ConnectStatus() {
  if (WiFi.status() == WL_DISCONNECTED)
  { WiFi.mode(WIFI_OFF);
    delay(150);
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(ssid, password);
    delay(150);
    Serial.println ("WiFi Start (Check_WiFi_ConnectStatus()");
    WiFiClient  client;
    server.begin();
  }
  // if (WiFi.status() != WL_CONNECTED) WiFi.begin(ssid, password);
  if (DebugMode == 1) {
    Serial.print("WiFi Status: "); Serial.println(WiFi.status());
  }
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
    strftime(hour_output, 30, "%T", localtime(&now));      // Formats time as: 14:05:49
  }
  else {
    strftime(day_output, 30, "%m-%d-%y", localtime(&now)); // Formats date as: 05-24-17
    strftime(hour_output, 30, "%r", localtime(&now));      // Formats time as: 2:05:49pm
  }
  Date_str = day_output;
  Time_str = hour_output;
}
//*****************************************************************************************
void BMP280_Read_Sensor() {
  if (Format == "M" || Format == "X") bme_temp = bme.readTemperature();
  else bme_temp = bme.readTemperature() * 9.00F / 5.00F + 32;
  if (Format == "M" || Format == "X") bme_pres = bme.readPressure() / 100.0F + pressure_offset;
  else bme_pres = (bme.readPressure() / 100.0F + pressure_offset) / 33.863886666667; // For inches
  //  bme_humi = bme.readHumidity();
}
//*****************************************************************************************
void WiFi_Lost() {
  unsigned long currentMillis = millis();
  if (currentMillis - LEDpreviousMillis > LEDinterval) {
    LEDpreviousMillis = currentMillis;
    if (count % 2 == 0)
      digitalWrite(ONBOARD_LED, HIGH);
    else
      digitalWrite(ONBOARD_LED, LOW);
    count ++;
  }
}
//*****************************************************************************************
void Write_TS() {
  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2, &etvoc, &errstat, &raw);
  float UVindex = uv.readUV(); UVindex /= 100.0; // the index is multiplied by 100 so to get the integer index, divide by 100!
  ThingSpeak.setField(1, String(bme.readTemperature(), 1));
  ThingSpeak.setField(2, String(bme_pres, 1));
  ThingSpeak.setField(3, String(si7021.readHumidity(), 1));
  ThingSpeak.setField(4, eco2);
  ThingSpeak.setField(5, etvoc);
  ThingSpeak.setField(6, String(UVindex, 1));

  ThingSpeak.setField(8, TScount);
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, writeAPIKey);
  if (x == 200) {
    Serial.println("Channel update successful."); TScount++;// TSSpiffsW();
    /* Future dev
       File TSW = SPIFFS.open("/spiffs.txt", FILE_WRITE);
         if (!TSW) {
          Serial.println("There was an error opening the file for writing");
          return;
         }
         if (TSW.print(TScount)) {
          Serial.print ("Write TScount: "); Serial.println (TScount);
         } else {
          Serial.println("TScount data write failed");
         }*/
    //    TSW.close();  // Future dev
  }
  else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
}
//*****************************************************************************************
// AJAX HTML
const char Web_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
  <style>
    .displayobject{
       font-family: sans-serif;
       margin: auto;
       text-align: center;
       width: 50%;
       border: 3px solid #000000;
       padding: 10px;
       background: white;
    }
    h1 {
      font-size: 50px;
      color: blue;
    }
    h4 {
      font-size: 30px;
      color: green;
    }
     h5 {
      font-size: 28px;
      color: black;
    }
     h6 {
      font-size: 28px;
      color: white;
    }

  </style>
  <body>
     <div class = "displayobject">
       <h1>EnviroCorder</h1>
       <h4>Temp: <span id="TEMPvalue">0</span>  /  Humidity: <span id="HUMIvalue">0</span>%</h4>
       <h4>Pressure: <span id="PRESvalue">0</span>  /  UV: <span id="UVvalue">0</span></h4>
       <h4>CO2: <span id="CO2value">0</span> ppm  /  VOC: <span id="VOCvalue">0</span> ppb</h4>
       <h5>Firmware: <span id="FWvalue">0</span></h5>
       <h5>Serial No: <span id="SNvalue">0</span></h5>
     </div>
     <script>
       setInterval(function() {getSensorData();}, 1000); // Call the update function every set interval e.g. 1000mS or 1-sec
  
       function getSensorData() {
          var xhttp = new XMLHttpRequest();
          xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("TEMPvalue").innerHTML = this.responseText;
          }
        };
        xhttp.open("GET", "TEMPread", true);
        xhttp.send();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("HUMIvalue").innerHTML = this.responseText;
          }
        };
        xhttp.open("GET", "HUMIread", true);
        xhttp.send();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("PRESvalue").innerHTML = this.responseText;}
        };  
        xhttp.open("GET", "PRESread", true);
        xhttp.send(); 
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("CO2value").innerHTML = this.responseText;}
        };  
        xhttp.open("GET", "CO2read", true);
        xhttp.send(); 
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("VOCvalue").innerHTML = this.responseText;}
        };  
        xhttp.open("GET", "VOCread", true);
        xhttp.send(); 
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("UVvalue").innerHTML = this.responseText;}
        };  
        xhttp.open("GET", "UVread", true);
        xhttp.send(); 
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("FWvalue").innerHTML = this.responseText;}
        };  
        xhttp.open("GET", "FWread", true);
        xhttp.send(); 
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("SNvalue").innerHTML = this.responseText;}
        };  
        xhttp.open("GET", "SNread", true);
        xhttp.send(); 
      }
    </script>
  </body>
</html>
)=====";
//==============================================================================================
// This routine is executed when you open a browser at the IP address, updates the sensor values.
//==============================================================================================
void handleRoot() {
  //String s = Web_page;             //Display HTML contents
  server.send(200, "text/html", Web_page); //Send web page
}
void handleTEMP() {
  server.send(200, "text/plain", String(bme_temp, 1) + "&#176;" + (Format == "M" || Format == "X" ? "C" : "F"));
}
void handleHUMI() {
  server.send(200, "text/plain", String (si7021.readHumidity(), 0));
}
void handlePRES() {
  server.send(200, "text/plain", String(bme_pres, (Format == "I" ? 1 : 0)) + (Format == "M" || Format == "X" ? " hPa" : " in"));
}
void handleCO2() {
  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2, &etvoc, &errstat, &raw);
  server.send(200, "text/plain", String(eco2));
}
void handleVOC() {
  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2, &etvoc, &errstat, &raw);
  server.send(200, "text/plain", String(etvoc));
}
void handleUV() {
  float UVindex = uv.readUV(); UVindex /= 100.0;
  server.send(200, "text/plain", String(UVindex, 1));
}
void handleFW() {
  server.send(200, "text/plain", String(fw));
}
void handleSN() {
  server.send(200, "text/plain", String(SerialNo));
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
*/
