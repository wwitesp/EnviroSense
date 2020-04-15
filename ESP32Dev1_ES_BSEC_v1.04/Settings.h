char* device = "EnviroSense";
char* model = "ESP32Dev1ESbsec";
char* ver = "v1.0";
char* fw = "v1.04.140420";
char* fwname = "ESP32Dev1_ES_BSEC_v1.04";
const char* ssid1 = "zzzz";   // WiFi SSID to connect to
const char* password1 = "zzzz"; // WiFi password needed for the SSID
const char* ssid2 = "zzzz";   // WiFi SSID to connect to
const char* password2 = "zzzz"; // WiFi password needed for the SSID
//const char* AP = device;
const char* APpassword = ""; // WiFi password needed for the SSID
char* screenweb = "On";
char* cycleweb = "On";
int ONBOARD_LED = 2; // onboard LED
int LEDState = LOW;
int ledCO2r = 16;
int ledCO2g = 4;
int ledCO2b = 15;
int ledIAQr = 18;
int ledIAQg = 5;
int ledIAQb = 17;
int wake = 23;
int IAQLED;
int CO2LED;
int TempLED;
int HumiLED;
int IAQscore = 01;
int count = 0;
int Uni_Flashcount = 0;
int SCRNcount = 0;
int resetcounter = 0;
int DebugMode = 0;
int TScount = 1;
int screenstate = 1; // Turn screen on/off
int screen = 1; // Call screen to display on base unit
int cycle = 1;
static char s[17]; // 16 bits plus terminating zero
char thingSpeakAddress[] = "api.thingspeak.com";
unsigned long ETCChannelNumber = 830856;
unsigned long ESChannelNumber = 986281;
unsigned int result; // Baseline
const char* ETCwriteAPIKey = "zzzz";   // Get the key for your channel to approve writing
const char* ESwriteAPIKey = "zzzz";   // Get the key for your channel to approve writing
const int UpdateThingSpeakInterval = 10 * 60; // e.g. 10 * 60 for a 10-Min update interval (10-mins x 60-secs)
unsigned long SCRNpreviousMillis;
unsigned long Uni_FlashLEDprevious;
unsigned long ResetpreviousMillis = 0;
unsigned long TSpreviousMillis = 0;   // will store last time TS was updated
unsigned long BME680previousMillis = 0;
//const long TSinterval = 1800000;   // 30min interval at which to run Write_TS()(milliseconds)
//const long TSinterval = 1500000;   // 25min interval at which to run Write_TS()(milliseconds)
//const long TSinterval = 1200000;   // 20min interval at which to run Write_TS()(milliseconds)
//const long TSinterval = 900000;   // 15min interval at which to run Write_TS()(milliseconds)
//const long TSinterval = 600000;   // 10min interval at which to run Write_TS()(milliseconds)
const long TSinterval = 300000;   // 5min interval at which to run Write_TS()(milliseconds)
const long Uni_FlashLEDinterval = 300;
const long IAQinterval = 10000;
const long BME680interval = 10000;
const long StartTimeout = 15000;
const long SCRNinterval = 5000;

float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score
int humidity_score, gas_score, CO2Health_Score, UVI_Score;
float gas_reference = 2500;
float hum_reference = 40;
int getgasreference_count = 0;
int gas_lower_limit = 10000;  // Bad air quality limit
int gas_upper_limit = 300000; // Good air quality

uint64_t chipid;
uint16_t eco2, etvoc, errstat, raw;
String output, iaq_acc_text;
Bsec iaqSensor;
String SerialNo;
String Format = "X";   // Time format M for dd-mm-yy and 23:59:59, "I" for mm-dd-yy and 12:59:59 PM, "X" for Metric units.
static String         Date_str, Time_str, IAQ, CO2Health, UVIScore, UVIExp_text, UVIScore_text;
volatile unsigned int local_Unix_time = 0, next_update_due = 0;
volatile unsigned int update_duration = 60 * 60;  // Time duration in seconds, so synchronise every hour
static float          UVindex, bmp280_humi, bme680_temp, bme680_humi, bme680_pres, air_quality_score, final_air_quality_score, C_Temp, C_Pressure, C_Humidity, iaq_score, iaq_acc;
static unsigned int   Last_Event_Time;
float ab_humi, dewpoint_c, temp_f, dewpoint_f, UVA, UVB, UVI;
String City             = "NOTTINGHAM";                    // Your home city See: http://bulk.openweathermap.org/sample/
String Country          = "GB";                            // Your _ISO-3166-1_two-letter_country_code country code, on OWM find your nearest city and the country code is displayed
// https://en.wikipedia.org/wiki/List_of_ISO_3166_country_codes
String Language         = "EN";                            // NOTE: Only the weather description is translated by OWM
// Examples: Arabic (AR) Czech (CZ) English (EN) Greek (EL) Persian(Farsi) (FA) Galician (GL) Hungarian (HU) Japanese (JA)
// Korean (KR) Latvian (LA) Lithuanian (LT) Macedonian (MK) Slovak (SK) Slovenian (SL) Vietnamese (VI)
String Hemisphere       = "north";                         // or "south"
String Units            = "M";                             // Use 'M' for Metric or I for Imperial
const char* Timezone    = "GMT0BST,M3.5.0/01,M10.5.0/02";  // Choose your time zone from: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
const char* ntpServer   = "0.uk.pool.ntp.org";             // Or, choose a time server close to you, but in most cases it's best to use pool.ntp.org to find an NTP server
// then the NTP system decides e.g. 0.pool.ntp.org, 1.pool.ntp.org as the NTP syem tries to find  the closest available servers
// EU "0.europe.pool.ntp.org"
// US "0.north-america.pool.ntp.org"
// See: https://www.ntppool.org/en/
//int   gmtOffset_sec     = 0;    // UK normal time is GMT, so GMT Offset is 0, for US (-5Hrs) is typically -18000, AU is typically (+8hrs) 28800
//int  daylightOffset_sec = 3600; // In the UK DST is +1hr or 3600-secs, other countries may use 2hrs 7200 or 30-mins 1800 or 5.5hrs 19800 Ahead of GMT use + offset behind - offset

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store

// Example time zones
//const char* Timezone = "MET-1METDST,M3.5.0/01,M10.5.0/02"; // Most of Europe
//const char* Timezone = "CET-1CEST,M3.5.0,M10.5.0/3";       // Central Europe
//const char* Timezone = "EST-2METDST,M3.5.0/01,M10.5.0/02"; // Most of Europe
//const char* Timezone = "EST5EDT,M3.2.0,M11.1.0";           // EST USA
//const char* Timezone = "CST6CDT,M3.2.0,M11.1.0";           // CST USA
//const char* Timezone = "MST7MDT,M4.1.0,M10.5.0";           // MST USA
//const char* Timezone = "NZST-12NZDT,M9.5.0,M4.1.0/3";      // Auckland
//const char* Timezone = "EET-2EEST,M3.5.5/0,M10.5.5/0";     // Asia
//const char* Timezone = "ACST-9:30ACDT,M10.1.0,M4.1.0/3":   // Australia
