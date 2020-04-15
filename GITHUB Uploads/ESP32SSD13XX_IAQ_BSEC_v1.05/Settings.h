int ONBOARD_LED = 2; // onboard blue LED
int WiFiLED = 0;  // Set WiFi disconnect status LED
char* device = "IAQ";
char* model = "ESP32_IAQBSEC";
char* ver = "v1.05";
char* fw = "v1.05.13042020";
char* fwname = "ESP32SSD13XX_IAQ_BSEC_v1.05";
char* criticalWiFi = "N";  // If Y, system reset performed after WiFi reconect fail followed by user defined timeout (criticalWiFiTimeOut) loop.
char* criticalWiFiTimeOut = "1000";  // default 1 second
char* screen_select = "Main_Screen";  // Call screen to display on base unit
const char* ssid = "zzzz";
const char* password = "zzzz";
int count = 0;
char* wifi = "WiFi";
int stop = 0;
int dwncount = 0;
int dwnflag = 0;
int upflag = 0;
const long updwntimer = 60000;
const long StartTimeout = 5000;
const long IAQStartTimeout = 10000;
int resetcounter = 0;
int DebugMode = 0;
unsigned long WIFIpreviousMillis = 0;
unsigned long LEDpreviousMillis = 0;
unsigned long IAQpreviousMillis = 0;
unsigned long BME680previousMillis = 0;
const long LEDinterval = 300;  // 3sec interval at which to flash LED (milliseconds)
String SerialNo;
// Change to your WiFi credentials and select your time zone
const char* Timezone = "GMT0BST,M3.5.0/01,M10.5.0/02";  // UK, see below for others and link to database
String Format = "X";  // Time format M for dd-mm-yy and 23:59:59, "I" for mm-dd-yy and 12:59:59 PM, "X" for Metric units.
static String Day_str, Time_str, WkDay_str, Mth_str, UpTime_str, DwnTime_str, PrevTime_str, PrevTemp, PrevHum, PrevPress, Prevco2, Prevvoc, PrevIAQacc, IAQ, PrevIAQ;
static float air_quality_score, final_air_quality_score, current_temp, current_pressure, current_humidity, current_co2, current_voc, iaq_score, iaq_acc;

volatile unsigned int local_Unix_time = 0, next_update_due = 0;
volatile unsigned int update_duration = 60 * 60;  // Time duration in seconds, so synchronise every hour
static unsigned int   Last_Event_Time;
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time TS was updated
const long interval = 1800000;           // 30min interval at which to run Write_TS()(milliseconds)
const long IAQinterval = 10000;
uint64_t chipid;

Bsec iaqSensor;

String output;

float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score
int   humidity_score, gas_score;
float gas_reference = 2500;
float hum_reference = 40;
int   getgasreference_count = 0;
int   gas_lower_limit = 10000;  // Bad air quality limit
int   gas_upper_limit = 300000; // Good air quality limit
