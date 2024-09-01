
#include <Arduino.h>
#include <U8g2lib.h>        // OLED
#include <WiFiUdp.h>        // UDP functionality
#define NTP_PORT 123        // Time Server Port

#include <RtcDS3231.h>      // RTC functions
#include <EepromAT24C32.h   // We will use clock's eeprom to store config

// Create a WiFi access point and provide a web server on it 

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#define APSSID "GPSTimeServer" // AP SSID - set to desired server name
#define APPSK "thereisnospoon" // AP password - set to desired server password
#define PPS_PIN D6             // Pin on which 1PPS line is attached
#define SYNC_INTERVAL 10       // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT 30        // time(sec) without GPS input before error
#define RTC_UPDATE_INTERVAL 30 // time(sec) between RTC SetTime events
#define PPS_BLINK_INTERVAL 50  // Set time pps LED should be on for blink effect
#define DISPLAY_1_ADDR 0x78
#define DISPLAY_2_ADDR 0x7A

#define LOCK_LED D3
#define PPS_LED 10
#define WIFI_LED D5
#define WIFI_BUTTON D4

#include <SoftwareSerial.h>
#include <TimeLib.h>     // Time functions  https://github.com/PaulStoffregen/Time
#include <TinyGPS.h>     // GPS parsing     https://github.com/mikalhart/TinyGPS
#include <Wire.h>        // OLED and DS3231 


RtcDS3231<TwoWire> Rtc(Wire);
EepromAt24c32<TwoWire> RtcEeprom(Wire);

// The U8G2 library has many constuctors. Use the one that's appropriate for your displays -MM

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_1(U8G2_R0, U8X8_PIN_NONE);    // OLED display library parameters
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_2(U8G2_R0, U8X8_PIN_NONE);    //   (rotation, reset pin) 

//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2_1(U8G2_R0, U8X8_PIN_NONE); 
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2_2(U8G2_R0, U8X8_PIN_NONE); 

TinyGPS gps;
SoftwareSerial ss(D7, D8);              // serial GPS handler
ESP8266WebServer server(80);
WiFiUDP Udp;                            // an Ethernet UDP instance

static const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];     // buffers for receiving and sending data
time_t displayTime = 0;                 // time that is currently displayed
time_t syncTime = 0;                    // time of last GPS or RTC synchronization
time_t lastSetRTC = 0;                  // time that RTC was last set
volatile int pps = 0;                   // GPS one-pulse-per-second flag
time_t dstStart = 0;                    // start of DST in unix time
time_t dstEnd = 0;                      // end of DST in unix time
time_t lastFix;                         // time of most recent NTP response
bool gpsLocked = false;                 // indicates recent sync with GPS
int currentYear = 0;                    // used for DST
long int pps_blink_time = 0;
IPAddress clientIP;                     // address of most recent client
byte statusWifi = 1;
const char *ssid = APSSID;
const char *password = APPSK;

// ISR Debounce
#define DEBOUNCE_TICKS 150    // use 150ms debounce time
word keytick = 0;             // record time of keypress

//#define DEBUG     // comment this in order to remove debug code from release version

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x);
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC);
#define DEBUG_PRINTLN(x) Serial.println(x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

// Function prototypes

bool KeyCheck();
void HandleRoot();
void EnableWifi();
void DisableWifi();
void ProcessWifi();
void PrintDigit(int d);
void PrintTime(time_t t);
void PrintRTCstatus();
void SetRTC(time_t t);
void ManuallySetRTC();
void UpdateRTC();
void ShowDate(time_t t);
void ShowTime(time_t t);
void ShowFix();
void ShowDateTime(time_t t);
void ShowSatellites();
void ShowSyncFlag();
void SyncWithGPS();
void SyncWithRTC();
void SyncCheck();
void IRAM_ATTR Isr();
void IRAM_ATTR Btw();
void ProcessKeypress();
void FeedGpsParser();
void UpdateDisplay();
void ProcessNTP();
void InitOLED();
