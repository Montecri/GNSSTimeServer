// WiFi enabled GPS NTP server - Cristiano Monteiro <cristianomonteiro@gmail.com> - 06.May.2021
// Based on the work of:
// Bruce E. Hall, W8BH <bhall66@gmail.com> - http://w8bh.net
// and
// https://forum.arduino.cc/u/ziggy2012/summary
//
// Contributions from:
// https://github.com/mmarkin
// https://github.com/sjthespian

#include <Arduino.h>
#include <U8g2lib.h>

// State data
#include <LittleFS.h>

// For time on the web page
#include <ctime> // For formatting time

// Needed for UDP functionality
#include <WiFiUdp.h>
// Time Server Port
#define NTP_PORT 123
static const int NTP_PACKET_SIZE = 48;
// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE];
// An Ethernet UDP instance
WiFiUDP Udp;

// Syslog support
#include <Syslog.h>

/* Create a WiFi access point and provide a web server on it. */
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <RtcUtility.h>
#include <EepromAT24C32.h> // We will use clock's eeprom to store config

// function prototypes
void processWifi(); // Need to declare this for handleUpdate()
void UpdateDisplay();
void processKeyHold();
void processKeyPress();

// GLOBAL DEFINES
#define HOSTNAME "ESP-NTP-Server" // Hostname used for syslog and DHCP
#define WIFIRETRIES 15            // Max number of wifi retry attempts
#define APSSID "GPSTimeServer"    // Default AP SSID
#define APPSK "thereisnospoon"    // Default password
#define PPS_PIN D6                // Pin on which 1PPS line is attached
#define SYNC_INTERVAL 10          // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT 30           // time(sec) without GPS input before error
// #define RTC_UPDATE_INTERVAL    SECS_PER_DAY             // time(sec) between RTC SetTime events
#define RTC_UPDATE_INTERVAL 30 // time(sec) between RTC SetTime events
#define PPS_BLINK_INTERVAL 50  // Set time pps led should be on for blink effect
// #define SYSLOG_SERVER "x.x.x.x" // syslog server name or IP
#define SYSLOG_PORT 514 // syslog port

#define LOCK_LED D3
#define PPS_LED 10
#define WIFI_LED D5
#define WIFI_BUTTON D4
#define BTN_HOLD_MS 2000 // Number of milliseconds to determine button being held
#define BTN_NONE 0       // No button press
#define BTN_PRESS 1      // Button pressed for < BTN_HOLD_MS
#define BTN_HOLD 2       // Button held for at least BTN_HOLD_MS

// INCLUDES
#include <SoftwareSerial.h>
#include <TimeLib.h>     // Time functions  https://github.com/PaulStoffregen/Time
#include <TinyGPSPlus.h> // GPS parsing     https://github.com/mikalhart/TinyGPSPlus
#include <Wire.h>        // OLED and DS3231 necessary
#include <RtcDS3231.h>   // RTC functions

RtcDS3231<TwoWire> Rtc(Wire);
EepromAt24c32<TwoWire> RtcEeprom(Wire);

// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // OLED display library parameters
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // OLED display library parameters

TinyGPSPlus gps;
SoftwareSerial ss(D7, D8); // Serial GPS handler
time_t displayTime = 0;    // time that is currently displayed
time_t syncTime = 0;       // time of last GPS or RTC synchronization
time_t lastSetRTC = 0;     // time that RTC was last set
volatile int pps = 0;      // GPS one-pulse-per-second flag
time_t dstStart = 0;       // start of DST in unix time
time_t dstEnd = 0;         // end of DST in unix time
bool gpsLocked = false;    // indicates recent sync with GPS
int currentYear = 0;       // used for DST
int displaynum = 0;        // Display pane currently displayed
#define NUMDISPLAYPANES 2  // Number of display panes available

long int pps_blink_time = 0;

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;
String wifissid;
String wifipassword;
String syslogserver;
uint8_t statusWifi = 1;

ESP8266WebServer server(80);

// rdate server
WiFiServer RFC868server(37);

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udpClient;
// Create syslogt instance with LOG_DAEMON
// Create a new empty syslog instance
Syslog syslog(udpClient, SYSLOG_PROTO_IETF);
/*
 * ISR Debounce
 */

// use 150ms debounce time
#define DEBOUNCE_TICKS 100

word keytick_down = 0; // record time of keypress
word keytick_up = 0;
int lastState = HIGH; // record last button state to support debouncing

// #define DEBUG // Comment this in order to remove debug code from release version
// #define DEBUG_GPS // Uncomment this to receive GPS messages in debug output

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

// Button ISR debouncing routine

// returns true if key pressed

void KeyCheck()
{
  // Button pressed and released
  int button_type = 0;
  if (keytick_up > 0 and keytick_down > 0 and ((keytick_down - keytick_up) >= DEBOUNCE_TICKS))
  {
    if ((keytick_down - keytick_up) > BTN_HOLD_MS)
    {
      button_type = BTN_HOLD;
    }
    else
    {
      button_type = BTN_PRESS;
    }

    // return button_type;
    switch (button_type)
    {
    case BTN_HOLD: // Malabarism to cover mechanical switch debouncing
      processKeyHold();
      break;
    case BTN_PRESS:
      processKeyPress();
      break;
    }
  }
  // return BTN_NONE;
}

// littleFS routines

String readData(const char *filename)
{
  DEBUG_PRINT("Reading data from ");
  DEBUG_PRINT(filename);
  File file = LittleFS.open(filename, "r");

  String data = "";
  if (!file)
  {
    DEBUG_PRINTLN("ERROR: File open failed!");
  }
  else
  {

    if (file.available())
    {
      data = file.readString();
    }
    DEBUG_PRINT(" : ");
    DEBUG_PRINTLN(data);

    file.close();
  }

  return data;
}

void writeData(const char *filename, String data)
{
  DEBUG_PRINT("Writing data to ");
  DEBUG_PRINT(filename);
  File file = LittleFS.open(filename, "w");

  if (!file)
  {
    DEBUG_PRINTLN("ERROR: File open failed!");
  }
  else
  {
    file.println(data);
    DEBUG_PRINT(" : ");
    DEBUG_PRINTLN(data);
  }

  file.close();
}

// WiFi and Web Routines

void handleUpdate()
{
  wifissid = server.arg("wifi_ssid");
  wifipassword = server.arg("wifi_psk");
  syslogserver = server.arg("syslog_server");
  DEBUG_PRINTLN("Settings Updated!");
  DEBUG_PRINTLN(wifissid);
  //  DEBUG_PRINTLN(wifipassword);
  // Save SSID and password in littlefs
  writeData("/wifissid", wifissid);
  if (wifipassword != "")
    writeData("/wifipsk", wifipassword);
  writeData("/syslogserver", syslogserver);

  processWifi();

  String content = "<a href='/'>Return to main page</a>";
  server.send(200, "text/html", content);
}

void handleRoot()
{
  char webpage[2048];
  char timestr[32];

  // Build string for web UI
  time_t t = now(); // get current time
  tm *ptm = gmtime(&t);
  strftime(timestr, 32, "%Y-%m-%d %H:%M:%S UTC", ptm);

  int sats = (gps.satellites.value() != 255) ? gps.satellites.value() : 0;
  String resol = gpsLocked ? String(gps.hdop.value()) : "";

  // latitude & longitude
  long lat = 0.0;
  long lng = 0.0;
  if (gpsLocked)
  {
    lat = gps.location.lat();
    lng = gps.location.lng();
  }

  char form[512] = {0};
  //  if (WiFi.getMode() == WIFI_AP) {
  sprintf(form, "<hr/><h3>Settings</h3><form action=\"updatewifi\">Syslog server: <input type=\"text\" name=\"syslog_server\" value=\"%s\"><br/>SSID: <input type=\"text\" name=\"wifi_ssid\" value=\"%s\"><br/>Password: <input type=\"password\" name=\"wifi_psk\"><br/><input type=\"submit\"></form>", syslogserver.c_str(), wifissid.c_str());
  //  }

  sprintf(webpage,
          "<html><head><title>NTP Server</title></head><body><h1>%s</h1>Satellites: %d  Resolution: %s<h3>Location</h3>Latitude: %7.4f, Longitude: %7.4f<br/>%s</body></html>",
          timestr,
          sats,
          resol.c_str(),
          (float)lat / 1000000,
          (float)lng / 1000000,
          form);
  server.send(200, "text/html", webpage);
}

void startHttpServer()
{
  server.on("/", handleRoot);
  server.on("/updatewifi", handleUpdate);
  server.begin();
  DEBUG_PRINTLN(F("HTTP server started"));
  syslog.log(LOG_INFO, "HTTP server started");
}

void enableWifiAP()
{
  // WiFi Initialization as an AP
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.mode(WIFI_AP);
  // WiFi.softAP(ssid, psk, channel, hidden, max_connection)
  // Setting maximum of 8 clients, 1 is default channel already, 0 is false for hidden SSID - The maximum allowed by ES8266 is 8 - Thanks to Mitch Markin for that
  WiFi.softAP(ssid, password, 1, 0, 8);

#ifdef DEBUG
  IPAddress myIP = WiFi.softAPIP();
  DEBUG_PRINT(F("AP IP address: "));
  DEBUG_PRINTLN(myIP);
#endif
  startHttpServer();
}

void enableWifi()
{
  // Get SSID and password from littleFS
  wifissid = readData("/wifissid");
  wifipassword = readData("/wifipsk"); // Password follows

  wifissid.trim();
  wifipassword.trim();
  // Connect to WiFi
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  DEBUG_PRINTLN("Connecting to WiFI");
  DEBUG_PRINTLN(wifissid);
  DEBUG_PRINTLN(wifipassword);
  WiFi.begin(wifissid, wifipassword);
  int retries = 0;
  while ((WiFi.status() != WL_CONNECTED) && (retries < WIFIRETRIES))
  {
    retries++;
    delay(500);
    DEBUG_PRINT(".");
  }
  if (retries >= WIFIRETRIES)
  {
    enableWifiAP();
  }
  if (WiFi.status() == WL_CONNECTED)
  {
#ifdef DEBUG
    IPAddress myIP = WiFi.localIP();
    if (myIP[0] == 0)
      myIP = WiFi.softAPIP();
    DEBUG_PRINTLN(F("WiFi connected!"));
    DEBUG_PRINT("IP address: ");
    DEBUG_PRINTLN(myIP);
    syslog.logf(LOG_INFO, "WiFi connected as %s", myIP.toString().c_str());
#endif
    startHttpServer();
  }
}

void disableWifi()
{
  server.stop();
  DEBUG_PRINTLN(F("HTTP server stopped"));
  syslog.log(LOG_WARNING, "WiFi disabled!");
  if (WiFi.getMode() == WIFI_AP)
  {
    WiFi.softAPdisconnect(true);
    WiFi.enableAP(false);
  }
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  DEBUG_PRINTLN(F("WiFi disabled"));
}

void processWifi()
{
  // Toggle WiFi on/off and corresponding LED
  DEBUG_PRINT(F("Status Wifi: "));
  DEBUG_PRINTLN(statusWifi);

  if (statusWifi)
  {
    enableWifi();
    digitalWrite(WIFI_LED, HIGH);
  }
  else
  {
    disableWifi();
    digitalWrite(WIFI_LED, LOW);
  }
}

// --------------------------------------------------------------------------------------------------
// SERIAL MONITOR ROUTINES
// These routines print the date/time information to the serial monitor
// Serial monitor must be initialized in setup() before calling

void PrintDigit(int d)
{
  if (d < 10)
    DEBUG_PRINT('0');
  DEBUG_PRINT(d);
}

void PrintTime(time_t t)
// display time and date to serial monitor
{
  PrintDigit(month(t));
  DEBUG_PRINT("-");
  PrintDigit(day(t));
  DEBUG_PRINT("-");
  PrintDigit(year(t));
  DEBUG_PRINT(" ");
  PrintDigit(hour(t));
  DEBUG_PRINT(":");
  PrintDigit(minute(t));
  DEBUG_PRINT(":");
  PrintDigit(second(t));
  DEBUG_PRINTLN(" UTC");
}

// --------------------------------------------------------------------------------------------------
//  RTC SUPPORT
//  These routines add the ability to get and/or set the time from an attached real-time-clock module
//  such as the DS1307 or the DS3231.  The module should be connected to the I2C pins (SDA/SCL).

void PrintRTCstatus()
// send current RTC information to serial monitor
{
  RtcDateTime Now = Rtc.GetDateTime();
  time_t t = Now.Epoch32Time();
  if (t)
  {
    DEBUG_PRINT("PrintRTCstatus: ");
    DEBUG_PRINTLN("Called PrintTime from PrintRTCstatus");
#ifdef DEBUG
    PrintTime(t);
#endif
  }
  else
  {
    DEBUG_PRINTLN("ERROR: cannot read the RTC.");
    syslog.log(LOG_ERR, "ERROR: cannot read the RTC.");
  }
}

// Update RTC from current system time
void SetRTC(time_t t)
{
  RtcDateTime timeToSet;

  timeToSet.InitWithEpoch32Time(t);

  Rtc.SetDateTime(timeToSet);
  if (Rtc.LastError() == 0)
  {
    DEBUG_PRINT("SetRTC: ");
    DEBUG_PRINTLN("Called PrintTime from SetRTC");
#ifdef DEBUG
    PrintTime(t);
#endif
  }
  else
  {
    DEBUG_PRINT("ERROR: cannot set RTC time");
    syslog.log(LOG_ERR, "ERROR: cannot set RTC time");
  }
}

void ManuallySetRTC()
// Use this routine to manually set the RTC to a specific UTC time.
// Since time is automatically set from GPS, this routine is mainly for
// debugging purposes.  Change numeric constants to the time desired.
{
  //  tmElements_t tm;
  //  tm.Year   = 2017 - 1970;                              // Year in unix years
  //  tm.Month  = 5;
  //  tm.Day    = 31;
  //  tm.Hour   = 5;
  //  tm.Minute = 59;
  //  tm.Second = 30;
  //  SetRTC(makeTime(tm));                                 // set RTC to desired time
}

void UpdateRTC()
// keep the RTC time updated by setting it every (RTC_UPDATE_INTERVAL) seconds
// should only be called when system time is known to be good, such as in a GPS sync event
{
  time_t t = now();                            // get current time
  if ((t - lastSetRTC) >= RTC_UPDATE_INTERVAL) // is it time to update RTC internal clock?
  {
    DEBUG_PRINT("Called SetRTC from UpdateRTC with ");
    DEBUG_PRINTLN(t);
    SetRTC(t);      // set RTC with current time
    lastSetRTC = t; // remember time of this event
  }
}

// --------------------------------------------------------------------------------------------------
// LCD SPECIFIC ROUTINES
// These routines are used to display time and/or date information on the LCD display
// Assumes the presence of a global object "lcd" of the type "LiquidCrystal" like this:
//    LiquidCrystal   lcd(6,9,10,11,12,13);
// where the six numbers represent the digital pin numbers for RS,Enable,D4,D5,D6,and D7 LCD pins

void ShowDate(time_t t)
{
  String data = "";

  int y = year(t);
  if (y < 10)
    data = data + "0";
  data = data + String(y) + "-";

  int m = month(t);
  if (m < 10)
    data = data + "0";
  data = data + String(m) + "-";

  int d = day(t);
  if (d < 10)
    data = data + "0";
  data = data + String(d);

  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 43, 107);

  u8g2.setFont(u8g2_font_logisoso16_tf); // choose a suitable font
  u8g2.drawStr(18, 43, data.c_str());

  DEBUG_PRINTLN("UpdateDisplay");
}

void ShowTime(time_t t)
{
  String hora = "";
  int h = hour(t);
  if (h < 10)
    hora = hora + "0";
  hora = hora + String(h) + ":";

  int m = minute(t);
  if (m < 10)
    hora = hora + "0";
  hora = hora + String(m) + ":";

  int s = second(t);
  if (s < 10)
    hora = hora + "0";
  hora = hora + String(s) + " UTC";

  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 64, 123);

  u8g2.setFont(u8g2_font_logisoso16_tf); // choose a suitable font
  u8g2.drawStr(16, 64, hora.c_str());
}

void ShowDateTime(time_t t)
{
  ShowDate(t);

  ShowTime(t);
}

void ShowSyncFlag()
{
  String sats = "";
  if (gps.satellites.value() != 255)
    sats = String(gps.satellites.value());
  else
    sats = "0";

  if (gpsLocked)
    digitalWrite(LOCK_LED, HIGH);
  else
    digitalWrite(LOCK_LED, LOW);

  String resol = "";
  if (gpsLocked)
    resol = String(gps.hdop.value());
  else
    resol = "0";

  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 16, 259);
  u8g2.drawGlyph(64, 16, 263);

  u8g2.setFont(u8g2_font_logisoso16_tf); // choose a suitable font
  u8g2.drawStr(18, 16, sats.c_str());
  u8g2.drawStr(82, 16, resol.c_str());
}

void ShowWifiNetwork()
{
  String wifinet = "";

  if (WiFi.getMode() == WIFI_STA)
    wifinet = wifissid;
  else
    wifinet = APSSID;

  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 43, 248);

  int wrap = 38;
  int ypos1 = 28;
  int ypos2 = 43;
  // Play games with the font and spacing based on the length of the SSID
  if (wifinet.length() > 30)
  {
    u8g2.setFont(u8g2_font_6x12_tf); // Small font for long SSID
    wrap = 18;
  }
  else if (wifinet.length() > 20)
  {
    u8g2.setFont(u8g2_font_7x13_tf); // Not quite as small
    wrap = 15;
  }
  else if (wifinet.length() > 11)
  {
    u8g2.setFont(u8g2_font_7x13_tf); // Not quite as small on one line
    ypos1 = 43;
  }
  else
  {
    u8g2.setFont(u8g2_font_10x20_tf); // Readable on one line
    ypos1 = 43;
  }
  u8g2.drawStr(16, ypos1, wifinet.substring(0, wrap).c_str());
  if (wifinet.length() > 20) // 2nd line needed if > 20
    u8g2.drawStr(16, ypos2, wifinet.substring(wrap).c_str());
}

void ShowIPAddress()
{
  IPAddress myIP = WiFi.localIP();
  if (myIP[0] == 0)
    myIP = WiFi.softAPIP();

  u8g2.setFont(u8g2_font_logisoso16_tr); // choose a suitable font
  u8g2.drawStr(0, 64, myIP.toString().c_str());
}

void InitLCD()
{
  u8g2.begin(); // Initialize OLED library
}

// --------------------------------------------------------------------------------------------------
// TIME SYNCHONIZATION ROUTINES
// These routines will synchonize time with GPS and/or RTC as necessary
// Sync with GPS occur when the 1pps interrupt signal from the GPS goes high.
// GPS synchonization events are attempted every (SYNC_INTERVAL) seconds.
// If a valid GPS signal is not received within (SYNC_TIMEOUT) seconds, the clock with synchonized
// with RTC instead.  The RTC time is updated with GPS data once every 24 hours.

void SyncWithGPS()
{
  int y;
  // byte h, m, s, mon, d, hundredths;
  byte h, m, s, mon, d;
  unsigned long age;
  y = gps.date.year();
  mon = gps.date.month();
  d = gps.date.day();
  h = gps.time.hour();
  m = gps.time.minute();
  s = gps.time.second();
  age = gps.location.age();
  // gps.crack_datetime(&y, &mon, &d, &h, &m, &s, NULL, &age); // get time from GPS
  //  cheise @ Github spotted the uneccessary and wrong '> 3000' condition. Fixed - 20230206
  //  if (age < 1000 or age > 3000)
  if (age < 1000) // dont use data older than 1 second
  {
    setTime(h, m, s, d, mon, y); // copy GPS time to system time
    DEBUG_PRINT("Time from GPS: ");
    DEBUG_PRINT(h);
    DEBUG_PRINT(":");
    DEBUG_PRINT(m);
    DEBUG_PRINT(":");
    DEBUG_PRINTLN(s);
    adjustTime(1);    // 1pps signal = start of next second
    syncTime = now(); // remember time of this sync
    if (!gpsLocked)
    {
      syslog.logf(LOG_INFO, "GPS sychronized - %d satellites", gps.satellites.value());
    }
    gpsLocked = true;                  // set flag that time is reflects GPS time
    UpdateRTC();                       // update internal RTC clock periodically
    DEBUG_PRINTLN("GPS synchronized"); // send message to serial monitor
  }
  else
  {
    DEBUG_PRINT("Age: ");
    DEBUG_PRINTLN(age);
  }
}

void SyncWithRTC()
{
  RtcDateTime time = Rtc.GetDateTime();
  long int a = time.Epoch32Time();
  setTime(a); // set system time from RTC
  DEBUG_PRINT("SyncFromRTC: ");
  DEBUG_PRINTLN(a);
  syncTime = now();                       // and remember time of this sync event
  DEBUG_PRINTLN("Synchronized from RTC"); // send message to serial monitor
}

void SyncCheck()
// Manage synchonization of clock to GPS module
// First, check to see if it is time to synchonize
// Do time synchonization on the 1pps signal
// This call must be made frequently (keep in main loop)
{
  unsigned long timeSinceSync = now() - syncTime; // how long has it been since last sync?
  if (pps && (timeSinceSync >= SYNC_INTERVAL))
  { // is it time to sync with GPS yet?
    DEBUG_PRINTLN("Called SyncWithGPS from SyncCheck");
    SyncWithGPS(); // yes, so attempt it.
  }
  pps = 0;                           // reset 1-pulse-per-second flag, regardless
  if (timeSinceSync >= SYNC_TIMEOUT) // GPS sync has failed
  {
    if (gpsLocked)
    {
      syslog.log(LOG_INFO, "GPS sych lost!");
    }
    gpsLocked = false; // flag that clock is no longer in GPS sync
    DEBUG_PRINTLN("Called SyncWithRTC from SyncCheck");
    SyncWithRTC(); // sync with RTC instead
  }
}

// --------------------------------------------------------------------------------------------------
// MAIN PROGRAM

void IRAM_ATTR isr() // INTERRUPT SERVICE REQUEST
{
  pps = 1;                     // Flag the 1pps input signal
  digitalWrite(PPS_LED, HIGH); // Ligth up led pps monitor
  pps_blink_time = millis();   // Capture time in order to turn led off so we can get the blink effect ever x milliseconds - On loop
  DEBUG_PRINTLN("pps");
}

// Handle button pressed interrupt
void IRAM_ATTR btw() // INTERRUPT SERVICE REQUEST
{
  int currentState = digitalRead(WIFI_BUTTON);
  if (currentState != lastState)
  {
    if (lastState == HIGH)
    {
      keytick_up = millis();
    }
    else
    {
      keytick_down = millis();
      KeyCheck();
    }
    lastState = currentState;
  }
  DEBUG_PRINTLN(F("BUTTON PRESSED!"));
}

void processKeyHold()
{
  if (statusWifi)
    statusWifi = 0;
  else
    statusWifi = 1;

  processWifi();
  DEBUG_PRINTLN(F("BUTTON HOLD PROCESSED!"));
}

void processKeyPress()
{
  DEBUG_PRINTLN(F("BUTTON CLICK PROCESSED!"));
  displaynum++;
  if (displaynum >= NUMDISPLAYPANES)
  {
    displaynum = 0;
  }
  UpdateDisplay();
}

void setup()
{
  pinMode(LOCK_LED, OUTPUT);
  pinMode(PPS_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(WIFI_BUTTON, INPUT_PULLUP);

  digitalWrite(LOCK_LED, LOW);
  digitalWrite(PPS_LED, LOW);
  digitalWrite(WIFI_LED, LOW);
  // if you are using ESP-01 then uncomment the line below to reset the pins to
  // the available pins for SDA, SCL
  Wire.begin(D2, D1); // due to limited pins, use pin 0 and 2 for SDA, SCL
  Rtc.Begin();
  RtcEeprom.Begin();

  LittleFS.begin(); // Init storage for WiFi SSID/PSK

  InitLCD(); // initialize LCD display

  ss.begin(9600); // set GPS baud rate to 9600 bps
#ifdef DEBUG
  Serial.begin(9600); // set serial monitor rate to 9600 bps
#endif

  //Serial.begin(9600);
  delay(2000);

  syslogserver = readData("/syslogserver"); // Password follows
  syslogserver.trim();

  IPAddress ip;
  ip.fromString(syslogserver);
  // Syslog syslog(udpClient, syslogserver, SYSLOG_PORT, HOSTNAME, "esp-ntp", LOG_DAEMON);
  syslog.server(ip, SYSLOG_PORT);
  syslog.deviceHostname(HOSTNAME);
  syslog.appName("esp-ntp");
  syslog.defaultPriority(LOG_DAEMON);

  DEBUG_PRINTLN("Iniciado");

  // Initialize RTC
  while (!Rtc.GetIsRunning())
  {
    Rtc.SetIsRunning(true);
    DEBUG_PRINTLN(F("RTC had to be force started"));
    syslog.log(LOG_WARNING, "RTC had to be force started");
  }

  DEBUG_PRINTLN(F("RTC started"));

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);

#ifdef DEBUG
  PrintRTCstatus(); // show RTC diagnostics
#endif
  SyncWithRTC();                         // start clock with RTC data
  attachInterrupt(PPS_PIN, isr, RISING); // enable GPS 1pps interrupt input
  attachInterrupt(WIFI_BUTTON, btw, CHANGE);

  processWifi();

  // Startup UDP
  Udp.begin(NTP_PORT);
  RFC868server.begin();
}

void FeedGpsParser()
// feed currently available data from GPS module into tinyGPS parser
{
  while (ss.available()) // look for data from GPS module
  {
    char c = ss.read(); // read in all available chars
    gps.encode(c);      // and feed chars to GPS parser
                        // Serial.write(c); // Uncomment for some extra debug info if in doubt about GPS feed
#ifdef DEBUG_GPS
    DEBUG_PRINT(c);
#endif
    //Serial.print(c);
  }
}

void UpdateDisplay()
//  Call this from the main loop
//  Updates display if time has changed
{
  time_t t = now();     // get current time
  if (t != displayTime) // has time changed?
  {
    u8g2.clearBuffer(); // Clear buffer contents
    switch (displaynum)
    {
    case 0:
      ShowDateTime(t); // Display the new UTC time
      ShowSyncFlag();  // show if display is in GPS sync
      break;
    case 1:
      ShowWifiNetwork();
      ShowIPAddress();
      break;
    }
    u8g2.sendBuffer(); // Send new information to display

    displayTime = t; // save current display value
    DEBUG_PRINTLN("Called PrintTime from UpdateDisplay");
#ifdef DEBUG
    PrintTime(t); // copy time to serial monitor
#endif
  }
}

////////////////////////////////////////

const uint8_t daysInMonth[] PROGMEM = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // const or compiler complains

const unsigned long seventyYears = 2208988800UL; // to convert unix time to epoch

// Replaced with better, less verbose, more elegant timestamp = t + seventyYears calculation, suggested by cheise @ Github - 20230206
// NTP since 1900/01/01
// static unsigned long int numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mm, uint8_t s)
// {
//   if (y >= 1970)
//     y -= 1970;
//   uint16_t days = d - 1;
//   for (uint8_t i = 1; i < m; ++i)
//     days += pgm_read_byte(daysInMonth + i - 1);
//   if (m > 2 && y % 4 == 0)
//     ++days;
//   days += 365 * y + (y + 3) / 4 - 1;
//   return days * 24L * 3600L + h * 3600L + mm * 60L + s + seventyYears;
// }

////////////////////////////////////////

// Process rdate request
// Return the time as the number of seconds since 1/1/1900
void processRFC868()
{
  uint32_t timestamp;
  WiFiClient client = RFC868server.accept();
  client.setNoDelay(true);

  if (client.connected())
  {
    syslog.logf(LOG_INFO, "RDATE request from %s", client.remoteIP().toString().c_str());
    // Serial.println(client.remoteIP().toString().c_str());

    // Send Data to connected client
    time_t t = now(); // get current time

    timestamp = t + seventyYears;

    uint8_t bytes[4];

    // Populate big endian byte array
    bytes[0] = (timestamp >> 24) & 0xFF;
    bytes[1] = (timestamp >> 16) & 0xFF;
    bytes[2] = (timestamp >> 8) & 0xFF;
    bytes[3] = (timestamp >> 0) & 0xFF;

    client.write((const uint8_t *)bytes, sizeof(timestamp));
    // Serial.println(timestamp);

    // Disconnect client
    client.stop();
  }
}

void processNTP()
{

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();

    syslog.logf(LOG_INFO, "NTP request from %s", Remote.toString().c_str());

#ifdef DEBUG
    Serial.println();
    Serial.print("Received UDP packet size ");
    Serial.println(packetSize);
    Serial.print("From ");

    for (int i = 0; i < 4; i++)
    {
      Serial.print(Remote[i], DEC);
      if (i < 3)
      {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.print(PortNum);

    byte LIVNMODE = packetBuffer[0];
    Serial.print("  LI, Vers, Mode :");
    Serial.print(LIVNMODE, HEX);

    byte STRATUM = packetBuffer[1];
    Serial.print("  Stratum :");
    Serial.print(STRATUM, HEX);

    byte POLLING = packetBuffer[2];
    Serial.print("  Polling :");
    Serial.print(POLLING, HEX);

    byte PRECISION = packetBuffer[3];
    Serial.print("  Precision :");
    Serial.println(PRECISION, HEX);

    for (int z = 0; z < NTP_PACKET_SIZE; z++)
    {
      Serial.print(packetBuffer[z], HEX);
      if (((z + 1) % 4) == 0)
      {
        Serial.println();
      }
    }
    Serial.println();

    syslog.logf(LOG_INFO, "   Received UDP packet size %d  LI, Vers, Mode : 0x%02x  Stratum: 0x%02x  Polling: 0x%02x  Precision: 0x%02x", packetSize, LIVNMODE, STRATUM, POLLING, PRECISION);
#endif

    packetBuffer[0] = 0b00100100; // LI, Version, Mode
    if (gpsLocked)
    {
      packetBuffer[1] = 1; // stratum 1 if synced with GPS
    }
    else
    {
      packetBuffer[1] = 16; // stratum 16 if not synced
    }
    // think that should be at least 4 or so as you do not use fractional seconds
    // packetBuffer[1] = 4;    // stratum
    packetBuffer[2] = 6;    // polling minimum
    packetBuffer[3] = 0xFA; // precision

    packetBuffer[4] = 0; // root delay
    packetBuffer[5] = 0;
    packetBuffer[6] = 8;
    packetBuffer[7] = 0;

    packetBuffer[8] = 0; // root dispersion
    packetBuffer[9] = 0;
    packetBuffer[10] = 0xC;
    packetBuffer[11] = 0;

    // int year;
    // byte month, day, hour, minute, second, hundredths;
    // unsigned long date, time, age;
    uint32_t timestamp, tempval;
    time_t t = now();

    // gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    // timestamp = numberOfSecondsSince1900Epoch(year,month,day,hour,minute,second);

    // timestamp = numberOfSecondsSince1900Epoch(year(t), month(t), day(t), hour(t), minute(t), second(t));
    //  Better, less verbose, more elegant timestamp calculation, suggested by cheise @ Github - 20230206
    timestamp = t + seventyYears;

#ifdef DEBUG
    Serial.println(timestamp);
    // print_date(gps);
#endif

    tempval = timestamp;

    if (gpsLocked)
    {
      packetBuffer[12] = 71; //"G";
      packetBuffer[13] = 80; //"P";
      packetBuffer[14] = 83; //"S";
      packetBuffer[15] = 0;  //"0";
    }
    else
    {
      // Set refid to IP address if not locked
      IPAddress myIP = WiFi.localIP();
      packetBuffer[12] = myIP[0];
      packetBuffer[13] = myIP[1];
      packetBuffer[14] = myIP[2];
      packetBuffer[15] = myIP[3];
    }

    // reference timestamp
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval)&0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

    // copy originate timestamp from incoming UDP transmit timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    // receive timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval)&0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    // transmitt timestamp
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval)&0xFF;

    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;

    // Reply to the IP address and port that sent the NTP request

    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
}

////////////////////////////////////////

void loop()
{
  FeedGpsParser();                                    // decode incoming GPS data
  SyncCheck();                                        // synchronize to GPS or RTC
  UpdateDisplay();                                    // if time has changed, display it
  if (millis() - pps_blink_time > PPS_BLINK_INTERVAL) // If x milliseconds passed, then it's time to switch led off for blink effect
    digitalWrite(PPS_LED, LOW);
  server.handleClient();
  processNTP();
  processRFC868();
}
