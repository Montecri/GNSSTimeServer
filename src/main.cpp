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
// #include <esp_wifi.h>

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

// Main Web Page and update page
// #include "index.h"
#include "index_bytearray.h"
#include "ota.h"

#define ETHERNET_ENABLED

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>

#define HOSTNAME "GNSSTimeServerV1" // Hostname used for syslog and DHCP

ESP8266WebServer server(80);
SoftwareSerial ss(D7, D8); // Serial GPS handler
#define PPS_LED 10
#define LOCK_LED 0U    // ESP8266 D3
#define WIFI_BUTTON 2U // ESP8266 D4
#define AP_CHANNEL 1
#elif defined(ESP32)
#include "WiFi.h"
#include <HardwareSerial.h>
#include <WebServer.h>
// Web OTA Upgrade
#include <Update.h>

HardwareSerial ss(2);
WebServer server(80);

#define PPS_LED 27

#if defined(ETHERNET_ENABLED)
#include <ETH.h>
#include "driver/spi_master.h"
#include <SPI.h>

#define MO 23
#define MI 19
#define CL 18
#define CS 5
#define INT 25

#define LOCK_LED 13
#define WIFI_BUTTON 26

#define AP_CHANNEL 3
#define HOSTNAME "GNSSTimeServerV2WE" // Hostname used for syslog and DHCP
#else
#define LOCK_LED 19
#define WIFI_BUTTON 5

#define AP_CHANNEL 2
// #define LOCK_LED 13
// #define WIFI_BUTTON 26
#define HOSTNAME "GNSSTimeServerV2W" // Hostname used for syslog and DHCP
#endif

// Priority tasks
// TaskHandle_t FGP; // FeedGPSParser
// TaskHandle_t SC;  // SyncCheck
// TaskHandle_t PN;  // ProcessNTP
// TaskHandle_t PR;  // ProcessRFC868

// Required for temperature reading
#ifdef __cplusplus
extern "C"
{
#endif
  uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();
#else
#error Unknown architecture
#endif

/* Create a WiFi access point and provide a web server on it. */
#include <WiFiClient.h>
#include <RtcUtility.h>
#include <EepromAT24C32.h> // We will use clock's eeprom to store config

// function prototypes
void processWifi(); // Need to declare this for handleUpdate()
void UpdateDisplay();
void processKeyHold();
void processKeyPress();

// GLOBAL DEFINES
#define WIFIRETRIES 15 // Max number of wifi retry attempts
// #define APSSID "GNSSTimeServer"    // Default AP SSID
// #define APPSK "thereisnospoon"     // Default password
#define PPS_PIN 12U      // ESP8266 D6 - Pin on which 1PPS line is attached
#define SYNC_INTERVAL 10 // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT 30  // time(sec) without GPS input before error
// #define RTC_UPDATE_INTERVAL    SECS_PER_DAY             // time(sec) between RTC SetTime events
#define RTC_UPDATE_INTERVAL 30 // time(sec) between RTC SetTime events
#define PPS_BLINK_INTERVAL 50  // Set time pps led should be on for blink effect
// #define SYSLOG_SERVER "x.x.x.x" // syslog server name or IP
#define SYSLOG_PORT 514 // syslog port

const char *www_username = "admin";
const char *www_password = "esp32";
// allows you to set the realm of authentication Default:"Login Required"
const char *www_realm = "Custom Auth Realm";
// the Content of the HTML response in case of Unautherized Access Default:empty
String authFailResponse = "Authentication Failed";
//const char *serverUpdate = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

#define WIFI_LED 14U     // ESP8266 D5
#define BTN_HOLD_MS 2000 // Number of milliseconds to determine button being held
#define BTN_NONE 0       // No button press
#define BTN_PRESS 1      // Button pressed for < BTN_HOLD_MS
#define BTN_HOLD 2       // Button held for at least BTN_HOLD_MS

// INCLUDES
#include <TimeLib.h>     // Time functions  https://github.com/PaulStoffregen/Time
#include <TinyGPSPlus.h> // GPS parsing     https://github.com/mikalhart/TinyGPSPlus
#include <Wire.h>        // OLED and DS3231 necessary
#include <RtcDS3231.h>   // RTC functions

RtcDS3231<TwoWire> Rtc(Wire);
EepromAt24c32<TwoWire> RtcEeprom(Wire);

// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // OLED display library parameters
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // OLED display library parameters

TinyGPSPlus gps;

time_t displayTime = 0;     // time that is currently displayed
time_t syncTime = 0;        // time of last GPS or RTC synchronization
time_t lastSetRTC = 0;      // time that RTC was last set
volatile int pps = 0;       // GPS one-pulse-per-second flag
time_t dstStart = 0;        // start of DST in unix time
time_t dstEnd = 0;          // end of DST in unix time
bool gpsLocked = false;     // indicates recent sync with GPS
bool buttonPressed = false; // To process button presses outside the ISR
int currentYear = 0;        // used for DST
int displaynum = 0;         // Display pane currently displayed
#define NUMDISPLAYPANES 2   // Number of display panes available

long int pps_blink_time = 0;

/* Set these to your desired credentials. */
const char *ssid = "GNSSTimeServer";
const char *password = "thereisnospoon";
String wifissid;
String wifipassword;
String syslogserver;
uint8_t statusWifi = 1;

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
#if defined(ARDUINO_ARCH_ESP8266)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#elif defined(ESP32)
#define DEBUG_PRINT(x) Serial.print(String(xPortGetCoreID()) + " - " + String(x))
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(String(xPortGetCoreID()) + " - " + String(x))
                      // #define DEBUG_PRINT(x) Serial.print(x)
                      // #define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
                      // #define DEBUG_PRINTLN(x) Serial.println(x)
#else
#error Unknown architecture
#endif
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

#if defined(ETHERNET_ENABLED)
bool setupW5500()
{
  WiFi.mode(WIFI_OFF);
  WiFi.begin();
  tcpip_adapter_set_default_eth_handlers();

  // Initialize TCP/IP network interface (should be called only once in application)
  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
  esp_netif_t *eth_netif = esp_netif_new(&cfg);
  // Set default handlers to process TCP/IP stuffs
  ESP_ERROR_CHECK(esp_eth_set_default_handlers(eth_netif));

  esp_eth_mac_t *eth_mac = NULL;
  esp_eth_phy_t *eth_phy = NULL;

  gpio_install_isr_service(0);

  spi_bus_config_t buscfg = {
      .mosi_io_num = MO,
      .miso_io_num = MI,
      .sclk_io_num = CL,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, 1));

  spi_device_handle_t spi_handle = NULL;
  spi_device_interface_config_t devcfg = {
      .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
      .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
      .mode = 0,
      .clock_speed_hz = 12 * 1000 * 1000,
      .spics_io_num = CS,
      .queue_size = 20};
  ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));
  /* w5500 ethernet driver is based on spi driver */
  eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);
  w5500_config.int_gpio_num = INT;

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.reset_gpio_num = -1;

  eth_mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
  if (eth_mac == NULL)
  {
    log_e("esp_eth_mac_new_esp32 failed");
    return false;
  }

  eth_phy = esp_eth_phy_new_w5500(&phy_config);
  if (eth_phy == NULL)
  {
    log_e("esp_eth_phy_new failed");
    return false;
  }

  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(eth_mac, eth_phy);
  esp_eth_handle_t eth_handle = NULL;
  ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

  uint8_t macArr[] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
  ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, macArr));

  /* attach Ethernet driver to TCP/IP stack */
  ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
  /* start Ethernet driver state machine */
  ESP_ERROR_CHECK(esp_eth_start(eth_handle));

  // Check if an IP was obtained
  int count = 0;
  boolean gotIP = false;

  for (count = 0; count < 30; count++)
  {
    digitalWrite(WIFI_LED, !digitalRead(WIFI_LED));
    if (ETH.localIP().toString() != "0.0.0.0")
    {
      gotIP = true;
      digitalWrite(WIFI_LED, HIGH);
      break;
    }
    delay(200);
  }

  if (!gotIP)
    digitalWrite(WIFI_LED, HIGH);

  return gotIP;
}
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
  if (!server.authenticate(www_username, www_password))
  {
    return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
  }
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

void timeToString(char *string, size_t size, int seconds)
{
  int days = seconds / 86400;
  seconds %= 86400;
  byte hours = seconds / 3600;
  seconds %= 3600;
  byte minutes = seconds / 60;
  seconds %= 60;
  snprintf(string, size, "%04d:%02d:%02d:%02d", days, hours, minutes, seconds);
}

String getMacAddress(int mt)
{
  uint8_t baseMac[6] = {0};

// Get MAC address for WiFi station
// ESP_MAC_WIFI_SOFTAP
// ESP_MAC_WIFI_STA
#if defined(ESP32)
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
#endif

  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);

  String macAddress = String(baseMacChr);
  // Serial.println(baseMacChr);
  return macAddress;
}

// String getSSID() {
//   wifi_config_t conf;
//   esp_wifi_get_config(WIFI_IF_STA, &conf);
//   return String(reinterpret_cast<const char*>(conf.sta.ssid));
// }

String convS(const char *a)
{
  int i;

  String res = "";
  i = 0;
  while (a[i] != '\0')
  {
    // for (i = 0; i < arr_size; i++) {
    res = res + a[i];
    i++;
  }
  return res;
}

// void handleRoot()
void handleJSON()
{
  syslog.logf(LOG_INFO, "JSON requested from %s", server.client().remoteIP().toString().c_str());

  char webpage[2048];
  char timestr[32];

  // Build string for web UI
  time_t t = now(); // get current time
  tm *ptm = gmtime(&t);
  strftime(timestr, 32, "%Y-%m-%d %H:%M:%S UTC", ptm);

  int sats = (gps.satellites.value() != 255) ? gps.satellites.value() : 0;
  String resol = gpsLocked ? String(gps.hdop.value()) : "";

  // latitude & longitude
  float lat = 0.0;
  float lng = 0.0;
  if (gpsLocked)
  {
    lat = gps.location.lat();
    lng = gps.location.lng();
  }

  IPAddress myIP = WiFi.localIP();
  if (myIP[0] == 0)
    myIP = WiFi.softAPIP();

  String wifinet = "";
  String mcaddr = "";
  String wifmode = "";
  int signal = 0;
  if (WiFi.getMode() == WIFI_STA)
  {
    wifmode = "Station";
    wifinet = wifissid;
    signal = WiFi.RSSI();
#if defined(ESP32)
    mcaddr = getMacAddress(ESP_MAC_WIFI_STA);
#endif
  }
  else
  {
    wifmode = "Access Point";
    wifinet = convS(ssid);
    signal = 0;
#if defined(ESP32)
    mcaddr = getMacAddress(ESP_MAC_WIFI_SOFTAP);
#endif
  }

  char uptime[15] = "";
#if defined(ESP32)
  timeToString(uptime, sizeof(uptime), esp_timer_get_time() / 1000000);
#endif

  String uptimeStr(uptime);

  int freeHeap = 0;
  int minimumHeap = 0;
  String idfVersion = "";
  float internalTemp = 0.0;
#if defined(ESP32)
  freeHeap = esp_get_free_heap_size() / 1024;
  minimumHeap = esp_get_minimum_free_heap_size() / 1024;
  idfVersion = esp_get_idf_version();
  internalTemp = (temprature_sens_read() - 32) / 1.8; // Temp is only correctly captured when WiFi active, ottherwise will always return 128 F (53.33 C)
#endif

  String locked = gpsLocked ? "True" : "False";

  // RTC Data
  String validDateTime = Rtc.IsDateTimeValid() ? "True" : "False";
  String isRTCRunning = Rtc.GetIsRunning() ? "True" : "False";
  // RtcDateTime Now = Rtc.GetDateTime();
  int8_t AgingOffset = Rtc.GetAgingOffset();
  float RTCTemperature = Rtc.GetTemperature().AsFloatDegC();

  sprintf(webpage,
          "{\"version\":\"%s\", \"uptime\":\"%s\", \"freeheap\":\"%d\", \"ip\":\"%s\", \"network\":\"%s\", \"type\":\"%s\",\
           \"signal\":\"%d\", \"timestamp\":\"%s\", \"lat\":\"%7.8f\", \"lon\":\"%7.8f\", \"sats\":\"%d\", \"hdop\":\"%s\",\
            \"syslog\":\"%s\", \"ssid\":\"%s\", \"minimumheap\":\"%d\", \"idfversion\":\"%s\", \"internaltemp\":\"%.1f\",\
             \"macaddress\":\"%s\", \"locked\":\"%s\", \"validdatetime\":\"%s\", \"rtcrunning\":\"%s\", \"agingoffset\":\"%d\",\
              \"rtctemperature\":\"%.1f\"}",
          "2.0i",
          uptimeStr.c_str(),
          freeHeap,
#if defined(ETHERNET_ENABLED)
          ("W: " + myIP.toString() + " - E: " + ETH.localIP().toString()).c_str(),
#else
          myIP.toString().c_str(),
#endif
          wifinet.c_str(),
          wifmode,
          signal,
          timestr,
          (float)lat,
          (float)lng,
          sats,
          resol,
          syslogserver.c_str(),
          wifissid.c_str(),
          minimumHeap,
          idfVersion,
          internalTemp,
          mcaddr.c_str(),
          locked,
          validDateTime,
          isRTCRunning,
          AgingOffset,
          RTCTemperature);
  server.send(200, "text/html", webpage);
}

void handleRoot()
{
  const char *dataType = "text/html";

  syslog.logf(LOG_INFO, "Webpage requested from %s", server.client().remoteIP().toString().c_str());

  server.sendHeader(F("Content-Encoding"), F("gzip"));

#if defined(ARDUINO_ARCH_ESP8266)
  server.send(200, dataType, (const char *)index_gz, index_gz_len);
#elif defined(ESP32)
  server.send_P(200, dataType, (const char *)index_gz, index_gz_len);
#else
#error Unknown architecture
#endif
  // server.send(200, "text/html", PAGE_Index);
}

void startHttpServer()
{
  server.on("/", handleRoot);
  server.on("/updatewifi", handleUpdate);
  server.on("/json", handleJSON);
#if defined(ESP32)
  server.on("/serverUpdate", HTTP_GET, []()
            {
        if (!server.authenticate(www_username, www_password))
        {
          return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
        }
    server.sendHeader("Connection", "close");
    //server.send(200, "text/html", serverUpdate); });
    server.send(200, "text/html", PAGE_Update); });
  /*handling uploading firmware file */
  server.on(
      "/update", HTTP_POST, []()
      {
        if (!server.authenticate(www_username, www_password))
        {
          return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
        }
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); },
      []()
      {
        if (!server.authenticate(www_username, www_password))
        {
          return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
        }
        HTTPUpload &upload = server.upload();
        if (upload.status == UPLOAD_FILE_START)
        {
          // Serial.printf("Update: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          { // start with max available size
            // Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
          /* flashing firmware to ESP*/
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
          {
            // Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
          if (Update.end(true))
          { // true to set the size to the current progress
            // Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          }
          else
          {
            // Update.printError(Serial);
          }
        }
      });
#endif
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
// ESP32 got a 10 as maximum
#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.softAP(ssid, password, AP_CHANNEL, 0, 8);
#elif defined(ESP32)
  WiFi.softAP(ssid, password, AP_CHANNEL, 0, 10);
#else
#error Unknown architecture
#endif

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
#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.begin(wifissid, wifipassword);
#elif defined(ESP32)
  WiFi.begin(wifissid.c_str(), wifipassword.c_str());
#else
#error Unknown architecture
#endif

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

#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.forceSleepBegin();
#elif defined(ESP32)
// Start ethernet when WiFi is disabled
#if defined(ETHERNET_ENABLED)
  // if (setupW5500())
  startHttpServer();
#endif
#else
#error Unknown architecture
#endif

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
#if defined(ARDUINO_ARCH_ESP8266)
  // Deprecated on ESP32
  time_t t = Now.Epoch32Time();
#elif defined(ESP32)
  time_t t = Now.Unix32Time();
#else
#error Unknown architecture
#endif

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

#if defined(ARDUINO_ARCH_ESP8266)
  // Deprecated on ESP32
  timeToSet.InitWithEpoch32Time(t);
#elif defined(ESP32)
  timeToSet.InitWithUnix32Time(t);
#else
#error Unknown architecture
#endif

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
  // else
  // digitalWrite(LOCK_LED, LOW);

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
    wifinet = convS(ssid);

  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 25, 248);

  int wrap = 38;
  int ypos1 = 10;
  int ypos2 = 25;
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
    ypos1 = 30;
  }
  else
  {
    u8g2.setFont(u8g2_font_10x20_tf); // Readable on one line
    ypos1 = 30;
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

#if defined(ETHERNET_ENABLED)
  u8g2.setFont(u8g2_font_7x13_tf); // Small font for long IP
  u8g2.drawStr(0, 45, ETH.localIP().toString().c_str());
#else
  u8g2.setFont(u8g2_font_logisoso16_tr); // choose a suitable font
#endif

  u8g2.drawStr(0, 60, myIP.toString().c_str());
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
#if defined(ARDUINO_ARCH_ESP8266)
  // Deprecated on ESP32
  long int a = time.Epoch32Time();
#elif defined(ESP32)
  long int a = time.Unix32Time();
#else
#error Unknown architecture
#endif
  setTime(a); // set system time from RTC
  DEBUG_PRINT("SyncFromRTC: ");
  DEBUG_PRINTLN(a);
  syncTime = now();                       // and remember time of this sync event
  DEBUG_PRINTLN("Synchronized from RTC"); // send message to serial monitor
}

#if defined(ARDUINO_ARCH_ESP8266)
void SyncCheck()
{
#elif defined(ESP32)
void SyncCheck(void *parameter)
{
  for (;;)
  {
#else
#error Unknown architecture
#endif
  // Manage synchonization of clock to GPS module
  // First, check to see if it is time to synchonize
  // Do time synchonization on the 1pps signal
  // This call must be made frequently (keep in main loop)
  // DEBUG_PRINTLN(F("SyncCheck Task Running..."));
  // Serial.println("2SyncCheck Task Running...");
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
#if defined(ESP32)
  vTaskDelay(1);
  // taskYIELD();
}
#endif
}

// --------------------------------------------------------------------------------------------------
// MAIN PROGRAM

void IRAM_ATTR isr() // INTERRUPT SERVICE REQUEST
{
  pps = 1;                     // Flag the 1pps input signal
  digitalWrite(PPS_LED, HIGH); // Ligth up led pps monitor
  pps_blink_time = millis();   // Capture time in order to turn led off so we can get the blink effect ever x milliseconds - On loop
  // DEBUG_PRINTLN("pps"); Sometimes cause: esp32 Guru Meditation Error: Core  1 panic'ed (Interrupt wdt timeout on CPU1).
}

// Handle button pressed interrupt
void IRAM_ATTR btw() // INTERRUPT SERVICE REQUEST
{
  int currentState = digitalRead(WIFI_BUTTON);
  if ((currentState != lastState) and (!buttonPressed))
  {
    if (lastState == HIGH)
    {
      keytick_up = millis();
    }
    else
    {
      keytick_down = millis();
      buttonPressed = true;
      // KeyCheck();
    }
    lastState = currentState;
  }
  // DEBUG_PRINTLN(F("BUTTON PRESSED!")); Sometimes cause: esp32 Guru Meditation Error: Core  1 panic'ed (Interrupt wdt timeout on CPU1).
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

#if defined(ARDUINO_ARCH_ESP8266)
void FeedGpsParser()
{
#elif defined(ESP32)
  void FeedGpsParser(void *parameter)
  {
    for (;;)
    {
#else
#error Unknown architecture
#endif
  // feed currently available data from GPS module into tinyGPS parser
  // unsigned long StartTime = millis();
  while (ss.available()) // look for data from GPS module
  {
    char c = ss.read(); // read in all available chars
    gps.encode(c);      // and feed chars to GPS parser
                        // Serial.write(c); // Uncomment for some extra debug info if in doubt about GPS feed
#ifdef DEBUG_GPS
    DEBUG_PRINT(c);
#endif
    // Will toggle LOCK_LED at each pass if not locked, so user can see the GPS is actually doing something
    if (!gpsLocked)
      digitalWrite(LOCK_LED, !digitalRead(LOCK_LED));
    // Serial.print(c);
  }
  // DEBUG_PRINTLN(millis() - StartTime);
#if defined(ESP32)
  vTaskDelay(1);
  // taskYIELD();
}
#endif
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
#if defined(ARDUINO_ARCH_ESP8266)
void processRFC868()
{
#elif defined(ESP32)
    void processRFC868(void *parameter)
    {
      for (;;)
      {
#else
#error Unknown architecture
#endif
  // DEBUG_PRINTLN(F("Reached processRFC868"));
  uint32_t timestamp;
  WiFiClient client = RFC868server.accept();
  // client.setNoDelay(true); // Unecessary for UDP
  if (client.connected())
  {
    // syslog.logf(LOG_INFO, "RDATE request from %s", client.remoteIP().toString().c_str());
    //  Serial.println(client.remoteIP().toString().c_str());

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
#if defined(ESP32)
  vTaskDelay(1);
  // taskYIELD();
}
#endif
}

#if defined(ARDUINO_ARCH_ESP8266)
void processNTP()
{
#elif defined(ESP32)
      void processNTP(void *parameter)
      {
        for (;;)
        {
#else
#error Unknown architecture
#endif
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();
    // Packets bigger than NTP_PACKET size (like the ones generated by ntpq -c sysinfo <SERVER_IP>, which is 136 bytes) will block the UDP instance if not flushed. Not ready to deal with them right now, maybe in the future. Need to flush the remaining bytes.
    Udp.flush();

#ifdef DEBUG
    Serial.println();
    DEBUG_PRINTLN(F("Received UDP packet size "));
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

    // syslog.logf(LOG_INFO, "NTP request from %s", Remote.toString().c_str());
  }
#if defined(ESP32)
  vTaskDelay(1);
  // taskYIELD();
}
#endif
}

void setup()
{
  DEBUG_PRINTLN(F("Starting setup..."));
  pinMode(LOCK_LED, OUTPUT);
  pinMode(PPS_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(WIFI_BUTTON, INPUT_PULLUP);

  digitalWrite(LOCK_LED, LOW);
  digitalWrite(PPS_LED, LOW);
  digitalWrite(WIFI_LED, LOW);

  // if you are using ESP-01 then uncomment the line below to reset the pins to
  // the available pins for SDA, SCL
  DEBUG_PRINTLN(F("Starting LittleFS"));
#if defined(ARDUINO_ARCH_ESP8266)
  // ss.begin (13U, 15U); // ESP8266 D7, D8
  ss.begin(9600);     // set GPS baud rate to 9600 bps
  Wire.begin(4U, 5U); // ESP8266 D2, D1 - due to limited pins, use pin 0 and 2 for SDA, SCL
  LittleFS.begin();   // Init storage for WiFi SSID/PSK -- true = FORMAT_LITTLEFS_IF_FAILED
#elif defined(ESP32)
          ss.begin(115200);
          Wire.begin(21U, 22U);
          LittleFS.begin(true); // Init storage for WiFi SSID/PSK -- true = FORMAT_LITTLEFS_IF_FAILED
#if defined(ETHERNET_ENABLED)
          setupW5500();
#endif
#else
#error Unknown architecture
#endif

  Rtc.Begin();
  RtcEeprom.Begin();

  InitLCD(); // initialize LCD display

#ifdef DEBUG
  Serial.begin(115200); // set serial monitor rate to 9600 bps
#endif

  // Serial.begin(9600);
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
  // DEBUG_PRINTLN(F(xPortGetCoreID()));

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

#if defined(ARDUINO_ARCH_ESP8266)
#elif defined(ESP32)
          // xTaskCreatePinnedToCore(
          // Task1code, /* Function to implement the task */
          //"Task1", /* Name of the task */
          // 10000,  /* Stack size in words */
          // NULL,  /* Task input parameter */
          // 0,  /* Priority of the task */
          //&Task1,  /* Task handle. */
          // 0); /* Core where the task should run */
          xTaskCreate(FeedGpsParser, "FeedGpsParser", 2048, NULL, 1, NULL); // decode incoming GPS data
          xTaskCreate(SyncCheck, "SyncCheck", 2048, NULL, 1, NULL);         // synchronize to GPS or RTC
          xTaskCreate(processNTP, "processNTP", 2048, NULL, 1, NULL);
          xTaskCreate(processRFC868, "processRFC868", 2048, NULL, 1, NULL);
#else
#error Unknown architecture
#endif
}

////////////////////////////////////////

void loop()
{
#if defined(ARDUINO_ARCH_ESP8266)
  FeedGpsParser(); // decode incoming GPS data
  SyncCheck();     // synchronize to GPS or RTC
  processNTP();
  processRFC868();
#elif defined(ESP32)
#else
#error Unknown architecture
#endif

  UpdateDisplay(); // if time has changed, display it
  server.handleClient();
  if (millis() - pps_blink_time > PPS_BLINK_INTERVAL) // If x milliseconds passed, then it's time to switch led off for blink effect
    digitalWrite(PPS_LED, LOW);
  if (buttonPressed)
  { // Process keycheck of button presses outside ISR to avoid crash
    buttonPressed = false;
    KeyCheck();
  }
}
