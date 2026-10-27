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
#include "CredentialValidation.h"
#include <U8g2lib.h>
#include <Ticker.h>
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

// Being defined as build flag on platformio.ini, but can be uncommented here for testing without build flags
// Please note the pin assignment for the ETHERNET_ENABLED build is slightly different from the WiFi only build
// If you flash the WiFi only build to an ETHERNET_ENABLED wired hardware your sync led (green - pin 19) will not get lit
// and the toggle/wifi button (pin 5) will not work. This can be frustrating if you don't pay attention and can lead to you
// spending a long time trying to debug what you think is a hardware issue when in fact it's just a software configuration issue.
// #define ETHERNET_ENABLED

// Create a U8g2log object
U8G2LOG u8g2log;
// assume 4x6 font, define width and height
#define U8LOG_WIDTH 32
#define U8LOG_HEIGHT 10

// allocate memory
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];

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

volatile long int pps_blink_time = 0;

#elif defined(ESP32)
#include "WiFi.h"
#include <HardwareSerial.h>
#include <WebServer.h>
// Web OTA Upgrade
#include <Update.h>

// Initialize GNSS Serial on second UART. First one remains available for debugging - Cristiano - 20250125
HardwareSerial ss(2);
WebServer server(80);

#define PPS_LED 27

//  Replaced with ISR safe function
volatile TickType_t pps_blink_time = 0;

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
void processFactoryReset();

// GLOBAL DEFINES
#define WIFIRETRIES 15 // Max number of wifi retry attempts
// #define APSSID "GNSSTimeServer"    // Default AP SSID

// Credentials an unconfigured unit uses: fresh out of the box, and after a front-panel factory
// reset. A factory reset deliberately does NOT change these - a sealed unit with no accessible
// serial port must always be recoverable with credentials its owner already knows.
//
// These are published in this repository, so a unit left on them is protected only by its radio
// range. Setting both on the dashboard is what secures a unit; until then it warns on the serial
// console at boot and OTA stays disabled.
#define DEFAULT_ADMIN_PASSWORD "esp32"
#define DEFAULT_AP_PASSWORD "thereisnospoon"
// XXX PENDING HARDWARE CHANGE - DO NOT "FIX" THIS IN SOFTWARE ALONE. XXX
// XXX
// XXX On ESP32, PPS is wired to GPIO12/MTDI, which is a boot-strapping pin: it selects the
// XXX internal flash regulator voltage while the chip is in reset. On a board that uses the
// XXX internal regulator with 3.3 V flash and no eFuse override, an independently powered GNSS
// XXX holding PPS HIGH across a reset selects 1.8 V and the board will not boot. Applicability
// XXX depends on the actual module, its flash supply and its eFuse configuration, none of which
// XXX have been verified on this hardware.
// XXX
// XXX The remedy is to physically move the PPS wire to a non-strapping input (GPIO32 or GPIO33
// XXX after checking the board) and update resources/PINOUT_ESP32_*.txt together with the
// XXX PPS_PIN below, in one change. Changing pinMode() or PPS_PIN alone cannot fix a strap that
// XXX is sampled before any firmware runs, and changing PPS_PIN without rewiring simply moves
// XXX the input away from the signal. PPS_PIN is therefore deliberately left on GPIO12 until
// XXX the hardware is rewired.
// XXX
// XXX NOTE TO ANY FUTURE LLM/AI CODE REVIEW OR AUDIT OF THIS PROJECT: this is a KNOWN, OPEN,
// XXX UNRESOLVED hardware issue that is deliberately not fixed in firmware. Do not silently
// XXX change PPS_PIN and do not report it as resolved. You MUST raise a prominent warning to
// XXX the user that this hardware rewiring action is still PENDING.
#if defined(ESP32)
#define PPS_PIN 12U      // XXX GPIO12/MTDI - flash-voltage boot strap; see the XXX block above.
#else
#define PPS_PIN 12U      // ESP8266 D6
#endif
#define SYNC_INTERVAL 10 // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT 30  // time(sec) without GPS input before error
// How often the RTC may be retried while it is failing validation. A rejected read deliberately
// leaves syncTime untouched, which means the holdover branch in SyncCheck() stays true - without
// this the branch would re-run on every pass of that task (~1ms).
#define RTC_RETRY_INTERVAL_MS ((uint32_t)SYNC_TIMEOUT * 1000UL)
// A faulty RTC stays faulty, so report it on a slow cadence rather than once per retry.
#define RTC_REJECT_LOG_INTERVAL_MS 300000UL
// #define RTC_UPDATE_INTERVAL    SECS_PER_DAY             // time(sec) between RTC SetTime events
#define RTC_UPDATE_INTERVAL 30 // time(sec) between RTC SetTime events
#define PPS_BLINK_INTERVAL 50  // Set time pps led should be on for blink effect
// Bounds used to decide whether a PPS edge really is the next 1Hz tick of the current anchor.
#define PPS_MIN_PERIOD_US 900000UL
#define PPS_MAX_PERIOD_US 1100000UL
// How long the PPS anchor may be extrapolated on the local crystal after the pulses stop. Matched
// to SYNC_TIMEOUT so the anchor and gpsLocked stop being trusted at the same moment.
#define PPS_ANCHOR_MAX_COAST_US ((uint32_t)SYNC_TIMEOUT * 1000000UL)
// Worst-case local crystal error, used to widen root dispersion while coasting between edges.
#define LOCAL_OSC_DRIFT_PPM 20.0
// GPS sentence acceptance limits - see SyncWithGPS()
#define GPS_RMC_MAX_AGE_MS 1500UL
// Plausibility window applied to any clock source (GPS sentence or RTC register contents).
#define MIN_VALID_YEAR 2024
#define MAX_VALID_YEAR 2099
// A rejected GPS sync is retried on every PPS edge, so throttle the syslog line it produces.
#define GPS_REJECT_LOG_INTERVAL_MS 60000UL
// A sentence must have been committed inside the second that the triggering PPS edge closes.
#define NMEA_EDGE_MIN_LEAD_US 5000UL
#define NMEA_EDGE_MAX_LEAD_US 995000UL
// #define SYSLOG_SERVER "x.x.x.x" // syslog server name or IP
#define SYSLOG_PORT 514 // syslog port

const char *www_username = "admin";
// allows you to set the realm of authentication Default:"Login Required"
const char *www_realm = "Custom Auth Realm";
// the Content of the HTML response in case of Unautherized Access Default:empty
String authFailResponse = "Authentication Failed";
// const char *serverUpdate = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

#define WIFI_LED 14U     // ESP8266 D5
#define BTN_HOLD_MS 2000           // Number of milliseconds to determine button being held
#define BTN_FACTORY_RESET_MS 10000 // Hold this long to erase the stored configuration
#define BTN_NONE 0                 // No button press
#define BTN_PRESS 1                // Button pressed for < BTN_HOLD_MS
#define BTN_HOLD 2                 // Button held for at least BTN_HOLD_MS
#define BTN_FACTORY_RESET 3        // Button held for at least BTN_FACTORY_RESET_MS

// INCLUDES
#include <TimeLib.h>     // Time functions  https://github.com/PaulStoffregen/Time
#include <TinyGPSPlus.h> // GPS parsing     https://github.com/mikalhart/TinyGPSPlus
#include "NmeaRmcClock.h" // complete, checksum-validated UTC records for synchronization
#include <Wire.h>        // OLED and DS3231 necessary
#include <RtcDS3231.h>   // RTC functions

RtcDS3231<TwoWire> Rtc(Wire);
EepromAt24c32<TwoWire> RtcEeprom(Wire);

// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // OLED display library parameters
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // OLED display library parameters

TinyGPSPlus gps;
NmeaRmcClock gpsClock;

time_t displayTime = 0;     // time that is currently displayed
volatile time_t syncTime = 0;        // time of last GPS or RTC synchronization
volatile uint32_t syncFraction = 0;  // NTP sub-second fraction at the moment of that sync (0 unless GPS-locked at sync time)
volatile time_t lastGpsSyncTime = 0; // time of last successful GPS lock specifically (not RTC resync) - used for holdover dispersion estimate
time_t lastSetRTC = 0;      // time that RTC was last set
const unsigned long seventyYears = 2208988800UL; // to convert unix time to the NTP epoch
// NOTE: NTP era-0 rolls over 2036-02-07 (32-bit seconds-since-1900 field wraps); this server does
// not implement era pivoting, so timestamps will be served incorrectly (1900-era) after that date.
// RFC 868 (processRFC868) is inherently limited the same way by its own 32-bit field and cannot be
// fixed - this is a protocol limit, not a bug here.

// PPS edge event, published by isr() and consumed by SyncCheck().
volatile bool ppsEventPending = false; // an edge has occurred and has not been consumed yet
volatile uint32_t ppsEventMicros = 0;  // micros() at that edge
// The time anchor: "UTC second ppsAnchorSecond began when micros() read ppsAnchorMicros".
volatile time_t ppsAnchorSecond = 0;
volatile uint32_t ppsAnchorMicros = 0;
volatile bool ppsAnchorValid = false;
time_t dstStart = 0;        // start of DST in unix time
time_t dstEnd = 0;          // end of DST in unix time
volatile bool gpsLocked = false;     // indicates recent sync with GPS
Ticker lockLedTicker;

// One complete on/off cycle per second while acquiring. A timer keeps this independent of
// incoming serial bytes, screen refreshes and the blocking Wi-Fi connection attempt.
void updateLockLed()
{
  static bool blinkOn = false;
  blinkOn = !blinkOn;
  digitalWrite(LOCK_LED, gpsLocked || blinkOn ? HIGH : LOW);
}
bool rtcAvailable = false;           // set once in setup(); false if the RTC never started - see C2
volatile bool buttonPressed = false; // To process button presses outside the ISR
volatile bool wifiReconfigPending = false; // Set by handleUpdate() to defer processWifi() until after the HTTP response is sent
int currentYear = 0;        // used for DST
volatile int displaynum = 0;         // Display pane currently displayed
#define NUMDISPLAYPANES 2   // Number of display panes available

const char *ssid = HOSTNAME;
String wifissid;
String wifipassword;
String wifiappassword;
String adminpassword;
bool credentialsReady = false;       // set only after unique credentials are durably stored
bool credentialsProvisioned = false; // completed by an authenticated configuration save
String syslogserver;
// Per-boot token handed to the dashboard through /json and required back on /updatewifi. The
// browser caches digest credentials, so without it any page the user visits could drive a
// configuration change on their behalf; a cross-origin page cannot read /json to learn the token.
uint32_t csrfToken = 0;

const char *activeAdminPassword()
{
  return adminpassword.c_str();
}

const char *activeApPassword()
{
  return wifiappassword.c_str();
}

volatile uint8_t statusWifi = 1;

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
#define EDGE_LOCKOUT_MS 20 // minimum time between accepted button edges, to reject contact bounce

volatile uint32_t keyPressTick = 0;   // time the button was last pressed down
volatile uint32_t keyReleaseTick = 0; // time the button was last released
volatile unsigned long lastEdgeTick = 0; // time of last accepted (non-bounce) edge
volatile int lastState = HIGH; // record last button state to support debouncing

// Get compile timestamp
const char compile_date[] = __DATE__ " " __TIME__;

// #define DEBUG // Comment this in order to remove debug code from release version
// #define DEBUG_GPS // Uncomment this to receive GPS messages in debug output

#ifdef DEBUG_GPS
#if defined(ARDUINO_ARCH_ESP8266)
#define DEBUG_GPS_PRINT(x) Serial.print(x)
#define DEBUG_GPS_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_GPS_PRINTLN(x) Serial.println(x)
#elif defined(ESP32)
#define DEBUG_GPS_PRINT(x) Serial.print(String(xPortGetCoreID()) + " - " + String(x))
#define DEBUG_GPS_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_GPS_PRINTLN(x) Serial.println(String(xPortGetCoreID()) + " - " + String(x))
#else
#error Unknown architecture
#endif
#else
#define DEBUG_GPS_PRINT(x)
#define DEBUG_GPS_PRINTDEC(x)
#define DEBUG_GPS_PRINTLN(x)
#endif

#ifdef DEBUG
#if defined(ARDUINO_ARCH_ESP8266)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#elif defined(ESP32)
#define DEBUG_PRINT(x) Serial.print(String(xPortGetCoreID()) + " - " + String(x))
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(String(xPortGetCoreID()) + " - " + String(x))
#else
#error Unknown architecture
#endif
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

// syslog.log() drives the shared udpClient (beginPacket/write/endPacket) and this is the only
// call site in the file, but it's reached from both the SyncCheck task and loopTask (web
// handlers) - see B3. Guarded with a mutex (not a spinlock) since it does a UDP send plus
// internal heap allocation, not a short arithmetic op.
#if defined(ESP32)
static SemaphoreHandle_t syslogMutex = NULL;
#define SYSLOG_ENTER_CRITICAL() xSemaphoreTake(syslogMutex, portMAX_DELAY)
#define SYSLOG_EXIT_CRITICAL() xSemaphoreGive(syslogMutex)
#else
#define SYSLOG_ENTER_CRITICAL()
#define SYSLOG_EXIT_CRITICAL()
#endif

#if defined(ETHERNET_ENABLED)
esp_netif_t *eth_netif = NULL; // populated by setupW5500(); queried directly since ETH.begin() is never called

IPAddress getEthIP()
{
  if (eth_netif == NULL)
    return IPAddress(0, 0, 0, 0);
  esp_netif_ip_info_t ipInfo;
  if (esp_netif_get_ip_info(eth_netif, &ipInfo) != ESP_OK)
    return IPAddress(0, 0, 0, 0);
  return IPAddress(ipInfo.ip.addr);
}
#endif

volatile bool syslogEnabled = false; // false when syslogserver is empty or unparsable - see E4/C4

void configureSyslogServer()
{
  IPAddress ip;
  bool enabled = (syslogserver.length() > 0) && ip.fromString(syslogserver);
  SYSLOG_ENTER_CRITICAL();
  syslogEnabled = enabled;
  if (syslogEnabled)
    syslog.server(ip, SYSLOG_PORT);
  SYSLOG_EXIT_CRITICAL();
}

void safeSyslog(uint16_t pri, const char *msg)
{
#if defined(ETHERNET_ENABLED)
  if (syslogEnabled && ((WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) || getEthIP() != IPAddress(0, 0, 0, 0)))
#else
  if (syslogEnabled && WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0))
#endif
  {
    SYSLOG_ENTER_CRITICAL();
    syslog.log(pri, msg);
    SYSLOG_EXIT_CRITICAL();
  }
}

void safeSyslogf(uint16_t pri, const char *format, ...)
{
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  safeSyslog(pri, buf);
}

// TimeLib's now()/setTime()/adjustTime() perform an unguarded read-modify-write on shared
// statics (see Time.cpp). On ESP32 they are called from 5 different task contexts across 2
// cores, so every access is funneled through this lock to prevent torn reads/writes of the
// served time. ESP8266 build is single-core superloop, so the lock is a no-op there.
#if defined(ESP32)
static portMUX_TYPE timeMux = portMUX_INITIALIZER_UNLOCKED;
#define TIME_ENTER_CRITICAL() portENTER_CRITICAL(&timeMux)
#define TIME_EXIT_CRITICAL() portEXIT_CRITICAL(&timeMux)
#else
#define TIME_ENTER_CRITICAL()
#define TIME_EXIT_CRITICAL()
#endif

time_t safeNow()
{
  TIME_ENTER_CRITICAL();
  time_t t = now();
  TIME_EXIT_CRITICAL();
  return t;
}

void safeSetTime(time_t t)
{
  TIME_ENTER_CRITICAL();
  setTime(t);
  TIME_EXIT_CRITICAL();
}

void safeSetTime(int hr, int mi, int sec, int dy, int mo, int yr)
{
  TIME_ENTER_CRITICAL();
  setTime(hr, mi, sec, dy, mo, yr);
  TIME_EXIT_CRITICAL();
}

void safeAdjustTime(long adjustment)
{
  TIME_ENTER_CRITICAL();
  adjustTime(adjustment);
  TIME_EXIT_CRITICAL();
}

// The PPS time anchor.
//
// NTP timestamps must NOT be composed from TimeLib's now() for the seconds plus a PPS-derived
// value for the fraction: the two have different second boundaries. setTime() resets TimeLib's
// boundary to the instant it runs (Time.cpp: "prevMillis = millis()"), which is one SyncCheck tick
// AFTER the PPS edge, while a PPS-derived fraction wraps on the edge itself. In the window between
// the two the fraction has already rolled into the next second but the integer has not, so the
// composed timestamp steps ~1s backward and then forward again - once per second, every second.
//
// Instead everything hangs off one anchor: "UTC second ppsAnchorSecond began when micros() read
// ppsAnchorMicros". Both halves of a timestamp come from a single reading of it, so they can never
// disagree. isr() rolls the anchor forward one second per edge, which also keeps the 32-bit
// micros() delta far away from its ~71 minute wrap.
#if defined(ESP32)
static portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;
#define PPS_ENTER_CRITICAL() portENTER_CRITICAL(&ppsMux)
#define PPS_EXIT_CRITICAL() portEXIT_CRITICAL(&ppsMux)
#define PPS_ENTER_CRITICAL_ISR() portENTER_CRITICAL_ISR(&ppsMux)
#define PPS_EXIT_CRITICAL_ISR() portEXIT_CRITICAL_ISR(&ppsMux)
#else
// ESP8266 is single core, so the ISR cannot be preempted and needs no lock of its own, but the
// superloop side still has to mask interrupts while it reads or writes the anchor.
#define PPS_ENTER_CRITICAL() noInterrupts()
#define PPS_EXIT_CRITICAL() interrupts()
#define PPS_ENTER_CRITICAL_ISR()
#define PPS_EXIT_CRITICAL_ISR()
#endif

// PPS interval statistics - the evidence needed before root dispersion can honestly be lowered.
//
// The GNSS PPS edges are exactly one second apart, so every microsecond by which consecutive
// captures differ from 1000000 is error introduced on THIS side: interrupt entry latency, the
// micros() quantum, and any critical section that masked interrupts across an edge. The receiver's
// own jitter (tens of nanoseconds) is invisible at this resolution.
//
// - spread (min..max) is the capture jitter, and the extremes are what a dispersion bound needs;
//   an error bound is about the tail, not the average.
// - the MEAN interval measures the local crystal instead: at 20 ppm each second reads as 1000020us.
//   That is a direct check on the LOCAL_OSC_DRIFT_PPM guess used while coasting.
//
// This cannot see a CONSTANT interrupt latency - a fixed delay on every edge cancels in the
// difference - so it bounds jitter, not absolute offset. Absolute offset needs a scope on the PPS
// line against a pin toggled in the ISR. Intervals outside the plausible window are excluded:
// those are missed pulses, not capture error, and they already invalidate the anchor.
volatile uint32_t ppsIntervalMinUs = 0;
volatile uint32_t ppsIntervalMaxUs = 0;
volatile uint32_t ppsIntervalCount = 0;
volatile uint64_t ppsIntervalSumUs = 0;
volatile uint32_t ppsIntervalOutliers = 0; // intervals rejected as missed/glitched pulses
volatile uint32_t ppsIntervalPrevMicros = 0;
volatile bool ppsIntervalPrevValid = false;

// Snapshot the statistics as a group; individually they can be torn by an edge landing mid-read.
void readPpsIntervalStats(uint32_t *minUs, uint32_t *maxUs, uint32_t *count, uint64_t *sumUs,
                          uint32_t *outliers)
{
  PPS_ENTER_CRITICAL();
  *minUs = ppsIntervalMinUs;
  *maxUs = ppsIntervalMaxUs;
  *count = ppsIntervalCount;
  *sumUs = ppsIntervalSumUs;
  *outliers = ppsIntervalOutliers;
  PPS_EXIT_CRITICAL();
}

// Publish a known-good UTC second against the PPS edge that started it. Returns false if that edge
// has already been superseded - if the caller was preempted long enough for another edge to arrive,
// the second it computed belongs to the newer edge and anchoring it here would be a second slow.
bool setPpsAnchor(time_t utcSecond, uint32_t edgeMicros)
{
  bool anchored = false;
  PPS_ENTER_CRITICAL();
  if (ppsEventMicros == edgeMicros)
  {
    ppsAnchorSecond = utcSecond;
    ppsAnchorMicros = edgeMicros;
    ppsAnchorValid = true;
    anchored = true;
  }
  PPS_EXIT_CRITICAL();
  return anchored;
}

void invalidatePpsAnchor()
{
  PPS_ENTER_CRITICAL();
  ppsAnchorValid = false;
  PPS_EXIT_CRITICAL();
}

// Take the pending PPS event together with the timestamps captured with it. Testing a flag and
// then clearing it as two separate statements drops any edge that lands in between, and pairs the
// surviving flag with whichever edge timestamps happen to be in the globals by then.
bool consumePpsEvent(uint32_t *edgeMicros)
{
  PPS_ENTER_CRITICAL();
  bool pending = ppsEventPending;
  *edgeMicros = ppsEventMicros;
  ppsEventPending = false;
  PPS_EXIT_CRITICAL();
  return pending;
}

// Current UTC as a complete NTP timestamp, both halves derived from one reading of the anchor.
// Returns false when there is no PPS-disciplined time to serve, in which case the caller must not
// claim stratum 1. coastMicros, when asked for, reports how long we have been extrapolating since
// the last real edge, so the caller can widen root dispersion to match.
bool ppsTimestamp(uint32_t *ntpSeconds, uint32_t *ntpFraction, uint32_t *coastMicros = NULL)
{
  PPS_ENTER_CRITICAL();
  bool valid = ppsAnchorValid;
  time_t anchorSecond = ppsAnchorSecond;
  uint32_t anchorMicros = ppsAnchorMicros;
  uint32_t sampleMicros = micros(); // sampled inside the lock so it cannot straddle an anchor update
  PPS_EXIT_CRITICAL();

  if (!valid)
    return false;

  uint32_t elapsed = sampleMicros - anchorMicros; // wraparound-safe unsigned subtraction
  if (elapsed > PPS_ANCHOR_MAX_COAST_US)
    return false; // pulses have been gone too long to keep extrapolating on the local crystal

  if (coastMicros)
    *coastMicros = elapsed;
  *ntpSeconds = (uint32_t)anchorSecond + (elapsed / 1000000UL) + (uint32_t)seventyYears;
  *ntpFraction = (uint32_t)(((uint64_t)(elapsed % 1000000UL) << 32) / 1000000UL);
  return true;
}

// The I2C bus (Wire) is shared by the RTC (SyncCheck task: SyncWithRTC/SetRTC) and the OLED
// (loopTask: UpdateDisplay's u8g2.sendBuffer(), plus handleJSON's RTC reads). A single Wire
// transaction is already serialized by the core's internal lock, but the RTC read is two
// transactions (register pointer write + read) that an OLED burst can interleave between - see
// B5. The OLED flush can take multiple milliseconds, too long to hold interrupts off for, so
// this uses a blocking mutex rather than the spinlocks above.
#if defined(ESP32)
static SemaphoreHandle_t i2cMutex = NULL;
#define I2C_ENTER_CRITICAL() xSemaphoreTake(i2cMutex, portMAX_DELAY)
#define I2C_EXIT_CRITICAL() xSemaphoreGive(i2cMutex)
#else
#define I2C_ENTER_CRITICAL()
#define I2C_EXIT_CRITICAL()
#endif

// TinyGPSPlus::encode() (write, from FeedGpsParser) is not thread-safe against the read sites in
// SyncWithGPS, ShowSyncFlag, and handleJSON - its commit() methods do non-atomic member-by-member
// assignment, so a reader can observe a torn update (e.g. new date + old time at midnight).
#if defined(ESP32)
static portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;
#define GPS_ENTER_CRITICAL() portENTER_CRITICAL(&gpsMux)
#define GPS_EXIT_CRITICAL() portEXIT_CRITICAL(&gpsMux)
#else
#define GPS_ENTER_CRITICAL()
#define GPS_EXIT_CRITICAL()
#endif

// gpsLocked/syncTime/syncFraction/lastGpsSyncTime are a logically-related group: SyncWithGPS(),
// SyncWithRTC(), and SyncCheck() write them (SyncCheck task) while processNTP() (a different,
// equal-priority task) reads several of them together to build one packet. Each field is a single
// aligned word so no individual read/write tears, but reading them as 4 SEPARATE unlocked
// statements means the group isn't a consistent snapshot - a lock transition mid-read can pair a
// stratum/LI decision from one sync event with a dispersion or refid from another. This lock makes
// the group's writes and the processNTP() read all-or-nothing.
#if defined(ESP32)
static portMUX_TYPE syncStateMux = portMUX_INITIALIZER_UNLOCKED;
#define SYNCSTATE_ENTER_CRITICAL() portENTER_CRITICAL(&syncStateMux)
#define SYNCSTATE_EXIT_CRITICAL() portEXIT_CRITICAL(&syncStateMux)
#else
#define SYNCSTATE_ENTER_CRITICAL()
#define SYNCSTATE_EXIT_CRITICAL()
#endif

String return_reset_reason(int reason)
{
  switch (reason)
  {
  case 0:
    return "ESP_RST_UNKNOWN";
    break; //!< Reset reason can not be determined
  case 1:
    return "ESP_RST_POWERON";
    break; //!< Reset due to power-on event
  case 2:
    return "ESP_RST_EXT";
    break; //!< Reset by external pin (not applicable for ESP32)
  case 3:
    return "ESP_RST_SW";
    break; //!< Software reset via esp_restart
  case 4:
    return "ESP_RST_PANIC";
    break; //!< Software reset due to exception/panic
  case 5:
    return "ESP_RST_INT_WDT";
    break; //!< Reset (software or hardware) due to interrupt watchdog
  case 6:
    return "ESP_RST_TASK_WDT";
    break; //!< Reset due to task watchdog
  case 7:
    return "ESP_RST_WDT";
    break; //!< Reset due to other watchdogs
  case 8:
    return "ESP_RST_DEEPSLEEP";
    break; //!< Reset after exiting deep sleep mode
  case 9:
    return "ESP_RST_BROWNOUT";
    break; //!< Brownout reset (software or hardware)
  case 10:
    return "ESP_RST_SDIO";
    break; //!< Reset over SDIO
  default:
    return "NO_MEAN";
  }
}

#if defined(ETHERNET_ENABLED)
bool setupW5500()
{
  WiFi.mode(WIFI_OFF);
  WiFi.begin();
  tcpip_adapter_set_default_eth_handlers();

  // Initialize TCP/IP network interface (should be called only once in application)
  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
  eth_netif = esp_netif_new(&cfg);
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

  // Derive the Ethernet MAC from this chip's factory-assigned base address. A fixed literal here
  // gives every device built from this firmware the same MAC, which breaks switch address learning
  // and DHCP the moment two of them share a network.
  uint8_t macArr[6] = {0};
  if (esp_read_mac(macArr, ESP_MAC_ETH) != ESP_OK)
  {
    log_e("esp_read_mac(ESP_MAC_ETH) failed - deriving from the WiFi station MAC instead");
    ESP_ERROR_CHECK(esp_read_mac(macArr, ESP_MAC_WIFI_STA));
    macArr[0] |= 0x02;  // locally administered
    macArr[0] &= ~0x01; // unicast
  }
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
    if (getEthIP().toString() != "0.0.0.0")
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

void KeyCheck(uint32_t pressTick, uint32_t releaseTick)
{
  // Button pressed and released
  int button_type = 0;
  if (pressTick > 0 and releaseTick > 0 and ((releaseTick - pressTick) >= DEBOUNCE_TICKS))
  {
    uint32_t heldMs = releaseTick - pressTick;
    if (heldMs >= BTN_FACTORY_RESET_MS)
    {
      button_type = BTN_FACTORY_RESET;
    }
    else if (heldMs > BTN_HOLD_MS)
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
    case BTN_FACTORY_RESET:
      processFactoryReset();
      break;
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

// More robust version of readData
String readData(const char *filename)
{
  DEBUG_PRINT("Reading data from ");
  DEBUG_PRINTLN(filename);

  if (!LittleFS.exists(filename))
  {
    DEBUG_PRINT("ERROR: File does not exist: ");
    DEBUG_PRINTLN(filename);
    return "";
  }

  File file = LittleFS.open(filename, "r");
  if (!file)
  {
    DEBUG_PRINT("ERROR: File open failed: ");
    DEBUG_PRINTLN(filename);
    return "";
  }

  String data = "";
  if (file.available())
  {
    data = file.readString();
    // Strip only the delimiter writeData() appended, not whitespace: trim() silently corrupts a
    // Wi-Fi passphrase or SSID that legitimately begins or ends with a space.
    if (data.endsWith("\n"))
      data.remove(data.length() - 1);
    if (data.endsWith("\r"))
      data.remove(data.length() - 1);
  }
  else
  {
    DEBUG_PRINTLN("WARNING: File is empty");
  }

  DEBUG_PRINT("Data read: ");
  DEBUG_PRINTLN(data);

  file.close();
  return data;
}

bool writeData(const char *filename, const String &data)
{
  // Rename a completed file over the previous value so an interrupted write
  // cannot erase a working credential or expose a partially written password.
  String temporary = String(filename) + ".tmp";
  File file = LittleFS.open(temporary, "w");
  if (!file)
    return false;
  size_t written = file.println(data);
  file.flush();
  bool complete = written == data.length() + 2 && file.getWriteError() == 0;
  file.close();
  if (!complete || !LittleFS.rename(temporary, filename))
  {
    LittleFS.remove(temporary);
    return false;
  }
  return readData(filename) == data;
}

bool initializeDeviceCredentials()
{
  // Compiled-in defaults, deliberately. An unconfigured unit - fresh out of the box or straight
  // out of a factory reset - answers to DEFAULT_ADMIN_PASSWORD and brings its access point up on
  // DEFAULT_AP_PASSWORD. A factory reset does not touch them, so a sealed enclosure with no
  // serial access is always recoverable with credentials the owner already knows.
  //
  // The trade-off is deliberate and documented: these defaults are published in this repository,
  // so any unit left on them is only as protected as its radio range. Changing both on the
  // dashboard is what makes a unit its own. See resources/AUDIT/project_audit-20260918-recheck.md
  // finding 3.
  wifiappassword = readData("/wifiappsk");
  adminpassword = readData("/adminpsk");

  // An empty or unusable stored value means nothing has been configured yet, so fall back to the
  // default rather than leaving the AP open or the dashboard unreachable. Nothing is written back:
  // "stored" is what distinguishes a configured unit from one still on defaults.
  if (!CredentialValidation::validAdminSubmission(adminpassword.length()) ||
      adminpassword.length() == 0)
    adminpassword = DEFAULT_ADMIN_PASSWORD;
  if (!CredentialValidation::validApPassword(wifiappassword.length()))
    wifiappassword = DEFAULT_AP_PASSWORD;

  credentialsReady = true;
  // Provisioning still gates OTA. It is satisfied by one authenticated save from the dashboard,
  // which the owner performs anyway to set the Wi-Fi network, and it means a unit sitting
  // untouched on its defaults cannot be reflashed over the network.
  String provisioned = readData("/credentials_provisioned");
  credentialsProvisioned = provisioned == "1";

  bool usingDefaultAdmin = adminpassword == DEFAULT_ADMIN_PASSWORD;
  bool usingDefaultAp = wifiappassword == DEFAULT_AP_PASSWORD;
  if (usingDefaultAdmin || usingDefaultAp)
  {
#if !defined(DEBUG) && !defined(DEBUG_GPS)
    // A debug build has already opened the console in setup(); reopening it at a different rate
    // here would leave the operator reading this through a mismatched terminal. The rates below
    // match monitor_speed for each env in platformio.ini.
#if defined(ESP32)
    Serial.begin(115200);
#else
    Serial.begin(9600);
#endif
    delay(50); // let the UART settle so the first line is not clipped
#endif
    Serial.println();
    Serial.println("WARNING: this unit is running on published default credentials.");
    Serial.print("AP name: ");
    Serial.println(ssid);
    Serial.print("AP password: ");
    Serial.println(usingDefaultAp ? F("(default) " DEFAULT_AP_PASSWORD) : F("(custom)"));
    Serial.print("Admin password: ");
    Serial.println(usingDefaultAdmin ? F("(default) " DEFAULT_ADMIN_PASSWORD) : F("(custom)"));
    Serial.println("Set your own on the dashboard - both fields - to secure this unit.");
    Serial.flush(); // this matters more than the boot messages that follow
  }
  return true;
}

// WiFi and Web Routines

#if defined(ESP32)
// Per-request OTA state. The Update object's own _error survives from one request to the next, so
// it cannot say whether THIS request delivered and committed a firmware image.
static bool otaStarted = false;
static bool otaCompleted = false;

// Firmware replacement is the most powerful thing this device exposes, so it stays closed until
// one authenticated configuration save has been completed - which the owner performs anyway to set
// the Wi-Fi network. A unit sitting untouched on its published defaults therefore cannot be
// reflashed over the network. This is a speed bump, not a wall: anyone who knows the default admin
// password can also perform that save. It is what remains of the provisioning requirement after
// the decision to keep recoverable, non-randomised defaults (see DEFAULT_ADMIN_PASSWORD).
static const char otaBlockedMessage[] =
    "Firmware updates are disabled until setup is completed: sign in and save the settings page once.";

bool otaAllowed()
{
  return credentialsReady && credentialsProvisioned;
}
#endif

void handleUpdate()
{
  if (!server.authenticate(www_username, activeAdminPassword()))
  {
    return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
  }
  // A configuration change must not be reachable by GET: the browser caches the digest
  // credentials, so a bare <img src=".../updatewifi?wifi_ssid=..."> on any page the user happens
  // to visit would otherwise reconfigure the device. POST plus the per-boot token closes that.
  if (server.method() != HTTP_POST)
  {
    server.send(405, "text/plain", "POST required");
    return;
  }
  char expectedToken[9];
  snprintf(expectedToken, sizeof(expectedToken), "%08X", csrfToken);
  if (server.arg("csrf_token") != expectedToken)
  {
    safeSyslogf(LOG_WARNING, "Rejected /updatewifi with a bad CSRF token from %s",
                server.client().remoteIP().toString().c_str());
    server.send(403, "text/plain", "Invalid or missing token - reload the configuration page");
    return;
  }

  String newApPassword = server.arg("wifi_ap_psk");
  String newAdminPassword = server.arg("admin_psk");
  String newSsid = server.arg("wifi_ssid");
  String newStationPassword = server.arg("wifi_psk");
  String newSyslogServer = server.arg("syslog_server");
  bool clearPassword = (server.arg("wifi_open") != "");

  // Validate every field before writing any of them, so a rejected request leaves the stored
  // configuration exactly as it was. Nothing here truncates: silently storing a shortened SSID or
  // password would answer "saved" while keeping a credential that cannot authenticate, and the
  // failure would only surface later as an unexplained association failure or a locked-out admin.
  const char *rejection = NULL;
  if (newSsid.length() > CredentialValidation::MAX_SSID_BYTES)
    rejection = "The WiFi SSID must be at most 32 characters";
  else if (clearPassword && newStationPassword.length() > 0)
    rejection = "Choose either an open network or a WiFi password, not both";
  else if (!CredentialValidation::validStationPassword(newStationPassword.c_str(),
                                                       newStationPassword.length()))
    rejection = "The WiFi password must be 8-63 characters, or exactly 64 hexadecimal "
                "digits for a raw WPA PSK - leave it empty to keep the stored one";
  else if (newApPassword.length() > 0 && !CredentialValidation::validApPassword(newApPassword.length()))
    rejection = "The AP password must be 8-63 characters (WPA2) - leave it empty to keep the stored one";
  else if (!CredentialValidation::validAdminSubmission(newAdminPassword.length()))
    rejection = "The admin password must be 8-63 characters - leave it empty to keep the stored one";
  else if (newSyslogServer.length() > 64)
    rejection = "The syslog server must be at most 64 characters";
  if (rejection)
  {
    server.send(400, "text/plain", rejection);
    return;
  }

  wifissid = newSsid;
  wifipassword = newStationPassword;
  syslogserver = newSyslogServer;
  DEBUG_PRINTLN("Settings Updated!");
  DEBUG_PRINTLN(wifissid);
  //  DEBUG_PRINTLN(wifipassword);
  // Save SSID and password in littlefs. An empty password field means "keep the stored one", since
  // the form never echoes the password back - so clearing it, to join an open network, needs its
  // own explicit request rather than being indistinguishable from leaving the field alone.
  writeData("/wifissid", wifissid);
  if (clearPassword)
  {
    wifipassword = "";
    writeData("/wifipsk", wifipassword);
  }
  else if (wifipassword != "")
  {
    writeData("/wifipsk", wifipassword);
  }
  else
  {
    wifipassword = readData("/wifipsk"); // keep what is stored, and keep the live copy in step
  }
  // The AP and admin passwords follow the same "empty means keep what is stored" rule; a factory
  // reset is how they get cleared back to the compiled-in defaults.
  if (newApPassword != "")
  {
    wifiappassword = newApPassword;
    writeData("/wifiappsk", wifiappassword);
  }
  if (newAdminPassword != "")
  {
    adminpassword = newAdminPassword;
    writeData("/adminpsk", adminpassword);
  }
  writeData("/syslogserver", syslogserver);

  // One authenticated, CSRF-checked save is what "provisioned" means here, and it is what unlocks
  // OTA. Recorded even when the passwords were left on their defaults: the point is that somebody
  // has deliberately configured this unit, not that the credentials are unique. See otaAllowed().
  if (!credentialsProvisioned)
  {
    if (writeData("/credentials_provisioned", "1"))
    {
      credentialsProvisioned = true;
      safeSyslog(LOG_NOTICE, "Provisioning completed - firmware updates are now enabled");
    }
    else
    {
      safeSyslog(LOG_ERR, "Could not record completed provisioning - firmware updates stay disabled");
    }
  }

  configureSyslogServer();

  String content = "<a href='/'>Return to main page</a>";
  server.send(200, "text/html", content);

  wifiReconfigPending = true; // apply the new WiFi settings on the next loop() pass, after the response is sent
}

void timeToString(char *string, size_t size, uint64_t seconds)
{
  unsigned long days = seconds / 86400;
  seconds %= 86400;
  byte hours = seconds / 3600;
  seconds %= 3600;
  byte minutes = seconds / 60;
  seconds %= 60;
  snprintf(string, size, "%04lu:%02u:%02u:%02u", days, (unsigned)hours,
           (unsigned)minutes, (unsigned)seconds);
}

uint64_t uptimeSeconds()
{
#if defined(ESP32)
  return esp_timer_get_time() / 1000000ULL;
#else
  // Called each loop, including while Wi-Fi is off, to extend the 49-day millis() rollover.
  static uint32_t previousMillis = 0;
  static uint64_t elapsedMillis = 0;
  uint32_t currentMillis = millis();
  elapsedMillis += (uint32_t)(currentMillis - previousMillis);
  previousMillis = currentMillis;
  return elapsedMillis / 1000ULL;
#endif
}

String getMacAddress(int mt)
{
  uint8_t baseMac[6] = {0};

// Get MAC address for WiFi station
// ESP_MAC_WIFI_SOFTAP
// ESP_MAC_WIFI_STA
#if defined(ESP32)
  esp_read_mac(baseMac, (esp_mac_type_t)mt);
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

// Escape a string for embedding in a JSON string literal. SSIDs, syslog server names and the
// like are free-form user input: a single unescaped quote or backslash produces a document that
// JSON.parse() rejects, which silently stops the dashboard updating at all.
String jsonEscape(const String &in)
{
  String out = "";
  out.reserve(in.length() + 8);
  for (unsigned int i = 0; i < in.length(); i++)
  {
    char c = in[i];
    switch (c)
    {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if ((uint8_t)c < 0x20)
      {
        char esc[7];
        snprintf(esc, sizeof(esc), "\\u%04X", (uint8_t)c);
        out += esc;
      }
      else
      {
        out += c;
      }
      break;
    }
  }
  return out;
}

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
  safeSyslogf(LOG_INFO, "JSON requested from %s", server.client().remoteIP().toString().c_str());

  char webpage[3072];
  char timestr[32];

  // Build string for web UI
  time_t t = safeNow(); // get current time
  tm *ptm = gmtime(&t);
  strftime(timestr, 32, "%Y-%m-%d %H:%M:%S UTC", ptm);

  GPS_ENTER_CRITICAL();
  auto satsValue = gps.satellites.value();
  auto hdopValue = gps.hdop.value();
  float gpsLat = gps.location.lat();
  float gpsLng = gps.location.lng();
  GPS_EXIT_CRITICAL();
  int sats = (satsValue != 255) ? satsValue : 0;
  String resol = gpsLocked ? String(hdopValue / 100.0, 2) : "N/A";

  // latitude & longitude
  float lat = 0.0;
  float lng = 0.0;
  if (gpsLocked)
  {
    lat = gpsLat;
    lng = gpsLng;
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
#else
    mcaddr = WiFi.macAddress();
#endif
  }
  else
  {
    wifmode = "Access Point";
    wifinet = convS(ssid);
    signal = 0;
#if defined(ESP32)
    mcaddr = getMacAddress(ESP_MAC_WIFI_SOFTAP);
#else
    mcaddr = WiFi.softAPmacAddress();
#endif
  }

  char uptime[24] = "";
  timeToString(uptime, sizeof(uptime), uptimeSeconds());

  String uptimeStr(uptime);

  // Anything this chip cannot measure stays "N/A" and the dashboard renders it as unavailable,
  // rather than publishing a zero that reads like a real measurement. The ESP8266 has no
  // minimum-heap watermark API and no internal temperature sensor.
  int freeHeap = 0;
  String minimumHeap = "N/A";
  String sdkVersion = "N/A";
  String internalTemp = "N/A";
#if defined(ESP32)
  freeHeap = esp_get_free_heap_size() / 1024;
  minimumHeap = String(esp_get_minimum_free_heap_size() / 1024);
  sdkVersion = esp_get_idf_version();
  // temperatureRead() needs the RF block powered, so it has nothing to report with the radio off.
  if (WiFi.getMode() != WIFI_OFF)
    internalTemp = String(temperatureRead(), 1);
#else
  freeHeap = ESP.getFreeHeap() / 1024;
  sdkVersion = ESP.getSdkVersion();
#endif

  String locked = gpsLocked ? "True" : "False";

  // PPS capture jitter, reported as deviation from the nominal 1000000us interval. This is the
  // measurement that has to justify any future reduction of the root dispersion baseline, so it is
  // published as observed - worst low, worst high, and the sample count behind them.
  uint32_t ppsMinUs = 0, ppsMaxUs = 0, ppsCount = 0, ppsOutliers = 0;
  uint64_t ppsSumUs = 0;
  readPpsIntervalStats(&ppsMinUs, &ppsMaxUs, &ppsCount, &ppsSumUs, &ppsOutliers);
  String ppsJitter = "N/A";
  String ppsPpm = "N/A";
  String ppsSamples = String(ppsCount);
  if (ppsCount > 0)
  {
    char jitter[48];
    snprintf(jitter, sizeof(jitter), "%+ld / %+ld us",
             (long)ppsMinUs - 1000000L, (long)ppsMaxUs - 1000000L);
    ppsJitter = jitter;
    // Mean interval vs. 1s is the local crystal's frequency offset, not capture error - a direct
    // check on the LOCAL_OSC_DRIFT_PPM assumed while coasting between edges.
    double meanUs = (double)ppsSumUs / (double)ppsCount;
    ppsPpm = String((float)((meanUs - 1000000.0) / 1000000.0 * 1e6), 2);
  }
  if (ppsOutliers > 0)
    ppsSamples += " (" + String(ppsOutliers) + " dropped)";

  // RTC Data. Each of these returns a plausible-looking value on a bus failure, so check
  // LastError() instead of publishing whatever fell out of a failed transaction.
  I2C_ENTER_CRITICAL();
  bool rtcDateTimeValid = Rtc.IsDateTimeValid();
  bool rtcReadOk = (Rtc.LastError() == 0);
  bool rtcRunning = Rtc.GetIsRunning();
  rtcReadOk = rtcReadOk && (Rtc.LastError() == 0);
  int8_t AgingOffset = Rtc.GetAgingOffset();
  rtcReadOk = rtcReadOk && (Rtc.LastError() == 0);
  float RTCTemperature = Rtc.GetTemperature().AsFloatDegC();
  rtcReadOk = rtcReadOk && (Rtc.LastError() == 0);
  I2C_EXIT_CRITICAL();

  String validDateTime = rtcReadOk ? (rtcDateTimeValid ? "True" : "False") : "Error";
  String isRTCRunning = rtcReadOk ? (rtcRunning ? "True" : "False") : "Error";
  if (!rtcReadOk)
  {
    AgingOffset = 0;
    RTCTemperature = 0.0;
  }

  int written = snprintf(webpage, sizeof(webpage),
          "{\"version\":\"%s\", \"uptime\":\"%s\", \"freeheap\":\"%d\", \"ip\":\"%s\", \"network\":\"%s\", \"type\":\"%s\",\
           \"signal\":\"%d\", \"timestamp\":\"%s\", \"lat\":\"%7.8f\", \"lon\":\"%7.8f\", \"sats\":\"%d\", \"hdop\":\"%s\",\
            \"syslog\":\"%s\", \"ssid\":\"%s\", \"minimumheap\":\"%s\", \"idfversion\":\"%s\", \"internaltemp\":\"%s\",\
             \"macaddress\":\"%s\", \"locked\":\"%s\", \"validdatetime\":\"%s\", \"rtcrunning\":\"%s\", \"agingoffset\":\"%d\",\
              \"rtctemperature\":\"%.1f\", \"ppsjitter\":\"%s\", \"ppsppm\":\"%s\",               \"ppssamples\":\"%s\", \"csrf\":\"%08X\"}",
          compile_date,
          uptimeStr.c_str(),
          freeHeap,
#if defined(ETHERNET_ENABLED)
          ("W: " + myIP.toString() + " - E: " + getEthIP().toString()).c_str(),
#else
          myIP.toString().c_str(),
#endif
          jsonEscape(wifinet).c_str(),
          wifmode.c_str(),
          signal,
          timestr,
          (float)lat,
          (float)lng,
          sats,
          resol.c_str(),
          jsonEscape(syslogserver).c_str(),
          jsonEscape(wifissid).c_str(),
          minimumHeap.c_str(),
          sdkVersion.c_str(),
          internalTemp.c_str(),
          mcaddr.c_str(),
          locked.c_str(),
          validDateTime.c_str(),
          isRTCRunning.c_str(),
          AgingOffset,
          RTCTemperature,
          ppsJitter.c_str(),
          ppsPpm.c_str(),
          ppsSamples.c_str(),
          csrfToken);
  if (written < 0 || (size_t)written >= sizeof(webpage))
  {
    // Truncated output would be invalid JSON, which the dashboard cannot parse at all.
    safeSyslog(LOG_ERR, "JSON status document did not fit its buffer");
    server.send(500, "text/plain", "status document too large");
    return;
  }
  server.send(200, "application/json", webpage);
}

void handleRoot()
{
  const char *dataType = "text/html";

  safeSyslogf(LOG_INFO, "Webpage requested from %s", server.client().remoteIP().toString().c_str());

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
  static bool handlersRegistered = false;
  if (!handlersRegistered)
  {
#if defined(GNSS_HTTP_HARDENED)
  // The framework parses a request body - including every multipart field and the firmware image -
  // before it ever looks for a route handler, so route-level authentication runs far too late to
  // keep an anonymous client out of the parser. This hook runs after the bounded headers and
  // before the first body byte is consumed: an unauthenticated body-bearing request is refused
  // here, and the connection is dropped. See scripts/http_hardening/.
  server.onBeforeBody([]() -> bool
                      {
    if (server.authenticate(www_username, activeAdminPassword()))
      return true;
    server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
    safeSyslogf(LOG_WARNING, "Refused an unauthenticated request body from %s",
                server.client().remoteIP().toString().c_str());
    return false; });
#endif
  server.on("/", handleRoot);
  server.on("/updatewifi", HTTP_POST, handleUpdate);
  server.on("/json", handleJSON);
#if defined(ESP32)
  server.on("/serverUpdate", HTTP_GET, []()
            {
        if (!server.authenticate(www_username, activeAdminPassword()))
        {
          return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
        }
        if (!otaAllowed())
        {
          return server.send(403, "text/plain", otaBlockedMessage);
        }
    server.sendHeader("Connection", "close");
    //server.send(200, "text/html", serverUpdate); });
    server.send(200, "text/html", PAGE_Update); });
  /*handling uploading firmware file */
  server.on(
      "/update", HTTP_POST, []()
      {
        if (!server.authenticate(www_username, activeAdminPassword()))
        {
          return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
        }
        if (!otaAllowed())
        {
          return server.send(403, "text/plain", otaBlockedMessage);
        }
    server.sendHeader("Connection", "close");
    // Reboot only for a flash we actually completed. Update.hasError() is false for a POST that
    // carried no firmware at all, so the old test answered OK and restarted the device without
    // having written anything - and _error persists between requests, so it is not a per-request
    // result either.
    bool flashed = otaCompleted;
    otaStarted = false;
    otaCompleted = false;
    server.send(200, "text/plain", flashed ? "OK" : "FAIL");
    if (!flashed)
    {
      safeSyslog(LOG_ERR, "OTA request finished without a completed firmware image");
      return;
    }
    delay(200); // let the response flush before restarting
    ESP.restart(); },
      []()
      {
        if (!server.authenticate(www_username, activeAdminPassword()))
        {
          return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
        }
        // Checked again here, not only in the completion handler: this callback is what arms and
        // feeds the flash writer, so an unprovisioned device must never reach Update.begin().
        if (!otaAllowed())
        {
          return;
        }
        HTTPUpload &upload = server.upload();
        if (upload.status == UPLOAD_FILE_START)
        {
          // Serial.printf("Update: %s\n", upload.filename.c_str());
          otaStarted = false;
          otaCompleted = false;
          if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          { // start with max available size
            safeSyslogf(LOG_ERR, "OTA Update.begin failed: %s", Update.errorString());
          }
          else
          {
            otaStarted = true;
          }
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
          /* flashing firmware to ESP*/
          if (otaStarted && !Update.hasError()) // skip once already doomed, so we don't hammer syslog on every remaining chunk
          {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
            {
              safeSyslogf(LOG_ERR, "OTA Update.write failed: %s", Update.errorString());
            }
          }
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
          if (otaStarted && Update.end(true))
          { // true to set the size to the current progress
            otaCompleted = true;
            // Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          }
          else if (otaStarted)
          {
            safeSyslogf(LOG_ERR, "OTA Update.end failed: %s", Update.errorString());
          }
        }
        else if (upload.status == UPLOAD_FILE_ABORTED)
        {
          // The client disconnected mid-upload. Without an explicit abort the updater stays armed
          // with _size set, and every later Update.begin() fails with "already running" until the
          // board is power cycled - so one dropped upload takes OTA recovery down with it.
          if (otaStarted)
          {
            Update.abort();
            safeSyslog(LOG_ERR, "OTA upload aborted by the client - updater reset");
          }
          otaStarted = false;
          otaCompleted = false;
        }
      });
#endif
  handlersRegistered = true;
  }
  server.begin();
  DEBUG_PRINTLN(F("HTTP server started"));
  safeSyslog(LOG_INFO, "HTTP server started");
}

void enableWifiAP()
{
  // WiFi Initialization as an AP
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.mode(WIFI_AP);
// WiFi.softAP(ssid, psk, channel, hidden, max_connection)
// Setting maximum of 8 clients, 1 is default channel already, 0 is false for hidden SSID - The maximum allowed by ES8266 is 8 - Thanks to Mitch Markin for that
// ESP32 got a 10 as maximum
  const char *apPsk = activeApPassword();
#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.softAP(ssid, apPsk, AP_CHANNEL, 0, 8);
#elif defined(ESP32)
  WiFi.softAP(ssid, apPsk, AP_CHANNEL, 0, 10);
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

  // No trim() here - readData() has already removed the storage delimiter, and a passphrase or
  // SSID may legitimately begin or end with a space.
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
  if (WiFi.status() != WL_CONNECTED)
  {
    enableWifiAP();
    return; // AP fallback already started its own HTTP server; don't fall through to the STA-connected branch
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
    safeSyslogf(LOG_INFO, "WiFi connected as %s", myIP.toString().c_str());
#endif
    startHttpServer();
  }
}

void disableWifi()
{
  server.stop();
  DEBUG_PRINTLN(F("HTTP server stopped"));
  safeSyslog(LOG_WARNING, "WiFi disabled!");
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
    safeSyslog(LOG_ERR, "ERROR: cannot read the RTC.");
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

  I2C_ENTER_CRITICAL();
  Rtc.SetDateTime(timeToSet);
  bool rtcSetOk = (Rtc.LastError() == 0);
  I2C_EXIT_CRITICAL();
  if (rtcSetOk)
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
    safeSyslog(LOG_ERR, "ERROR: cannot set RTC time");
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
  time_t t = safeNow();                            // get current time
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
  GPS_ENTER_CRITICAL();
  auto satsValue = gps.satellites.value();
  auto hdopValue = gps.hdop.value();
  GPS_EXIT_CRITICAL();

  String sats = "";
  if (satsValue != 255)
    sats = String(satsValue);
  else
    sats = "0";

  String resol = "";
  if (gpsLocked)
    resol = String(hdopValue / 100.0, 2);
  else
    resol = "--";

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
  u8g2.drawStr(0, 45, getEthIP().toString().c_str());
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

// Called from SyncCheck() with the PPS edge that triggered this attempt, so the GPS sentence can
// be tied to a specific edge instead of to "whenever this happened to run".
void SyncWithGPS(uint32_t edgeMicros)
{
  NmeaRmcClock::Sample sample = {};
  GPS_ENTER_CRITICAL();
  bool validSample = gpsClock.sampleForEdge(edgeMicros, millis(), GPS_RMC_MAX_AGE_MS,
                                           NMEA_EDGE_MIN_LEAD_US, NMEA_EDGE_MAX_LEAD_US,
                                           MIN_VALID_YEAR, MAX_VALID_YEAR, sample);
  GPS_EXIT_CRITICAL();
  if (!validSample)
  {
    DEBUG_PRINTLN("GPS sync rejected: no complete, current RMC record preceding this PPS edge");
    // A rejection repeats on every PPS edge until a sync succeeds, so throttle the syslog line.
    static uint32_t lastRejectLogMillis = 0;
    uint32_t nowMillis = millis();
    if (lastRejectLogMillis == 0 || (nowMillis - lastRejectLogMillis) >= GPS_REJECT_LOG_INTERVAL_MS)
    {
      lastRejectLogMillis = nowMillis;
      safeSyslog(LOG_WARNING, "GPS sync rejected: no complete, current RMC record preceding this PPS edge");
    }
    return;
  }

  tmElements_t tmGps;
  tmGps.Year = (uint8_t)(sample.year - 1970); // tmElements_t counts years from 1970
  tmGps.Month = sample.month;
  tmGps.Day = sample.day;
  tmGps.Hour = sample.hour;
  tmGps.Minute = sample.minute;
  tmGps.Second = sample.second;
  tmGps.Wday = 0; // makeTime() recomputes this
  time_t ppsSecond = makeTime(tmGps) + 1; // the edge starts the second after the one just reported

  // Anchor the NTP clock to the edge itself. TimeLib is set to the same second, but only drives the
  // display and the RTC update from here on - it is no longer on the NTP timestamp path.
  if (!setPpsAnchor(ppsSecond, edgeMicros))
  {
    DEBUG_PRINTLN("GPS sync discarded - a newer PPS edge arrived while it was being processed");
    return;
  }
  safeSetTime(ppsSecond);

  DEBUG_PRINT("Time from GPS: ");
  DEBUG_PRINT(sample.hour);
  DEBUG_PRINT(":");
  DEBUG_PRINT(sample.minute);
  DEBUG_PRINT(":");
  DEBUG_PRINTLN(sample.second);

  // Group the sync-state writes into one critical section so processNTP() never reads a partial
  // update (e.g. new syncTime paired with a stale syncFraction from the previous sync) - finding I
  SYNCSTATE_ENTER_CRITICAL();
  syncTime = ppsSecond;        // remember time of this sync
  lastGpsSyncTime = ppsSecond; // GPS-specific sync time, for the holdover dispersion estimate
  bool wasLocked = gpsLocked;
  gpsLocked = true; // set flag that time reflects GPS time
  syncFraction = 0; // the sync instant IS the PPS edge, so its NTP fraction is exactly zero
  SYNCSTATE_EXIT_CRITICAL();

  if (!wasLocked)
  {
    GPS_ENTER_CRITICAL();
    auto satsValue = gps.satellites.value();
    GPS_EXIT_CRITICAL();
    safeSyslogf(LOG_INFO, "GPS sychronized - %d satellites", satsValue);
  }
  UpdateRTC();                       // update internal RTC clock periodically
  DEBUG_PRINTLN("GPS synchronized"); // send message to serial monitor
}

void SyncWithRTC()
{
  I2C_ENTER_CRITICAL();
  RtcDateTime time = Rtc.GetDateTime();
  bool readOk = (Rtc.LastError() == 0);
  // IsDateTimeValid() reads the oscillator-stop flag and ANDs in its own bus result, so it has to
  // run under the same mutex as the read above.
  bool rtcTrusted = readOk && Rtc.IsDateTimeValid();
  I2C_EXIT_CRITICAL();

  // RtcDS3231::GetDateTime() returns RtcDateTime(0) - i.e. 2000-01-01 - on any I2C failure, and a
  // running oscillator says nothing about whether the stored time survived a battery loss. Taking
  // the reading unconditionally therefore lets a transient bus glitch throw the system clock back
  // to the year 2000. Keep the clock we already have and report the fault instead.
  if (!rtcTrusted || !time.IsValid() || time.Year() < MIN_VALID_YEAR || time.Year() > MAX_VALID_YEAR)
  {
    DEBUG_PRINTLN("RTC read rejected - system clock left unchanged");
    // The DS3231 oscillator-stop flag is only cleared by SetDateTime(), so a board that has not had
    // a GPS lock since it last lost RTC power fails this check on every single attempt. Report it,
    // but do not repeat it at the retry cadence. The throttle window only starts once syslog can
    // actually deliver, so the first reachable report is never swallowed by a boot-time attempt.
    static uint32_t lastRejectLogMillis = 0;
    uint32_t nowMillis = millis();
    if (syslogEnabled && (lastRejectLogMillis == 0 || (nowMillis - lastRejectLogMillis) >= RTC_REJECT_LOG_INTERVAL_MS))
    {
      lastRejectLogMillis = nowMillis;
      safeSyslogf(LOG_ERR, "RTC read rejected (bus %s, oscillator flag %s, year %u) - system clock left unchanged",
                  readOk ? "ok" : "error", rtcTrusted ? "ok" : "invalid", (unsigned)time.Year());
    }
    return;
  }

#if defined(ARDUINO_ARCH_ESP8266)
  // Deprecated on ESP32
  time_t a = time.Epoch32Time();
#elif defined(ESP32)
  time_t a = time.Unix32Time();
#else
#error Unknown architecture
#endif
  safeSetTime(a); // set system time from RTC
  DEBUG_PRINT("SyncFromRTC: ");
  DEBUG_PRINTLN(a);
  time_t syncedNow = safeNow(); // captured before the sync-state lock - safeNow() takes its own separate lock
  SYNCSTATE_ENTER_CRITICAL();
  syncTime = syncedNow;         // and remember time of this sync event
  syncFraction = 0;             // no PPS discipline during an RTC sync, so there is no sub-second phase
  SYNCSTATE_EXIT_CRITICAL();
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
  // Take the flag and the edge timestamps together - testing the flag and clearing it separately
  // both loses edges and lets the surviving flag be paired with the wrong edge.
  uint32_t edgeMicros = 0;
  bool havePps = consumePpsEvent(&edgeMicros);
  unsigned long timeSinceSync = safeNow() - syncTime; // how long has it been since last sync?
  if (havePps && (timeSinceSync >= SYNC_INTERVAL))
  { // is it time to sync with GPS yet?
    DEBUG_PRINTLN("Called SyncWithGPS from SyncCheck");
    SyncWithGPS(edgeMicros); // yes, so attempt it.
    timeSinceSync = safeNow() - syncTime; // refresh: SyncWithGPS() may have just updated syncTime
  }
  if (timeSinceSync >= SYNC_TIMEOUT) // GPS sync has failed
  {
    if (gpsLocked)
    {
      safeSyslog(LOG_INFO, "GPS sych lost!");
    }
    SYNCSTATE_ENTER_CRITICAL();
    gpsLocked = false; // flag that clock is no longer in GPS sync
    SYNCSTATE_EXIT_CRITICAL();
    invalidatePpsAnchor(); // and stop serving PPS-derived timestamps too
    if (rtcAvailable)
    {
      // Pace the retries. A successful sync updates syncTime, which drops this whole branch for
      // another SYNC_TIMEOUT - but a REJECTED read deliberately leaves syncTime alone, so the
      // branch stays true and SyncWithRTC() would otherwise run on every pass of this task (~1ms),
      // hammering the I2C bus and flooding the syslog socket until lwIP runs out of buffers.
      // On a healthy RTC this is a no-op and the holdover cadence is unchanged.
      static uint32_t lastRtcAttemptMillis = 0;
      uint32_t nowMillis = millis();
      if (lastRtcAttemptMillis == 0 || (nowMillis - lastRtcAttemptMillis) >= RTC_RETRY_INTERVAL_MS)
      {
        lastRtcAttemptMillis = nowMillis;
        DEBUG_PRINTLN("Called SyncWithRTC from SyncCheck");
        SyncWithRTC(); // sync with RTC instead
      }
    }
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
  // micros() only: on ESP32 millis() divides a 64-bit timer value, and the helper that does it is
  // not guaranteed to be in IRAM, so calling it here could fault an edge that lands while flash is
  // busy (a LittleFS write, an OTA flash).
  uint32_t edgeMicros = micros();

  PPS_ENTER_CRITICAL_ISR();
  // Interval against the previous edge, for the capture-jitter statistics. Measured from the raw
  // edge timestamps rather than the anchor, so it keeps working while the anchor is invalid.
  if (ppsIntervalPrevValid)
  {
    uint32_t interval = edgeMicros - ppsIntervalPrevMicros; // unsigned: correct across the wrap
    if (interval >= PPS_MIN_PERIOD_US && interval <= PPS_MAX_PERIOD_US)
    {
      if (ppsIntervalCount == 0 || interval < ppsIntervalMinUs)
        ppsIntervalMinUs = interval;
      if (ppsIntervalCount == 0 || interval > ppsIntervalMaxUs)
        ppsIntervalMaxUs = interval;
      ppsIntervalSumUs += interval;
      ppsIntervalCount++;
    }
    else
    {
      ppsIntervalOutliers++; // a missed or glitched pulse, not capture error
    }
  }
  ppsIntervalPrevMicros = edgeMicros;
  ppsIntervalPrevValid = true;

  // Publish the event and the timestamp captured with it as one unit, so SyncCheck() can never
  // pair a surviving flag with an edge it did not actually observe.
  ppsEventMicros = edgeMicros;
  ppsEventPending = true;
  if (ppsAnchorValid)
  {
    // Roll the anchor forward one second per edge, which is what keeps it disciplined between the
    // (comparatively rare) GPS syncs. An edge that is not roughly 1s after the current anchor means
    // pulses were missed or the input glitched, so drop the anchor rather than silently counting
    // the wrong number of seconds - the next GPS sync re-establishes it.
    uint32_t sinceAnchor = edgeMicros - ppsAnchorMicros;
    if (sinceAnchor >= PPS_MIN_PERIOD_US && sinceAnchor <= PPS_MAX_PERIOD_US)
    {
      ppsAnchorSecond = ppsAnchorSecond + 1;
      ppsAnchorMicros = edgeMicros;
    }
    else
    {
      ppsAnchorValid = false;
    }
  }
  PPS_EXIT_CRITICAL_ISR();

#if defined(ARDUINO_ARCH_ESP8266)
  pps_blink_time = millis(); // Capture time in order to turn led off so we can get the blink effect ever x milliseconds
#elif defined(ESP32)
    pps_blink_time = xTaskGetTickCountFromISR();
#else
#error Unknown architecture
#endif
  // digitalWrite(PPS_LED, HIGH); // Ligth up led pps monitor - Moved to loop (lower priority) - 20231113
  // pps_blink_time = millis();   // Capture time in order to turn led off so we can get the blink effect ever x milliseconds - On loop
  //  DEBUG_PRINTLN("pps"); Sometimes cause: esp32 Guru Meditation Error: Core  1 panic'ed (Interrupt wdt timeout on CPU1).
}

// Handle button pressed interrupt
void IRAM_ATTR btw() // INTERRUPT SERVICE REQUEST
{
  int currentState = digitalRead(WIFI_BUTTON);
  unsigned long nowMs = millis();
  int previousState = lastState;
  lastState = currentState; // always track the pin's true state, even on edges we ignore below

  if ((currentState != previousState) && (nowMs - lastEdgeTick >= EDGE_LOCKOUT_MS) && !buttonPressed)
  {
    lastEdgeTick = nowMs;
    if (previousState == HIGH)
    {
      keyPressTick = nowMs;
    }
    else
    {
      keyReleaseTick = nowMs;
      buttonPressed = true;
      // KeyCheck();
    }
  }
  // DEBUG_PRINTLN(F("BUTTON PRESSED!")); Sometimes cause: esp32 Guru Meditation Error: Core  1 panic'ed (Interrupt wdt timeout on CPU1).
}

void processKeyHold()
{
  if (statusWifi)
    statusWifi = 0;
  else
    statusWifi = 1;

  writeData("/statusWifi", String(statusWifi)); // Save last WiFi status so it can be restored on next boot

  processWifi();
  DEBUG_PRINTLN(F("BUTTON HOLD PROCESSED!"));
}

// Triggered by holding the front-panel button for BTN_FACTORY_RESET_MS. Erases every provisioned
// setting and reboots, which returns the device to the compiled-in defaults: no stored Wi-Fi
// credentials, so it comes back up as an access point on DEFAULT_AP_PASSWORD with the dashboard on
// DEFAULT_ADMIN_PASSWORD. Those two are deliberately NOT randomised - a sealed unit with no serial
// access has to stay recoverable with credentials its owner already knows.
void processFactoryReset()
{
  DEBUG_PRINTLN(F("FACTORY RESET REQUESTED!"));
  safeSyslog(LOG_WARNING, "Factory reset from the front panel - erasing the stored configuration");

  // Say so on the display before the reboot: this drops the Wi-Fi credentials, so what comes back
  // is an access point the user may not be expecting.
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.drawStr(0, 24, "FACTORY");
  u8g2.drawStr(0, 48, "RESET");
  I2C_ENTER_CRITICAL();
  u8g2.sendBuffer();
  I2C_EXIT_CRITICAL();

  // Every persisted setting lives on LittleFS (/wifissid, /wifipsk, /wifiappsk, /adminpsk,
  // /syslogserver, /statusWifi), so one format clears the lot.
  if (!LittleFS.format())
  {
    DEBUG_PRINTLN(F("LittleFS format failed!"));
    safeSyslog(LOG_ERR, "Factory reset: LittleFS format failed");
  }

  delay(2000); // leave the message up long enough to be read
  ESP.restart();
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
  int bytesProcessed = 0;
  while (ss.available() && bytesProcessed < 256) // look for data from GPS module; capped so a misbehaving GNSS can't starve lower-priority tasks
  {
    char c = ss.read(); // read in all available chars
    bytesProcessed++;
    GPS_ENTER_CRITICAL();
    gps.encode(c); // and feed chars to GPS parser
    gpsClock.encode(c, micros(), millis()); // independent date/time fields from this exact RMC
    GPS_EXIT_CRITICAL();
                        // Serial.write(c);    // Uncomment for some extra debug info if in doubt about GPS feed
#ifdef DEBUG_GPS
    DEBUG_GPS_PRINT(c);
    // Serial.write(c);
#endif
    // Lock indication is driven by lockLedTicker, independent of serial traffic.
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
  time_t t = safeNow();     // get current time
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
    I2C_ENTER_CRITICAL();
    u8g2.sendBuffer(); // Send new information to display
    I2C_EXIT_CRITICAL();

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
    // safeSyslogf(LOG_INFO, "RDATE request from %s", client.remoteIP().toString().c_str());
    //  Serial.println(client.remoteIP().toString().c_str());

    // Send Data to connected client
    time_t t = safeNow(); // get current time

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
  if (packetSize >= NTP_PACKET_SIZE)
  {
    // A COMPLETE receive timestamp, both halves from a single anchor reading, captured as close to
    // packet arrival as possible. Sharing one integer between the receive and transmit timestamps
    // while sampling their fractions at different instants lets a second boundary in between make
    // the transmit timestamp older than the receive timestamp, which clients reject outright.
    //
    // Taken BEFORE Udp.read(), which is a memcpy out of the socket's buffer and contributes
    // nothing but delay to T2. Note this is only the last slice of the real arrival-to-timestamp
    // gap: parsePacket() above already did the recvfrom(), a malloc() and a buffer copy, and the
    // task had to be scheduled before that. Hardware RX timestamping is the only way to close the
    // rest, so do not read the advertised precision as the accuracy of this instant.
    uint32_t rxSeconds = 0;
    uint32_t rxFraction = 0;
    uint32_t rxCoastMicros = 0;
    bool haveRx = ppsTimestamp(&rxSeconds, &rxFraction, &rxCoastMicros);
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();
    // Packets bigger than NTP_PACKET size (like the ones generated by ntpq -c sysinfo <SERVER_IP>, which is 136 bytes) will block the UDP instance if not flushed. Not ready to deal with them right now, maybe in the future. Need to flush the remaining bytes.
    Udp.flush();

    byte requestMode = packetBuffer[0] & 0x07;
    if (requestMode == 3) // only answer client-mode (RFC 5905 section 7.3) requests
    {

    // Snapshot the sync-state group once so this whole packet is built from one consistent view,
    // instead of re-reading gpsLocked/syncTime/syncFraction/lastGpsSyncTime independently at each
    // use site below (which could otherwise mix fields from two different sync events) - finding I
    SYNCSTATE_ENTER_CRITICAL();
    bool lockedSnapshot = gpsLocked;
    time_t syncTimeSnapshot = syncTime;
    uint32_t syncFractionSnapshot = syncFraction;
    time_t lastGpsSyncSnapshot = lastGpsSyncTime;
    SYNCSTATE_EXIT_CRITICAL();

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

    safeSyslogf(LOG_INFO, "   Received UDP packet size %d  LI, Vers, Mode : 0x%02x  Stratum: 0x%02x  Polling: 0x%02x  Precision: 0x%02x", packetSize, LIVNMODE, STRATUM, POLLING, PRECISION);
#endif

    // Stratum 1 requires BOTH a recent sync and a usable PPS anchor. gpsLocked on its own stays
    // true for up to SYNC_TIMEOUT after the pulses stop, and with no anchor the timestamps degrade
    // to whole seconds - up to a full second of error, advertised as 47ms of dispersion.
    bool disciplined = lockedSnapshot && haveRx;
    if (!haveRx)
    {
      // Undisciplined fallback: whole seconds from TimeLib, which is all we have. Stratum 16 and
      // LI=3 below tell the client to discard it anyway.
      rxSeconds = (uint32_t)safeNow() + (uint32_t)seventyYears;
      rxFraction = 0;
    }

    packetBuffer[0] = disciplined ? 0b00100100 : 0b11100100; // LI=0 (synced) or 3/alarm (unsynchronized); VN=4, Mode=4
    if (disciplined)
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
    packetBuffer[3] = disciplined ? 0xF4 : 0x00; // precision: 2^-12 (~244us, PPS-disciplined) when locked, 2^0 = 1s in holdover

    // Root delay: RFC 5905 defines this as the total round-trip delay to the REFERENCE clock, not
    // to the client. The reference here is a PPS line on a GPIO pin, so that delay is zero - which
    // is also what ntpd reports for a local refclock. The inherited 0x00000800 (= 2048/65536 =
    // 31.25 ms) was a value copied from a stratum-2 example and never described this server: it
    // inflated the root distance clients compute (rootdelay/2 + rootdisp) by 15.6 ms for nothing.
    packetBuffer[4] = 0; // root delay = 0
    packetBuffer[5] = 0;
    packetBuffer[6] = 0;
    packetBuffer[7] = 0;

    // int year;
    // byte month, day, hour, minute, second, hundredths;
    // unsigned long date, time, age;
    time_t t = safeNow();

    // root dispersion: fixed ~47ms baseline (matches the original hardcoded value) when
    // GPS-locked; grows with time-since-last-GPS-fix during holdover, using the DS3231's
    // worst-case datasheet drift spec (3.5 ppm) as a pessimistic upper-bound estimate of
    // accumulated clock uncertainty (not a live measurement).
    const float RTC_HOLDOVER_DRIFT_PPM = 3.5;
    // XXX UNCHARACTERIZED BASELINE - deliberately left high until it is measured. XXX
    //
    // 46.875 ms is inherited, not derived from anything about this hardware. Adding up what is
    // actually known - receiver PPS spec (tens of ns), micros() quantum (1 us) and the advertised
    // 2^-12 precision (244 us) - suggests the true figure is nearer 1 ms, which would cut the root
    // distance clients compute (rootdelay/2 + rootdisp) by a factor of ~45.
    //
    // It is NOT being lowered here, because the error is asymmetric: overstating dispersion only
    // makes clients weight this server conservatively, while understating it makes them trust it
    // MORE than it deserves and skew their own clocks by more than they were told to expect - a
    // failure that is silent at both ends. Lowering it needs evidence, in this order:
    //   1. the PPS capture jitter now published on the dashboard and in /json (min/max deviation
    //      from 1000000us), left running for 24-48h to catch the tail rather than the average;
    //   2. absolute ISR entry latency, from a scope or logic analyser comparing the PPS line
    //      against a pin toggled at the top of isr() - the interval statistics cannot see a
    //      constant latency, only a varying one;
    //   3. end-to-end offset against an independent GPS-disciplined reference (a Pi with kernel
    //      PPS and chrony is ~100x better than this device, so it is a valid yardstick) over a
    //      quiet wired LAN, which is the only step that also covers the T2/T3 software
    //      timestamping error folded into this figure.
    // Take the worst case of each, add margin, and round up.
    float dispersionSec = 0.046875; // = original hardcoded 0x00000C00
    if (disciplined)
    {
      // Missed pulses mean the anchor is being extrapolated on the local crystal; say so rather
      // than implying the anchor is still exact however long the pulses have been gone.
      if (rxCoastMicros > 1000000UL)
        dispersionSec += (rxCoastMicros * 1e-6f) * LOCAL_OSC_DRIFT_PPM * 1e-6f;
    }
    else if (lastGpsSyncSnapshot != 0 && t > lastGpsSyncSnapshot)
    {
      unsigned long holdoverSecs = t - lastGpsSyncSnapshot;
      dispersionSec += holdoverSecs * RTC_HOLDOVER_DRIFT_PPM * 1e-6;
    }
    uint32_t dispersionNtp = (uint32_t)(dispersionSec * 65536.0);
    packetBuffer[8] = (dispersionNtp >> 24) & 0xFF;
    packetBuffer[9] = (dispersionNtp >> 16) & 0xFF;
    packetBuffer[10] = (dispersionNtp >> 8) & 0xFF;
    packetBuffer[11] = dispersionNtp & 0xFF;

#ifdef DEBUG
    Serial.println(rxSeconds);
    // print_date(gps);
#endif

    if (disciplined)
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

    // reference timestamp - time the system clock was last set/corrected (GPS or RTC sync),
    // per RFC 5905, rather than the current instant
    uint32_t refTimestamp = (uint32_t)syncTimeSnapshot + seventyYears;
    packetBuffer[16] = (refTimestamp >> 24) & 0XFF;
    packetBuffer[17] = (refTimestamp >> 16) & 0xFF;
    packetBuffer[18] = (refTimestamp >> 8) & 0xFF;
    packetBuffer[19] = refTimestamp & 0xFF;

    packetBuffer[20] = (syncFractionSnapshot >> 24) & 0xFF;
    packetBuffer[21] = (syncFractionSnapshot >> 16) & 0xFF;
    packetBuffer[22] = (syncFractionSnapshot >> 8) & 0xFF;
    packetBuffer[23] = syncFractionSnapshot & 0xFF;

    // copy originate timestamp from incoming UDP transmit timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    // receive timestamp - the pair captured together when the request arrived
    packetBuffer[32] = (rxSeconds >> 24) & 0XFF;
    packetBuffer[33] = (rxSeconds >> 16) & 0xFF;
    packetBuffer[34] = (rxSeconds >> 8) & 0xFF;
    packetBuffer[35] = rxSeconds & 0xFF;

    packetBuffer[36] = (rxFraction >> 24) & 0xFF;
    packetBuffer[37] = (rxFraction >> 16) & 0xFF;
    packetBuffer[38] = (rxFraction >> 8) & 0xFF;
    packetBuffer[39] = rxFraction & 0xFF;

    // Reply to the IP address and port that sent the NTP request.
    //
    // beginPacket() runs BEFORE the transmit timestamp is taken: on the first call it mallocs the
    // 1460-byte transmit buffer, and everything it does is delay between reading the clock and
    // handing the datagram to the stack. Taking T3 after it, immediately before the write, is the
    // closest this can get in software - endPacket() -> sendto() -> stack -> PHY still happens
    // afterwards and only hardware TX timestamping could account for it.
    //
    // T2 (above) and T3 bias the client's computed offset, ((T2-T1) + (T3-T4))/2, in OPPOSITE
    // directions, so the two placements were partly cancelling before. They are corrected as a
    // pair for that reason; moving only one would trade a smaller delay figure for a worse offset.
    Udp.beginPacket(Remote, PortNum);

    // transmit timestamp - its own complete reading, taken as close to the send as possible
    uint32_t txSeconds = rxSeconds;
    uint32_t txFraction = rxFraction;
    if (haveRx)
      ppsTimestamp(&txSeconds, &txFraction); // keeps the receive pair if the anchor just went away

    packetBuffer[40] = (txSeconds >> 24) & 0XFF;
    packetBuffer[41] = (txSeconds >> 16) & 0xFF;
    packetBuffer[42] = (txSeconds >> 8) & 0xFF;
    packetBuffer[43] = txSeconds & 0xFF;

    packetBuffer[44] = (txFraction >> 24) & 0xFF;
    packetBuffer[45] = (txFraction >> 16) & 0xFF;
    packetBuffer[46] = (txFraction >> 8) & 0xFF;
    packetBuffer[47] = txFraction & 0xFF;

    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();

    // safeSyslogf(LOG_INFO, "NTP request from %s", Remote.toString().c_str());
    }
  }
  else if (packetSize > 0)
  {
    Udp.flush(); // drain and discard undersized/malformed datagrams
  }
#if defined(ESP32)
  vTaskDelay(1);
  // taskYIELD();
}
#endif
}

void setup()
{
#if defined(ESP32)
  i2cMutex = xSemaphoreCreateMutex();
  syslogMutex = xSemaphoreCreateMutex();
#endif
#if defined(DEBUG) || defined(DEBUG_GPS)
  Serial.begin(115200); // set serial monitor rate to 115200 bps
#endif
  // Serial.begin(115200); // set serial monitor rate to 115200 bps

  DEBUG_PRINTLN(F("Starting setup..."));
  pinMode(LOCK_LED, OUTPUT);
  pinMode(PPS_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(WIFI_BUTTON, INPUT_PULLUP);

  digitalWrite(LOCK_LED, HIGH);
  lockLedTicker.attach_ms(500, updateLockLed);
  digitalWrite(PPS_LED, HIGH);
  digitalWrite(WIFI_LED, HIGH);

  InitLCD(); // initialize LCD display

  // Start printing boot diagnostic messages to help track issues
  u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
  u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer); // connect to u8g2, assign buffer
  u8g2log.setLineHeightOffset(0);                               // set extra space between lines in pixel, this can be negative
  u8g2log.setRedrawMode(0);                                     // 0: Update screen with newline, 1: Update screen for every char

#if defined(ESP32)
  // Arduino's network initializer is idempotent and also starts its event machinery. WiFi OFF
  // at boot never calls it, but the NTP/RFC868 sockets still need the TCP/IP mailbox initialized.
  extern bool tcpipInit();
  if (!tcpipInit())
  {
    u8g2log.print("Network init failed!\n");
    while (true)
      delay(1000);
  }
#endif

#if defined(ARDUINO_ARCH_ESP8266)
#elif defined(ESP32)
          u8g2log.print("Reset reason: ");
          u8g2log.print(return_reset_reason(esp_reset_reason()));
          delay(300);
#else
#error Unknown architecture
#endif
  // u8g2log.print("\nDiagnostics messages:\n");

  // if you are using ESP-01 then uncomment the line below to reset the pins to
  // the available pins for SDA, SCL
  DEBUG_PRINTLN(F("Starting LittleFS"));

  u8g2log.print("\nStarting COM and FS...");
#if defined(ARDUINO_ARCH_ESP8266)
  // ss.begin (13U, 15U); // ESP8266 D7, D8
  ss.begin(9600);     // set GPS baud rate to 9600 bps
  Wire.begin(4U, 5U); // ESP8266 D2, D1 - due to limited pins, use pin 0 and 2 for SDA, SCL
  if (!LittleFS.begin()) // LittleFSConfig defaults to auto-format-on-fail on this core
  {
    DEBUG_PRINTLN(F("ERROR: LittleFS mount failed"));
    u8g2log.print("LittleFS FAILED!\n");
  }
  u8g2log.print(" Done\n");
#elif defined(ESP32)
          Wire.begin(21U, 22U);
          if (!LittleFS.begin(true)) // true = FORMAT_LITTLEFS_IF_FAILED
          {
            DEBUG_PRINTLN(F("ERROR: LittleFS mount failed"));
            u8g2log.print("LittleFS FAILED!\n");
          }
          u8g2log.print(" Done\n");
#if defined(ETHERNET_ENABLED)
// For some reason Freenove board is resetting the baud rate on ETH build - 20250124
#define BAUD_RATE 115200
          u8g2log.print("Starting ETH...");
          setupW5500();
          // Extra delay to avoid panic restart possible due to unfinished intialization
          // Need to investigate better in the future - Cristiano - 20250125
          delay(1000);
          u8g2log.print(" Done\n");
#endif

#ifndef BAUD_RATE
#define BAUD_RATE 115200
#endif
          ss.begin(BAUD_RATE);

#else
#error Unknown architecture
#endif
  delay(300);

  // Must run after LittleFS is mounted and before anything that answers on the network: it is what
  // decides whether this unit is running on stored credentials or on the compiled-in defaults, and
  // warns on the serial console if it is still on the defaults.
  u8g2log.print("Device credentials...");
  if (!initializeDeviceCredentials())
  {
    // Coming up with credentials that were not durably stored would leave the device reachable
    // with whatever is in flash while reporting something else, so stop here instead.
    DEBUG_PRINTLN(F("ERROR: could not establish device credentials"));
    u8g2log.print("\nCREDENTIALS FAILED!\n");
    while (true)
      delay(1000);
  }
  u8g2log.print(credentialsProvisioned ? " Done\n" : " SETUP\n");
  delay(300);

  u8g2log.print("Starting RTC...");
  Rtc.Begin();
  RtcEeprom.Begin();
  // Initialize RTC
  const int RTC_START_RETRIES = 5;
  int rtcRetry = 0;
  while (!Rtc.GetIsRunning() && rtcRetry < RTC_START_RETRIES)
  {
    Rtc.SetIsRunning(true);
    DEBUG_PRINTLN(F("RTC had to be force started"));
    // safeSyslog(LOG_WARNING, "RTC had to be force started");
    u8g2log.print("RTC had to be force started\n");
    rtcRetry++;
  }
  rtcAvailable = Rtc.GetIsRunning();
  if (!rtcAvailable)
  {
    DEBUG_PRINTLN(F("RTC not responding - continuing without it"));
    u8g2log.print("RTC not responding!\n");
  }

  DEBUG_PRINTLN(F("RTC started"));

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);

#ifdef DEBUG
  PrintRTCstatus(); // show RTC diagnostics
#endif
  u8g2log.print("Done\n");
  delay(300);

  // Serial.begin(9600);
  // delay(2000);

  // DEBUG_PRINTLN(F(xPortGetCoreID()));

  u8g2log.print("Start RTC Sync...");
  if (rtcAvailable)
    SyncWithRTC(); // start clock with RTC data
  u8g2log.print("Done\n");
  delay(300);

  u8g2log.print("Start PPS ISR...");
  attachInterrupt(PPS_PIN, isr, RISING); // enable GPS 1pps interrupt input - attachInterrupt(digitalPinToInterrupt(pin), ISR, mode) (recommended)
  u8g2log.print("Done\n");
  delay(300);

  u8g2log.print("Start WiFi ISR...");
  attachInterrupt(WIFI_BUTTON, btw, CHANGE);
  u8g2log.print("Done\n");
  delay(300);

  // Seed the CSRF token before processWifi() brings the HTTP server up: /json is what hands the
  // token to the dashboard, and a page served before this ran would carry one that never matches.
#if defined(ESP32)
  csrfToken = esp_random();
#elif defined(ARDUINO_ARCH_ESP8266)
  csrfToken = (uint32_t)RANDOM_REG32;
#endif
  if (csrfToken == 0)
    csrfToken = 0xA5A5A5A5UL; // never hand out an all-zero token

  u8g2log.print("Start WiFi process...");

  String statusWifiString = readData("/statusWifi"); // Restore last WiFi status
  if (statusWifiString == "0" || statusWifiString == "1")
  {
    statusWifi = atoi(statusWifiString.c_str());
  }
  // The AP passphrase is needed the moment an access point comes up and the admin password the
  // moment the HTTP server starts answering requests; initializeDeviceCredentials() has already
  // loaded both above. Re-reading them here would only add a way to blank them on a read failure.
  processWifi();
  u8g2log.print("Done\n");
  delay(300);

  u8g2log.print("Starting SYSLOG...");
  syslogserver = readData("/syslogserver"); // Password follows
  syslogserver.trim();

  // Syslog syslog(udpClient, syslogserver, SYSLOG_PORT, HOSTNAME, "esp-ntp", LOG_DAEMON);
  configureSyslogServer();
  syslog.deviceHostname(HOSTNAME);
  syslog.appName(HOSTNAME);
  syslog.defaultPriority(LOG_DAEMON);
  u8g2log.print("Done\n");
  delay(300);

  DEBUG_PRINTLN("Iniciado");

  // Startup UDP
  u8g2log.print("Start NTP/RFC868...");
  if (!Udp.begin(NTP_PORT))
  {
    u8g2log.print("NTP socket FAILED!\n");
    safeSyslog(LOG_ERR, "Unable to bind NTP socket");
  }
  RFC868server.begin();
  u8g2log.print("Done\n");
  delay(300);

#if defined(ARDUINO_ARCH_ESP8266)
#elif defined(ESP32)
          u8g2log.print("Starting tasks...");
          // xTaskCreatePinnedToCore(
          // Task1code, /* Function to implement the task */
          //"Task1", /* Name of the task */
          // 10000,  /* Stack size in words */
          // NULL,  /* Task input parameter */
          // 0,  /* Priority of the task */
          //&Task1,  /* Task handle. */
          // 0); /* Core where the task should run */
          xTaskCreate(FeedGpsParser, "FeedGpsParser", 4096, NULL, 1, NULL); // decode incoming GPS data
          xTaskCreate(SyncCheck, "SyncCheck", 4096, NULL, 3, NULL);         // synchronize to GPS or RTC - above loopTask so display/web work can't delay it
          xTaskCreate(processNTP, "processNTP", 4096, NULL, 3, NULL);       // above loopTask so display/web work can't delay NTP replies
          xTaskCreate(processRFC868, "processRFC868", 4096, NULL, 1, NULL);
          u8g2log.print("Done\n");
          delay(300);
#else
#error Unknown architecture
#endif

  // u8g2log.print("\n");
  u8g2log.print("All complete!");
  delay(300);
}

////////////////////////////////////////

void loop()
{
  uptimeSeconds(); // Keep the ESP8266 rollover extension current even without dashboard clients.
#if defined(ARDUINO_ARCH_ESP8266)
  FeedGpsParser(); // decode incoming GPS data
  SyncCheck();     // synchronize to GPS or RTC
  processNTP();
  processRFC868();
#elif defined(ESP32)
#else
#error Unknown architecture
#endif

  if (wifiReconfigPending)
  {
    wifiReconfigPending = false;
    processWifi();
  }

  UpdateDisplay(); // if time has changed, display it
  server.handleClient();

#if defined(ARDUINO_ARCH_ESP8266)
  if (millis() - pps_blink_time > PPS_BLINK_INTERVAL) // If x milliseconds passed, then it's time to switch led off for blink effect
#elif defined(ESP32)
          if (xTaskGetTickCount() - pps_blink_time > PPS_BLINK_INTERVAL) // If x milliseconds passed, then it's time to switch led off for blink effect
#else
#error Unknown architecture
#endif
    digitalWrite(PPS_LED, LOW);
  else
    digitalWrite(PPS_LED, HIGH);
  if (buttonPressed)
  { // Process keycheck of button presses outside ISR to avoid crash
    noInterrupts();
    uint32_t pressTick = keyPressTick;
    uint32_t releaseTick = keyReleaseTick;
    buttonPressed = false;
    interrupts();
    KeyCheck(pressTick, releaseTick);
  }
}
