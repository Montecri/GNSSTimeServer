/* WiFi enabled GPS NTP server - Cristiano Monteiro <cristianomonteiro@gmail.com> - 06.May.2021
   Based on the work of:
   Bruce E. Hall, W8BH <bhall66@gmail.com> - http://w8bh.net
   and
   https://forum.arduino.cc/u/ziggy2012/summary

   Mitch Markin, 02.Sep.2022:
   date bug from ziggy2012 code fixed,
   unused variables commented out to clear compiler warnings, 
   Brett Oliver's PIR code addded to turn OLED display on and off,
   got rid of string class in display functions,
   ICACHE_RAM_ATTR changed to IRAM_ADDR to clear compiler warnings,
   added second OLED display for client IP information
*/

#include "definitions.h"

void setup()
{
  Serial.begin(9600);
  delay(2000);
  Serial.println("\nGPS Time Server");
  Serial.println("Iniciado");  // Portugese for "Initiated" -MM

  pinMode(LOCK_LED, OUTPUT);
  pinMode(PPS_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(WIFI_BUTTON, INPUT_PULLUP);

  digitalWrite(LOCK_LED, LOW);
  digitalWrite(PPS_LED, LOW);
  digitalWrite(WIFI_LED, LOW);
  
  Wire.begin(D2, D1);    // SDA on ESP pin 4, SCL on ESP pin 5
  Rtc.Begin();
  RtcEeprom.Begin();

  InitOLED();

  ss.begin(9600);        // set GPS baud rate to 9600 bps
  
  // Initialize RTC
  while (!Rtc.GetIsRunning())
  {
    Rtc.SetIsRunning(true);
    DEBUG_PRINTLN(F("RTC had to be force started"));
  }

  DEBUG_PRINTLN(F("RTC started"));

  // Never assume the Rtc was last configured by you, 
  // so just clear them to your needed state
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);

  #ifdef DEBUG
    PrintRTCstatus(); // show RTC diagnostics
  #endif

  SyncWithRTC();                              // start clock with RTC data
  attachInterrupt(PPS_PIN, Isr, RISING);      // enable GPS 1pps interrupt input
  attachInterrupt(WIFI_BUTTON, Btw, FALLING);
  
  ProcessWifi();

  // Startup UDP
  Udp.begin(NTP_PORT);
}

// --------------------------------------------------------------------------------------------------
// MAIN PROGRAM

void loop()
{
  FeedGpsParser();                                    // decode incoming GPS data
  SyncCheck();                                        // synchronize to GPS or RTC
  server.handleClient();
  ProcessNTP();
  UpdateDisplay();                                    // if time has changed, display it

  if (millis() - pps_blink_time > PPS_BLINK_INTERVAL) // if x milliseconds passed, then it's time to switch LED off for blink effect
    digitalWrite(PPS_LED, LOW);

  if (KeyCheck()) // Malabarism to cover mechanical switch debouncing
    ProcessKeypress();  
}

// --------------------------------------------------------------------------------------------------
// Feed currently available data from GPS module into tinyGPS parser

void FeedGpsParser()
{
  while (ss.available()) // look for data from GPS module
  {
    char c = ss.read(); // read in all available chars
    gps.encode(c);      // and feed chars to GPS parser
    //Serial.write(c);    // uncomment for some extra debug info if in doubt about GPS feed
  }
}

// Button ISR debouncing routine, returns true if key pressed

bool KeyCheck()
{
  if (keytick != 0)
  {
    if ((millis() - keytick) > DEBOUNCE_TICKS)
    {
      DEBUG_PRINT(F("KEYTICK: "));
      DEBUG_PRINTLN(keytick);
      keytick = 0;
      DEBUG_PRINTLN(F("KEYCHECK IS TRUE"));
      return true;
    }
  }
  return false;
}

// --------------------------------------------------------------------------------------------------
// WiFi Routines

// Just a little test message. Go to http://192.168.4.1 in a web browser
// connected to this access point to see it

void HandleRoot()
{
  server.send(200, "text/html", "<h1>You are connected.</h1>");
}

void EnableWifi()
{
  // WiFi Initialization
  // You can remove the password parameter if you want the AP to be open

  WiFi.mode(WIFI_AP);
  //WiFi.softAP(ssid, password);            // default, maximum 4 clients
  WiFi.softAP(ssid, password, 1, 0, 8);   // maximum 8 clients -MM

  #ifdef DEBUG
    IPAddress myIP = WiFi.softAPIP();  
    Serial.print(F("AP IP address: "));
    Serial.println(myIP);
  #endif

  server.on("/", HandleRoot);
  server.begin();
  DEBUG_PRINTLN(F("HTTP server started"));
}

void DisableWifi()
{
  server.stop();
  DEBUG_PRINTLN(F("HTTP server stopped"));
  WiFi.softAPdisconnect(true);
  WiFi.enableAP(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  DEBUG_PRINTLN(F("WiFi disabled")); 
}

void ProcessWifi()
{
  // Toggle WiFi on/off and corresponding LED
  DEBUG_PRINT(F("Status Wifi: "));
  DEBUG_PRINTLN(statusWifi);

  if (statusWifi)
  {
    EnableWifi();
    digitalWrite(WIFI_LED, HIGH);
  }
  else
  {
    DisableWifi();
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
// Display time and date to serial monitor
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
//  such as the DS1307 or the DS3231. The module should be connected to the I2C pins (SDA/SCL).

void PrintRTCstatus()
// Send current RTC information to serial monitor
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
    DEBUG_PRINTLN("ERROR: cannot read the RTC.");
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
    DEBUG_PRINT("ERROR: cannot set RTC time");
}

// Use this routine to manually set the RTC to a specific UTC time.
// Since time is automatically set from GPS, this routine is mainly for
// debugging purposes.  Change numeric constants to the time desired.

void ManuallySetRTC()
{
  //  tmElements_t tm;
  //  tm.Year   = 2017 - 1970;      // year in unix years
  //  tm.Month  = 5;
  //  tm.Day    = 31;
  //  tm.Hour   = 5;
  //  tm.Minute = 59;
  //  tm.Second = 30;
  //  SetRTC(makeTime(tm));         // set RTC to desired time
}

// Keep the RTC time updated by setting it every (RTC_UPDATE_INTERVAL) seconds
// should only be called when system time is known to be good, such as in a GPS sync event

void UpdateRTC()
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
// OLED SPECIFIC ROUTINES
// These routines are used to display time and/or date information on the OLED display

void InitOLED()
{
  u8g2_1.setI2CAddress(DISPLAY_1_ADDR);
  u8g2_1.begin();
  //u8g2_1.setContrast(255);    // you can play with this to try to equalize brightness of both displays
                                // but it doesn't have much effect -MM
           
  u8g2_2.setI2CAddress(DISPLAY_2_ADDR);
  u8g2_2.begin();
  //u8g2_2.setContrast(255); 
}

void ShowDate(time_t t)
{
  int y = year(t);
  int m = month(t);
  int d = day(t);
  
  char buffer[11];
  snprintf(buffer, 11, (PGM_P)F("%.4i-%.2i-%.2i"), y, m, d);  

  u8g2_1.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2_1.drawGlyph(0, 41, 107);

  u8g2_1.setFont(u8g2_font_logisoso16_tf);   // choose a suitable font
  u8g2_1.drawStr(18, 41, buffer);

  DEBUG_PRINTLN("UpdateDisplay");
}

void ShowTime(time_t t)
{
  int h = hour(t);
  int m = minute(t);
  int s = second(t);  
  char buffer[13];
  
  snprintf(buffer, 13, (PGM_P)F("%.2i:%.2i:%.2i UTC"), h, m, s); 

  u8g2_1.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2_1.drawGlyph(0, 64, 123);

  u8g2_1.setFont(u8g2_font_logisoso16_tf);     
  u8g2_1.drawStr(18, 64, buffer); 
}

void ShowSatellites()
{
  uint8_t sats;
  if (gps.satellites() != 255) sats = gps.satellites();
  else sats = 0;

  uint32_t resol;
  if (gpsLocked) resol = gps.hdop();
  else resol = 0;

  u8g2_1.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2_1.drawGlyph(0, 18, 259);
  u8g2_1.drawGlyph(64, 18, 263);

  u8g2_1.setFont(u8g2_font_logisoso16_tf);    
  u8g2_1.setCursor(18, 18);
  u8g2_1.print(sats);
  u8g2_1.setCursor(82, 18);
  u8g2_1.print(resol);
}

void ShowFix()
{
  char buffer[16]; 
  uint8_t stationCount = wifi_softap_get_station_num();   

  u8g2_2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2_2.drawGlyph(0, 18, 208); 
  u8g2_2.drawGlyph(0, 64, 247);  

  u8g2_2.setFont(u8g2_font_logisoso16_tf);

  u8g2_2.setCursor(20, 18);
  u8g2_2.print(stationCount);

  u8g2_2.setCursor(0, 41);
  u8g2_2.print(clientIP);

  snprintf(buffer, 12, "%.2i:%.2i:%.2i", hour(lastFix), minute(lastFix), second(lastFix));  
  u8g2_2.drawStr(20, 64, buffer);                                 
}

void ShowDateTime(time_t t)
{
  ShowDate(t);
  ShowTime(t);
}

void ShowSyncFlag()
{
  if (gpsLocked) digitalWrite(LOCK_LED, HIGH);
  else digitalWrite(LOCK_LED, LOW);
}

//  Call this from the main loop
//  Updates display if time has changed

void UpdateDisplay()
{
  time_t t = now();         // get current time
  if (t != displayTime)     // has time changed?
  {
    displayTime = t;        // save current display value

    u8g2_1.clearBuffer();   // clear buffer contents   
    ShowDateTime(t);        // display the new UTC time
    ShowSatellites();       // display satellites in view and resolution
    ShowSyncFlag();         // show if display is in GPS sync
    u8g2_1.sendBuffer();    // send new information to display     
    
    u8g2_2.clearBuffer();   // clear buffer contents
    ShowFix();
    u8g2_2.sendBuffer();    // send new information to display 
    
    DEBUG_PRINTLN("Called PrintTime from UpdateDisplay");

    uint16_t PIRValue = analogRead(A0);

    if (PIRValue < 500)
    {
      u8g2_1.setPowerSave(1); 
      u8g2_2.setPowerSave(1);      
      DEBUG_PRINTLN(" Powersave activated, screen off");
      DEBUG_PRINTLN(PIRValue);
    }
    else if (PIRValue > 600)
    {
      u8g2_1.setPowerSave(0); 
      u8g2_2.setPowerSave(0);          
      DEBUG_PRINTLN(" Powersave deactivated, screen on");
      DEBUG_PRINTLN(PIRValue);
    }    

    #ifdef DEBUG
      PrintTime(t); // copy time to serial monitor
    #endif
  }
}

// --------------------------------------------------------------------------------------------------
// TIME SYNCHONIZATION ROUTINES
// These routines will synchonize time with GPS and/or RTC as necessary.
// Sync with GPS occurs when the 1pps interrupt signal from the GPS goes high.
// GPS synchonization events are attempted every (SYNC_INTERVAL) seconds.
// If a valid GPS signal is not received within (SYNC_TIMEOUT) seconds, the clock is synchonized
// with RTC instead.  The RTC time is updated with GPS data once every 24 hours.

void SyncWithGPS()
{
  int y;
  //byte h, m, s, mon, d, hundredths;    // hundredths is not used here
  byte h, m, s, mon, d;
  unsigned long age;
  gps.crack_datetime(&y, &mon, &d, &h, &m, &s, NULL, &age); // get time from GPS

  if (age < 1000 or age > 3000)        // dont use data older than 1 second
  {
    setTime(h, m, s, d, mon, y);       // copy GPS time to system time
    DEBUG_PRINT("Time from GPS: ");
    DEBUG_PRINT(h);
    DEBUG_PRINT(":");
    DEBUG_PRINT(m);
    DEBUG_PRINT(":");
    DEBUG_PRINTLN(s);
    adjustTime(1);                     // 1pps signal = start of next second
    syncTime = now();                  // remember time of this sync
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
  syncTime = now();                          // remember time of this sync event
  DEBUG_PRINTLN("Synchronized from RTC");    // send message to serial monitor
}

void SyncCheck()
// Manage synchonization of clock to GPS module
// First, check to see if it is time to synchonize
// Do time synchonization on the 1pps signal
// This call must be made frequently (keep in main loop)
{
  unsigned long timeSinceSync = now() - syncTime;    // how long has it been since last sync?
  if (pps && (timeSinceSync >= SYNC_INTERVAL))
  { // is it time to sync with GPS yet?
    DEBUG_PRINTLN("Called SyncWithGPS from SyncCheck");
    SyncWithGPS(); // yes, so attempt it.
  }
  pps = 0;                              // reset 1-pulse-per-second flag, regardless
  if (timeSinceSync >= SYNC_TIMEOUT)    // GPS sync has failed
  {
    gpsLocked = false; // flag that clock is no longer in GPS sync
    DEBUG_PRINTLN("Called SyncWithRTC from SyncCheck");
    SyncWithRTC(); // sync with RTC instead
  }
}

void IRAM_ATTR 
Isr()           // INTERRUPT SERVICE REQUEST
{
  pps = 1;                     // flag the 1pps input signal
  digitalWrite(PPS_LED, HIGH); // light up LED pps monitor
  pps_blink_time = millis();   // capture time in order to turn LED off so we can get the blink effect every x milliseconds - On loop
  DEBUG_PRINTLN("pps");
}

// Handle button pressed interrupt
void IRAM_ATTR Btw()           // INTERRUPT SERVICE REQUEST
{
  keytick = millis();
  DEBUG_PRINTLN(F("BUTTON PRESSED!"));
}

void ProcessKeypress()
{
  if (statusWifi)
    statusWifi = 0;
  else
    statusWifi = 1;

  ProcessWifi();
  DEBUG_PRINTLN(F("BUTTON CLICK PROCESSED!"));
}

// --------------------------------------------------------------------------------------------------
// NTP since 1900/01/01

const uint8_t daysInMonth[] PROGMEM = 
  { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };     // const or compiler complains, the array also has to be global

const uint32_t seventyYears = 2208988800UL;    // to convert Unix time to NTP 

// Mitch's function

uint32_t CalculateNTP(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mm, uint8_t s)
{
  uint16_t yearsSince1970 = y;                 // calculate the number of years from 1970 to the current year 
  if (y >= 1970) 
    yearsSince1970 = y - 1970;  

  uint16_t days = d - 1;                       // start with the number of elapsed days in the current month 
                                               // don't count the current day, its hours, minutes and seconds will be added later
  for (uint8_t i = 1; i < m; i++)              // add the days between January 1 and the start of the current month
    days += pgm_read_byte(daysInMonth + i - 1);

  if ((y % 4 == 0) && m > 2)                   // add a day if the current year is a leap year and the current month is > Feb. 
    days++;

  days += 365 * yearsSince1970;                // add the days before the current year  
  days += (yearsSince1970 + 1) / 4;            // add an extra day for each leap year before the current year

  uint32_t secs = days * 24L * 3600L;          // calculate the number of seconds in the total days 
  secs += (h * 3600L) + (mm * 60L) + s;        // now add the current day's hours, minutes, and seconds 
                                               // (this is a UNIX Epoch timestamp)

  secs += seventyYears;                        // add the number of seconds before 1970
  return secs;                                 // return the NTP Epoch timestamp 
} 

// ziggy's function with date bug fixed

uint32_t numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mm, uint8_t s)
{  
  if (y >= 1970) y -= 1970;

  uint16_t days = d - 1;

  for (uint8_t i = 1; i < m; ++i) days += pgm_read_byte(daysInMonth + i - 1);

  if (m > 2 && y % 4 == 0) ++days;

  days += 365 * y + (y + 3) / 4 - 1;
  return days * 24L * 3600L + h * 3600L + mm * 60L + s + seventyYears;
}

// --------------------------------------------------------------------------------------------------

void ProcessNTP()
{
  // If there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    clientIP = Udp.remoteIP();                 // store in global variable for OLED display
    int PortNum = Udp.remotePort();

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

      //byte LIVNMODE = packetBuffer[0];
      Serial.print("  LI, Vers, Mode :");
      Serial.print(packetBuffer[0], HEX);

      //byte STRATUM = packetBuffer[1];
      Serial.print("  Stratum :");
      Serial.print(packetBuffer[1], HEX);

      //byte POLLING = packetBuffer[2];
      Serial.print("  Polling :");
      Serial.print(packetBuffer[2], HEX);

      //byte PRECISION = packetBuffer[3];
      Serial.print("  Precision :");
      Serial.println(packetBuffer[3], HEX);

      for (int z = 0; z < NTP_PACKET_SIZE; z++)
      {
        Serial.print(packetBuffer[z], HEX);
        if (((z + 1) % 4) == 0)
        {
          Serial.println();
        }
      }
      Serial.println();
    #endif

    packetBuffer[0] = 0b00100100; // 00 (no leapsecond indicated), 100 (version 4), 100 (server mode)
    packetBuffer[1] = 4;          // stratum (MM - the original authors think it should be 4 or so, not 1)
    packetBuffer[2] = 6;          // polling minimum
    packetBuffer[3] = 0xFA;       // precision

    packetBuffer[7] = 0;          // root delay
    packetBuffer[8] = 0;
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;

    packetBuffer[11] = 0;         // root dispersion
    packetBuffer[12] = 0;
    packetBuffer[13] = 0xC;
    packetBuffer[14] = 0;

    uint32_t timestamp, tempval;
    time_t t = now();
    lastFix = t;    
    timestamp = CalculateNTP(year(t), month(t), day(t), hour(t), minute(t), second(t));
    

    #ifdef DEBUG
      Serial.print("Timestamp: ");
      Serial.print(hour(t));    Serial.print(":");
      Serial.print(minute(t));  Serial.print(":");
      Serial.print(second(t));  Serial.print("   ");
      Serial.println(timestamp);    
    #endif

    tempval = timestamp;

    packetBuffer[12] = 71; // "G";
    packetBuffer[13] = 80; // "P";
    packetBuffer[14] = 83; // "S";
    packetBuffer[15] = 0;  // "0";

    // Reference timestamp

    packetBuffer[16] = (tempval >> 24) & 0xFF;
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval) & 0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

    // Copy originate timestamp from incoming UDP transmit timestamp

    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    // Receive timestamp
    
    packetBuffer[32] = (tempval >> 24) & 0xFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval) & 0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    // Transmit timestamp

    packetBuffer[40] = (tempval >> 24) & 0xFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval) & 0xFF;

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

// --------------------------------------------------------------------------------------------------
//  END
