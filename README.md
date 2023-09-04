# Stratum-1 Time Server Built From Scratch-GNSSTimeServer

<p align="center"><img src="https://github.com/Montecri/GPSTimeServer/assets/38574378/d716c6b1-41b5-4bbf-84f4-9e0a0178015a"></p> 

<p align="center"><img src="https://user-images.githubusercontent.com/38574378/117382664-69117f00-aeb5-11eb-818f-4dcee22dbfc9.gif"></p>


<p align="center"><a href="http://www.youtube.com/watch?feature=player_embedded&v=bdvNMfCw1Pw" target="_blank">
 <img src="http://img.youtube.com/vi/bdvNMfCw1Pw/mqdefault.jpg" alt="Watch the video" width="320" border="10" />
</a> - <a href="http://www.youtube.com/watch?feature=player_embedded&v=qk_Gvh3UzQg" target="_blank">
 <img src="http://img.youtube.com/vi/qk_Gvh3UzQg/mqdefault.jpg" alt="Watch the video" width="320" border="10" />
</a></p>

## WiFi enabled, GNSS fed, NTP/RDATE server based on ESP8266/ESP32 and Arduino

<br>

🎖️ Featured by [Hackaday](https://hackaday.com/2021/07/25/portable-gps-time-server-powered-by-the-esp8266/)

🎖️ Selected for [Maker Faire Rome 2023](https://makerfairerome.eu/en/)

<br>

<b>Functionalities:</b>

- Internal RTC synched from GNSS (Global Navigation Satellite System) satellites constellation (GPS, BeiDou, GLONASS and Galileo; backed by CR2032 battery)
- RTOS multi-tasking (when used with ESP32); core tasks (GNSS synching, Clock updating, processing NTP/RFC868 requests) running with a higher priority than common tasks (display update, PPS led blinking, web page serving)
- Rich dashboard with information about MCU, RTC, GNSS, WiFi (uptime, free heap, minimum heap, temperature, version, ip, mac address, network name, network type, signal strength, timestamp lat, long, lock status, sats, precision, etc)
- Highly optimized web page (minimalist single HTML with no external dependency, embedded graphics (as base64 encoded strings, including favicon), embedded script and layout, gzip-compressed - Python conversion/compression script supplied), requiring a password for critical functions. Only 32 Kb in size total, further reduced to ~20Kb through GZip compression. All modern browsers support gzipped pages, the compression takes place before firmware compiling, so no computing cycle is wasted by the device
  - Data separated from layout, once the page is loaded, data will be updated every 5 seconds from a lightweight JSON string, no full page reloads are needed
- Over-The-Air (OTA) Web Based firmware upgrade
- Dual voltage powered
- Backup battery for the whole unit (18650)
- NTP and RDATE (RFC868) protocols supported
- WiFi functionality as a client or access point
- Syslog logging
- Configuration webpage for WiFi and Syslog, saving to persistent storage (LittleFS)
- Informative display with a timestamp, satellites in view, accuracy, WiFi server, IP address
- Dual function button, short press switch information on display, long press disables WiFi
- Status at a glance, with dedicated 5mm LEDs for GPS lock, PPS signal, and WiFi status, as well as visible internal leds for charging and GPS module statuses
  - Yellow led will be on or off to indicate respective WiFi status
  - Green led will blink/pulsate at different rates to indicate a lock is being acquired and stay steady green when locked
  - Red led will blink once a second once lock is acquired, in sync with the PPS signal

> [!NOTE]
> Please note that some functionalities are available only on ESP32, due to either ESP8266 limitation or the functionality not being backported yet.

<b>Parts list:</b>

- Amica NodeMCU (ESP8266 / ESP-12) or ESP32Duino (ESP32-WROOM-32)
- DS3231 RTC
- Neo-6m V2 or ATGM336H GPS
- 1.3" SH1106 OLED Display
- For mains powered:
  - Hi-Link 5V/3W
  - Mini-360 DC-DC Buck converter
  - TP4056 Module
  - 18650 battery holder
  - 0.5A/230v fuse
  - 10D561K varistor
- For power adapter powered:
  - 18650 battery shield
  - 5.5mm x 2.1 mm or any other suitable socket
- Red, Green and Yellow LEDs
- Resistors (150, 100 and 150 Ohms respectively for above leds)
- Switch key and momentary push button

<b>New dashboard screenshot:</b>
<p align="center"><img src="https://github.com/Montecri/GNSSTimeServer/assets/38574378/c2b418d3-9c57-4498-8c56-a7258ae198d4" width=60%></p>

<b>And the new 3D printable case:</b>
<p align="center"><img src="https://github.com/Montecri/GNSSTimeServer/assets/38574378/15880404-a610-4608-9146-c4bb9b7f2ab2" width=80%></p>

<br/><br/>
<br/><br/>

<b>Check also the Dual Display PIR Enabled branch option:</b>
https://github.com/Montecri/GPSTimeServer/tree/Dual-Display

---
<p align="center"><img src="https://user-images.githubusercontent.com/38574378/132773469-08fb7b59-2f9d-4641-9665-c8d50d3904bc.png"><b>   ATTENTION   </b><img src="https://user-images.githubusercontent.com/38574378/132773469-08fb7b59-2f9d-4641-9665-c8d50d3904bc.png"></p> 

Several DS3231 modules being sold today contain a hazardous design flaw in which it supplies a voltage to the battery cradle regardless if it came with a rechargeable battery or not. If it came with a CR2032 battery (non-rechargeable) the consequence is that it will swell, explode, or worse. If it came with a LIR2032 battery (rechargeable), the module is fed with 5v will generate an unsafe charging voltage for that battery.

There are workarounds for that so you don't need to toss your module away, the most popular being removing a diode and/or resistor.

There's a long discussion on the thread below about the root cause and possible fixes:

https://forum.arduino.cc/t/zs-042-ds3231-rtc-module/268862/33

---

Libraries:

-	paulstoffregen/Time@^1.6
-	makuna/RTC@^2.3.5
-	mikalhart/TinyGPSPlus@^1.0.3
-	olikraus/U8g2@^2.28.8
- arcao/Syslog@^2.0.0

Source code based on:

- http://w8bh.net/avr/clock2.pdf
- https://forum.arduino.cc/t/ntp-time-server/192816

Contributions from:

@mmarkin
@sjthespian

![sketch_bb-menor](https://user-images.githubusercontent.com/38574378/117375890-66f3f400-aea6-11eb-9389-1b9b0b01f88f.png)
![power supply - UPDATED_bb-menor](https://user-images.githubusercontent.com/38574378/231014475-e1dc9185-a66d-4c0c-b681-dbc1ae9cf767.png)


https://www.linkedin.com/pulse/iot-maker-tale-stratum-1-time-server-built-from-scratch-monteiro/
