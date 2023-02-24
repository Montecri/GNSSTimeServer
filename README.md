# GPSTimeServer - Dual Display

![IMG_3050_MOV_AdobeExpress](https://user-images.githubusercontent.com/32185145/219906084-103e5c18-b03a-4c53-9235-e16533b12cdc.gif)

WiFi enabled GPS fed NTP server based on NodeMCU Amica and Arduino framework

The code is in two files, main.cpp in the src directory and definitions.h in the includes directory. 
It is higly recommended to use PlatformIO to compile it. That way all the libraries needed will be 
installed automatically.

As on Cristiano's original version, the first OLED display shows how many satellites 
are "in view" and the resolution of the position reported. It also shows the UTC time and date. The pushbutton
switch disables or enables WiFi connectivity. The yellow LED indicates WiFi is enabled.
The green LED indicates the GPS data is valid and the server's system time is in sync with it. The red 
LED pulses every second when GPS signals are present.

The second OLED display was added so each time an NTP request is received the server can show 
the client's IP address and when the response was sent. It also shows how many clients are connected. 
The maximum number of WiFi clients an ESP8266 can handle is eight. The second display doesn't have to be 
present. The code works without it, however if there are multiple clients it helps to verify they are all
connected and their NTP requests are being answered. 

The I2C address of the second OLED display has to be changed. For example with the recommended SSD1306 
displays this means changing it from the default 0x78 to 0x7A. This is done by relocating a resistor 
on its circuit board. It's a tiny surface-mount part so it requires some delicate desoldering and 
resoldering. There is a graphic on the board that shows where the resistor should be placed to choose 
the new address.

The U8g2 library the code uses works with many different OLED displays. 
You just need to use the constructor from the library and the I2C addresses that match the displays being used.
For example SH1106 modules could also be used. Constructors for SH1106 modules are included in the definitions.h file. 
To use them comment out the SSD1306 constructors and uncomment the SH1106 constructors. 
The I2C addresses specified in the code may also need to be changed to 0x3C and 0x3D. 

OLED displays wear out if left on for extended periods of time, especially if they continually show 
information that doesn't change much. So there is provision to turn them off when nobody is there to see them. 
As suggested by Brett Oliver, this can be done by connecting a PIR motion sensor to the A0 pin on the NodeMCU. 
A simple switch could also be used. Details are shown below.

To use this server, set your clock to connect to WiFi using the SSID and password specified in the 
definitions.h file. Then set the IP address for the time server it calls to 192.168.4.1. That is the 
address an ESP8266 uses when it's in server mode. The server will assign IP addresses to clients on 
its network starting with 192.168.4.2.

A custom enclosure was built for the server using walnut and acrylic. 

![front_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/IMG_2770.jpg)

![enclosure_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/IMG_2849.JPG)

---

Parts list:

- Amica NodeMCU (ESP8266 / ESP-12 E)
- DS3231 RTC
- Neo-6m V2 GPS
- 0.96" OLED Display (SSD1306 or similar. 
  If two are used be sure that it is possible to change the I2C address on one of them.)
- Hi-Link 5V/3W
- Mini-360 DC-DC Buck converter
- TP4056 Module (optional for portable use)
- 18650 battery holder (optional for portable use)
- HC-SR501 PIR motion sensor module (optional)
- Red, Green and Yellow LEDs
- Resistors (150, 100 and 150 Ohms respectivelly for above leds)
- Switch key and momentary push button

---

<p align="center"><img src="https://user-images.githubusercontent.com/38574378/132773469-08fb7b59-2f9d-4641-9665-c8d50d3904bc.png"><b>   ATTENTION   </b><img src="https://user-images.githubusercontent.com/38574378/132773469-08fb7b59-2f9d-4641-9665-c8d50d3904bc.png"></p> 

Several DS3231 modules being sold today contain a hazardous design flaw in which it supplies a voltage to the battery cradle regardless if it came with 
a rechargeable battery or not. If it came with a CR2032 battery (non-rechargeable) the consequence is that it will swell, explode, or worse. If it came 
with a LIR2032 battery (rechargeable), the module being fed with 5v will generate an unsafe charging voltage for that battery.

There are workarounds for that so you don't need to toss your module away, the most popular being removing a diode and/or resistor.

There's a long discussion on the thread below about root cause and possible fixes:

https://forum.arduino.cc/t/zs-042-ds3231-rtc-module/268862/33

---

Libraries:

-	paulstoffregen/Time@^1.6
-	makuna/RTC@^2.3.5
-	mikalhart/TinyGPS@0.0.0-alpha+sha.db4ef9c97a
-	olikraus/U8g2@^2.28.8

Source code based on:

- http://w8bh.net/avr/clock2.pdf
- https://forum.arduino.cc/t/ntp-time-server/192816

---

Wiring diagrams

This time server does not have to be portable so it is AC powered only. 

A PIR motion sensor can be connected to A0 on the NodeMCU to automatically turn the OLED displays on only when someone
is near to see them. Thanks to Brett Oliver who engineered the mod on his version of the project. Alternatively, an SPDT 
switch that connects A0 to either ground or +3.3 volts could be used. If it is not desired to turn the displays off, 
just connect A0 permanently to +3.3 volts or comment out the "if (PIRvalue < 500)" block of statements in the main.cpp file. 

![sketch_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/Schematic.png)

Breadboard layout

![sketch_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/sketch_bb.png)

Optional AC/DC power supply 

![power supply_bb-menor](https://user-images.githubusercontent.com/38574378/117375897-6a877b00-aea6-11eb-8022-d2b06e11bd37.png)

----

Here are some pictures of the inside of the project. This version doesn't have to be portable so it's AC powered and
the battery power supply wasn't implemented. It also uses a switch instead of a motion sensor to turn the displays on and off.

![inside_front_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/IMG_2767.JPG)

![inside_rear_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/IMG_2771.JPG)

---

CAD drawing for the enclosure

![CAD_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/Enclosure.PNG)

---

More information about the time server project can be found here:

https://www.linkedin.com/pulse/iot-maker-tale-stratum-1-time-server-built-from-scratch-monteiro/
