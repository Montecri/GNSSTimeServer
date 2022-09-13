# GPSTimeServer - Dual Display

![ezgif com-gif-maker0](https://user-images.githubusercontent.com/38574378/117382664-69117f00-aeb5-11eb-818f-4dcee22dbfc9.gif)

WiFi enabled GPS fed NTP server based on NodeMCU Amica and Arduino framework

The second OLED display was added to show the client's IP address and when
the response was sent each time the server receives an NTP request. It also shows 
how many clients are connected. The maximum number of WiFi clients an ESP8266 can handle is eight.
The second display doesn't have to be present. The code works without it. 

The I2C address of the second OLED display has to be changed from 0x78 to 0x7A. This is done 
by relocating a resistor on its circuit board. It's a tiny surface-mount part and this isn't 
easy to do. There should be a graphic on the board that shows where the resistor
should be placed to choose the new address.

Also use the appropriate U8g2 library constructor for your displays. It's in
the definitions.h file. SSD1306 displays don't work properly with the SH1106 
constructor. 

There is provision to turn the displays on and off. This can be done by
connecting a PIR motion sensor or a switch to the A0 pin on the NodeMCU. 

The code is in two files, main.cpp and definitions.h. 

A custom enclosure was built for the server using walnut and acrylic. 

![front_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/IMG_2780.jpg)

---

Parts list:

- Amica NodeMCU (ESP8266 / ESP-12)
- DS3231 RTC
- Neo-6m V2 GPS
- 0.96" OLED Display (x2)
- Hi-Link 5V/3W
- Mini-360 DC-DC Buck converter
- TP4056 Module (optional for portable use)
- 18650 battery holder (optional for portable use)
- PIR motion sensor module (optional)
- Red, Green and Yellow LEDs
- Resistors (150, 100 and 150 Ohms respectivelly for above leds)
- Switch key and momentary push button

---
<p align="center"><img src="https://user-images.githubusercontent.com/38574378/132773469-08fb7b59-2f9d-4641-9665-c8d50d3904bc.png"><b>   ATTENTION   </b><img src="https://user-images.githubusercontent.com/38574378/132773469-08fb7b59-2f9d-4641-9665-c8d50d3904bc.png"></p> 

Several DS3231 modules being sold today contain a hazardous design flaw in which it supplies a voltage to the battery cradle regardless if it came with a rechargeable battery or not. If it came with a CR2032 battery (non-rechargeable) the consequence is that it will swell, explode, or worse. If it came with a LIR2032 battery (rechargeable), the module being fed with 5v will generate an unsafe charging voltage for that battery.

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

![sketch_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/Dual%20Display.png)

AC/DC power supply

![power supply_bb-menor](https://user-images.githubusercontent.com/38574378/117375897-6a877b00-aea6-11eb-8022-d2b06e11bd37.png)

AC only power supply

![ac power supply_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/Power.png)

Motion sensor connections to turn the displays on an off.  You can also just connect a SPDT switch 
to supply either +3.3 volts or ground to A0. If you don't want to turn off the displays either connect A0 permanently to
+3.3 volts or comment out the setPowerSave code in the UpdateDisplay() function in the main.cpp file.

![PIR_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/PIR.png)

---

Here are some pictures of the inside of the project. This version is AC-powered. It doesn't have to be 
portable so the battery power supply wasn't implemented. It also uses a switch instead of a motion sensor to turn the displays on and off.

![inside_front_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/IMG_2767.JPG)

![inside_rear_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/IMG_2771.JPG)

![enclosure_bb-menor](https://github.com/Montecri/GPSTimeServer/blob/Dual-Display/images/IMG_2735.JPG)

---

More information about this project can be found here:

https://www.linkedin.com/pulse/iot-maker-tale-stratum-1-time-server-built-from-scratch-monteiro/
