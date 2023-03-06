# SensESP-NMEA2000-Battery-Monitor
This is the Battery Monitor for Nautical Sun. 
This version has 2 battery monitors (voltage, current and temp) and an outside temperature sensor.  This device outputs in NMEA2000 and WiFi to a Signal K server.  the device is built on SensESP using an ESP32. 

Hardware side
what I bought was:

- [Firebeetle 2 ESP32-E](https://www.dfrobot.com/product-2231.html)

- [Buck converter](https://amzn.to/3Z54veM)

- [INA219 Current sensor boards](https://amzn.to/3EIlT1d)	

- [Waveshare CAN board](https://amzn.to/3Kud8ew)	

- [1-Wire Temperature sensors](https://amzn.to/3kkkZ3f)	

- [Ceramic Capacitors](https://amzn.to/3KjGs7s)	

- [Resistors](https://amzn.to/3IdH7VA)

- [500µohm Shunt](https://www.newark.com/vishay/wsms2908l5000jk/through-hole-current-sense-resistor/dp/27T3036?ost=wsms2908l5000jk)

I needed 2 Current Sensor Boards, so look carefully on the board you will see pads labelled A0 and A1 for battery A, I left the pads alone (default Address), for Battery B, I connected the A0 pad to the pad next to it (0x41)

See the video on (TBA)

The files should replace the files of the same name in the template. locations are as follows:

src/main.cpp

platformio.ini

Comprehensive documentation for SensESP, including how to get started with your own project, is available at the [SensESP documentation site](https://signalk.org/SensESP/).

See [Schematic.jpg](https://github.com/Techstyleuk/SensESP-NMEA2000-Battery-Monitor/blob/main/Nautical%20Sun%20schematic.jpg) for the schematic and [INA219_detail.jpg](https://github.com/Techstyleuk/SensESP_3_Battery_Monitor/blob/main/INA219_detail.jpg) for more info on the current board setup

Here is a picture of the shunt on a test battery, the shunt was much smaller (physicallY) than I was expecting and I really need bigger holes, so a different one may make its way to the production version:
![Shunt1](https://github.com/Techstyleuk/SensESP_3_Battery_Monitor/blob/main/Shunt1.JPG)
The resistors are for filtering (with the capacitor), they will be inside the box

this version has an additional CAN board which provides NMEA2000 messeages, remember to remove the 120ohm resistor if you are integrating with a NMEA2000 backbone
