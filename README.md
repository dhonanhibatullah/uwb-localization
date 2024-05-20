# **UWB Localization**
This project consists of app, firmwares, and the ideas behind my thesis work on UWB localization.

## **[1] app/firmware**
The firmware inside this directory can be compiled via Arduino IDE or Arduino CLI. There are several libraries need to be installed before compiling, such as:

1. [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
2. [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_GFX.h) 
3. [tzapu WiFiManager](https://github.com/tzapu/WiFiManager)
4. [knolleary pubsubclient](https://github.com/knolleary/pubsubclient)
5. [thotro arduino-dw1000](https://github.com/thotro/arduino-dw1000)
6. [Makerfabs DW3000](https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000) (already included)

Note that the firmware is developed with [FreeRTOS](https://www.freertos.org/index.html) for higher flexibility and asynchronous needs.

## **[2] app/ros**
The ROS2 workspace is used for simulating the scenario of anchor-tag localization. The ROS2 distro used in this project is [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) which is the Long-Term ROS2 support for Ubuntu 22.04.

## **[3] app/web**
The folder contains the web application for serving and interfacing the information received from the UWB devices which installed with the firmware. 

## **[4] docs/**
Read the documentations and demos of this project here!