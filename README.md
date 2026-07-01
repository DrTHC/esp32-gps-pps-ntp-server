# ESP32 GPS PPS NTP Server

Arduino IDE firmware for an ESP32 GPS/PPS-disciplined NTP server.

## Features

- GPS UTC parsing with TinyGPSPlus
- PPS input discipline
- UDP NTP server on port 123
- Web dashboard and `/status`
- ArduinoOTA
- DHCP networking
- GPS TCP bridge on port 5000
- Read-only bridge mode by default

## Setup

Edit these values before flashing:


static const char WIFI_SSID[] = "YOUR_WIFI_SSID";
static const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

Default Pins: 

GPS RX: GPIO16
GPS TX: GPIO17
PPS: GPIO27

## Please Note

This project is shared as a working prototype/reference build, not as a polished commercial product or actively supported package.

You are welcome to use it, modify it, fork it, improve it, or adapt it for your own GPS/PPS NTP setup. Hardware, GPS modules, antennas, Wi-Fi conditions, and timing behaviour can vary, so some adjustment may be needed for your own build.

Issues and pull requests are welcome if they are useful, but please do not treat this repository as a guaranteed support channel or expect custom fixes for individual setups. This is provided as a project you can build on, not a product with a helpdesk attached.

If you improve the code, timing logic, dashboard, documentation, or hardware layout, contributions are appreciated.