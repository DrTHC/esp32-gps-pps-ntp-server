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