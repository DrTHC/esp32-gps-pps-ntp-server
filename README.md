# ESP32 GPS/PPS NTP Server

Arduino IDE firmware for an ESP32-based GPS/PPS-disciplined NTP server.

This project uses GPS UTC data and a PPS pulse to maintain an internal disciplined timebase, serve NTP on the local network, and provide a web dashboard for monitoring GPS, PPS, NTP, bridge, and system status.

---

## Features

- GPS UTC parsing using TinyGPSPlus
- PPS input discipline
- UDP NTP server on port `123`
- Web dashboard
- `/status` JSON endpoint
- ArduinoOTA support
- DHCP networking
- GPS TCP bridge on port `5000`
- Read-only bridge mode by default
- Optional GPS maintenance mode for u-center access
- Post-maintenance GPS/PPS revalidation
- Basic holdover behaviour when GPS UTC becomes stale but PPS remains fresh

---

## Hardware Used

This was built around an ESP32 and a GPS module with PPS output.

Default wiring:

| Function | ESP32 GPIO |
|---|---:|
| GPS RX | GPIO16 |
| GPS TX | GPIO17 |
| GPS PPS | GPIO27 |

> Check your own ESP32 board and GPS module pinout before wiring. Some GPS modules use different voltage levels or pin labels, because apparently standardisation was too much to ask.

---

## Setup

Edit these values before flashing:

```cpp
static const char WIFI_SSID[] = "YOUR_WIFI_SSID";
static const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";


## Please Note

This project is shared as a working prototype/reference build.

It is **not** a commercial product, not a finished package, and not something I am offering ongoing support for.

You are welcome to use it, fork it, modify it, improve it, or adapt it for your own GPS/PPS NTP setup. That is the point of sharing it.

However, please do not treat this repository as a bug-report-and-I-will-fix-it support channel. Hardware, GPS modules, antennas, Wi-Fi networks, timing behaviour, and ESP32 boards all vary, so your build may need changes.

Issues and pull requests are welcome if they help improve the project, but I cannot guarantee fixes, support, or compatibility with individual setups.

In short: this is a project you can build on, not a product with a helpdesk attached.