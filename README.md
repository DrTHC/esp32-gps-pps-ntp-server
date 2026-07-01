# ESP32 GPS PPS NTP Server

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

Check your own ESP32 board and GPS module pinout before wiring. Some GPS modules use different voltage levels or pin labels, because apparently standardisation was too much to ask.

---

## Setup

Edit these values before flashing:

```cpp
static const char WIFI_SSID[] = "YOUR_WIFI_SSID";
static const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
```

Then flash the sketch using the Arduino IDE.

Required libraries:

- TinyGPSPlus
- ArduinoOTA
- ESP32 Arduino core

---

## Network Services

Once running, the ESP32 provides:

| Service | Default |
|---|---|
| Web dashboard | `http://<ESP32_IP>/` |
| JSON status | `http://<ESP32_IP>/status` |
| NTP server | UDP port `123` |
| GPS TCP bridge | TCP port `5000` |
| ArduinoOTA | Enabled after Wi-Fi connects |

The IP address is assigned by DHCP. I recommend creating a DHCP reservation on your router or firewall so the device keeps the same address.

---

## Web Dashboard

The dashboard shows:

- Current UTC display
- Sync state
- PPS status
- GPS UTC age
- GPS receiver telemetry
- NTP request count
- Last NTP client
- GPS TCP bridge status
- Maintenance/revalidation state
- Wi-Fi/system information
- Live logs

The dashboard clock is a visual display interpolated in the browser. NTP replies use the ESP32 internal disciplined timebase, not the browser clock.

---

## GPS TCP Bridge / u-center

The firmware includes a TCP bridge on port `5000` for viewing GPS receiver data in tools such as u-center.

Example:

```text
tcp://<ESP32_IP>:5000
```

By default, the bridge runs in **read-only mode**:

```text
GPS receiver -> ESP32 -> TCP client
```

In read-only mode, tools like u-center can observe GPS data, but commands sent from the TCP client are discarded.

Maintenance mode can be enabled from the dashboard when you deliberately want TCP client commands forwarded to the GPS receiver.

When maintenance mode is disabled, the firmware revalidates GPS/PPS timing before returning to normal discipline.

---

## Timing / Accuracy Notes

The ESP32 internal timebase is disciplined using GPS UTC and PPS.

For a basic Windows check, use:

```powershell
w32tm /stripchart /computer:<ESP32_IP> /period:1 /dataonly /samples:60
```

A longer test:

```powershell
w32tm /stripchart /computer:<ESP32_IP> /period:1 /dataonly /samples:300
```

Client-side results will vary depending on:

- Wi-Fi latency
- Windows scheduling
- network jitter
- NTP client behaviour
- signal quality
- GPS antenna placement

Do not treat the web dashboard clock as a precision measurement tool. It is there for monitoring and display.

---

## GPS Antenna Notes

Small ceramic GPS antennas may take several minutes to acquire satellites, especially indoors, inside enclosures, or with limited sky view.

GPS timing may become usable before full fix telemetry such as satellites, HDOP, and position appears.

For best results:

- place the antenna face-up
- give it the clearest possible sky view
- keep it away from ESP32 Wi-Fi antennas
- keep it away from noisy wiring and power cables
- avoid metal enclosures
- consider an external active GPS antenna for permanent outdoor installs

Plastic waterproof boxes are usually fine, but condensation, thick plastic, nearby metal, and poor sky view can all make acquisition slower.

---

## Important Safety / Network Notes

Do not expose this NTP server directly to the public internet.

This is intended for local network use.

If using OTA, make sure your network is trusted. OTA is convenient, which naturally means it can also become a tiny remote footgun if you expose it carelessly.

---

## Please Note

This project is shared as a working prototype/reference build.

It is **not** a commercial product, not a finished package, and not something I am offering ongoing support for.

You are welcome to use it, fork it, modify it, improve it, or adapt it for your own GPS/PPS NTP setup. That is the point of sharing it.

However, please do not treat this repository as a bug-report-and-I-will-fix-it support channel. Hardware, GPS modules, antennas, Wi-Fi networks, timing behaviour, and ESP32 boards all vary, so your build may need changes.

Issues and pull requests are welcome if they help improve the project, but I cannot guarantee fixes, support, or compatibility with individual setups.

In short: this is a project you can build on, not a product with a helpdesk attached.

---

