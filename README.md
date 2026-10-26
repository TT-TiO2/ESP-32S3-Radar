# 📡 ESP32-S3 ADS-B Aircraft Radar

![Build](https://img.shields.io/badge/build-passing-brightgreen) ![Platform](https://img.shields.io/badge/platform-ESP32--S3-blue) ![License](https://img.shields.io/badge/license-MIT-green) ![Version](https://img.shields.io/badge/version-v10-orange) ![APIs](https://img.shields.io/badge/APIs-free%20%26%20open-lightgrey)

---

## Description

A multi-screen embedded display built on the **CrowPanel 7.0" ESP32-S3** that turns a live ADS-B aircraft feed into an interactive, touch-driven radar — with a full weather station on the second screen.

Forked from [mircemk's original radar project](https://projecthub.arduino.cc/mircemk) on Arduino Project Hub, this build layers a complete custom UI on top of the base radar: tap any aircraft dot or sidebar row to pull up its flight route and model, filter by altitude, zoom the range, relocate the radar anywhere in the world, and switch to a weather screen that looks and feels like the iPhone Weather app — all running on a single embedded board with no API keys and no cloud backend.

The project is designed as an ongoing platform. Each version adds one focused feature so issues are easy to isolate and the sketch always has a known-good baseline to fall back to.

---

## Key Features

### Radar Screen
- **Live ADS-B feed** from adsb.fi, updating every few seconds, centered on any GPS coordinate you type in
- **Tap to select** any aircraft dot on the radar or any row in the sidebar list to open its detail panel
- **Flight detail panel** — model name, airline/operator, registration, origin → destination route with city names (from adsbdb.com), altitude, speed, heading, distance, bearing, and squawk code
- **Altitude filter** — toggle high / medium / low / ground bands with no extra API calls; the last active band cannot be switched off
- **Range zoom** — pick 10–100 km in 10 km steps; each change fetches a fresh aircraft pool at the matching API radius
- **Paged sidebar** — lists all aircraft sorted nearest-first, with page arrows and a PAGE x/y counter
- **Prev/next arrows** in the detail panel to step between aircraft without closing it
- **Plane types by shape** — filled circle (airliner), hollow ring (small plane), × (helicopter), diamond (military); dot color still encodes altitude
- **Emergency squawk banner** — flashing red strip when any plane squawks 7500, 7600, or 7700; tapping it selects the plane; emergency planes bypass the altitude filter
- **Airport markers** — violet ★ with the airport code for every airport in range, drawn under the planes (~70 US airports built in, including the full Philadelphia area)
- **Favorite places** — PLACES menu with 4 saved slots; tap to jump, or save/clear slots in place
- **Auto-dim** — backlight fades to 25% after the real local sunset and back up before sunrise (from Open-Meteo); any touch restores full brightness for 30 seconds
- **Custom location screen** — tap the compass to type any coordinates; the board looks up the place name (OpenStreetMap) and time zone (Open-Meteo), moves the radar and clock, and saves everything across reboots

### Weather Screen
- **Full-screen, separate screen** — no overlap with radar data; only fetches while open
- **Dark navy, iPhone-weather style** — smooth DejaVu anti-aliased fonts, gradient background, rounded cards, weather icons drawn from shapes
- **DAILY tab** — current temperature in F and C, conditions, feels like, wind with compass arrow, humidity bar, precipitation chance and amount, and today's high/low
- **Hourly strip on DAILY** — the next 12 hours of today (no tomorrow overlap): hour, icon, temperature, chance of rain, and a bar that turns bright blue when rain is measurable
- **WEEKLY tab** — 7-day list with icons, rain chance, F high/low with a color-coded range bar, and C range
- **Tap any day in WEEKLY** for a full 24-hour view: temperature line, per-hour rain bars, icons every 3 hours, a one-line rain summary, and a NOW marker on today
- **24-hour clock** in the top-right corner of the weather screen, in the saved location's time zone
- **All hourly times in 24-hour format** (16:00, not 4 PM)
- **15-minute refresh** with automatic retry and a visible stale-data warning

### System
- **Screen switcher** — top-left corner (no box, same style as the compass): tap to see the screen number, tap again to advance, switch after 3 seconds
- **Only the active screen fetches data** — no background polling; each screen starts fresh on entry
- **Remembers the last screen** across reboots
- **Auto-recovery** — WiFi reconnect, fetch watchdog, backoff on failures, LIVE / STALE / NO WIFI / NO API states
- **All settings saved to flash** — location, range, filter, favorite places, and last screen survive a reboot
- **No API keys required** — all data sources are free and open

---

## Version History

| Version | Date | What changed |
|---------|------|-------------|
| **v1** | Sep 15 | Baseline: live radar, touch-to-select, emergency squawk ring, auto-recovery |
| **v2** | Sep 19 | Route and model lookup from adsbdb.com; buffered network jobs |
| **v3** | Sep 23 | Paged sidebar list, prev/next arrows in the detail panel, buffered JSON download (fixes campus-network parse errors) |
| **v4** | Sep 27 | Filter and zoom menu opened from the RANGE AC box; settings saved across reboots |
| **v5** | Oct 1 | Smaller RANGE AC box so it doesn't clip the outer radar ring; amber tint replaces the "FILTERED" label |
| **v6** | Oct 4 | Custom location screen; place name and time zone lookup; daylight saving re-check every 6 hours |
| **v7** | Oct 8 | Multi-screen foundation: screen manager, top-left switcher, full-screen weather screen (DAILY + WEEKLY) |
| **v8** | Oct 10 | Hourly weather strip on DAILY; tap-a-day 24-hour view in WEEKLY |
| **v9** | Oct 12 | 24-hour clock on the weather screen; all hour labels in 24-hour format |
| **v10** | Oct 20 | Plane types by dot shape; squawk banner; airport markers; favorite places menu; auto-dim from real sunrise/sunset; remembers last screen |

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| **Microcontroller** | ESP32-S3 (CrowPanel 7.0", 800×480, OPI PSRAM) |
| **Display library** | LovyanGFX 1.2.33 |
| **Touch** | GT911 over I2C |
| **Language** | Arduino C++ (ESP32 core 3.3.12) |
| **JSON parsing** | ArduinoJson v6 |
| **ADS-B data** | [adsb.fi](https://adsb.fi) — free, no key |
| **Weather** | [Open-Meteo](https://open-meteo.com) — free, no key |
| **Route / model** | [adsbdb.com](https://adsbdb.com) — free, no key |
| **Location name** | [OpenStreetMap Nominatim](https://nominatim.openstreetmap.org) — free |
| **Storage** | ESP32 NVS (Preferences library) |

---

## Getting Started

### Prerequisites

- **Arduino IDE** 2.x (or PlatformIO)
- **ESP32 Arduino core** 3.3.12 — install via Boards Manager (`esp32` by Espressif)
- **LovyanGFX** 1.2.33 — install via Library Manager
- **ArduinoJson** v6 — install via Library Manager
- A **CrowPanel 7.0" ESP32-S3** board (800×480, GT911 touch, OPI PSRAM)

### Installation

```bash
# 1. Clone the repository
git clone https://github.com/your-username/ESP32-S3-Radar.git
cd ESP32-S3-Radar
```

Open `Radar_v10.ino` in the Arduino IDE.

Configure your credentials at the top of the sketch:

```cpp
// ── WiFi credentials ────────────────────────────────────────
const char* WIFI_SSID = "YOUR_NETWORK_NAME";
const char* WIFI_PASS = "YOUR_NETWORK_PASSWORD";
```

> **⚠️ Security note:** never commit real credentials to a public repo.  
> Add `secrets.h` to `.gitignore` and `#include` it instead, or use Arduino's `Preferences` to store credentials at runtime.

### Board Settings (Arduino IDE)

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| PSRAM | OPI PSRAM |
| Flash size | 16MB (or match your board) |
| Partition Scheme | Huge APP (3MB)
| Upload speed | 115200 |
| Port | your COM / tty port |

Select the correct port, then **Upload** (`Ctrl+U` / `Cmd+U`).

---

## Usage

Once flashed, the board starts on the radar screen automatically.

### Radar screen

| Action | Result |
|--------|--------|
| Tap any dot or sidebar row | Opens the flight detail panel |
| Tap RANGE AC box | Opens the RADAR MENU (filter, zoom, places) |
| Tap compass (bottom-left) | Opens the location entry screen |
| Tap top-left corner | Shows screen number; tap again to advance, wait 3 s to switch |
| Prev / Next arrows in detail | Steps between aircraft |
| Page arrows in sidebar header | Pages through the aircraft list |
| Tap squawk banner | Selects the emergency aircraft |

### Weather screen

| Action | Result |
|--------|--------|
| DAILY / WEEKLY tabs | Switch forecast views |
| Tap any day in WEEKLY | Opens that day's 24-hour detail |
| "Week" button or WEEKLY tab | Returns to the 7-day list |
| Top-left corner | Same switcher as the radar |

### Serial Monitor

Open at **115200 baud** to see boot info, fetch timing, JSON byte counts, and error messages.

```
Radar v10
MAC: AA:BB:CC:DD:EE:FF
WiFi connected: 192.168.1.42
Fetch ok: 38 planes (11024 bytes)
Weather ok: 14.2°C, 58% humidity, 7 days, 168 hourly
Screen -> 2
```

---

## Project Structure

```
ESP32-S3-Radar/
│
├── Radar_v10.ino          ← current stable sketch (all features)
│
├── archive/               ← previous versions kept for reference
│   ├── Radar_v1.ino
│   ├── Radar_v2.ino
│   ├── Radar_v3.ino
│   ├── Radar_v4.ino
│   ├── Radar_v5.ino
│   ├── Radar_v6.ino
│   ├── Radar_v7.ino
│   ├── Radar_v8.ino
│   └── Radar_v9.ino
│
├── README.md              ← this file
└── LICENSE
```

> Each version in `archive/` is a self-contained snapshot. The current sketch is always the highest-numbered `.ino` in the root.

---

## Roadmap

Features that are designed and queued, in rough priority order:

- [ ] **Storm radar sub-screen** — RainViewer tiles (free) rendered via PNGdec, opened from the weather screen's top-right corner
- [ ] **Weather alerts** — NWS ticker banner for tornado warnings, flood watches, etc. (US locations only)
- [ ] **Aircraft photo** — replace the lower detail panel with a photo of the aircraft type when one is available
- [ ] **Settings screen** — units (km/nm, ft/m, mph/kt), display preferences, filter presets
- [ ] **Terrain map** — elevation layer under the radar dots
- [ ] **Expanded airport list** — more regional airports, or a hosted dataset to avoid reflashing for updates

---

## Contributing

Contributions are welcome. To keep the version-by-version structure intact, please follow these conventions:

1. **Fork** the repo and create a branch: `git checkout -b feature/my-feature`
2. **Keep changes focused** — one feature or fix per PR, mirroring the single-version-single-change approach used throughout the project
3. **Test on hardware** before opening a PR; note in the PR description whether it was tested on a real CrowPanel or only compiled
4. **Update the version table** in `README.md` with a one-line summary of your change
5. Open a **Pull Request** with a clear description of what changed and why

For bugs, open an [issue](https://github.com/your-username/ESP32-S3-Radar/issues) with your board settings, Arduino core version, and the Serial Monitor output around the failure.

---

## Credits

| Source | Used for |
|--------|---------|
| [mircemk — Arduino Project Hub](https://projecthub.arduino.cc/mircemk) | Original radar base |
| [adsb.fi](https://adsb.fi) | Live ADS-B aircraft feed |
| [Open-Meteo](https://open-meteo.com) | Weather forecast and time zone |
| [adsbdb.com](https://adsbdb.com) | Flight route and aircraft model data (David Taylor & Jim Mason) |
| [OpenStreetMap Nominatim](https://nominatim.openstreetmap.org) | Location name lookup (© OpenStreetMap contributors, ODbL) |

---

## License

This project is licensed under the **MIT License** — see [`LICENSE`](LICENSE) for details.
