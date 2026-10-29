// Realtime Airplane Radar UI - STEP 4
// CrowPanel 7.0" ESP32-S3 800x480 RGB
// Real aircraft data + background API update + sweep reveal effect
// Credit. Orginial base was mircemk May, 2026
// Updated by (TT-Tio2) for Philadelphia / Boathouse Row

#define LGFX_USE_V1
// Radar v5
#define RADAR_VERSION "v5"

#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include "time.h"

// ---------- WIFI (put your own credentials here before uploading) ----------
const char* ssid = "SSID";
const char* password = "PASSWORD";

// (declared up here because the Arduino auto-prototype step needs the type before any function)
enum LinkState { LINK_LIVE, LINK_STALE, LINK_NO_WIFI, LINK_NO_API };

// ---------- CONNECTION HEALTH ----------
const unsigned long FETCH_INTERVAL_MS     = 6000;    // normal gap between API calls (adsb.fi allows 1/sec)
const unsigned long STALE_AFTER_MS        = 30000;   // no fresh data this long -> "STALE DATA"
const unsigned long DEAD_AFTER_MS         = 120000;  // no fresh data this long -> "NO API", planes cleared
const unsigned long FETCH_HARD_TIMEOUT_MS = 30000;   // kill a fetch task that hangs
const unsigned long WIFI_RETRY_MS         = 20000;   // gap between WiFi reconnect attempts
const unsigned long RESTART_AFTER_MS      = 600000;  // no good data for 10 min -> reboot

// ---------- TOUCH ----------
const unsigned long SELECTION_TIMEOUT_MS = 90000;  // detail panel goes back to the list after this long with no taps
// If taps land in the wrong place, flip these (watch the "TAP x,y" lines in Serial).
// Tapping the top-left corner should print roughly 0,0 and bottom-right roughly 799,479.
const bool TOUCH_SWAP_XY = false;
const bool TOUCH_FLIP_X  = false;
const bool TOUCH_FLIP_Y  = false;



// Center location: Boathouse Row, Philadelphia, PA (Drexel Rowing area)
const double HOME_LAT = 39.9721;  
const double HOME_LON = -75.1878; 

// Radar range: changed from the RANGE AC menu in 10 km steps, remembered across reboots
const int RANGE_MIN_KM  = 10;
const int RANGE_MAX_KM  = 100;
const int RANGE_STEP_KM = 10;
volatile int rangeKm = 100;                                   // current radar range
int apiRadiusNm(int km) { return (int)ceil(km / 1.852); }     // 100 km -> 54 nm, 10 km -> 6 nm

class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB   _bus;
  lgfx::Panel_RGB _panel;
  lgfx::Light_PWM _light;

  LGFX(void) {
    {
      auto cfg = _panel.config();
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width   = 800;
      cfg.panel_height  = 480;
      _panel.config(cfg);
    }

    {
      auto cfg = _bus.config();
      cfg.panel = &_panel;

      cfg.pin_d0  = GPIO_NUM_15;
      cfg.pin_d1  = GPIO_NUM_7;
      cfg.pin_d2  = GPIO_NUM_6;
      cfg.pin_d3  = GPIO_NUM_5;
      cfg.pin_d4  = GPIO_NUM_4;
      cfg.pin_d5  = GPIO_NUM_9;
      cfg.pin_d6  = GPIO_NUM_46;
      cfg.pin_d7  = GPIO_NUM_3;
      cfg.pin_d8  = GPIO_NUM_8;
      cfg.pin_d9  = GPIO_NUM_16;
      cfg.pin_d10 = GPIO_NUM_1;
      cfg.pin_d11 = GPIO_NUM_14;
      cfg.pin_d12 = GPIO_NUM_21;
      cfg.pin_d13 = GPIO_NUM_47;
      cfg.pin_d14 = GPIO_NUM_48;
      cfg.pin_d15 = GPIO_NUM_45;

      cfg.pin_henable = GPIO_NUM_41;
      cfg.pin_vsync   = GPIO_NUM_40;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;

      cfg.freq_write = 12000000;

      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch  = 40;
      cfg.vsync_front_porch = 1;
      cfg.vsync_pulse_width = 31;
      cfg.vsync_back_porch  = 13;

      cfg.pclk_active_neg = 1;
      cfg.de_idle_high    = 0;
      cfg.pclk_idle_high  = 0;

      _bus.config(cfg);
      _panel.setBus(&_bus);
    }

    {
      auto cfg = _light.config();
      cfg.pin_bl = GPIO_NUM_2;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }

    setPanel(&_panel);
  }
};

LGFX lcd;
LGFX_Sprite radarCanvas(&lcd);

// ---------- LAYOUT ----------
const int MARGIN = 5;

const int RADAR_X = MARGIN;
const int RADAR_Y = MARGIN;
const int RADAR_W = 470;
const int RADAR_H = 470;

const int INFO_X = 485;
const int INFO_Y = MARGIN;
const int INFO_W = 310;
const int INFO_H = 470;

// Detail panel (replaces the aircraft list when a plane is selected)
const int DETAIL_X = INFO_X + 10;
const int DETAIL_Y = INFO_Y + 10;
const int DETAIL_W = INFO_W - 20;
const int DETAIL_H = 285;
const int CLOSE_W  = 50;
const int CLOSE_H  = 40;
const int CLOSE_X  = DETAIL_X + DETAIL_W - CLOSE_W;
const int CLOSE_Y  = DETAIL_Y;

// Prev / next bar at the bottom of the detail panel
const int NAV_W      = 88;
const int NAV_H      = 32;
const int NAV_Y      = DETAIL_Y + 250;
const int NAV_PREV_X = DETAIL_X;
const int NAV_NEXT_X = DETAIL_X + DETAIL_W - NAV_W;

// Page arrows in the header of the aircraft list
const int PAGER_W       = 56;
const int PAGER_H       = 30;
const int PAGER_Y       = DETAIL_Y;
const int PAGER_LEFT_X  = DETAIL_X;
const int PAGER_RIGHT_X = DETAIL_X + DETAIL_W - PAGER_W;

// The RANGE AC box on the radar (tap it to open the menu)
const int RB_X1 = 388, RB_X2 = 472, RB_Y1 = 4, RB_Y2 = 92;

// Menu panels (they cover the same area as the detail panel)
const int BACK_X = DETAIL_X, BACK_Y = DETAIL_Y, BACK_W = 50, BACK_H = 40;
const int MENU_BTN_H     = 76;
const int MENU_FILTER_Y  = DETAIL_Y + 56;
const int MENU_ZOOM_Y    = DETAIL_Y + 144;
const int FILT_ROW_Y0    = DETAIL_Y + 50;
const int FILT_ROW_H     = 44;
const int FILT_ROW_STEP  = 52;
const int FILT_ALL_W     = 64;
const int FILT_ALL_H     = 26;
const int FILT_ALL_X     = DETAIL_X + DETAIL_W - FILT_ALL_W;
const int FILT_ALL_Y     = DETAIL_Y + 256;
const int ZOOM_GRID_Y    = DETAIL_Y + 100;
const int ZOOM_BTN_W     = 54;
const int ZOOM_BTN_H     = 46;
const int ZOOM_BTN_STEP_X = 59;
const int ZOOM_BTN_STEP_Y = 54;

const int CX = 240;
const int CY = 240;

const float X_CORR = 0.93;
const int R = 212;

const int OUTER_GAP = 3;
const int OUTER_RING_THICKNESS = 3;

// ---------- SWEEP ----------
float sweepAngle = 0;
unsigned long lastSweepUpdate = 0;

const int TRAIL_WIDTH = 60;
const int SWEEP_SPEED = 25;

unsigned long lastClockUpdate = 0;

// ---------- COLORS ----------
uint16_t greenBright;
uint16_t green;
uint16_t greenDim;
uint16_t greenDark;
uint16_t panelBg;

// ---------- AIRCRAFT ----------
struct AircraftPoint {
  bool valid;
  float distanceKm;
  float bearingDeg;
  int altitudeFt;
  float speedKt;
  float trackDeg;
  char flight[12];
  char category[5];
  char typeCode[8];
  char hex[8];        // ICAO hex id - how we track a plane between updates
  char reg[12];       // registration / tail number
  char squawk[5];
  int baroRate;       // ft/min
  int dbFlags;        // bit 0 = military
  bool onGround;
  bool vis;           // drawn on the radar this frame
  int16_t sx, sy;     // where it was drawn (for tap hit-testing)
  char desc[24];      // full model name from the feed (when the feed has one)
  char ownOp[24];     // operator from the feed (when the feed has one)
};

// Extra details looked up on demand (adsbdb.com) for the selected plane
struct FlightInfo {
  char hex[8];
  char call[12];
  uint8_t state;          // 0 = none, 1 = looking up, 2 = found, 3 = nothing found
  unsigned long when;
  char model[32];
  char airline[32];
  char orig[5];
  char dest[5];
  char origCity[24];
  char destCity[24];
};

const int MAX_PLANES = 50;

AircraftPoint planes[MAX_PLANES];
AircraftPoint pendingPlanes[MAX_PLANES];

int planeCount = 0;
int pendingPlaneCount = 0;

volatile bool fetchInProgress = false;
volatile bool pendingDataReady = false;
volatile int  consecutiveFails = 0;
volatile int  lastHttpCode = 0;
volatile unsigned long lastGoodFetchMs = 0;   // 0 = never
unsigned long lastFetchStartMs = 0;
unsigned long fetchStartMs = 0;
bool timeConfigured = false;

TaskHandle_t fetchTaskHandle = NULL;
SemaphoreHandle_t dataMutex = NULL;

// Allocated once and reused every fetch, so the heap doesn't fragment over days
DynamicJsonDocument jsonDoc(57344);

// ---------- TAP-TO-SELECT STATE ----------
const int LIST_ROWS = 10;
int  listIdx[LIST_ROWS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};  // plane shown on each sidebar row
int  listPage = 0;              // which page of the sidebar list is showing (0 = nearest 10)
int  sortedIdx[MAX_PLANES];     // all valid planes, nearest first (rebuilt before every use)
int  sortedCount = 0;
bool hasSelection = false;
char selectedHex[8] = "";
AircraftPoint selSnap;          // last known data for the selected plane
bool selLost = false;           // selected plane dropped out of the feed
unsigned long lastInteractMs = 0;

// ---------- MENU / FILTER / ZOOM STATE ----------
const int MENU_NONE = 0, MENU_MAIN = 1, MENU_FILTER = 2, MENU_ZOOM = 3;
int  menuMode = MENU_NONE;                      // which panel covers the top of the sidebar
// Altitude bands (same colors as the dots): 0 = 30,000 ft and up, 1 = 10,000-30,000, 2 = below 10,000, 3 = ground/unknown
bool bandOn[4] = {true, true, true, true};
volatile bool refetchNow = false;               // zoom changed: pull a fresh pool right away
bool rangeApplyWanted = false;                  // ...and show it as soon as it lands
volatile int pendingRangeKm = 100;              // range the waiting data was fetched for
Preferences prefs;

// (takes plain values, not an AircraftPoint: the Arduino IDE puts function prototypes
//  above the struct, so a struct in a function signature won't compile)
int altBand(int altitudeFt) {
  if (altitudeFt >= 30000) return 0;
  if (altitudeFt >= 10000) return 1;
  if (altitudeFt >= 0) return 2;
  return 3;
}

bool filterActive() {
  return !(bandOn[0] && bandOn[1] && bandOn[2] && bandOn[3]);
}

// A plane is drawn / listed only if it is inside the current range AND its altitude band is switched on.
// Filtering only hides planes from the pool we already have - it never asks the API for more.
bool planeShown(int i) {
  const AircraftPoint& p = planes[i];
  return p.valid && p.distanceKm <= rangeKm && bandOn[altBand(p.altitudeFt)];
}

// Planes in the pool that are inside the current range, before the altitude filter
int poolCount() {
  int n = 0;
  for (int i = 0; i < planeCount; i++) {
    if (planes[i].valid && planes[i].distanceKm <= rangeKm) n++;
  }
  return n;
}

uint8_t bandMask() {
  uint8_t m = 0;
  for (int i = 0; i < 4; i++) if (bandOn[i]) m |= (1 << i);
  return m;
}

void saveSettings() {
  prefs.begin("radar", false);
  prefs.putInt("range", rangeKm);
  prefs.putUChar("bands", bandMask());
  prefs.end();
}

void loadSettings() {
  prefs.begin("radar", true);
  int r = prefs.getInt("range", 100);
  uint8_t m = prefs.getUChar("bands", 0x0F);
  prefs.end();

  if (r < RANGE_MIN_KM || r > RANGE_MAX_KM || r % RANGE_STEP_KM != 0) r = 100;
  if ((m & 0x0F) == 0) m = 0x0F;                 // never start with everything hidden
  rangeKm = r;
  for (int i = 0; i < 4; i++) bandOn[i] = (m >> i) & 1;
}

// ---------- ROUTE / MODEL LOOKUP STATE (adsbdb.com) ----------
const int INFO_CACHE = 8;
FlightInfo infoCache[INFO_CACHE];
int  infoNext = 0;
int  pendingSlot = -1;                 // lookup waiting to start
int  activeSlot = -1;                  // lookup running now
volatile bool lookupInProgress = false;
volatile bool lookupDone = false;      // tells the main loop to redraw the panel
unsigned long lookupStartMs = 0;
unsigned long lookupBlockedUntil = 0;  // back off if adsbdb says slow down
TaskHandle_t lookupTaskHandle = NULL;
static StaticJsonDocument<4096> lookupDoc;

// ---------- HELPERS ----------
int rxCorr(int radius) {
  return (int)(radius * X_CORR);
}

double deg2radD(double deg) {
  return deg * M_PI / 180.0;
}

double rad2degD(double rad) {
  return rad * 180.0 / M_PI;
}

float angleDiffBehindSweep(float sweep, float targetBearing) {
  float d = sweep - targetBearing;
  while (d < 0) d += 360;
  while (d >= 360) d -= 360;
  return d;
}

float distanceKm(double lat1, double lon1, double lat2, double lon2) {
  const double Rearth = 6371.0;

  double dLat = deg2radD(lat2 - lat1);
  double dLon = deg2radD(lon2 - lon1);

  double a =
    sin(dLat / 2) * sin(dLat / 2) +
    cos(deg2radD(lat1)) * cos(deg2radD(lat2)) *
    sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
  return Rearth * c;
}

float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2radD(lon2 - lon1)) * cos(deg2radD(lat2));
  double x =
    cos(deg2radD(lat1)) * sin(deg2radD(lat2)) -
    sin(deg2radD(lat1)) * cos(deg2radD(lat2)) * cos(deg2radD(lon2 - lon1));

  double brng = rad2degD(atan2(y, x));
  if (brng < 0) brng += 360.0;
  return brng;
}

void canvasCorrectedCircle(int cx, int cy, int r, uint16_t col) {
  int lastX = cx + rxCorr(r);
  int lastY = cy;

  for (int a = 1; a <= 360; a++) {
    float rad = a * DEG_TO_RAD;
    int x = cx + cos(rad) * rxCorr(r);
    int y = cy + sin(rad) * r;

    radarCanvas.drawLine(lastX, lastY, x, y, col);

    lastX = x;
    lastY = y;
  }
}

void canvasCorrectedRadialLine(int angleDeg, int r1, int r2, uint16_t col) {
  float rad = (angleDeg - 90) * DEG_TO_RAD;

  int x1 = CX + cos(rad) * rxCorr(r1);
  int y1 = CY + sin(rad) * r1;
  int x2 = CX + cos(rad) * rxCorr(r2);
  int y2 = CY + sin(rad) * r2;

  radarCanvas.drawLine(x1, y1, x2, y2, col);
}

// ---------- API ----------
void copyStr(char* dst, size_t n, const char* src) {
  strncpy(dst, src, n - 1);
  dst[n - 1] = '\0';
}

void reportFailure(const char* why, int code = 0) {
  consecutiveFails = consecutiveFails + 1;
  lastHttpCode = code;
  Serial.printf("Fetch failed (%s) code=%d fails=%d heap=%u maxblock=%u wifi=%d\n",
                why, code, (int)consecutiveFails,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(),
                (int)WiFi.status());
}

bool fetchAircraftData() {
  if (WiFi.status() != WL_CONNECTED) {
    reportFailure("no wifi");
    return false;
  }

  int usedRange = rangeKm;   // the range this fetch is for (the menu may change it mid-fetch)

  String url = "https://opendata.adsb.fi/api/v2/lat/";
  url += String(HOME_LAT, 4);
  url += "/lon/";
  url += String(HOME_LON, 4);
  url += "/dist/";
  url += String(apiRadiusNm(usedRange));

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(10000);

  if (!http.begin(client, url)) {
    reportFailure("http begin");
    return false;
  }

  // Headers go AFTER begin()
  http.addHeader("User-Agent", "ESP32-AirRadar/1.0");
  http.addHeader("Accept", "application/json");
  http.useHTTP10(true);   // needed for stream parsing
  http.setTimeout(15000);
  int code = http.GET();
  Serial.printf("API HTTP Code: %d\n", code);

  if (code != 200) {
    if (code < 0) Serial.println(http.errorToString(code));
    http.end();
    reportFailure("http", code);
    return false;
  }

  // Only keep the fields we actually use
  StaticJsonDocument<1536> filter;
  const char* keys[] = {"ac", "aircraft"};
  for (const char* k : keys) {
    JsonObject f = filter[k].createNestedObject();
    f["lat"] = true;  f["lon"] = true;
    f["alt_baro"] = true; f["alt_geom"] = true;
    f["gs"] = true;   f["track"] = true;
    f["flight"] = true; f["category"] = true; f["t"] = true;
    f["hex"] = true;  f["r"] = true;  f["squawk"] = true;
    f["baro_rate"] = true; f["dbFlags"] = true;
    f["desc"] = true; f["ownOp"] = true;
  }

  // Download the whole body first, then parse from memory (a live TLS stream
  // can stall or close early on some networks -> "IncompleteInput")
  const size_t BODY_CAP = 120000;
  static char* body = nullptr;
  if (!body) {
    body = (char*)ps_malloc(BODY_CAP + 1);          // PSRAM
    if (!body) body = (char*)malloc(BODY_CAP + 1);  // fall back to heap
  }
  if (!body) {
    http.end();
    reportFailure("no body buffer");
    return false;
  }

  int expected = http.getSize();                    // -1 if chunked/unknown
  WiFiClient* stream = http.getStreamPtr();
  size_t got = 0;
  unsigned long lastData = millis();
  while ((http.connected() || stream->available()) && got < BODY_CAP &&
         millis() - lastData < 10000) {
    int avail = stream->available();
    if (avail > 0) {
      size_t want = min((size_t)avail, BODY_CAP - got);
      got += stream->readBytes(body + got, want);
      lastData = millis();
    } else {
      if (expected > 0 && got >= (size_t)expected) break;
      delay(5);
    }
  }
  body[got] = '\0';
  http.end();
  Serial.printf("Body: %u bytes (Content-Length %d)\n", (unsigned)got, expected);

  DeserializationError err =
      deserializeJson(jsonDoc, (const char*)body, got, DeserializationOption::Filter(filter));

  if (err) {
    Serial.print("JSON error: ");
    Serial.println(err.c_str());
    Serial.println(body + (got > 80 ? got - 80 : 0));   // last 80 chars
    reportFailure("json");
    return false;
  }

  JsonArray arr = jsonDoc["ac"].as<JsonArray>();
  if (arr.isNull()) {
    arr = jsonDoc["aircraft"].as<JsonArray>();
  }
  if (arr.isNull()) {
    reportFailure("no aircraft list");
    return false;
  }

  static AircraftPoint tempPlanes[MAX_PLANES];   // static: too big for the task stack now
  int tempCount = 0;

  for (JsonObject ac : arr) {
    if (tempCount >= MAX_PLANES) break;

    if (!ac["lat"].is<float>() || !ac["lon"].is<float>()) continue;

    double lat = ac["lat"];
    double lon = ac["lon"];

    float d = distanceKm(HOME_LAT, HOME_LON, lat, lon);
    if (d > usedRange) continue;

    AircraftPoint &p = tempPlanes[tempCount];

    p.valid = true;
    p.distanceKm = d;
    p.bearingDeg = bearingDeg(HOME_LAT, HOME_LON, lat, lon);

    if (ac["alt_baro"].is<int>()) p.altitudeFt = ac["alt_baro"];
    else if (ac["alt_geom"].is<int>()) p.altitudeFt = ac["alt_geom"];
    else p.altitudeFt = -1;

    p.speedKt = ac["gs"] | 0.0;
    p.trackDeg = ac["track"] | 0.0;

    const char* fl = ac["flight"] | "";
    strncpy(p.flight, fl, sizeof(p.flight) - 1);
    p.flight[sizeof(p.flight) - 1] = '\0';

    const char* cat = ac["category"] | "";
    strncpy(p.category, cat, sizeof(p.category) - 1);
    p.category[sizeof(p.category) - 1] = '\0';

    const char* typ = ac["t"] | "";
    strncpy(p.typeCode, typ, sizeof(p.typeCode) - 1);
    p.typeCode[sizeof(p.typeCode) - 1] = '\0';

    const char* hx = ac["hex"] | "";
    strncpy(p.hex, hx, sizeof(p.hex) - 1);
    p.hex[sizeof(p.hex) - 1] = '\0';

    const char* rg = ac["r"] | "";
    strncpy(p.reg, rg, sizeof(p.reg) - 1);
    p.reg[sizeof(p.reg) - 1] = '\0';

    const char* sq = ac["squawk"] | "";
    strncpy(p.squawk, sq, sizeof(p.squawk) - 1);
    p.squawk[sizeof(p.squawk) - 1] = '\0';

    p.baroRate = ac["baro_rate"] | 0;
    p.dbFlags  = ac["dbFlags"] | 0;
    p.onGround = ac["alt_baro"].is<const char*>();   // "ground" comes through as a string
    const char* ds = ac["desc"] | "";
    copyStr(p.desc, sizeof(p.desc), ds);

    const char* ow = ac["ownOp"] | "";
    copyStr(p.ownOp, sizeof(p.ownOp), ow);

    p.vis = false;
    p.sx = 0;
    p.sy = 0;

    tempCount++;
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  pendingPlaneCount = tempCount;
  for (int i = 0; i < tempCount; i++) {
    pendingPlanes[i] = tempPlanes[i];
  }
  pendingRangeKm = usedRange;
  pendingDataReady = true;
  xSemaphoreGive(dataMutex);

  consecutiveFails = 0;
  lastGoodFetchMs = millis();

  Serial.printf("Background aircraft found: %d (heap=%u, json=%u)\n",
                tempCount, (unsigned)ESP.getFreeHeap(), (unsigned)jsonDoc.memoryUsage());
  return true;
}

void aircraftFetchTask(void *parameter) {
  fetchAircraftData();
  fetchTaskHandle = NULL;
  fetchInProgress = false;
  vTaskDelete(NULL);
}

void startAircraftFetchBackground() {
  if (fetchInProgress) return;

  fetchInProgress = true;
  fetchStartMs = millis();
  lastFetchStartMs = fetchStartMs;

  BaseType_t ok = xTaskCreatePinnedToCore(
    aircraftFetchTask,
    "aircraftFetchTask",
    16384,
    NULL,
    1,
    &fetchTaskHandle,
    0
  );

  // If the heap is too fragmented to make the task, don't stay "in progress" forever
  if (ok != pdPASS) {
    fetchTaskHandle = NULL;
    fetchInProgress = false;
    reportFailure("task create");
  }
}

// ---------- ROUTE / MODEL LOOKUP (adsbdb.com) ----------
// Only runs when you tap a plane, results are cached, and it never overlaps an aircraft update.
int findInfo(const char* hex) {
  for (int i = 0; i < INFO_CACHE; i++) {
    if (infoCache[i].hex[0] && strcmp(infoCache[i].hex, hex) == 0) return i;
  }
  return -1;
}

int allocInfo() {
  for (int tries = 0; tries < INFO_CACHE; tries++) {
    infoNext = (infoNext + 1) % INFO_CACHE;
    if (infoNext != activeSlot && infoNext != pendingSlot) return infoNext;
  }
  return 0;
}

// Returns the HTTP code (200 = parsed into the slot)
int adsbdbGet(const String& url, int slot) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(10000);

  if (!http.begin(client, url)) return -1;

  http.addHeader("User-Agent", "ESP32-AirRadar/1.0");
  http.addHeader("Accept", "application/json");
  http.useHTTP10(true);

  int code = http.GET();
  if (code != 200) {
    http.end();
    return code;
  }

  // Only keep what we show
  StaticJsonDocument<768> filter;
  JsonObject r = filter["response"].to<JsonObject>();
  JsonObject a = r["aircraft"].to<JsonObject>();
  a["type"] = true;
  a["manufacturer"] = true;
  a["registered_owner"] = true;
  JsonObject f = r["flightroute"].to<JsonObject>();
  f["airline"]["name"] = true;
  f["origin"]["iata_code"] = true;
  f["origin"]["icao_code"] = true;
  f["origin"]["municipality"] = true;
  f["destination"]["iata_code"] = true;
  f["destination"]["icao_code"] = true;
  f["destination"]["municipality"] = true;

  DeserializationError err =
      deserializeJson(lookupDoc, http.getStream(), DeserializationOption::Filter(filter));
  http.end();

  if (err) {
    Serial.print("Lookup JSON error: ");
    Serial.println(err.c_str());
    return -2;
  }

  JsonObject ac = lookupDoc["response"]["aircraft"];
  JsonObject fr = lookupDoc["response"]["flightroute"];

  FlightInfo &fi = infoCache[slot];

  const char* man = ac["manufacturer"] | "";
  const char* typ = ac["type"] | "";
  String model = String(man) + " " + String(typ);
  model.trim();
  copyStr(fi.model, sizeof(fi.model), model.c_str());

  const char* al = fr["airline"]["name"] | "";
  if (!al[0]) al = ac["registered_owner"] | "";
  copyStr(fi.airline, sizeof(fi.airline), al);

  const char* oc = fr["origin"]["iata_code"] | "";
  if (!oc[0]) oc = fr["origin"]["icao_code"] | "";
  copyStr(fi.orig, sizeof(fi.orig), oc);
  const char* ocity = fr["origin"]["municipality"] | "";
  copyStr(fi.origCity, sizeof(fi.origCity), ocity);

  const char* dc = fr["destination"]["iata_code"] | "";
  if (!dc[0]) dc = fr["destination"]["icao_code"] | "";
  copyStr(fi.dest, sizeof(fi.dest), dc);
  const char* dcity = fr["destination"]["municipality"] | "";
  copyStr(fi.destCity, sizeof(fi.destCity), dcity);

  return 200;
}

void runLookup(int slot) {
  FlightInfo &fi = infoCache[slot];

  char hex[8];
  char call[12];
  copyStr(hex, sizeof(hex), fi.hex);
  copyStr(call, sizeof(call), fi.call);
  for (char* c = hex; *c; c++) *c = toupper(*c);

  fi.model[0] = '\0';
  fi.airline[0] = '\0';
  fi.orig[0] = '\0';
  fi.dest[0] = '\0';
  fi.origCity[0] = '\0';
  fi.destCity[0] = '\0';

  String url = String("https://api.adsbdb.com/v0/aircraft/") + hex;
  if (call[0]) url += String("?callsign=") + call;

  int code = adsbdbGet(url, slot);

  // Aircraft not in their database: try the flight route on its own
  if (code == 404 && call[0]) {
    url = String("https://api.adsbdb.com/v0/callsign/") + call;
    code = adsbdbGet(url, slot);
  }

  Serial.printf("Lookup %s %s -> %d\n", hex, call, code);

  if (code == 429) lookupBlockedUntil = millis() + 60000;

  bool found = (code == 200) && (fi.model[0] || fi.airline[0] || fi.orig[0] || fi.dest[0]);
  fi.state = found ? 2 : 3;
  fi.when = millis();
}

void lookupTask(void *parameter) {
  runLookup(activeSlot);
  lookupTaskHandle = NULL;
  lookupInProgress = false;
  lookupDone = true;
  vTaskDelete(NULL);
}

void maybeStartLookup() {
  if (pendingSlot < 0) return;
  if (lookupInProgress || fetchInProgress) return;   // one network job at a time
  if (WiFi.status() != WL_CONNECTED) return;

  if (millis() < lookupBlockedUntil) {               // adsbdb asked us to slow down
    infoCache[pendingSlot].state = 3;
    pendingSlot = -1;
    lookupDone = true;
    return;
  }

  activeSlot = pendingSlot;
  pendingSlot = -1;
  lookupInProgress = true;
  lookupStartMs = millis();

  BaseType_t ok = xTaskCreatePinnedToCore(lookupTask, "lookupTask", 16384, NULL, 1, &lookupTaskHandle, 0);
  if (ok != pdPASS) {
    lookupTaskHandle = NULL;
    lookupInProgress = false;
    infoCache[activeSlot].state = 3;
    lookupDone = true;
  }
}

void checkLookupWatchdog() {
  if (lookupInProgress && millis() - lookupStartMs > 25000) {
    Serial.println("Lookup task hung - killing it");
    if (lookupTaskHandle) vTaskDelete(lookupTaskHandle);
    lookupTaskHandle = NULL;
    lookupInProgress = false;
    infoCache[activeSlot].state = 3;
    lookupDone = true;
  }
}

// Ask for model/route details for the selected plane (cached for a while)
void requestLookup() {
  String call = String(selSnap.flight);
  call.trim();

  int i = findInfo(selSnap.hex);
  if (i >= 0) {
    if (infoCache[i].state == 1) return;               // already on its way
    unsigned long maxAge = (infoCache[i].state == 2) ? 1800000UL : 120000UL;
    bool sameFlight = strcmp(infoCache[i].call, call.c_str()) == 0;
    if (sameFlight && infoCache[i].state != 0 && millis() - infoCache[i].when < maxAge) return;   // cached
  } else {
    i = allocInfo();
  }

  if (pendingSlot >= 0 && pendingSlot != i) infoCache[pendingSlot].state = 0;   // drop an older waiting request

  copyStr(infoCache[i].hex, sizeof(infoCache[i].hex), selSnap.hex);
  copyStr(infoCache[i].call, sizeof(infoCache[i].call), call.c_str());
  infoCache[i].state = 1;
  infoCache[i].when = millis();
  pendingSlot = i;
}

bool isEmergencySquawk(const char* sq) {
  return strcmp(sq, "7500") == 0 || strcmp(sq, "7600") == 0 || strcmp(sq, "7700") == 0;
}

// After new data lands, find the selected plane again by its hex id
void refreshSelection() {
  if (!hasSelection) return;

  selLost = true;
  for (int i = 0; i < planeCount; i++) {
    if (strcmp(planes[i].hex, selectedHex) == 0) {
      selSnap = planes[i];
      selLost = false;
      break;
    }
  }
}

void applyPendingAircraftData() {
  if (!pendingDataReady) return;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  planeCount = pendingPlaneCount;
  for (int i = 0; i < planeCount; i++) {
    planes[i] = pendingPlanes[i];
  }
  pendingDataReady = false;
  xSemaphoreGive(dataMutex);

  refreshSelection();

  Serial.printf("Applied aircraft data: %d\n", planeCount);
}

// ---------- CONNECTION HEALTH ----------

LinkState getLinkState() {
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  unsigned long age = millis() - lastGoodFetchMs;

  if (lastGoodFetchMs != 0 && age <= STALE_AFTER_MS) return LINK_LIVE;
  if (!wifiUp) return LINK_NO_WIFI;
  if (lastGoodFetchMs != 0 && age <= DEAD_AFTER_MS) return LINK_STALE;
  return LINK_NO_API;
}

// Don't leave ghost planes on screen once the data is truly dead
void expireOldData() {
  if (lastGoodFetchMs != 0 && millis() - lastGoodFetchMs > DEAD_AFTER_MS && planeCount > 0) {
    planeCount = 0;
  }
}

void ensureWifi() {
  static unsigned long lastCheck = 0;
  static unsigned long lastAttempt = 0;

  if (millis() - lastCheck < 2000) return;
  lastCheck = millis();

  if (WiFi.status() == WL_CONNECTED) {
    if (!timeConfigured) {
      // US Eastern Time Zone rule for NTP time sync
      configTzTime("EST5EDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
      timeConfigured = true;
    }
    return;
  }

  if (millis() - lastAttempt < WIFI_RETRY_MS) return;
  lastAttempt = millis();

  Serial.println("WiFi down - reconnecting");
  WiFi.disconnect();
  WiFi.begin(ssid, password);
}

void maybeStartFetch() {
  if (fetchInProgress || lookupInProgress) return;   // one network job at a time
  if (WiFi.status() != WL_CONNECTED) return;

  // Zoom changed: don't wait for the normal gap (but stay under adsb.fi's 1 request/second)
  if (refetchNow && millis() - lastFetchStartMs >= 1500) {
    refetchNow = false;
    startAircraftFetchBackground();
    return;
  }

  unsigned long gap = FETCH_INTERVAL_MS;
  if (consecutiveFails > 0) {
    gap = FETCH_INTERVAL_MS * (unsigned long)(consecutiveFails + 1);   // back off when failing
    if (gap > 60000UL) gap = 60000UL;
  }

  if (millis() - lastFetchStartMs < gap) return;
  startAircraftFetchBackground();
}

void checkFetchWatchdog() {
  if (fetchInProgress && millis() - fetchStartMs > FETCH_HARD_TIMEOUT_MS) {
    Serial.println("Fetch task hung - killing it");
    if (fetchTaskHandle) vTaskDelete(fetchTaskHandle);
    fetchTaskHandle = NULL;
    fetchInProgress = false;
    reportFailure("hung");
  }
}

// Last resort: no good data for a long time -> reboot and start clean
void checkSafetyNet() {
  if (millis() - lastGoodFetchMs > RESTART_AFTER_MS) {
    Serial.println("No data for too long - restarting");
    delay(200);
    ESP.restart();
  }
}

// ---------- RADAR CANVAS DRAW ----------
void drawGridToCanvas() {
  radarCanvas.fillSprite(TFT_BLACK);

  for (int d = -240; d <= 240; d += 40) {
    int x = CX + rxCorr(d);
    if (x >= RADAR_X && x <= RADAR_X + RADAR_W) {
      radarCanvas.drawFastVLine(x, RADAR_Y, RADAR_H, greenDark);
    }
  }

  for (int d = -240; d <= 240; d += 40) {
    int y = CY + d;
    if (y >= RADAR_Y && y <= RADAR_Y + RADAR_H) {
      radarCanvas.drawFastHLine(RADAR_X, y, RADAR_W, greenDark);
    }
  }

  radarCanvas.drawRect(RADAR_X, RADAR_Y, RADAR_W, RADAR_H, green);
}

void drawSweepToCanvas() {
  for (float i = TRAIL_WIDTH; i >= 0; i -= 0.5) {
    float a = sweepAngle - i;
    if (a < 0) a += 360;

    float intensity = (float)(TRAIL_WIDTH - i) / TRAIL_WIDTH;

    uint8_t g = 6 + intensity * 120;
    uint16_t col = radarCanvas.color565(0, g, 0);

    float rad = (a - 90) * DEG_TO_RAD;

    int x = CX + cos(rad) * rxCorr(R);
    int y = CY + sin(rad) * R;

    radarCanvas.drawLine(CX, CY, x, y, col);
  }

  float rad = (sweepAngle - 90) * DEG_TO_RAD;
  int sx = CX + cos(rad) * rxCorr(R);
  int sy = CY + sin(rad) * R;

  radarCanvas.drawLine(CX, CY, sx, sy, greenBright);
  radarCanvas.drawLine(CX + 1, CY, sx + 1, sy, greenBright);
}

void drawRadarCirclesToCanvas() {
  canvasCorrectedCircle(CX, CY, 35, green);
  canvasCorrectedCircle(CX, CY, 70, green);
  canvasCorrectedCircle(CX, CY, 110, green);
  canvasCorrectedCircle(CX, CY, 150, green);
  canvasCorrectedCircle(CX, CY, 190, green);
  canvasCorrectedCircle(CX, CY, R, greenBright);

  for (int i = 0; i < OUTER_RING_THICKNESS; i++) {
    canvasCorrectedCircle(CX, CY, R + OUTER_GAP + 1 + i, greenBright);
  }
}

void drawCrossLinesToCanvas() {
  radarCanvas.drawFastHLine(CX - rxCorr(R), CY, rxCorr(R) * 2, greenBright);
  radarCanvas.drawFastVLine(CX, CY - R, R * 2, greenBright);

  for (int d = 20; d < R; d += 40) {
    radarCanvas.drawFastVLine(CX + rxCorr(d), CY - 7, 14, greenBright);
    radarCanvas.drawFastVLine(CX - rxCorr(d), CY - 7, 14, greenBright);

    radarCanvas.drawFastHLine(CX - 7, CY + d, 14, greenBright);
    radarCanvas.drawFastHLine(CX - 7, CY - d, 14, greenBright);
  }

  for (int a = 30; a < 360; a += 30) {
    if (a == 90 || a == 180 || a == 270) continue;
    canvasCorrectedRadialLine(a, 0, R, greenDim);
  }
}

void drawDistanceLabelsToCanvas() {
  radarCanvas.setTextDatum(middle_center);
  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(greenBright, TFT_BLACK);

  int radii[] = {35, 70, 110, 150, 190};
  int km[5];
  for (int i = 0; i < 5; i++) km[i] = rangeKm * (i + 1) / 5;   // rings = fifths of the current range

  const int LABEL_OFFSET = 9;

  for (int i = 0; i < 5; i++) {
    radarCanvas.drawString(String(km[i]) + "km", CX + 18, CY - radii[i] - LABEL_OFFSET);
    radarCanvas.drawString(String(km[i]) + "km", CX + 18, CY + radii[i] + LABEL_OFFSET);
  }

  radarCanvas.drawString("0", CX + 15, CY + 12);
}

void drawAngleScaleToCanvas() {
  for (int a = 0; a < 360; a++) {
    int len = 4;
    if (a % 10 == 0) len = 13;
    else if (a % 5 == 0) len = 8;

    canvasCorrectedRadialLine(a, R - len, R, greenBright);
  }

  radarCanvas.setTextColor(greenBright, TFT_BLACK);
  radarCanvas.setTextSize(2);
  radarCanvas.setTextDatum(middle_center);

  for (int a = 0; a < 360; a += 30) {
    float rad = (a - 90) * DEG_TO_RAD;

    int tx = CX + cos(rad) * rxCorr(R - 28);
    int ty = CY + sin(rad) * (R - 28);

    radarCanvas.drawNumber(a, tx, ty);
  }
}

void drawAircraftTargetsToCanvas() {
  for (int i = 0; i < planeCount; i++) {
    planes[i].vis = false;
    if (!planeShown(i)) continue;      // outside the range or filtered out

    bool isSel = hasSelection && strcmp(planes[i].hex, selectedHex) == 0;

    float ageAngle = angleDiffBehindSweep(sweepAngle, planes[i].bearingDeg);

    if (ageAngle > 260 && !isSel) continue;   // selected plane stays visible

    float fade = 1.0;
    if (ageAngle > 200) {
      fade = 1.0 - ((ageAngle - 200.0) / 60.0);
      if (fade < 0.20) fade = 0.20;
    }

    float rr = (planes[i].distanceKm / (float)rangeKm) * R;
    float rad = (planes[i].bearingDeg - 90) * DEG_TO_RAD;

    int x = CX + cos(rad) * rxCorr(rr);
    int y = CY + sin(rad) * rr;

    planes[i].sx = x;
    planes[i].sy = y;
    planes[i].vis = true;

    uint8_t r = 0;
    uint8_t g = 255 * fade;
    uint8_t b = 35 * fade;

    if (planes[i].altitudeFt >= 30000) {
      r = 0; g = 180 * fade; b = 255 * fade;
    } else if (planes[i].altitudeFt >= 10000) {
      r = 0; g = 255 * fade; b = 35 * fade;
    } else if (planes[i].altitudeFt >= 0) {
      r = 255 * fade; g = 220 * fade; b = 0;
    } else {
      r = 255 * fade; g = 40 * fade; b = 40 * fade;
    }

    uint16_t col = radarCanvas.color565(r, g, b);

    int size = 4;
    if (planes[i].altitudeFt > 30000) size = 5;
    if (planes[i].distanceKm < rangeKm * 0.2f) size = 6;   // nearest fifth of the range

    radarCanvas.fillCircle(x, y, size, col);
    radarCanvas.drawCircle(x, y, size + 2, greenDim);

    if (isSel) {
      radarCanvas.drawCircle(x, y, size + 6, TFT_WHITE);
      radarCanvas.drawCircle(x, y, size + 7, TFT_WHITE);
    }

    if (isEmergencySquawk(planes[i].squawk) && ((millis() / 400) & 1)) {
      uint16_t alarm = radarCanvas.color565(255, 40, 40);
      radarCanvas.drawCircle(x, y, size + 10, alarm);
      radarCanvas.drawCircle(x, y, size + 11, alarm);
    }

    if (planes[i].trackDeg >= 0) {
      float tr = (planes[i].trackDeg - 90) * DEG_TO_RAD;
      int x2 = x + cos(tr) * 12;
      int y2 = y + sin(tr) * 12;
      radarCanvas.drawLine(x, y, x2, y2, col);
    }

    if (isSel || (ageAngle >= 0 && ageAngle < 35)) {
      radarCanvas.setTextDatum(top_left);
      radarCanvas.setTextSize(1);
      radarCanvas.setTextColor(TFT_WHITE, TFT_BLACK);

      String label = String(planes[i].flight);
      label.trim();
      if (label.length() == 0) label = "UNKNOWN";

      radarCanvas.drawString(label, x + 9, y - 12);
    }
  }

  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(2);

  LinkState ls = getLinkState();
  uint16_t amber = radarCanvas.color565(255, 200, 0);
  uint16_t red   = radarCanvas.color565(255, 80, 80);

  if (ls == LINK_LIVE || ls == LINK_STALE) {
    radarCanvas.setTextColor(ls == LINK_LIVE ? green : amber, TFT_BLACK);
    radarCanvas.drawString(ls == LINK_LIVE ? "REAL ADS-B" : "STALE DATA", 12, 10);

    radarCanvas.setTextColor(greenBright, TFT_BLACK);
    radarCanvas.drawString("AC: " + String(planeCount), 12, 32);
  } else {
    radarCanvas.setTextColor(red, TFT_BLACK);
    radarCanvas.drawString(ls == LINK_NO_WIFI ? "NO WIFI" : "NO API", 14, 14);

    radarCanvas.setTextSize(1);
    radarCanvas.drawString("RETRYING... (" + String((int)consecutiveFails) + ")", 14, 36);
    radarCanvas.setTextSize(2);
  }

  if (fetchInProgress) {
    radarCanvas.setTextSize(2);
    radarCanvas.setTextColor(greenDim, TFT_BLACK);
    radarCanvas.drawString("UPDATING", 10, 54);
  }

  int ccx = 50;
  int ccy = 427;
  uint16_t compassBright = greenBright;
  uint16_t compassDim = greenDim;
  uint16_t compassDark = greenDark;

  radarCanvas.setTextDatum(middle_center);

  radarCanvas.drawCircle(ccx, ccy, 17, compassDim);
  radarCanvas.drawCircle(ccx, ccy, 18, compassDark);

  radarCanvas.fillTriangle(ccx, ccy - 16, ccx - 5, ccy - 4, ccx + 5, ccy - 4, compassBright);
  radarCanvas.drawTriangle(ccx, ccy - 16, ccx - 5, ccy - 4, ccx + 5, ccy - 4, TFT_WHITE);
  radarCanvas.fillTriangle(ccx, ccy + 16, ccx - 5, ccy + 4, ccx + 5, ccy + 4, compassDim);
  radarCanvas.fillTriangle(ccx - 16, ccy, ccx - 4, ccy - 5, ccx - 4, ccy + 5, compassDim);
  radarCanvas.fillTriangle(ccx + 16, ccy, ccx + 4, ccy - 5, ccx + 4, ccy + 5, compassDim);

  radarCanvas.fillCircle(ccx, ccy, 2, compassBright);
  radarCanvas.drawCircle(ccx, ccy, 4, compassDim);

  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(compassBright, TFT_BLACK);
  radarCanvas.drawString("N", ccx, ccy - 29);

  radarCanvas.setTextColor(compassDim, TFT_BLACK);
  radarCanvas.drawString("S", ccx, ccy + 29);
  radarCanvas.drawString("W", ccx - 29, ccy - 3);
  radarCanvas.drawString("E", ccx + 29, ccy - 3);
}

void drawRangeBarsToCanvas() {
  const int bins = 5;
  int counts[bins] = {0};
  float binKm = rangeKm / 5.0f;                // each bar = a fifth of the current range

  for (int i = 0; i < planeCount; i++) {
    if (!planeShown(i)) continue;

    int bin = (int)(planes[i].distanceKm / binKm);
    if (bin < 0) bin = 0;
    if (bin >= bins) bin = bins - 1;

    counts[bin]++;
  }

  const int startX = 395;
  const int baseY  = 62;
  const int barW   = 10;
  const int barGap = 14;

  const int tickH = 3;
  const int tickGap = 2;
  const int maxTicks = 8;

  // Faint outline: this box is a button (tap it for the filter / zoom menu).
  // Kept small so it stays clear of the outer radar ring; it turns amber while a filter is on.
  uint16_t boxCol = filterActive() ? radarCanvas.color565(255, 200, 0) : greenDark;
  radarCanvas.drawRect(392, 8, 77, 78, boxCol);

  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(filterActive() ? boxCol : green, TFT_BLACK);
  radarCanvas.drawString("RANGE AC", startX, 12);

  for (int b = 0; b < bins; b++) {
    int x = startX + b * barGap;
    int c = counts[b];
    if (c > maxTicks) c = maxTicks;

    for (int t = 0; t < c; t++) {
      int y = baseY - t * (tickH + tickGap);
      radarCanvas.fillRect(x, y, barW, tickH, greenBright);
    }

    radarCanvas.drawFastHLine(x, baseY + 7, barW, greenDark);
  }

  radarCanvas.setTextColor(green, TFT_BLACK);
  radarCanvas.drawString(String(rangeKm / 5),     startX - 1,              baseY + 14);
  radarCanvas.drawString(String(rangeKm * 3 / 5), startX + 2 * barGap - 1, baseY + 14);
  radarCanvas.drawString(String((int)rangeKm),    startX + 4 * barGap - (rangeKm >= 100 ? 4 : 1), baseY + 14);
}

void drawSweepHeadingIndicatorToCanvas() {
  int hx = 430;
  int hy = 420;

  int angleInt = (int)sweepAngle;
  if (angleInt < 0) angleInt += 360;
  if (angleInt >= 360) angleInt -= 360;

  radarCanvas.setTextDatum(middle_center);

  radarCanvas.drawCircle(hx, hy, 28, greenDark);
  radarCanvas.drawCircle(hx, hy, 29, greenDim);

  float a = (sweepAngle - 90) * DEG_TO_RAD;

  int tipX = hx + cos(a) * 23;
  int tipY = hy + sin(a) * 23;

  float leftA  = a + 2.45;
  float rightA = a - 2.45;

  int leftX = hx + cos(leftA) * 10;
  int leftY = hy + sin(leftA) * 10;
  int rightX = hx + cos(rightA) * 10;
  int rightY = hy + sin(rightA) * 10;

  radarCanvas.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, greenBright);
  radarCanvas.drawLine(hx, hy, tipX, tipY, TFT_WHITE);
  radarCanvas.fillCircle(hx, hy, 3, greenBright);

  radarCanvas.setTextSize(2);
  radarCanvas.setTextColor(greenBright, TFT_BLACK);

  char buf[8];
  sprintf(buf, "%03d", angleInt);
  radarCanvas.drawString(buf, hx, hy + 42);

  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(green, TFT_BLACK);
  radarCanvas.drawString("SWEEP", hx, hy - 38);
}

void drawRadarFrameToCanvas() {
  drawGridToCanvas();
  drawSweepToCanvas();
  drawRadarCirclesToCanvas();
  drawCrossLinesToCanvas();
  drawDistanceLabelsToCanvas();
  drawAngleScaleToCanvas();
  drawAircraftTargetsToCanvas();
  drawRangeBarsToCanvas();
  drawSweepHeadingIndicatorToCanvas();
}

void drawTimePanel() {
  int boxX = INFO_X + 18;
  int boxY = INFO_Y + 392;
  int boxW = INFO_W - 36;
  int boxH = 68;

  lcd.fillRect(boxX, boxY, boxW, boxH, TFT_BLACK);
  lcd.drawFastHLine(INFO_X + 10, boxY - 17, INFO_W - 20, greenDark);

  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("TIME:", boxX + 110, boxY - 7);

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeStr[12];
    sprintf(timeStr, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    lcd.setTextSize(5);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString(timeStr, boxX + 20, boxY + 22);
  } else {
    lcd.setTextSize(4);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString("--:--:--", boxX + 20, boxY + 22);
  }
}

void drawLocationPanel() {
  int boxX = INFO_X + 18;
  int boxY = INFO_Y + 305;   
  int boxW = INFO_W - 36;
  int boxH = 70;

  lcd.fillRect(boxX, boxY, boxW, boxH, TFT_BLACK);
  lcd.drawFastHLine(INFO_X + 10, boxY - 10, INFO_W - 20, greenDark);

  // Updated location label for Philadelphia / Boathouse Row area
  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("PHILLY", boxX + boxW / 2, boxY);

  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

  String coord = String(HOME_LAT, 4) + " / " + String(HOME_LON, 4);
  lcd.drawString(coord, boxX + boxW / 2, boxY + 32);
}

// All valid planes sorted nearest-first (insertion sort, at most MAX_PLANES)
void rebuildSortedList() {
  sortedCount = 0;
  for (int i = 0; i < planeCount && i < MAX_PLANES; i++) {
    if (!planeShown(i)) continue;      // inside the range and passing the altitude filter
    int j = sortedCount++;
    while (j > 0 && planes[sortedIdx[j - 1]].distanceKm > planes[i].distanceKm) {
      sortedIdx[j] = sortedIdx[j - 1];
      j--;
    }
    sortedIdx[j] = i;
  }
}

int listPageCount() {
  int pages = (sortedCount + LIST_ROWS - 1) / LIST_ROWS;
  return pages < 1 ? 1 : pages;
}

// Position of a plane (by hex id) in the sorted list, or -1 if it isn't in the feed
int findSortedPos(const char* hex) {
  if (!hex || !hex[0]) return -1;
  for (int k = 0; k < sortedCount; k++) {
    if (strcmp(planes[sortedIdx[k]].hex, hex) == 0) return k;
  }
  return -1;
}

void drawArrowButton(int x, int y, int w, int h, bool left) {
  lcd.fillRect(x, y, w, h, TFT_BLACK);
  lcd.drawRect(x, y, w, h, green);
  int cx = x + w / 2;
  int cy = y + h / 2;
  if (left) lcd.fillTriangle(cx + 7, cy - 9, cx + 7, cy + 9, cx - 8, cy, greenBright);
  else      lcd.fillTriangle(cx - 7, cy - 9, cx - 7, cy + 9, cx + 8, cy, greenBright);
}

void drawDetectedAircraftPanel() {
  int topY = INFO_Y + 10;       
  int bottomY = INFO_Y + 295;   
  int boxX = INFO_X + 10;
  int boxW = INFO_W - 20;
  int boxH = bottomY - topY;

  lcd.fillRect(boxX, topY, boxW, boxH, TFT_BLACK);

  rebuildSortedList();
  int pages = listPageCount();
  if (listPage >= pages) listPage = pages - 1;
  if (listPage < 0) listPage = 0;

  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(filterActive() ? lcd.color565(255, 200, 0) : greenBright, TFT_BLACK);   // amber = filter on
  if (pages > 1) {
    drawArrowButton(PAGER_LEFT_X,  PAGER_Y, PAGER_W, PAGER_H, true);
    drawArrowButton(PAGER_RIGHT_X, PAGER_Y, PAGER_W, PAGER_H, false);
    lcd.drawString("PAGE " + String(listPage + 1) + "/" + String(pages),
                   INFO_X + INFO_W / 2, topY + 7);
  } else {
    lcd.drawString("DETECTED AIRCRAFTS", INFO_X + INFO_W / 2, topY + 2);
  }

  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);

  for (int row = 0; row < LIST_ROWS; row++) {
    int slot = listPage * LIST_ROWS + row;
    listIdx[row] = (slot < sortedCount) ? sortedIdx[slot] : -1;
  }

  int y = topY + 34;

  for (int row = 0; row < LIST_ROWS; row++) {
    if (listIdx[row] == -1) break;

    AircraftPoint &p = planes[listIdx[row]];

    String call = String(p.flight);
    call.trim();
    if (call.length() == 0) call = "UNKNOWN";

    uint16_t dotCol = greenBright;
    if (p.altitudeFt >= 30000) dotCol = lcd.color565(0, 180, 255);
    else if (p.altitudeFt >= 10000) dotCol = lcd.color565(0, 255, 35);
    else if (p.altitudeFt >= 0) dotCol = lcd.color565(255, 220, 0);
    else dotCol = lcd.color565(255, 40, 40);

    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString(call.substring(0, 7), boxX + 2, y);

    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.drawString(String((int)p.distanceKm) + "km", boxX + 76, y);

    if (p.altitudeFt >= 0) lcd.drawString(String(p.altitudeFt) + "ft", boxX + 118, y);
    else lcd.drawString("---ft", boxX + 118, y);

    lcd.drawString(String((int)p.speedKt) + "kt", boxX + 188, y);

    lcd.setTextColor(green, TFT_BLACK);
    
    String typ = String(p.typeCode);
    typ.trim();
    if (typ.length() == 0) typ = String(p.category);

    lcd.drawString(typ.substring(0, 5), boxX + 235, y);

    lcd.fillCircle(boxX + 272, y + 5, 4, dotCol);
    lcd.drawFastHLine(boxX, y + 17, boxW, greenDark);

    y += 26;
  }

  if (sortedCount == 0) {
    lcd.setTextDatum(top_center);
    lcd.setTextSize(1);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString(poolCount() == 0 ? "NO AIRCRAFT IN RANGE" : "NO AIRCRAFT MATCH THE FILTER",
                   INFO_X + INFO_W / 2, topY + 52);
  }
}

void drawDetailPanel() {
  AircraftPoint &p = selSnap;
  int x0 = DETAIL_X;
  int y0 = DETAIL_Y;
  int w  = DETAIL_W;

  uint16_t red   = lcd.color565(255, 80, 80);
  uint16_t amber = lcd.color565(255, 200, 0);

  // what adsbdb told us about this plane (if we asked yet)
  int ii = findInfo(p.hex);
  int lstate = (ii >= 0) ? infoCache[ii].state : 0;

  lcd.fillRect(x0, y0, w, DETAIL_H, TFT_BLACK);

  // close button
  lcd.drawRect(CLOSE_X, CLOSE_Y, CLOSE_W, CLOSE_H, green);
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("X", CLOSE_X + CLOSE_W / 2, CLOSE_Y + CLOSE_H / 2);

  // callsign
  String call = String(p.flight);
  call.trim();
  if (call.length() == 0) { call = String(p.hex); call.toUpperCase(); }
  if (call.length() == 0) call = "UNKNOWN";

  lcd.setTextDatum(top_left);
  lcd.setTextSize(3);
  lcd.setTextColor(selLost ? red : TFT_YELLOW, TFT_BLACK);
  lcd.drawString(call.substring(0, 8), x0 + 2, y0 + 8);

  // model: adsbdb > feed description > type code
  String model = "";
  if (lstate == 2 && infoCache[ii].model[0]) model = String(infoCache[ii].model);
  else if (p.desc[0]) model = String(p.desc);
  else {
    model = String(p.typeCode);
    model.trim();
    if (model.length() == 0) model = String(p.category);
  }

  lcd.setTextSize(2);
  lcd.setTextColor(green, TFT_BLACK);
  lcd.drawString(model.substring(0, 23), x0 + 2, y0 + 40);

  // airline / operator + registration
  String op = "";
  if (lstate == 2 && infoCache[ii].airline[0]) op = String(infoCache[ii].airline);
  else if (p.ownOp[0]) op = String(p.ownOp);
  String reg = String(p.reg);
  reg.trim();
  String who = op;
  if (reg.length()) who += (op.length() ? "  |  " : "") + reg;

  lcd.setTextSize(1);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString(who.substring(0, 42), x0 + 2, y0 + 62);

  if (p.dbFlags & 1) {
    lcd.setTextColor(amber, TFT_BLACK);
    lcd.drawString("MIL", x0 + w - 24, y0 + 62);
  }

  // route
  bool haveRoute = (lstate == 2 && (infoCache[ii].orig[0] || infoCache[ii].dest[0]));
  if (haveRoute) {
    String o = infoCache[ii].orig[0] ? String(infoCache[ii].orig) : String("???");
    String d = infoCache[ii].dest[0] ? String(infoCache[ii].dest) : String("???");

    lcd.setTextSize(3);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.drawString(o + " > " + d, x0 + 2, y0 + 76);

    String oc = String(infoCache[ii].origCity);
    String dc = String(infoCache[ii].destCity);
    lcd.setTextSize(1);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString((oc + " > " + dc).substring(0, 46), x0 + 2, y0 + 104);
  } else {
    lcd.setTextSize(2);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString(lstate == 1 ? "LOOKING UP ROUTE..." : "ROUTE UNAVAILABLE", x0 + 2, y0 + 82);
  }

  lcd.drawFastHLine(x0, y0 + 120, w, greenDark);

  // data rows
  auto row = [&](int r, const char* label, const String& val, uint16_t col) {
    int ry = y0 + 128 + r * 21;
    lcd.setTextSize(2);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString(label, x0 + 2, ry);
    lcd.setTextColor(col, TFT_BLACK);
    lcd.drawString(val, x0 + 78, ry);
  };

  uint16_t altCol = TFT_WHITE;
  String alt;
  if (p.onGround) alt = "GROUND";
  else if (p.altitudeFt >= 0) alt = String(p.altitudeFt) + " ft";
  else alt = "---";
  if (p.altitudeFt >= 30000) altCol = lcd.color565(0, 180, 255);
  else if (p.altitudeFt >= 10000) altCol = lcd.color565(0, 255, 35);
  else if (p.altitudeFt >= 0) altCol = amber;

  String vs = String(p.baroRate > 0 ? "+" : "") + String(p.baroRate) + " fpm";

  row(0, "ALT",  alt, altCol);
  row(1, "SPD",  String((int)p.speedKt) + " kt", TFT_WHITE);
  row(2, "HDG",  String((int)p.trackDeg) + " deg", TFT_WHITE);
  row(3, "V/S",  vs, TFT_WHITE);
  row(4, "DIST", String(p.distanceKm, 1) + " km @ " + String((int)p.bearingDeg), TFT_WHITE);

  if (isEmergencySquawk(p.squawk)) {
    const char* what = strcmp(p.squawk, "7500") == 0 ? "HIJACK" :
                       strcmp(p.squawk, "7600") == 0 ? "NO RADIO" : "EMERGENCY";
    row(5, "SQK", String(p.squawk) + " " + what, red);
  } else {
    row(5, "SQK", p.squawk[0] ? String(p.squawk) : String("----"), TFT_WHITE);
  }

  // prev / next bar: arrows, position in the nearest-first list, status
  rebuildSortedList();
  int pos = findSortedPos(p.hex);
  drawArrowButton(NAV_PREV_X, NAV_Y, NAV_W, NAV_H, true);
  drawArrowButton(NAV_NEXT_X, NAV_Y, NAV_W, NAV_H, false);

  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  String posTxt = (pos >= 0 ? String(pos + 1) : String("-")) + "/" + String(sortedCount);
  lcd.drawString(posTxt, x0 + w / 2, NAV_Y + 1);

  lcd.setTextSize(1);
  if (selLost) {
    lcd.setTextColor(red, TFT_BLACK);
    lcd.drawString("SIGNAL LOST", x0 + w / 2, NAV_Y + 20);
  } else {
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString("TAP X TO CLOSE", x0 + w / 2, NAV_Y + 20);
  }
  lcd.setTextDatum(top_left);
}

// ---------- MENU PANELS (RANGE AC box -> FILTER / ZOOM) ----------
void drawCloseX() {
  lcd.drawRect(CLOSE_X, CLOSE_Y, CLOSE_W, CLOSE_H, green);
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("X", CLOSE_X + CLOSE_W / 2, CLOSE_Y + CLOSE_H / 2);
}

String filterSummary() {
  if (!filterActive()) return "ALL ALTITUDES";
  const char* n[4] = {"HIGH", "MID", "LOW", "UNK"};
  String out = "";
  for (int i = 0; i < 4; i++) {
    if (!bandOn[i]) continue;
    if (out.length()) out += "+";
    out += n[i];
  }
  return out;
}

void drawMenuButton(int x, int y, int w, int h, const char* label, const String& sub) {
  lcd.fillRect(x, y, w, h, TFT_BLACK);
  lcd.drawRect(x, y, w, h, green);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(3);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString(label, x + 14, y + 12);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(sub, x + 14, y + 46);
  int cx = x + w - 22;
  int cy = y + h / 2;
  lcd.fillTriangle(cx - 6, cy - 10, cx - 6, cy + 10, cx + 7, cy, greenBright);   // ">" = opens a page
}

void drawMenuPanel() {
  int x0 = DETAIL_X, y0 = DETAIL_Y, w = DETAIL_W;
  lcd.fillRect(x0, y0, w, DETAIL_H, TFT_BLACK);
  drawCloseX();

  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("RADAR MENU", x0 + 2, y0 + 12);

  drawMenuButton(x0, MENU_FILTER_Y, w, MENU_BTN_H, "FILTER", filterSummary());
  drawMenuButton(x0, MENU_ZOOM_Y,   w, MENU_BTN_H, "ZOOM",   String((int)rangeKm) + " KM RANGE");

  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString("TAP X OR EMPTY RADAR TO CLOSE", x0 + 2, y0 + 262);
}

void drawFilterPanel() {
  int x0 = DETAIL_X, y0 = DETAIL_Y, w = DETAIL_W;
  lcd.fillRect(x0, y0, w, DETAIL_H, TFT_BLACK);
  drawCloseX();
  drawArrowButton(BACK_X, BACK_Y, BACK_W, BACK_H, true);

  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("ALTITUDE", x0 + 62, y0 + 12);

  const char* names[4] = {"ABOVE 30,000 FT", "10,000-30,000 FT", "BELOW 10,000 FT", "GROUND / UNKNOWN"};
  uint16_t cols[4] = {lcd.color565(0, 180, 255), lcd.color565(0, 255, 35),
                      lcd.color565(255, 220, 0), lcd.color565(255, 40, 40)};

  for (int i = 0; i < 4; i++) {
    int ry = FILT_ROW_Y0 + i * FILT_ROW_STEP;
    lcd.drawRect(x0, ry, w, FILT_ROW_H, bandOn[i] ? green : greenDark);

    lcd.drawRect(x0 + 8, ry + 8, 28, 28, cols[i]);              // checkbox in the band's dot color
    if (bandOn[i]) lcd.fillRect(x0 + 12, ry + 12, 20, 20, cols[i]);

    lcd.setTextColor(bandOn[i] ? TFT_WHITE : greenDim, TFT_BLACK);
    lcd.drawString(names[i], x0 + 46, ry + 14);
  }

  rebuildSortedList();
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("SHOWING " + String(sortedCount) + "/" + String(poolCount()), x0 + 2, FILT_ALL_Y + 5);

  lcd.drawRect(FILT_ALL_X, FILT_ALL_Y, FILT_ALL_W, FILT_ALL_H, filterActive() ? green : greenDark);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(filterActive() ? greenBright : greenDim, TFT_BLACK);
  lcd.drawString("ALL", FILT_ALL_X + FILT_ALL_W / 2, FILT_ALL_Y + FILT_ALL_H / 2);
  lcd.setTextDatum(top_left);
}

void drawZoomPanel() {
  int x0 = DETAIL_X, y0 = DETAIL_Y, w = DETAIL_W;
  lcd.fillRect(x0, y0, w, DETAIL_H, TFT_BLACK);
  drawCloseX();
  drawArrowButton(BACK_X, BACK_Y, BACK_W, BACK_H, true);

  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("ZOOM", x0 + 62, y0 + 12);

  lcd.setTextDatum(top_center);
  lcd.setTextSize(4);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString(String((int)rangeKm) + " KM", x0 + w / 2, y0 + 52);

  for (int i = 0; i < 10; i++) {
    int km = (i + 1) * RANGE_STEP_KM;
    int bx = x0 + (i % 5) * ZOOM_BTN_STEP_X;
    int by = ZOOM_GRID_Y + (i / 5) * ZOOM_BTN_STEP_Y;
    bool cur = (km == rangeKm);

    if (cur) lcd.fillRect(bx, by, ZOOM_BTN_W, ZOOM_BTN_H, greenDark);
    lcd.drawRect(bx, by, ZOOM_BTN_W, ZOOM_BTN_H, cur ? greenBright : green);
    lcd.setTextDatum(middle_center);
    lcd.setTextSize(2);
    lcd.setTextColor(cur ? greenBright : TFT_WHITE, cur ? greenDark : TFT_BLACK);
    lcd.drawString(String(km), bx + ZOOM_BTN_W / 2, by + ZOOM_BTN_H / 2);
  }

  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString("API RADIUS " + String(apiRadiusNm(rangeKm)) + " NM", x0 + 2, y0 + 214);
  lcd.drawString(String(poolCount()) + " AIRCRAFT", x0 + 2, y0 + 238);
  lcd.setTextSize(1);
}

// Top half of the sidebar: menu pages, detail panel when a plane is selected, plane list otherwise
void drawInfoTop() {
  if (menuMode == MENU_MAIN) drawMenuPanel();
  else if (menuMode == MENU_FILTER) drawFilterPanel();
  else if (menuMode == MENU_ZOOM) drawZoomPanel();
  else if (hasSelection) drawDetailPanel();
  else drawDetectedAircraftPanel();
}

void drawRightInfoPanel() {
  lcd.fillRect(INFO_X, INFO_Y, INFO_W, INFO_H, TFT_BLACK);
  lcd.drawRect(INFO_X, INFO_Y, INFO_W, INFO_H, green);

  drawInfoTop();
  drawLocationPanel();
  drawTimePanel();
}

// ---------- TOUCH (GT911 on the same I2C bus the sketch already opens) ----------
uint8_t touchAddr = 0;
bool touchOk = false;

bool gt911Read(uint16_t reg, uint8_t* buf, int len) {
  Wire.beginTransmission(touchAddr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)touchAddr, len) != len) return false;
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void gt911ClearStatus() {
  Wire.beginTransmission(touchAddr);
  Wire.write((uint8_t)0x81);
  Wire.write((uint8_t)0x4E);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
}

void initTouch() {
  // Print everything on the bus so we can see what's there
  Serial.print("I2C devices:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();

  const uint8_t addrs[] = {0x5D, 0x14};
  for (uint8_t a : addrs) {
    touchAddr = a;
    uint8_t id[4] = {0, 0, 0, 0};
    if (gt911Read(0x8140, id, 4)) {
      Serial.printf("Touch: GT911 found at 0x%02X (id %c%c%c%c)\n", a, id[0], id[1], id[2], id[3]);
      touchOk = true;
      return;
    }
  }
  Serial.println("Touch: GT911 not found - radar still works, taps are disabled");
}

// True once at the START of each touch (not while the finger is held)
bool touchTapStarted(int* x, int* y) {
  static bool down = false;
  if (!touchOk) return false;

  uint8_t st;
  if (!gt911Read(0x814E, &st, 1)) return false;
  if (!(st & 0x80)) return false;            // no new report yet

  uint8_t n = st & 0x0F;
  bool newTap = false;

  if (n > 0 && n <= 5) {
    uint8_t d[4];
    if (gt911Read(0x8150, d, 4)) {
      int tx = d[0] | (d[1] << 8);
      int ty = d[2] | (d[3] << 8);
      if (TOUCH_SWAP_XY) { int t = tx; tx = ty; ty = t; }
      if (TOUCH_FLIP_X) tx = 799 - tx;
      if (TOUCH_FLIP_Y) ty = 479 - ty;
      *x = tx;
      *y = ty;
      newTap = !down;
    }
    down = true;
  } else {
    down = false;
  }

  gt911ClearStatus();
  return newTap;
}

void selectPlane(int idx) {
  if (planes[idx].hex[0] == '\0') return;     // no id = can't track it

  strncpy(selectedHex, planes[idx].hex, sizeof(selectedHex) - 1);
  selectedHex[sizeof(selectedHex) - 1] = '\0';
  hasSelection = true;
  selLost = false;
  selSnap = planes[idx];
  menuMode = MENU_NONE;                          // picking a plane replaces any open menu page

  // keep the sidebar list on the page that holds this plane
  rebuildSortedList();
  int pos = findSortedPos(selectedHex);
  if (pos >= 0) listPage = pos / LIST_ROWS;

  requestLookup();
  drawInfoTop();
}

// Move the selection to the next (+1) or previous (-1) plane, nearest-first, wrapping around
void stepSelection(int dir) {
  rebuildSortedList();
  if (sortedCount == 0) return;
  int pos = findSortedPos(selectedHex);          // -1 if the selected plane left the feed
  for (int t = 0; t < sortedCount; t++) {
    if (pos < 0) pos = (dir > 0) ? 0 : sortedCount - 1;
    else         pos = (pos + dir + sortedCount) % sortedCount;
    int idx = sortedIdx[pos];
    if (planes[idx].hex[0] == '\0') continue;                      // can't track it
    if (strcmp(planes[idx].hex, selectedHex) == 0) continue;       // that's the current one
    selectPlane(idx);
    return;
  }
}

void clearSelection() {
  hasSelection = false;
  selectedHex[0] = '\0';
  drawInfoTop();
}

void openMenu() {
  hasSelection = false;                          // the menu takes over the detail panel's spot
  selectedHex[0] = '\0';
  menuMode = MENU_MAIN;
  drawInfoTop();
}

void closeMenu() {
  menuMode = MENU_NONE;
  drawInfoTop();
}

// New range (10 km steps): hides planes outside it right away and asks the API for a fresh pool
void setRange(int km) {
  if (km < RANGE_MIN_KM) km = RANGE_MIN_KM;
  if (km > RANGE_MAX_KM) km = RANGE_MAX_KM;
  km = (km / RANGE_STEP_KM) * RANGE_STEP_KM;
  if (km == rangeKm) return;

  rangeKm = km;
  saveSettings();
  listPage = 0;
  refetchNow = true;
  rangeApplyWanted = true;
  Serial.printf("Range -> %d km (API radius %d nm)\n", km, apiRadiusNm(km));
  drawInfoTop();
}

// Taps inside the sidebar while a menu page is open
void handleMenuTap(int x, int y) {
  int x0 = DETAIL_X, w = DETAIL_W;

  // back arrow (filter / zoom pages)
  if (menuMode != MENU_MAIN && x >= BACK_X && x <= BACK_X + BACK_W + 10 &&
      y >= BACK_Y && y <= BACK_Y + BACK_H + 10) {
    menuMode = MENU_MAIN;
    drawInfoTop();
    return;
  }

  if (menuMode == MENU_MAIN) {
    if (x < x0 || x > x0 + w) return;
    if (y >= MENU_FILTER_Y && y <= MENU_FILTER_Y + MENU_BTN_H) { menuMode = MENU_FILTER; drawInfoTop(); }
    else if (y >= MENU_ZOOM_Y && y <= MENU_ZOOM_Y + MENU_BTN_H) { menuMode = MENU_ZOOM; drawInfoTop(); }
    return;
  }

  if (menuMode == MENU_FILTER) {
    for (int i = 0; i < 4; i++) {
      int ry = FILT_ROW_Y0 + i * FILT_ROW_STEP;
      if (y < ry || y > ry + FILT_ROW_H || x < x0 || x > x0 + w) continue;

      int onCount = 0;
      for (int k = 0; k < 4; k++) if (bandOn[k]) onCount++;
      if (!(bandOn[i] && onCount == 1)) {        // can't switch off the last band
        bandOn[i] = !bandOn[i];
        saveSettings();
        listPage = 0;
      }
      drawInfoTop();
      return;
    }

    if (x >= FILT_ALL_X - 6 && x <= FILT_ALL_X + FILT_ALL_W && y >= FILT_ALL_Y - 6 && y <= FILT_ALL_Y + FILT_ALL_H + 6) {
      for (int k = 0; k < 4; k++) bandOn[k] = true;
      saveSettings();
      listPage = 0;
      drawInfoTop();
    }
    return;
  }

  if (menuMode == MENU_ZOOM) {
    for (int i = 0; i < 10; i++) {
      int bx = x0 + (i % 5) * ZOOM_BTN_STEP_X;
      int by = ZOOM_GRID_Y + (i / 5) * ZOOM_BTN_STEP_Y;
      if (x >= bx && x <= bx + ZOOM_BTN_W && y >= by && y <= by + ZOOM_BTN_H) {
        setRange((i + 1) * RANGE_STEP_KM);
        return;
      }
    }
  }
}

void onTap(int x, int y) {
  lastInteractMs = millis();
  Serial.printf("TAP %d,%d\n", x, y);

  // 0) the RANGE AC box on the radar opens the filter / zoom menu
  if (x >= RB_X1 && x <= RB_X2 && y >= RB_Y1 && y <= RB_Y2) {
    openMenu();
    return;
  }

  // 1) close button on the detail panel or a menu page (a little extra reach for fingers)
  if ((hasSelection || menuMode != MENU_NONE) && x >= CLOSE_X - 10 && x <= CLOSE_X + CLOSE_W + 10 &&
      y >= CLOSE_Y && y <= CLOSE_Y + CLOSE_H + 10) {
    if (menuMode != MENU_NONE) closeMenu();
    else clearSelection();
    return;
  }

  // 1a) anything else in the sidebar while a menu page is open
  if (menuMode != MENU_NONE && x >= RADAR_X + RADAR_W) {
    handleMenuTap(x, y);
    return;
  }

  // 1b) prev / next arrows on the detail panel
  if (hasSelection && y >= NAV_Y - 6 && y <= NAV_Y + NAV_H + 6) {
    if (x >= NAV_PREV_X - 6 && x <= NAV_PREV_X + NAV_W + 6) { stepSelection(-1); return; }
    if (x >= NAV_NEXT_X - 6 && x <= NAV_NEXT_X + NAV_W + 6) { stepSelection(+1); return; }
  }

  // 2) the radar: nearest visible dot within ~28 px, or empty space to deselect
  if (x < RADAR_X + RADAR_W) {
    int best = -1;
    int bestD2 = 28 * 28;
    for (int i = 0; i < planeCount; i++) {
      if (!planes[i].valid || !planes[i].vis) continue;
      int dx = x - planes[i].sx;
      int dy = y - planes[i].sy;
      int d2 = dx * dx + dy * dy;
      if (d2 < bestD2) { bestD2 = d2; best = i; }
    }
    if (best >= 0) selectPlane(best);
    else if (hasSelection) clearSelection();
    else if (menuMode != MENU_NONE) closeMenu();
    return;
  }

  // 3a) page arrows in the list header
  if (!hasSelection && y >= PAGER_Y - 4 && y < DETAIL_Y + 30) {
    rebuildSortedList();
    int pages = listPageCount();
    if (pages > 1) {
      if (x <= PAGER_LEFT_X + PAGER_W + 6) {
        listPage = (listPage + pages - 1) % pages;
        drawInfoTop();
      } else if (x >= PAGER_RIGHT_X - 6) {
        listPage = (listPage + 1) % pages;
        drawInfoTop();
      }
    }
    return;
  }

  // 3) a row in the sidebar list (rows start at DETAIL_Y + 34, 26 px apart)
  if (!hasSelection && y >= DETAIL_Y + 30 && y < INFO_Y + 295) {
    int row = (y - (DETAIL_Y + 30)) / 26;
    if (row >= 0 && row < LIST_ROWS && listIdx[row] >= 0 && listIdx[row] < planeCount) {
      selectPlane(listIdx[row]);
    }
  }
}

void handleTouch() {
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll < 20) return;
  lastPoll = millis();

  int x, y;
  if (touchTapStarted(&x, &y)) onTap(x, y);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  Serial.println("Radar " RADAR_VERSION);

  loadSettings();
  Serial.printf("Range %d km, altitude bands 0x%02X\n", (int)rangeKm, bandMask());

  Wire.begin(19, 20);
  Wire.beginTransmission(0x18);
  Wire.write(0x01);
  Wire.write(0x3B);
  Wire.endTransmission();

  lcd.init();
  lcd.setBrightness(180);

  initTouch();

  greenBright = lcd.color565(0, 255, 35);
  green       = lcd.color565(0, 180, 25);
  greenDim    = lcd.color565(0, 90, 18);
  greenDark   = lcd.color565(0, 45, 10);
  panelBg     = lcd.color565(0, 20, 0);

  radarCanvas.setPsram(true);
  radarCanvas.setColorDepth(16);

  if (!radarCanvas.createSprite(480, 480)) {
    Serial.println("Radar sprite create failed!");
  }

  lcd.fillScreen(TFT_BLACK);

  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setTextDatum(middle_center);
  lcd.drawString("Connecting WiFi...", 240, 240);

  dataMutex = xSemaphoreCreateMutex();

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);        // don't save credentials to flash on every connect
  WiFi.setAutoReconnect(true);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());   // handy if your network needs the device registered
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);          // modem sleep causes random drops on some routers

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(300);
  }

  lcd.fillScreen(TFT_BLACK);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");

    // US Eastern Time Zone rule for NTP time sync
    configTzTime("EST5EDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
    timeConfigured = true;

    lastFetchStartMs = millis();
    fetchAircraftData();
    applyPendingAircraftData();
  } else {
    Serial.println("WiFi failed - will keep retrying in the background");
  }

  drawRadarFrameToCanvas();
  radarCanvas.pushSprite(0, 0);

  drawRightInfoPanel();
}

// ---------- LOOP ----------
void loop() {
  ensureWifi();
  checkFetchWatchdog();
  maybeStartFetch();
  checkSafetyNet();
  handleTouch();
  maybeStartLookup();
  checkLookupWatchdog();
  if (lookupDone) {
    lookupDone = false;
    if (hasSelection) drawInfoTop();   // route/model just arrived
  }

  if ((hasSelection || menuMode != MENU_NONE) && millis() - lastInteractMs > SELECTION_TIMEOUT_MS) {
    if (menuMode != MENU_NONE) closeMenu();
    else clearSelection();
  }

  // After a zoom change, show the fresh pool as soon as it lands instead of waiting for the sweep to wrap
  if (rangeApplyWanted && pendingDataReady && pendingRangeKm == rangeKm) {
    rangeApplyWanted = false;
    applyPendingAircraftData();
    drawInfoTop();
  }

  if (millis() - lastSweepUpdate > SWEEP_SPEED) {
    lastSweepUpdate = millis();

    drawRadarFrameToCanvas();
    radarCanvas.pushSprite(0, 0);

    sweepAngle += 2;

    if (sweepAngle >= 360) {
      sweepAngle -= 360;

      applyPendingAircraftData();
      expireOldData();
      drawInfoTop();
    }
  }

  if (millis() - lastClockUpdate > 1000) {
    lastClockUpdate = millis();
    drawTimePanel();
  }
}
