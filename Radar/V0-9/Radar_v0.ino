// Realtime Airplane Radar UI
// CrowPanel 7.0" ESP32-S3 800x480 RGB
// Real aircraft data + background API update + sweep reveal effect
// Credit. Orginial base was mircemk May, 2026
// Updated by (TT-Tio2) for Philadelphia / Boathouse Row Plus
//V0
// some bug fixes including API upgrades

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>
#include "time.h"

// ---------- WIFI (put your own credentials here before uploading) ----------
const char* ssid = "SSID"
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



// Center location: Boathouse Row, Philadelphia, PA (Drexel Rowing area)
const double HOME_LAT = 39.9721;  
const double HOME_LON = -75.1878; 

// Radar range
const float RADAR_RANGE_KM = 100.0;
const int API_RADIUS_NM = 54;   // about 100 km

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
DynamicJsonDocument jsonDoc(32768);

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

  String url = "https://opendata.adsb.fi/api/v2/lat/";
  url += String(HOME_LAT, 4);
  url += "/lon/";
  url += String(HOME_LON, 4);
  url += "/dist/";
  url += String(API_RADIUS_NM);

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

  int code = http.GET();
  Serial.printf("API HTTP Code: %d\n", code);

  if (code != 200) {
    if (code < 0) Serial.println(http.errorToString(code));
    http.end();
    reportFailure("http", code);
    return false;
  }

  // Only keep the fields we actually use
  StaticJsonDocument<768> filter;
  const char* keys[] = {"ac", "aircraft"};
  for (const char* k : keys) {
    JsonObject f = filter[k].createNestedObject();
    f["lat"] = true;  f["lon"] = true;
    f["alt_baro"] = true; f["alt_geom"] = true;
    f["gs"] = true;   f["track"] = true;
    f["flight"] = true; f["category"] = true; f["t"] = true;
  }

  DeserializationError err =
      deserializeJson(jsonDoc, http.getStream(), DeserializationOption::Filter(filter));
  http.end();

  if (err) {
    Serial.print("JSON error: ");
    Serial.println(err.c_str());
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

  AircraftPoint tempPlanes[MAX_PLANES];
  int tempCount = 0;

  for (JsonObject ac : arr) {
    if (tempCount >= MAX_PLANES) break;

    if (!ac["lat"].is<float>() || !ac["lon"].is<float>()) continue;

    double lat = ac["lat"];
    double lon = ac["lon"];

    float d = distanceKm(HOME_LAT, HOME_LON, lat, lon);
    if (d > RADAR_RANGE_KM) continue;

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

    tempCount++;
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  pendingPlaneCount = tempCount;
  for (int i = 0; i < tempCount; i++) {
    pendingPlanes[i] = tempPlanes[i];
  }
  pendingDataReady = true;
  xSemaphoreGive(dataMutex);

  consecutiveFails = 0;
  lastGoodFetchMs = millis();

  Serial.printf("Background aircraft found: %d (heap=%u)\n",
                tempCount, (unsigned)ESP.getFreeHeap());
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

void applyPendingAircraftData() {
  if (!pendingDataReady) return;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  planeCount = pendingPlaneCount;
  for (int i = 0; i < planeCount; i++) {
    planes[i] = pendingPlanes[i];
  }
  pendingDataReady = false;
  xSemaphoreGive(dataMutex);

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
  if (fetchInProgress) return;
  if (WiFi.status() != WL_CONNECTED) return;

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
  int km[]    = {20, 40, 60, 80, 100};

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
    if (!planes[i].valid) continue;

    float ageAngle = angleDiffBehindSweep(sweepAngle, planes[i].bearingDeg);

    if (ageAngle > 260) continue;

    float fade = 1.0;
    if (ageAngle > 200) {
      fade = 1.0 - ((ageAngle - 200.0) / 60.0);
      if (fade < 0.20) fade = 0.20;
    }

    float rr = (planes[i].distanceKm / RADAR_RANGE_KM) * R;
    float rad = (planes[i].bearingDeg - 90) * DEG_TO_RAD;

    int x = CX + cos(rad) * rxCorr(rr);
    int y = CY + sin(rad) * rr;

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
    if (planes[i].distanceKm < 20) size = 6;

    radarCanvas.fillCircle(x, y, size, col);
    radarCanvas.drawCircle(x, y, size + 2, greenDim);

    if (planes[i].trackDeg >= 0) {
      float tr = (planes[i].trackDeg - 90) * DEG_TO_RAD;
      int x2 = x + cos(tr) * 12;
      int y2 = y + sin(tr) * 12;
      radarCanvas.drawLine(x, y, x2, y2, col);
    }

    if (ageAngle >= 0 && ageAngle < 35) {
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

  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;

    int bin = (int)(planes[i].distanceKm / 20.0);
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

  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(green, TFT_BLACK);
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
  radarCanvas.drawString("20",  startX - 1,             baseY + 14);
  radarCanvas.drawString("60",  startX + 2 * barGap - 1, baseY + 14);
  radarCanvas.drawString("100", startX + 4 * barGap - 4, baseY + 14);
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

void drawDetectedAircraftPanel() {
  int topY = INFO_Y + 10;       
  int bottomY = INFO_Y + 295;   
  int boxX = INFO_X + 10;
  int boxW = INFO_W - 20;
  int boxH = bottomY - topY;

  lcd.fillRect(boxX, topY, boxW, boxH, TFT_BLACK);

  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("DETECTED AIRCRAFTS", INFO_X + INFO_W / 2, topY + 2);

  const int MAX_ROWS = 10;
  int idx[MAX_ROWS];
  for (int i = 0; i < MAX_ROWS; i++) idx[i] = -1;

  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;

    for (int j = 0; j < MAX_ROWS; j++) {
      if (idx[j] == -1 || planes[i].distanceKm < planes[idx[j]].distanceKm) {
        for (int k = MAX_ROWS - 1; k > j; k--) idx[k] = idx[k - 1];
        idx[j] = i;
        break;
      }
    }
  }

  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);

  int y = topY + 34;

  for (int row = 0; row < MAX_ROWS; row++) {
    if (idx[row] == -1) break;

    AircraftPoint &p = planes[idx[row]];

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

  if (planeCount == 0) {
    lcd.setTextDatum(top_center);
    lcd.setTextSize(1);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString("NO AIRCRAFT IN RANGE", INFO_X + INFO_W / 2, topY + 52);
  }
}

void drawRightInfoPanel() {
  lcd.fillRect(INFO_X, INFO_Y, INFO_W, INFO_H, TFT_BLACK);
  lcd.drawRect(INFO_X, INFO_Y, INFO_W, INFO_H, green);

  drawDetectedAircraftPanel();
  drawLocationPanel();
  drawTimePanel();
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  Wire.begin(19, 20);
  Wire.beginTransmission(0x18);
  Wire.write(0x01);
  Wire.write(0x3B);
  Wire.endTransmission();

  lcd.init();
  lcd.setBrightness(180);

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

  if (millis() - lastSweepUpdate > SWEEP_SPEED) {
    lastSweepUpdate = millis();

    drawRadarFrameToCanvas();
    radarCanvas.pushSprite(0, 0);

    sweepAngle += 2;

    if (sweepAngle >= 360) {
      sweepAngle -= 360;

      applyPendingAircraftData();
      expireOldData();
      drawDetectedAircraftPanel();
    }
  }

  if (millis() - lastClockUpdate > 1000) {
    lastClockUpdate = millis();
    drawTimePanel();
  }
}
