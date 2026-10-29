// Realtime Airplane Radar UI — v11
// CrowPanel 7.0" ESP32-S3 800x480 RGB
// Real aircraft data + background API update + sweep reveal effect
//
// Originally by mircemk (May 2026)
// https://projecthub.arduino.cc/mircemk
//
// Extended and maintained by TT-Tio2
// https://github.com/TT-Tio2
//
// Default location: Philadelphia, Boathouse Row

#define LGFX_USE_V1
// Radar v11
#define RADAR_VERSION "v11"

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



// Original center: Boathouse Row, Philadelphia, PA (Drexel Rowing area).
// The radar starts here, the PHILLY button returns here, and a failed location lookup falls back here.
// The live center is homeLat / homeLon below; it can be changed from the compass screen.
const double DEFAULT_LAT = 39.9721;
const double DEFAULT_LON = -75.1878;
const char* const DEFAULT_NAME = "PHILLY";
const char* const DEFAULT_TZ   = "EST5EDT,M3.2.0,M11.1.0";   // US Eastern, with daylight saving

// Sent with place-name lookups (OpenStreetMap's Nominatim asks apps to identify themselves)
const char* const GEO_USER_AGENT = "ESP32-AirRadar/1.0 (personal hobby project)";

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
LGFX_Sprite wxCanvas(&lcd);     // v7: full-screen canvas for the weather screen (created the first time it is opened)
void drawSwitcher(LGFX_Sprite& g, uint16_t idleCol, uint16_t numCol, bool smooth);

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
const int PAGER_W       = 48;
const int PAGER_H       = 30;
const int PAGER_Y       = DETAIL_Y;
const int PAGER_LEFT_X  = DETAIL_X;
const int PAGER_RIGHT_X = DETAIL_X + 190;      // v11: the airport button sits in the top right corner (where the X is)

// The RANGE AC box on the radar (tap it to open the menu)
const int RB_X1 = 388, RB_X2 = 472, RB_Y1 = 4, RB_Y2 = 92;

// Menu panels (they cover the same area as the detail panel)
const int BACK_X = DETAIL_X, BACK_Y = DETAIL_Y, BACK_W = 50, BACK_H = 40;
const int MENU_BTN_H     = 56;                 // v11: a little shorter, to make room for the TERRAIN bar
const int MENU_FILTER_Y  = DETAIL_Y + 50;
const int MENU_ZOOM_Y    = DETAIL_Y + 112;
const int MENU_FAV_Y     = DETAIL_Y + 174;
const int MENU_TERR_Y    = DETAIL_Y + 236;     // v11: TERRAIN on / off bar
const int MENU_TERR_H    = 32;
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

// Compass (bottom-left of the radar): tap it to open the location screen (no box drawn around it)
const int CMP_X1 = 12, CMP_X2 = 92, CMP_Y1 = 390, CMP_Y2 = 468;

// Location screen (covers the same area as the detail panel)
const int LOC_HOME_W    = 84;
const int LOC_HOME_H    = 36;
const int LOC_FIELD_Y0  = DETAIL_Y + 42;
const int LOC_FIELD_H   = 34;
const int LOC_FIELD_STEP = 38;
const int LOC_FIELD_X   = DETAIL_X + 56;
const int LOC_STATUS_Y  = DETAIL_Y + 119;
const int LOC_KP_Y0     = DETAIL_Y + 131;
const int LOC_KP_ROW_H  = 33;
const int LOC_KP_COL_W  = 72;

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
const int MENU_NONE = 0, MENU_MAIN = 1, MENU_FILTER = 2, MENU_ZOOM = 3, MENU_LOC = 4;
int  menuMode = MENU_NONE;                      // which panel covers the top of the sidebar
// Altitude bands (same colors as the dots): 0 = 30,000 ft and up, 1 = 10,000-30,000, 2 = below 10,000, 3 = ground/unknown
bool bandOn[4] = {true, true, true, true};
volatile bool refetchNow = false;               // zoom changed: pull a fresh pool right away
bool rangeApplyWanted = false;                  // ...and show it as soon as it lands
volatile int pendingRangeKm = 100;              // range the waiting data was fetched for
Preferences prefs;
bool isEmergencySquawk(const char* sq);

// ---------- v11: TERRAIN, AIRCRAFT PHOTO, AIRPORT INFO ----------
const int MENU_AIRPORT = 6;                    // sidebar page for a tapped airport
const char* const NEWJOB_USER_AGENT = "ESP32-AirRadar/1.1 (personal hobby project)";

// Terrain: topographic contour lines under the radar (AWS elevation tiles). Built once per location / zoom into terrainSprite.
bool  terrainOn = true;                        // TERRAIN switch in the RANGE AC menu (remembered across reboots)
LGFX_Sprite terrainSprite(&lcd);               // the finished map, same size as the radar canvas
LGFX_Sprite tileSprite(&lcd);                  // one decoded 256 x 256 tile (24 bit), read back as elevation
bool  terrainSpriteOk = false;
LGFX_Sprite terrainTop(&lcd);                  // only the pixels that must stay visible when the sweep passes (lines, shore, water); black = see-through
bool  terrainTopOk = false;
bool  tileSpriteOk = false;
int16_t* terrainGrid = nullptr;                // elevation (m) for every radar pixel while a map is being made
volatile bool terrainReady = false;            // terrainSprite holds a finished map for the terrainBuilt* values below
float terrainBuiltLat = 0, terrainBuiltLon = 0;
int   terrainBuiltRange = 0;
volatile bool terrainInProgress = false;       // a terrain task (one tile, or the final drawing) is running
TaskHandle_t terrainTaskHandle = NULL;
unsigned long terrainStartMs = 0;
float terrainWantLat = 0, terrainWantLon = 0;  // what the last check saw, to notice a new location / zoom
int   terrainWantRange = 0;
unsigned long terrainWantChangedMs = 0;
bool  terrainPlanOk = false;                   // the tile plan below is for the current location / zoom
float terrainPlanLat = 0, terrainPlanLon = 0;
int   terrainPlanRange = 0;
int   terrainZ = 0, terrainTx0 = 0, terrainTy0 = 0, terrainNx = 0, terrainNy = 0;
volatile int  terrainFails = 0;
volatile bool terrainGaveUp = false;
volatile unsigned long terrainRetryAt = 0;
volatile int terrainAction = 0;                // what the task does: 1 = download + decode one tile, 2 = draw the finished map
int   terrainActZ = 0, terrainActX = 0, terrainActY = 0, terrainActSlot = 0;
int   terrainRnZ = 0, terrainRnRange = 0;      // the plan the drawing task works from (the main loop leaves these alone while it runs)
float terrainRnLat = 0, terrainRnLon = 0;

const int TERRAIN_CACHE = 9;                   // decoded tiles kept in PSRAM (128 KB each), so a zoom step often needs no download
struct TileSlot {
  int16_t* elev;                               // 256 x 256 elevation in metres
  int z, x, y;                                 // z = -1: empty / being filled
  unsigned long used;
};
TileSlot tileCache[TERRAIN_CACHE];

// Aircraft photo (Planespotters): kept in RAM only, for the plane you are looking at
bool  photoMode = false;                       // the photo covers the bottom of the sidebar
volatile bool photoInProgress = false;
volatile bool photoDone = false;
bool  photoWanted = false;
int   photoState = 0;                          // 0 = nothing yet, 1 = loading, 2 = have a photo, 3 = no photo on file, 4 = failed
char  photoHex[8] = "";                        // plane the state / picture belongs to
char  photoRunHex[8] = "";                     // plane the running task is working on
char  photoWho[48] = "";                       // photographer
uint8_t* photoBuf = nullptr;
size_t photoLen = 0;
TaskHandle_t photoTaskHandle = NULL;
unsigned long photoStartMs = 0;
const size_t PHOTO_CAP = 90000;

// Airport page: CENTER button (moves the radar to the airport) and the airport button state
int   apRecenter = 0;                          // 0 = idle, 1 = working, 2 = failed (shown on the CENTER button)
bool  geoRecenter = false;                     // the running place lookup is for the CENTER button
char  geoRecenterName[28] = "";

// Airport info: live weather report for a tapped airport (aviationweather.gov, no key)
int   apSel = -1;                              // airport shown in the sidebar page
int   mtState = 0;                             // 0 = idle, 1 = loading, 2 = have a report, 3 = no report, 4 = failed
int   mtAirport = -1;                          // airport the report / state belongs to
int   mtRunAirport = -1;                       // airport the running task is working on
unsigned long mtWhen = 0;
bool  metarWanted = false;
volatile bool metarInProgress = false;
volatile bool metarDone = false;
TaskHandle_t metarTaskHandle = NULL;
unsigned long metarStartMs = 0;
char  mtCat[6] = "";                           // flight category: VFR / MVFR / IFR / LIFR
float mtTempC = NAN;
int   mtWdir = -1, mtWspd = -1, mtWgst = 0;    // wdir -1 = variable
char  mtVis[10] = "";
char  mtSky[24] = "";
char  mtRaw[130] = "";
long  mtObs = 0;                               // observation time, UNIX seconds

bool extraNetBusy() { return photoInProgress || metarInProgress || terrainInProgress; }

// ---------- v10: plane types, squawk banner, places, sunrise / sunset, auto-dim ----------
const int PC_AIRLINER = 0, PC_SMALL = 1, PC_HELI = 2, PC_MIL = 3;      // plane classes (they decide the dot shape)

// squawk banner: bottom of the radar, left of the sweep indicator
const int BN_X = 100, BN_Y = 424, BN_W = 292, BN_H = 44;
bool bannerShown = false;
char bannerHex[8] = "";

// favorite places: RANGE AC box -> menu -> PLACES
const int MENU_FAV = 5;
const int FAV_SLOTS = 4;
struct FavPlace {
  bool  used;
  float lat;
  float lon;
  char  name[28];
  char  tz[32];
};
FavPlace favs[FAV_SLOTS];
int favCount();
void drawFavPanel();
int favMode = 0;                                   // 0 = normal, 1 = SAVE HERE armed, 2 = CLEAR armed

// sunrise / sunset of the saved location, in minutes after local midnight
volatile int  sunriseMin = -1;
volatile int  sunsetMin  = -1;
volatile bool sunKnown   = false;
volatile unsigned long sunNextCheckMs = 0;        // 0 = look them up as soon as possible
volatile int  geoSunriseMin = -1;                 // from the latest place / time zone lookup
volatile int  geoSunsetMin  = -1;
bool geoSunOnly = false;                          // true = the lookup only refreshes sunrise / sunset

// auto-dim: fades down after sunset, back up before sunrise; a touch brings full brightness back for a while
const int BRIGHT_DAY   = 180;                     // same as the boot brightness
const int BRIGHT_NIGHT = 45;
const unsigned long WAKE_MS = 30000UL;
const int DIM_FADE_MIN = 30;                      // minutes the fade takes
unsigned long lastTouchMs = 0;
int curBright = BRIGHT_DAY;

// ---------- SCREENS (v7): 1 = RADAR, 2 = WEATHER ----------
// Only the active screen does any network work. Leaving a screen stops its fetching, entering one starts fresh.
const int SCREEN_RADAR = 0, SCREEN_WEATHER = 1;
const int NUM_SCREENS = 2;
int  curScreen = SCREEN_RADAR;

// Screen switcher: the top-left corner (no box). Tap = shows the screen number, tap again = next number,
// SW_HOLD_MS after the last tap it switches to the number on display.
const int SW_ZONE_W = 130, SW_ZONE_H = 80;
const unsigned long SW_HOLD_MS = 3000;
bool swShown   = false;                 // the number is on screen
bool swLeaving = false;                 // timer ran out: waiting for the leaving screen's network jobs to finish
int  swNum     = 1;                     // number on display (1-based)
int  swTarget  = 0;                     // screen index we are switching to
unsigned long swLastTapMs = 0;
unsigned long radarEnteredMs = 0;       // when the radar screen was last entered (safety-net reference)

// ---------- WEATHER (v7): Open-Meteo, no API key ----------
const int WX_DAILY = 0, WX_WEEKLY = 1;
const unsigned long WX_REFRESH_MS      = 900000UL;   // refresh every 15 min while the weather screen is open
const unsigned long WX_HARD_TIMEOUT_MS = 25000UL;    // kill a weather fetch that hangs

struct WxDay {
  char  date[11];      // "YYYY-MM-DD" (local date at the place)
  int   code;          // WMO weather code
  float hiC, loC;
  int   popMax;        // max chance of precipitation, %
  float precipIn;      // total precipitation, inches
};

struct WxHour {
  int16_t  tC10;       // temperature, tenths of a degree C
  uint8_t  pop;        // chance of precipitation, %
  uint8_t  code;       // WMO weather code
  uint16_t pre1000;    // precipitation, thousandths of an inch
};

struct WeatherData {
  bool  valid;
  float lat, lon;      // the place this data is for
  float tempC, feelsC, windMph, precipIn;
  int   hum, windDir, code;
  bool  isDay;
  int   dayCount;
  WxDay day[7];
  int   sunriseMin, sunsetMin;   // today's sunrise / sunset, minutes after local midnight (-1 = unknown)
  int   nowHour;       // local hour of the 'current' reading (0-23), -1 if unknown
  int   hrCount;       // hourly values received (24 per day, day 0 starts at local midnight)
  WxHour hr[168];
};

WeatherData wx;                          // newest good data (written by the fetch task under dataMutex)
WeatherData wxView;                      // snapshot the draw code reads
volatile bool wxInProgress = false;
volatile bool wxDirty      = true;       // screen needs a redraw
volatile int  wxFails      = 0;
unsigned long wxStartMs    = 0;
unsigned long lastWxStartMs = 0;         // 0 = fetch as soon as possible
TaskHandle_t  wxTaskHandle = NULL;
int  wxTab = WX_DAILY;
int  wxSelDay = -1;                      // WEEKLY tab: -1 = the week list, 0-6 = that day's hourly view
bool wxCanvasOk = false;

// ---------- RADAR CENTER (changed from the compass screen, remembered across reboots) ----------
volatile float homeLat = (float)DEFAULT_LAT;
volatile float homeLon = (float)DEFAULT_LON;
char placeName[28] = "";                        // shown in the location panel (filled in by loadSettings)
char tzString[32]  = "";                        // POSIX time zone rule used by the clock
volatile float pendingLat = (float)DEFAULT_LAT; // center the waiting data was fetched for
volatile float pendingLon = (float)DEFAULT_LON;

// Place-name + time-zone lookup (runs as a background task, one network job at a time)
const int GEO_IDLE = 0, GEO_WAITING = 1, GEO_RUNNING = 2, GEO_OK = 3, GEO_FAIL = 4;
volatile int geoState = GEO_IDLE;
float geoReqLat = 0, geoReqLon = 0;             // coordinates being looked up
bool  geoTzOnly = false;                        // true = just refreshing the time zone offset (daylight saving)
char  geoName[28] = "";                         // results
char  geoTz[32]   = "";
unsigned long geoStartMs = 0;
TaskHandle_t geoTaskHandle = NULL;
unsigned long tzNextCheckMs = 0;                // next daylight-saving re-check for a custom location

bool geoBusy() { return geoState == GEO_WAITING || geoState == GEO_RUNNING; }

// Location screen typing state
char locBuf[2][13] = {"", ""};                  // [0] = latitude text, [1] = longitude text
int  locField = 0;
bool locReplace[2] = {false, false};            // next key replaces the whole field (like "select all")
const char* locStatus = "";
bool locStatusRed = false;

bool atDefaultLocation() {
  return fabs((double)homeLat - DEFAULT_LAT) < 0.00002 && fabs((double)homeLon - DEFAULT_LON) < 0.00002;
}

void applyTimezone(const char* posix) {
  strncpy(tzString, posix, sizeof(tzString) - 1);
  tzString[sizeof(tzString) - 1] = '\0';
  setenv("TZ", tzString, 1);
  tzset();
}

// POSIX time zone text from a UTC offset in seconds. POSIX flips the sign: UTC+5:30 is "<+0530>-5:30".
void tzFromOffset(int offsetSec, char* out, size_t n) {
  int a = abs(offsetSec);
  int h = a / 3600;
  int m = (a % 3600) / 60;
  char nm[10], ofs[10];
  if (m) {
    snprintf(nm, sizeof(nm), "%c%02d%02d", offsetSec < 0 ? '-' : '+', h, m);
    snprintf(ofs, sizeof(ofs), "%d:%02d", h, m);
  } else {
    snprintf(nm, sizeof(nm), "%c%02d", offsetSec < 0 ? '-' : '+', h);
    snprintf(ofs, sizeof(ofs), "%d", h);
  }
  snprintf(out, n, "<%s>%s%s", nm, offsetSec < 0 ? "" : "-", ofs);
}

void setDefaultLocationValues() {
  homeLat = (float)DEFAULT_LAT;
  homeLon = (float)DEFAULT_LON;
  strncpy(placeName, DEFAULT_NAME, sizeof(placeName) - 1);
  placeName[sizeof(placeName) - 1] = '\0';
  applyTimezone(DEFAULT_TZ);
}

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
  return p.valid && p.distanceKm <= rangeKm && (bandOn[altBand(p.altitudeFt)] || isEmergencySquawk(p.squawk));   // v10: emergencies ignore the altitude filter
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
  prefs.putFloat("lat", homeLat);
  prefs.putFloat("lon", homeLon);
  prefs.putString("place", placeName);
  prefs.putString("tz", tzString);
  prefs.putUChar("terrain", terrainOn ? 1 : 0);   // v11
  prefs.end();
}

void loadSettings() {
  prefs.begin("radar", true);
  int r = prefs.getInt("range", 100);
  uint8_t m = prefs.getUChar("bands", 0x0F);
  float la = prefs.getFloat("lat", NAN);
  float lo = prefs.getFloat("lon", NAN);
  String pn = prefs.getString("place", "");
  String tz = prefs.getString("tz", "");
  terrainOn = prefs.getUChar("terrain", 1) != 0;   // v11
  prefs.end();

  // saved location: use it only if it is complete and sane, otherwise start at the original coordinates
  if (!isnan(la) && !isnan(lo) && la >= -90 && la <= 90 && lo >= -180 && lo <= 180 &&
      pn.length() > 0 && pn.length() < sizeof(placeName) && tz.length() > 0 && tz.length() < sizeof(tzString)) {
    homeLat = la;
    homeLon = lo;
    strncpy(placeName, pn.c_str(), sizeof(placeName) - 1);
    placeName[sizeof(placeName) - 1] = '\0';
    applyTimezone(tz.c_str());
  } else {
    setDefaultLocationValues();
  }

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
  float uLat = homeLat;      // ...and the center it is for (the compass screen may change it)
  float uLon = homeLon;

  String url = "https://opendata.adsb.fi/api/v2/lat/";
  url += String(uLat, 4);
  url += "/lon/";
  url += String(uLon, 4);
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

    float d = distanceKm(uLat, uLon, lat, lon);
    if (d > usedRange) continue;

    AircraftPoint &p = tempPlanes[tempCount];

    p.valid = true;
    p.distanceKm = d;
    p.bearingDeg = bearingDeg(uLat, uLon, lat, lon);

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
  pendingLat = uLat;
  pendingLon = uLon;
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
  if (lookupInProgress || fetchInProgress || geoBusy() || extraNetBusy()) return;   // one network job at a time
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

// ---------- LOCATION LOOKUP: place name (OpenStreetMap Nominatim) + time zone (Open-Meteo) ----------

// Plain uppercase ASCII for the built-in font (Latin-1 letters lose their accents, other scripts are dropped)
void toDisplayAscii(const char* in, char* out, size_t n) {
  static const char LATIN1[] = "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTsaaaaaaaceeeeiiiidnooooo/ouuuuyty";
  size_t o = 0;
  for (size_t i = 0; in[i] && o < n - 1; i++) {
    unsigned char c = (unsigned char)in[i];
    unsigned char c2 = (unsigned char)in[i + 1];
    if (c < 0x80) {
      out[o++] = (char)toupper(c);
    } else if (c == 0xC3 && c2 >= 0x80 && c2 <= 0xBF) {
      out[o++] = (char)toupper(LATIN1[c2 - 0x80]);
      i++;
    }
  }
  out[o] = '\0';
}

// GET a small JSON reply into doc, keeping only the fields named in the filter. Returns 200 when it worked.
int geoGetJson(const String& url, JsonDocument& doc, const JsonDocument& filter) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(10000);
  http.setUserAgent(GEO_USER_AGENT);
  http.useHTTP10(true);

  if (!http.begin(client, url)) return -1;
  int code = http.GET();
  if (code != 200) {
    Serial.printf("Geo lookup HTTP %d\n", code);
    http.end();
    return code;
  }

  String body = http.getString();           // these replies are tiny
  http.end();

  DeserializationError err = deserializeJson(doc, body, DeserializationOption::Filter(filter));
  if (err) {
    Serial.print("Geo JSON error: ");
    Serial.println(err.c_str());
    return -2;
  }
  return 200;
}

// Finds the place name and the UTC offset for a point. tzOnly skips the name (used for daylight-saving re-checks).
bool geoLookup(float lat, float lon, bool tzOnly, char* nameOut, size_t nameN, char* tzOut, size_t tzN) {
  if (!tzOnly) {
    String url = "https://nominatim.openstreetmap.org/reverse?format=jsonv2&zoom=10&addressdetails=1&accept-language=en&lat=";
    url += String(lat, 4);
    url += "&lon=";
    url += String(lon, 4);

    const char* keys[] = {"city", "town", "village", "municipality", "hamlet", "suburb", "county", "state", "country"};
    const int nKeys = 9;

    StaticJsonDocument<512> filter;
    filter["error"] = true;
    JsonObject fa = filter["address"].to<JsonObject>();
    for (int i = 0; i < nKeys; i++) fa[keys[i]] = true;
    fa["country_code"] = true;
    fa["ISO3166-2-lvl4"] = true;

    StaticJsonDocument<1024> doc;
    if (geoGetJson(url, doc, filter) != 200) return false;

    const char* place = "";
    JsonObject addr = doc["address"];
    for (int i = 0; i < nKeys; i++) {
      const char* v = addr[keys[i]] | "";
      if (v[0]) { place = v; break; }
    }

    // region: state code for the US (US-PA -> PA), otherwise the country code
    char region[8] = "";
    const char* cc = addr["country_code"] | "";
    const char* lvl4 = addr["ISO3166-2-lvl4"] | "";
    if (strcmp(cc, "us") == 0 && strlen(lvl4) >= 4 && lvl4[2] == '-') {
      strncpy(region, lvl4 + 3, 3);
      region[3] = '\0';
    } else if (cc[0]) {
      strncpy(region, cc, 3);
      region[3] = '\0';
      for (int i = 0; region[i]; i++) region[i] = (char)toupper((unsigned char)region[i]);
    }

    String raw = place;
    if (place[0] && region[0]) {
      raw += ", ";
      raw += region;
    }

    nameOut[0] = '\0';
    if (raw.length()) {
      toDisplayAscii(raw.c_str(), nameOut, nameN);
      if (strlen(nameOut) > 20) nameOut[20] = '\0';        // fits the panel at text size 2
    }
    if (!nameOut[0]) {                                      // open ocean etc.: no place to name, show the coordinates
      snprintf(nameOut, nameN, "%.3f, %.3f", lat, lon);
    }
  }

  String url2 = "https://api.open-meteo.com/v1/forecast?current=temperature_2m&timezone=auto&forecast_days=1&daily=sunrise,sunset&latitude=";
  url2 += String(lat, 4);
  url2 += "&longitude=";
  url2 += String(lon, 4);

  StaticJsonDocument<128> f2;
  f2["utc_offset_seconds"] = true;
  JsonObject fdaily = f2["daily"].to<JsonObject>();
  fdaily["sunrise"] = true;
  fdaily["sunset"] = true;
  StaticJsonDocument<384> d2;
  if (geoGetJson(url2, d2, f2) != 200) return false;
  if (!d2["utc_offset_seconds"].is<int>()) return false;

  tzFromOffset(d2["utc_offset_seconds"].as<int>(), tzOut, tzN);

  // v10: sunrise / sunset as minutes after local midnight ("2026-10-25T07:12")
  geoSunriseMin = -1;
  geoSunsetMin = -1;
  const char* sr = d2["daily"]["sunrise"][0] | "";
  const char* ss = d2["daily"]["sunset"][0] | "";
  if (strlen(sr) >= 16 && strlen(ss) >= 16) {
    geoSunriseMin = atoi(sr + 11) * 60 + atoi(sr + 14);
    geoSunsetMin  = atoi(ss + 11) * 60 + atoi(ss + 14);
  }
  return true;
}

void geoTask(void* pv) {
  char nm[28] = "";
  char tz[32] = "";
  bool ok = geoLookup(geoReqLat, geoReqLon, geoTzOnly, nm, sizeof(nm), tz, sizeof(tz));
  if (ok) {
    strcpy(geoName, nm);
    strcpy(geoTz, tz);
  }
  geoTaskHandle = NULL;
  geoState = ok ? GEO_OK : GEO_FAIL;
  vTaskDelete(NULL);
}

void maybeStartGeo() {
  if (geoState != GEO_WAITING) return;
  if (fetchInProgress || lookupInProgress || extraNetBusy()) return;      // one network job at a time
  if (WiFi.status() != WL_CONNECTED) {
    geoState = GEO_FAIL;
    return;
  }

  geoState = GEO_RUNNING;
  geoStartMs = millis();
  BaseType_t ok = xTaskCreatePinnedToCore(geoTask, "geoTask", 16384, NULL, 1, &geoTaskHandle, 0);
  if (ok != pdPASS) {
    geoTaskHandle = NULL;
    geoState = GEO_FAIL;
  }
}

void checkGeoWatchdog() {
  if (geoState == GEO_RUNNING && millis() - geoStartMs > 30000) {
    Serial.println("Geo lookup hung - killing it");
    if (geoTaskHandle) vTaskDelete(geoTaskHandle);
    geoTaskHandle = NULL;
    geoState = GEO_FAIL;
  }
}

// Custom locations: re-check the UTC offset now and then so daylight saving changes are picked up
void maybeScheduleTzCheck() {
  if (geoState != GEO_IDLE || atDefaultLocation()) return;
  if (millis() < tzNextCheckMs || WiFi.status() != WL_CONNECTED) return;
  geoReqLat = homeLat;
  geoReqLon = homeLon;
  geoTzOnly = true;
  geoState = GEO_WAITING;
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

  if (pendingLat != homeLat || pendingLon != homeLon) {   // fetched for an old center: throw it away
    pendingDataReady = false;
    refetchNow = true;
    return;
  }

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
      configTzTime(tzString, "pool.ntp.org", "time.nist.gov");
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
  if (fetchInProgress || lookupInProgress || geoBusy() || extraNetBusy()) return;   // one network job at a time
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
  // v7: the radar doesn't fetch while another screen is open, so count from when it was last entered
  unsigned long ref = lastGoodFetchMs ? lastGoodFetchMs : radarEnteredMs;
  if (millis() - ref > RESTART_AFTER_MS) {
    Serial.println("No data for too long - restarting");
    delay(200);
    ESP.restart();
  }
}

// ---------- RADAR CANVAS DRAW ----------
void drawGridToCanvas() {
  if (terrainOn && terrainKeyIsBuilt()) terrainSprite.pushSprite(&radarCanvas, 0, 0);   // v11: the topographic map is the background
  else radarCanvas.fillSprite(TFT_BLACK);

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

  terrainOverlayOnSweep();                       // v11: terrain pixels win over the sweep
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

// ---------- v10: AIRPORT MARKERS (v11: tap a star for its info page) ----------
// Built-in list (no API): major US airports plus the Philadelphia area. The radar draws the ones inside the current range.
struct Airport {
  const char* code;     // 3-letter code shown on the radar
  const char* icao;     // 4-letter code (used for the weather report)
  const char* name;
  const char* city;
  float lat;
  float lon;
};

const Airport AIRPORTS[] = {
  // Philadelphia area
  {"PHL", "KPHL", "PHILADELPHIA INTL", "PHILADELPHIA, PA", 39.8744f, -75.2424f},
  {"PNE", "KPNE", "NORTHEAST PHILADELPHIA", "PHILADELPHIA, PA", 40.0819f, -75.0106f},
  {"ILG", "KILG", "NEW CASTLE", "WILMINGTON, DE", 39.6787f, -75.6065f},
  {"TTN", "KTTN", "TRENTON-MERCER", "TRENTON, NJ", 40.2767f, -74.8135f},
  {"ACY", "KACY", "ATLANTIC CITY INTL", "ATLANTIC CITY, NJ", 39.4576f, -74.5772f},
  {"MIV", "KMIV", "MILLVILLE", "MILLVILLE, NJ", 39.3678f, -75.0722f},
  {"LNS", "KLNS", "LANCASTER", "LANCASTER, PA", 40.1217f, -76.2961f},
  {"RDG", "KRDG", "READING RGNL", "READING, PA", 40.3785f, -75.9652f},
  {"ABE", "KABE", "LEHIGH VALLEY INTL", "ALLENTOWN, PA", 40.6521f, -75.4408f},
  {"MDT", "KMDT", "HARRISBURG INTL", "HARRISBURG, PA", 40.1935f, -76.7634f},
  {"DOV", "KDOV", "DOVER AFB", "DOVER, DE", 39.1295f, -75.4660f},
  {"BLM", "KBLM", "MONMOUTH EXECUTIVE", "BELMAR, NJ", 40.1869f, -74.1249f},
  // Northeast
  {"EWR", "KEWR", "NEWARK LIBERTY INTL", "NEWARK, NJ", 40.6925f, -74.1687f},
  {"JFK", "KJFK", "JOHN F KENNEDY INTL", "NEW YORK, NY", 40.6398f, -73.7789f},
  {"LGA", "KLGA", "LAGUARDIA", "NEW YORK, NY", 40.7769f, -73.8740f},
  {"TEB", "KTEB", "TETERBORO", "TETERBORO, NJ", 40.8501f, -74.0608f},
  {"ISP", "KISP", "LONG ISLAND MACARTHUR", "ISLIP, NY", 40.7952f, -73.1002f},
  {"HPN", "KHPN", "WESTCHESTER COUNTY", "WHITE PLAINS, NY", 41.0670f, -73.7076f},
  {"SWF", "KSWF", "STEWART INTL", "NEWBURGH, NY", 41.5041f, -74.1048f},
  {"AVP", "KAVP", "WILKES-BARRE/SCRANTON", "AVOCA, PA", 41.3385f, -75.7234f},
  {"BWI", "KBWI", "BALTIMORE/WASHINGTON", "BALTIMORE, MD", 39.1774f, -76.6684f},
  {"DCA", "KDCA", "REAGAN NATIONAL", "WASHINGTON, DC", 38.8512f, -77.0402f},
  {"IAD", "KIAD", "WASHINGTON DULLES", "DULLES, VA", 38.9531f, -77.4565f},
  {"RIC", "KRIC", "RICHMOND INTL", "RICHMOND, VA", 37.5052f, -77.3197f},
  {"ORF", "KORF", "NORFOLK INTL", "NORFOLK, VA", 36.8946f, -76.2012f},
  {"BOS", "KBOS", "BOSTON LOGAN", "BOSTON, MA", 42.3656f, -71.0096f},
  {"PVD", "KPVD", "T F GREEN", "PROVIDENCE, RI", 41.7240f, -71.4281f},
  {"BDL", "KBDL", "BRADLEY INTL", "WINDSOR LOCKS, CT", 41.9389f, -72.6832f},
  {"ALB", "KALB", "ALBANY INTL", "ALBANY, NY", 42.7483f, -73.8017f},
  {"SYR", "KSYR", "SYRACUSE HANCOCK", "SYRACUSE, NY", 43.1112f, -76.1063f},
  {"ROC", "KROC", "ROCHESTER INTL", "ROCHESTER, NY", 43.1189f, -77.6724f},
  {"BUF", "KBUF", "BUFFALO NIAGARA", "BUFFALO, NY", 42.9405f, -78.7322f},
  {"PIT", "KPIT", "PITTSBURGH INTL", "PITTSBURGH, PA", 40.4915f, -80.2329f},
  // Midwest
  {"CLE", "KCLE", "CLEVELAND HOPKINS", "CLEVELAND, OH", 41.4117f, -81.8498f},
  {"CMH", "KCMH", "JOHN GLENN COLUMBUS", "COLUMBUS, OH", 39.9980f, -82.8919f},
  {"CVG", "KCVG", "CINCINNATI/N KENTUCKY", "HEBRON, KY", 39.0488f, -84.6678f},
  {"DTW", "KDTW", "DETROIT METRO", "DETROIT, MI", 42.2124f, -83.3534f},
  {"ORD", "KORD", "CHICAGO O'HARE", "CHICAGO, IL", 41.9742f, -87.9073f},
  {"MDW", "KMDW", "CHICAGO MIDWAY", "CHICAGO, IL", 41.7868f, -87.7522f},
  {"MSP", "KMSP", "MINNEAPOLIS-ST PAUL", "MINNEAPOLIS, MN", 44.8848f, -93.2223f},
  {"STL", "KSTL", "ST LOUIS LAMBERT", "ST LOUIS, MO", 38.7487f, -90.3700f},
  {"MCI", "KMCI", "KANSAS CITY INTL", "KANSAS CITY, MO", 39.2976f, -94.7139f},
  // South
  {"ATL", "KATL", "HARTSFIELD-JACKSON", "ATLANTA, GA", 33.6407f, -84.4277f},
  {"CLT", "KCLT", "CHARLOTTE DOUGLAS", "CHARLOTTE, NC", 35.2140f, -80.9431f},
  {"RDU", "KRDU", "RALEIGH-DURHAM", "RALEIGH, NC", 35.8776f, -78.7875f},
  {"MIA", "KMIA", "MIAMI INTL", "MIAMI, FL", 25.7959f, -80.2870f},
  {"FLL", "KFLL", "FORT LAUDERDALE", "FORT LAUDERDALE, FL", 26.0726f, -80.1527f},
  {"MCO", "KMCO", "ORLANDO INTL", "ORLANDO, FL", 28.4312f, -81.3081f},
  {"TPA", "KTPA", "TAMPA INTL", "TAMPA, FL", 27.9755f, -82.5332f},
  {"JAX", "KJAX", "JACKSONVILLE INTL", "JACKSONVILLE, FL", 30.4941f, -81.6879f},
  {"MSY", "KMSY", "LOUIS ARMSTRONG", "NEW ORLEANS, LA", 29.9934f, -90.2580f},
  {"BNA", "KBNA", "NASHVILLE INTL", "NASHVILLE, TN", 36.1263f, -86.6774f},
  {"MEM", "KMEM", "MEMPHIS INTL", "MEMPHIS, TN", 35.0424f, -89.9767f},
  {"DFW", "KDFW", "DALLAS/FORT WORTH", "DALLAS, TX", 32.8998f, -97.0403f},
  {"IAH", "KIAH", "GEORGE BUSH INTERCONT", "HOUSTON, TX", 29.9902f, -95.3368f},
  {"HOU", "KHOU", "HOUSTON HOBBY", "HOUSTON, TX", 29.6454f, -95.2789f},
  {"AUS", "KAUS", "AUSTIN-BERGSTROM", "AUSTIN, TX", 30.1945f, -97.6699f},
  {"SAT", "KSAT", "SAN ANTONIO INTL", "SAN ANTONIO, TX", 29.5337f, -98.4698f},
  // West
  {"DEN", "KDEN", "DENVER INTL", "DENVER, CO", 39.8561f, -104.6737f},
  {"SLC", "KSLC", "SALT LAKE CITY INTL", "SALT LAKE CITY, UT", 40.7899f, -111.9791f},
  {"PHX", "KPHX", "PHOENIX SKY HARBOR", "PHOENIX, AZ", 33.4342f, -112.0116f},
  {"LAS", "KLAS", "HARRY REID INTL", "LAS VEGAS, NV", 36.0840f, -115.1537f},
  {"LAX", "KLAX", "LOS ANGELES INTL", "LOS ANGELES, CA", 33.9416f, -118.4085f},
  {"SAN", "KSAN", "SAN DIEGO INTL", "SAN DIEGO, CA", 32.7336f, -117.1897f},
  {"SFO", "KSFO", "SAN FRANCISCO INTL", "SAN FRANCISCO, CA", 37.6213f, -122.3790f},
  {"SJC", "KSJC", "MINETA SAN JOSE", "SAN JOSE, CA", 37.3626f, -121.9290f},
  {"OAK", "KOAK", "OAKLAND INTL", "OAKLAND, CA", 37.7126f, -122.2197f},
  {"SEA", "KSEA", "SEATTLE-TACOMA", "SEATTLE, WA", 47.4502f, -122.3088f},
  {"PDX", "KPDX", "PORTLAND INTL", "PORTLAND, OR", 45.5898f, -122.5951f},
  {"ANC", "PANC", "TED STEVENS ANCHORAGE", "ANCHORAGE, AK", 61.1743f, -149.9983f},
  {"HNL", "PHNL", "DANIEL K INOUYE", "HONOLULU, HI", 21.3187f, -157.9225f}
};
const int NUM_AIRPORTS = sizeof(AIRPORTS) / sizeof(AIRPORTS[0]);

int16_t apSx[NUM_AIRPORTS];                    // v11: where each star was drawn this frame (for tap hit-testing)
int16_t apSy[NUM_AIRPORTS];
bool    apVis[NUM_AIRPORTS];

void drawStarToCanvas(int cx, int cy, int r, uint16_t col) {
  int px[5], py[5];
  for (int k = 0; k < 5; k++) {
    float a = (-90 + 72 * k) * DEG_TO_RAD;
    px[k] = cx + cosf(a) * r;
    py[k] = cy + sinf(a) * r;
  }
  for (int k = 0; k < 5; k++) {
    int n = (k + 2) % 5;                        // every second point makes the star
    radarCanvas.drawLine(px[k], py[k], px[n], py[n], col);
  }
  radarCanvas.fillCircle(cx, cy, 1, col);
}

// A star and the airport code for every airport inside the current range (drawn under the planes)
void drawAirportsToCanvas() {
  uint16_t col = radarCanvas.color565(190, 150, 255);
  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(1);
  for (int a = 0; a < NUM_AIRPORTS; a++) {
    apVis[a] = false;
    float d = distanceKm(homeLat, homeLon, AIRPORTS[a].lat, AIRPORTS[a].lon);
    if (d > rangeKm) continue;
    float b = bearingDeg(homeLat, homeLon, AIRPORTS[a].lat, AIRPORTS[a].lon);
    float rr = (d / (float)rangeKm) * R;
    float rad = (b - 90) * DEG_TO_RAD;
    int x = CX + cos(rad) * rxCorr(rr);
    int y = CY + sin(rad) * rr;
    apVis[a] = true;
    apSx[a] = x;
    apSy[a] = y;
    bool sel = (menuMode == MENU_AIRPORT && a == apSel);       // v11: the airport on the info page is drawn in white with a ring
    uint16_t c = sel ? (uint16_t)TFT_WHITE : col;
    drawStarToCanvas(x, y, 6, c);
    if (sel) radarCanvas.drawCircle(x, y, 11, c);
    radarCanvas.setTextColor(c);
    radarCanvas.drawString(AIRPORTS[a].code, x + 9, y - 4);
  }
}

// ---------- v11: shared download helper ----------
// GET a URL into a buffer (PSRAM). Returns the HTTP code (200 = the whole body arrived).
int httpGetBuffered(const String& url, uint8_t* buf, size_t cap, size_t* gotOut) {
  *gotOut = 0;
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(15000);
  http.setUserAgent(NEWJOB_USER_AGENT);
  http.useHTTP10(true);

  if (!http.begin(client, url)) return -1;
  int code = http.GET();
  if (code != 200) {
    http.end();
    return code;
  }

  int expected = http.getSize();                    // -1 if unknown
  if (expected > 0 && (size_t)expected > cap) {
    http.end();
    return -3;                                      // too big for the buffer
  }

  WiFiClient* stream = http.getStreamPtr();
  size_t got = 0;
  unsigned long lastData = millis();
  while ((http.connected() || stream->available()) && got < cap && millis() - lastData < 10000) {
    int avail = stream->available();
    if (avail > 0) {
      size_t want = min((size_t)avail, cap - got);
      got += stream->readBytes(buf + got, want);
      lastData = millis();
    } else {
      if (expected > 0 && got >= (size_t)expected) break;
      delay(5);
    }
  }
  http.end();
  *gotOut = got;
  if (expected > 0 && got < (size_t)expected) return -4;     // cut short
  return (got > 0) ? 200 : -5;
}

// ---------- v11: TERRAIN (topographic contour lines from AWS elevation tiles, no API key) ----------
float terrainMppTile(int z, float lat) {
  return 156543.03392f * cosf(lat * DEG_TO_RAD) / (float)(1 << z);       // metres per tile pixel
}

bool terrainKeyIsBuilt() {
  return terrainReady && terrainBuiltRange == (int)rangeKm &&
         fabsf(terrainBuiltLat - (float)homeLat) < 0.00001f && fabsf(terrainBuiltLon - (float)homeLon) < 0.00001f;
}

bool terrainLoading() {
  return terrainOn && !terrainGaveUp && !terrainKeyIsBuilt() && WiFi.status() == WL_CONNECTED;
}

// Position of a point in "global tile pixels" at a zoom level (Web Mercator)
void terrainCenterPx(float lat, float lon, int z, double* X, double* Y) {
  double n = (double)(1 << z);
  double latr = lat * M_PI / 180.0;
  *X = (lon + 180.0) / 360.0 * 256.0 * n;
  *Y = (1.0 - log(tan(latr) + 1.0 / cos(latr)) / M_PI) / 2.0 * 256.0 * n;
}

// Which tiles cover the radar disc for terrainPlan*: zoom level and tile range (at most TERRAIN_CACHE tiles)
bool terrainMakePlan() {
  float lat = terrainPlanLat, lon = terrainPlanLon;
  int rk = terrainPlanRange;
  if (fabsf(lat) > 80.0f) return false;

  float mppV = rk * 1000.0f / R;                                          // metres per radar pixel
  float zf = log2f(156543.03392f * cosf(lat * DEG_TO_RAD) / mppV);
  int z = (int)floorf(zf + 0.5f);
  if (z > 12) z = 12;
  if (z < 3) z = 3;

  for (;;) {
    double Xc, Yc;
    terrainCenterPx(lat, lon, z, &Xc, &Yc);
    double hw = rk * 1000.0 / terrainMppTile(z, lat);                     // radar radius in tile pixels
    int tx0 = (int)floor((Xc - hw) / 256.0), tx1 = (int)floor((Xc + hw) / 256.0);
    int ty0 = (int)floor((Yc - hw) / 256.0), ty1 = (int)floor((Yc + hw) / 256.0);
    int nx = tx1 - tx0 + 1, ny = ty1 - ty0 + 1;
    int lim = 1 << z;
    if (tx0 < 0 || ty0 < 0 || tx1 >= lim || ty1 >= lim) return false;     // off the edge of the map
    if (nx * ny > TERRAIN_CACHE) {
      if (z > 3) { z--; continue; }                                       // coarser tiles cover more ground
      return false;
    }
    terrainZ = z;
    terrainTx0 = tx0;
    terrainTy0 = ty0;
    terrainNx = nx;
    terrainNy = ny;
    return true;
  }
}

int tileFind(int z, int x, int y) {
  for (int i = 0; i < TERRAIN_CACHE; i++) {
    if (tileCache[i].elev && tileCache[i].z == z && tileCache[i].x == x && tileCache[i].y == y) return i;
  }
  return -1;
}

bool tileInPlan(int i) {
  return tileCache[i].elev && tileCache[i].z == terrainZ &&
         tileCache[i].x >= terrainTx0 && tileCache[i].x < terrainTx0 + terrainNx &&
         tileCache[i].y >= terrainTy0 && tileCache[i].y < terrainTy0 + terrainNy;
}

// A slot to load a tile into: an unused one, else the oldest one the current plan does not need
int tileAllocSlot() {
  for (int i = 0; i < TERRAIN_CACHE; i++) {
    if (!tileCache[i].elev) {
      tileCache[i].elev = (int16_t*)ps_malloc(256 * 256 * sizeof(int16_t));
      if (!tileCache[i].elev) continue;
      tileCache[i].z = -1;
      return i;
    }
  }
  int best = -1;
  for (int i = 0; i < TERRAIN_CACHE; i++) {
    if (tileCache[i].z >= 0 && tileInPlan(i)) continue;
    if (best < 0 || tileCache[i].used < tileCache[best].used) best = i;
  }
  return best;
}

// PNG -> metres. Terrarium tiles store the height in the colour: metres = R * 256 + G + B / 256 - 32768
bool terrainDecodeTile(const uint8_t* data, size_t len, int16_t* dst) {
  if (!tileSpriteOk) {
    tileSprite.setPsram(true);
    tileSprite.setColorDepth(24);
    tileSpriteOk = tileSprite.createSprite(256, 256);
    if (!tileSpriteOk) return false;
  }
  tileSprite.fillSprite(TFT_BLACK);
  if (!tileSprite.drawPng(data, (uint32_t)len, 0, 0)) return false;
  const uint8_t* b = (const uint8_t*)tileSprite.getBuffer();
  if (!b) return false;

  // The sprite keeps the three bytes of a pixel in RGB or BGR order depending on the library version.
  // Try both on a sample of the tile and keep the order that gives sane heights.
  int okA = 0, okB = 0;
  for (int i = 0; i < 256 * 256; i += 97) {
    const uint8_t* p = b + i * 3;
    float a = p[0] * 256.0f + p[1] - 32768.0f;
    float c = p[2] * 256.0f + p[1] - 32768.0f;
    if (a > -1000.0f && a < 9000.0f) okA++;
    if (c > -1000.0f && c < 9000.0f) okB++;
  }
  bool rFirst = (okA >= okB);

  for (int i = 0; i < 256 * 256; i++) {
    const uint8_t* p = b + i * 3;
    int hi = rFirst ? p[0] : p[2];
    int lo = rFirst ? p[2] : p[0];
    float m = hi * 256.0f + p[1] + lo / 256.0f - 32768.0f;
    if (m < -1000.0f) m = -1000.0f;
    if (m > 9000.0f) m = 9000.0f;
    dst[i] = (int16_t)lroundf(m);
  }
  return true;
}

bool terrainFetchTile(int z, int x, int y, int slot) {
  static uint8_t* pngBuf = nullptr;
  const size_t PNG_CAP = 300000;
  if (!pngBuf) pngBuf = (uint8_t*)ps_malloc(PNG_CAP);
  if (!pngBuf) return false;

  tileCache[slot].z = -1;
  String url = String("https://s3.amazonaws.com/elevation-tiles-prod/terrarium/") + z + "/" + x + "/" + y + ".png";
  size_t got = 0;
  int code = httpGetBuffered(url, pngBuf, PNG_CAP, &got);
  Serial.printf("Terrain tile %d/%d/%d -> HTTP %d, %u bytes\n", z, x, y, code, (unsigned)got);
  if (code != 200) return false;
  if (!terrainDecodeTile(pngBuf, got, tileCache[slot].elev)) {
    Serial.println("Terrain tile decode failed");
    return false;
  }
  tileCache[slot].z = z;
  tileCache[slot].x = x;
  tileCache[slot].y = y;
  tileCache[slot].used = millis();
  return true;
}

// Heights for every radar pixel -> contour lines (every 5th one brighter) on a faint brown tint, water dark blue.
// No numbers or labels on purpose: the radar is busy enough.
bool terrainRender() {
  if (!terrainSpriteOk) {
    terrainSprite.setPsram(true);
    terrainSprite.setColorDepth(16);
    terrainSpriteOk = terrainSprite.createSprite(480, 480);
  }
  if (!terrainSpriteOk) return false;
  if (!terrainTopOk) {
    terrainTop.setPsram(true);
    terrainTop.setColorDepth(16);
    terrainTopOk = terrainTop.createSprite(480, 480);
  }
  if (!terrainTopOk) return false;
  if (!terrainGrid) terrainGrid = (int16_t*)ps_malloc(480 * 480 * sizeof(int16_t));
  if (!terrainGrid) return false;

  const int16_t NO_ELEV = -32768;
  for (int i = 0; i < 480 * 480; i++) terrainGrid[i] = NO_ELEV;

  int z = terrainRnZ;
  int rk = terrainRnRange;
  float lat = terrainRnLat, lon = terrainRnLon;
  double Xc, Yc;
  terrainCenterPx(lat, lon, z, &Xc, &Yc);
  float mppT = terrainMppTile(z, lat);
  float sx = (rk * 1000.0f / (R * X_CORR)) / mppT;      // tile pixels per radar pixel, sideways
  float sy = (rk * 1000.0f / R) / mppT;                 // ...and up / down
  int rx = rxCorr(R);

  int lastTx = -99999, lastTy = -99999, lastSlot = -1;
  int minLand = 32000, maxLand = -32000;

  // 1) height under every pixel of the radar disc (bilinear from the tiles)
  for (int py = CY - R; py <= CY + R; py++) {
    if (py < 0 || py >= 480) continue;
    float ey = (py - CY) / (float)R;
    for (int px = CX - rx; px <= CX + rx; px++) {
      if (px < 0 || px >= 480) continue;
      float ex = (px - CX) / (float)rx;
      if (ex * ex + ey * ey > 1.0f) continue;

      double X = Xc + (px - CX) * sx;
      double Y = Yc + (py - CY) * sy;
      int tx = (int)floor(X / 256.0), ty = (int)floor(Y / 256.0);
      if (tx != lastTx || ty != lastTy) {
        lastTx = tx;
        lastTy = ty;
        lastSlot = tileFind(z, tx, ty);
      }
      if (lastSlot < 0) continue;

      float u = (float)(X - tx * 256.0) - 0.5f;
      float v = (float)(Y - ty * 256.0) - 0.5f;
      int u0 = (int)floorf(u), v0 = (int)floorf(v);
      float fu = u - u0, fv = v - v0;
      int u1 = u0 + 1, v1 = v0 + 1;
      if (u0 < 0) u0 = 0;
      if (u0 > 255) u0 = 255;
      if (u1 < 0) u1 = 0;
      if (u1 > 255) u1 = 255;
      if (v0 < 0) v0 = 0;
      if (v0 > 255) v0 = 255;
      if (v1 < 0) v1 = 0;
      if (v1 > 255) v1 = 255;

      const int16_t* t = tileCache[lastSlot].elev;
      float e = ((t[v0 * 256 + u0] * (1 - fu) + t[v0 * 256 + u1] * fu) * (1 - fv) +
                 (t[v1 * 256 + u0] * (1 - fu) + t[v1 * 256 + u1] * fu) * fv);
      int ei = (int)lroundf(e);
      terrainGrid[py * 480 + px] = (int16_t)ei;
      if (ei > 0) {
        if (ei < minLand) minLand = ei;
        if (ei > maxLand) maxLand = ei;
      }
    }
  }

  // 2) contour spacing from the local relief (about a dozen lines across the view)
  if (maxLand < minLand) { minLand = 0; maxLand = 0; }
  int relief = maxLand - (minLand < 0 ? 0 : minLand);
  static const int steps[] = {5, 10, 20, 25, 50, 100, 200, 250, 500, 1000};
  int interval = 1000;
  for (int k = 0; k < 10; k++) {
    if (steps[k] * 12 >= relief) { interval = steps[k]; break; }
  }

  uint16_t lineCol  = (uint16_t)terrainSprite.color565(122, 86, 38);      // contour line (tan)
  uint16_t idxCol   = (uint16_t)terrainSprite.color565(184, 132, 62);     // every 5th contour
  uint16_t shoreCol = (uint16_t)terrainSprite.color565(30, 105, 165);     // shoreline
  uint16_t waterCol = (uint16_t)terrainSprite.color565(0, 22, 58);        // sea level and below

  // 3) draw
  terrainSprite.fillSprite(TFT_BLACK);
  terrainTop.fillSprite(TFT_BLACK);                                      // black = see-through
  for (int py = CY - R; py <= CY + R; py++) {
    if (py < 0 || py >= 480) continue;
    for (int px = CX - rx; px <= CX + rx; px++) {
      if (px < 0 || px >= 480) continue;
      int e = terrainGrid[py * 480 + px];
      if (e == NO_ELEV) continue;

      uint16_t col;
      bool top = false;                                                 // true = drawn over the sweep too
      if (e <= 0) {                                                     // water
        top = true;
        col = waterCol;
        for (int k = 0; k < 2; k++) {
          int nx2 = px + (k == 0 ? 1 : 0), ny2 = py + (k == 1 ? 1 : 0);
          if (nx2 >= 480 || ny2 >= 480) continue;
          int en = terrainGrid[ny2 * 480 + nx2];
          if (en != NO_ELEV && en > 0) col = shoreCol;
        }
      } else {                                                          // land
        float t = (relief > 0) ? (e - minLand) / (float)relief : 0.0f;
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        col = (uint16_t)terrainSprite.color565(14 + (int)(t * 32), 9 + (int)(t * 21), 4 + (int)(t * 8));

        int b = e / interval;
        bool line = false, index = false, shore = false;
        for (int k = 0; k < 2; k++) {
          int nx2 = px + (k == 0 ? 1 : 0), ny2 = py + (k == 1 ? 1 : 0);
          if (nx2 >= 480 || ny2 >= 480) continue;
          int en = terrainGrid[ny2 * 480 + nx2];
          if (en == NO_ELEV) continue;
          if (en <= 0) { shore = true; continue; }
          int b2 = en / interval;
          if (b2 != b) {
            line = true;
            int hi = (b > b2) ? b : b2;
            int lo = (b > b2) ? b2 : b;
            if (hi / 5 > lo / 5) index = true;
          }
        }
        if (shore) col = shoreCol;
        else if (index) col = idxCol;
        else if (line) col = lineCol;
        if (shore || index || line) top = true;
      }
      terrainSprite.drawPixel(px, py, col);
      if (top) terrainTop.drawPixel(px, py, col);
    }
  }

  free(terrainGrid);                                  // 460 KB back to PSRAM until the next map
  terrainGrid = nullptr;

  terrainBuiltLat = lat;
  terrainBuiltLon = lon;
  terrainBuiltRange = rk;
  terrainReady = true;
  Serial.printf("Terrain map ready: zoom %d, relief %d m, contour every %d m\n", z, relief, interval);
  return true;
}

void terrainTask(void* parameter) {
  bool ok;
  if (terrainAction == 1) ok = terrainFetchTile(terrainActZ, terrainActX, terrainActY, terrainActSlot);
  else                    ok = terrainRender();

  if (ok) {
    terrainFails = 0;
  } else {
    terrainFails = terrainFails + 1;
    if (terrainFails >= 3) {
      terrainGaveUp = true;                     // stop trying until the location or zoom changes
      Serial.println("Terrain: giving up for this view");
    } else {
      terrainRetryAt = millis() + 6000UL * (unsigned long)terrainFails;
    }
  }
  terrainTaskHandle = NULL;
  terrainInProgress = false;
  vTaskDelete(NULL);
}

void startTerrainTask(int action) {
  terrainAction = action;
  terrainInProgress = true;
  terrainStartMs = millis();
  BaseType_t ok = xTaskCreatePinnedToCore(terrainTask, "terrainTask", 20480, NULL, 1, &terrainTaskHandle, 0);
  if (ok != pdPASS) {
    terrainTaskHandle = NULL;
    terrainInProgress = false;
    terrainFails = terrainFails + 1;
    terrainRetryAt = millis() + 5000UL;
  }
}

// The sweep is drawn over the terrain, so put the terrain's lines, shoreline and water back on top inside the
// sweep's area (a box around the trail, not the whole radar, to keep the frame fast).
void terrainOverlayOnSweep() {
  if (!terrainOn || !terrainKeyIsBuilt() || !terrainTopOk) return;

  float a0 = sweepAngle - TRAIL_WIDTH;
  float a1 = sweepAngle;
  int rxs = rxCorr(R);
  int minX = CX, maxX = CX, minY = CY, maxY = CY;

  auto grow = [&](float a) {
    float rad = (a - 90) * DEG_TO_RAD;
    int x = CX + (int)lroundf(cosf(rad) * rxs);
    int y = CY + (int)lroundf(sinf(rad) * R);
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  };
  grow(a0);
  grow(a1);
  for (int k = (int)ceilf(a0 / 90.0f); k * 90 <= (int)a1; k++) grow(k * 90.0f);   // the trail crosses N / E / S / W

  minX -= 3; minY -= 3; maxX += 3; maxY += 3;
  if (minX < 0) minX = 0;
  if (minY < 0) minY = 0;
  if (maxX > 479) maxX = 479;
  if (maxY > 479) maxY = 479;

  radarCanvas.setClipRect(minX, minY, maxX - minX + 1, maxY - minY + 1);
  terrainTop.pushSprite(&radarCanvas, 0, 0, TFT_BLACK);       // black pixels are skipped
  radarCanvas.clearClipRect();
}

// Runs every radar loop. Waits until the zoom / location stops changing, then fetches the missing tiles one at a time
// (a plane update never has to wait for more than one tile), then draws the map.
void serviceTerrain(bool allowNet) {
  if (!terrainOn) return;

  float la = homeLat, lo = homeLon;
  int rk = rangeKm;
  if (la != terrainWantLat || lo != terrainWantLon || rk != terrainWantRange) {
    terrainWantLat = la;
    terrainWantLon = lo;
    terrainWantRange = rk;
    terrainWantChangedMs = millis();
    terrainGaveUp = false;
    terrainFails = 0;
    terrainRetryAt = 0;
    terrainPlanOk = false;
  }

  if (terrainKeyIsBuilt() || terrainGaveUp) return;
  if (!allowNet || terrainInProgress) return;
  if (millis() - terrainWantChangedMs < 1500UL || millis() < terrainRetryAt) return;
  if (fetchInProgress || lookupInProgress || geoBusy() || wxInProgress || photoInProgress || metarInProgress) return;
  if (photoWanted || metarWanted) return;                 // things you tapped for come first
  if (WiFi.status() != WL_CONNECTED) return;

  if (!terrainPlanOk) {
    terrainPlanLat = la;
    terrainPlanLon = lo;
    terrainPlanRange = rk;
    terrainPlanOk = terrainMakePlan();
    if (!terrainPlanOk) {
      terrainGaveUp = true;
      Serial.println("Terrain: no map for this view");
      return;
    }
    Serial.printf("Terrain plan: zoom %d, %d x %d tiles\n", terrainZ, terrainNx, terrainNy);
  }

  for (int ty = 0; ty < terrainNy; ty++) {
    for (int tx = 0; tx < terrainNx; tx++) {
      int x = terrainTx0 + tx, y = terrainTy0 + ty;
      if (tileFind(terrainZ, x, y) >= 0) continue;
      int slot = tileAllocSlot();
      if (slot < 0) {
        terrainGaveUp = true;
        return;
      }
      terrainActZ = terrainZ;
      terrainActX = x;
      terrainActY = y;
      terrainActSlot = slot;
      startTerrainTask(1);
      return;
    }
  }

  // every tile is here: draw the map
  terrainRnZ = terrainZ;
  terrainRnRange = terrainPlanRange;
  terrainRnLat = terrainPlanLat;
  terrainRnLon = terrainPlanLon;
  terrainReady = false;
  startTerrainTask(2);
}

// ---------- v11: AIRCRAFT PHOTO (Planespotters) ----------
const int CAM_W = 42, CAM_H = 26;
int camX() { return DETAIL_X + DETAIL_W - CAM_W; }
int camY() { return DETAIL_Y + 75; }

// A little camera at the right edge of the route row (the SMALL / HELI / MIL tag sits above it and does not move)
void drawCamIcon(bool on) {
  int x = camX(), y = camY();
  uint16_t col = on ? greenBright : green;
  lcd.fillRect(x, y, CAM_W, CAM_H, on ? greenDark : TFT_BLACK);
  lcd.drawRect(x, y, CAM_W, CAM_H, col);
  int cx = x + CAM_W / 2, cy = y + CAM_H / 2 + 1;
  lcd.drawRoundRect(cx - 12, cy - 7, 24, 15, 2, col);
  lcd.fillRect(cx - 5, cy - 10, 10, 3, col);
  lcd.drawCircle(cx, cy, 4, col);
  lcd.fillCircle(cx, cy, 1, col);
}

// Size of a JPEG without decoding it. progressive = true for files the decoder can't draw.
bool jpegInfo(const uint8_t* b, size_t n, int* w, int* h, bool* progressive) {
  *progressive = false;
  if (n < 4 || b[0] != 0xFF || b[1] != 0xD8) return false;
  size_t i = 2;
  while (i + 9 < n) {
    if (b[i] != 0xFF) { i++; continue; }
    uint8_t m = b[i + 1];
    if (m == 0xFF) { i++; continue; }
    if (m == 0xD8 || m == 0x01 || (m >= 0xD0 && m <= 0xD7)) { i += 2; continue; }
    if (m >= 0xC0 && m <= 0xCF && m != 0xC4 && m != 0xC8 && m != 0xCC) {
      *h = (b[i + 5] << 8) | b[i + 6];
      *w = (b[i + 7] << 8) | b[i + 8];
      *progressive = (m == 0xC2);
      return (*w > 0 && *h > 0);
    }
    size_t len = (b[i + 2] << 8) | b[i + 3];
    i += 2 + len;
  }
  return false;
}

// Plain ASCII for the built-in font, keeping upper / lower case (photographer names)
void toAsciiKeepCase(const char* in, char* out, size_t n) {
  static const char LATIN1[] = "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTsaaaaaaaceeeeiiiidnooooo/ouuuuyty";
  size_t o = 0;
  for (size_t i = 0; in[i] && o < n - 1; i++) {
    unsigned char c = (unsigned char)in[i];
    unsigned char c2 = (unsigned char)in[i + 1];
    if (c >= 0x20 && c < 0x7F) {
      out[o++] = (char)c;
    } else if (c == 0xC3 && c2 >= 0x80 && c2 <= 0xBF) {
      out[o++] = LATIN1[c2 - 0x80];
      i++;
    }
  }
  out[o] = '\0';
}

// Looks the plane up on Planespotters by its hex id, then downloads the small photo into RAM (nothing is saved)
void photoRun() {
  char hex[8];
  copyStr(hex, sizeof(hex), photoRunHex);
  for (char* c = hex; *c; c++) *c = toupper(*c);

  int st = 4;
  char who[48] = "";
  size_t len = 0;

  if (!photoBuf) photoBuf = (uint8_t*)ps_malloc(PHOTO_CAP);
  if (photoBuf) {
    size_t got = 0;
    String url = String("https://api.planespotters.net/pub/photos/hex/") + hex;
    int code = httpGetBuffered(url, photoBuf, PHOTO_CAP, &got);
    Serial.printf("Photo lookup %s -> HTTP %d (%u bytes)\n", hex, code, (unsigned)got);

    if (code == 404) {
      st = 3;
    } else if (code == 200) {
      StaticJsonDocument<256> filter;
      filter["photos"][0]["thumbnail"]["src"] = true;
      filter["photos"][0]["photographer"] = true;
      DynamicJsonDocument doc(2048);
      DeserializationError err = deserializeJson(doc, (const char*)photoBuf, got, DeserializationOption::Filter(filter));
      if (!err) {
        JsonObject p0 = doc["photos"][0];
        const char* src = p0["thumbnail"]["src"] | "";
        if (!src[0]) {
          st = 3;                                              // no photo on file for this aircraft
        } else {
          String imgUrl = String(src);
          toAsciiKeepCase(p0["photographer"] | "", who, sizeof(who));

          int w = 0, h = 0;
          bool prog = false;
          code = httpGetBuffered(imgUrl, photoBuf, PHOTO_CAP, &got);
          Serial.printf("Photo download -> HTTP %d (%u bytes)\n", code, (unsigned)got);
          if (code == 200 && jpegInfo(photoBuf, got, &w, &h, &prog) && !prog) {
            len = got;
            st = 2;
          }
        }
      }
    }
  }

  if (strcmp(photoRunHex, photoHex) == 0) {                    // still the plane you asked about
    copyStr(photoWho, sizeof(photoWho), who);
    photoLen = len;
    photoState = st;
  }
}

void photoTask(void* parameter) {
  photoRun();
  photoTaskHandle = NULL;
  photoInProgress = false;
  photoDone = true;
  vTaskDelete(NULL);
}

void maybeStartPhoto() {
  if (!photoWanted || photoInProgress) return;
  if (fetchInProgress || lookupInProgress || geoBusy() || wxInProgress || metarInProgress || terrainInProgress) return;
  photoWanted = false;
  if (WiFi.status() != WL_CONNECTED) {
    photoState = 4;
    photoDone = true;
    return;
  }
  copyStr(photoRunHex, sizeof(photoRunHex), photoHex);
  photoInProgress = true;
  photoStartMs = millis();
  BaseType_t ok = xTaskCreatePinnedToCore(photoTask, "photoTask", 16384, NULL, 1, &photoTaskHandle, 0);
  if (ok != pdPASS) {
    photoTaskHandle = NULL;
    photoInProgress = false;
    photoState = 4;
    photoDone = true;
  }
}

// The photo fills the bottom of the sidebar (over the location and time panels)
void drawPhotoZone() {
  if (!photoMode) return;
  int zx = INFO_X + 2, zy = INFO_Y + 298, zw = INFO_W - 4, zh = INFO_H - 301;
  int cx = INFO_X + INFO_W / 2;
  lcd.fillRect(zx, zy, zw, zh, TFT_BLACK);
  lcd.setTextDatum(middle_center);

  bool drawn = false;
  if (photoState == 2 && photoBuf && photoLen > 0) {
    int w = 0, h = 0;
    bool prog = false;
    if (jpegInfo(photoBuf, photoLen, &w, &h, &prog) && !prog) {
      float sc = 1.0f;
      if (w > zw - 10 || h > 140) sc = fminf((zw - 10) / (float)w, 140.0f / (float)h);
      int dw = (int)(w * sc), dh = (int)(h * sc);
      int ix = zx + (zw - dw) / 2;
      int iy = zy + 4 + (140 - dh) / 2;
      if (lcd.drawJpg(photoBuf, (uint32_t)photoLen, ix, iy, 0, 0, 0, 0, sc, sc)) {
        lcd.drawRect(ix - 1, iy - 1, dw + 2, dh + 2, greenDark);

        char line[60];
        snprintf(line, sizeof(line), "PHOTO: %s", photoWho[0] ? photoWho : "UNKNOWN");
        line[47] = '\0';
        lcd.setTextDatum(top_center);
        lcd.setTextSize(1);
        lcd.setTextColor(greenBright, TFT_BLACK);
        lcd.drawString(line, cx, INFO_Y + 446);
        lcd.setTextColor(greenDim, TFT_BLACK);
        lcd.drawString("PLANESPOTTERS.NET", cx, INFO_Y + 457);
        drawn = true;
      }
    }
  }

  if (!drawn) {
    lcd.setTextDatum(middle_center);
    lcd.setTextSize(2);
    lcd.setTextColor(greenDim, TFT_BLACK);
    if (photoState == 0 || photoState == 1) {
      lcd.drawString("LOADING PHOTO...", cx, zy + 70);
    } else if (photoState == 3) {
      lcd.drawString("NO PHOTO ON FILE", cx, zy + 62);
      lcd.setTextSize(1);
      lcd.drawString("FOR THIS AIRCRAFT", cx, zy + 86);
    } else {
      lcd.drawString("PHOTO UNAVAILABLE", cx, zy + 62);
      lcd.setTextSize(1);
      lcd.drawString("TAP THE CAMERA TO RETRY", cx, zy + 86);
    }
  }
  lcd.setTextDatum(top_left);
}

// Puts the photo away and brings the location and time panels back
void photoHide() {
  if (!photoMode) return;
  photoMode = false;
  drawLocationPanel();
  drawTimePanel();
}

void photoToggle() {
  if (!hasSelection || !selSnap.hex[0]) return;
  if (photoMode) {
    photoHide();
    drawInfoTop();                                             // the camera icon goes back to normal
    return;
  }

  bool same = (strcmp(photoHex, selSnap.hex) == 0);
  if (!same || photoState == 0 || photoState == 4) {           // new plane (or last try failed): ask again
    copyStr(photoHex, sizeof(photoHex), selSnap.hex);
    photoState = 1;
    photoLen = 0;
    photoWanted = true;
  }
  photoMode = true;
  drawPhotoZone();
  drawInfoTop();                                               // highlight the camera icon
}

// ---------- v11: AIRPORT INFO PAGE ----------
const char* compass8(float b) {
  static const char* n[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return n[((int)((b + 22.5f) / 45.0f)) % 8];
}

// The airport star closest to a tap (within about 26 px), or -1
int airportAt(int x, int y) {
  int best = -1;
  int bestD2 = 26 * 26;
  for (int a = 0; a < NUM_AIRPORTS; a++) {
    if (!apVis[a]) continue;
    int dx = x - apSx[a], dy = y - apSy[a];
    int d2 = dx * dx + dy * dy;
    if (d2 < bestD2) { bestD2 = d2; best = a; }
  }
  return best;
}

// Airports inside the current range, nearest first (what the arrows on the airport page step through)
int   apListN = 0;
int   apListIdx[NUM_AIRPORTS];
float apListDist[NUM_AIRPORTS];

void rebuildAirportList() {
  apListN = 0;
  for (int a = 0; a < NUM_AIRPORTS; a++) {
    float d = distanceKm(homeLat, homeLon, AIRPORTS[a].lat, AIRPORTS[a].lon);
    if (d > rangeKm) continue;
    int p = apListN++;
    while (p > 0 && apListDist[p - 1] > d) {                 // insertion sort
      apListDist[p] = apListDist[p - 1];
      apListIdx[p] = apListIdx[p - 1];
      p--;
    }
    apListDist[p] = d;
    apListIdx[p] = a;
  }
}

int apListPos(int a) {
  for (int i = 0; i < apListN; i++) {
    if (apListIdx[i] == a) return i;
  }
  return -1;
}

// Ask for a weather report unless we have a fresh one (a report under 10 minutes old is reused)
void apRequestWeather(int a) {
  bool fresh = (mtAirport == a) && (mtState == 1 || ((mtState == 2 || mtState == 3) && millis() - mtWhen < 600000UL));
  if (!fresh) {
    mtAirport = a;
    mtState = 1;
    metarWanted = true;
  }
}

// Tap on an airport star on the radar
void openAirportPanel(int a) {
  photoHide();
  hasSelection = false;                          // the airport page takes over the detail panel's spot
  selectedHex[0] = '\0';
  rebuildAirportList();
  apSel = a;
  menuMode = MENU_AIRPORT;
  favMode = 0;
  apRecenter = 0;
  apRequestWeather(a);
  drawInfoTop();
}

// The airport button in the top right corner of the list screen: start with the nearest airport
void openAirportBrowse() {
  photoHide();
  hasSelection = false;
  selectedHex[0] = '\0';
  rebuildAirportList();
  apSel = (apListN > 0) ? apListIdx[0] : -1;
  menuMode = MENU_AIRPORT;
  favMode = 0;
  apRecenter = 0;
  if (apSel >= 0) apRequestWeather(apSel);
  drawInfoTop();
}

void stepAirport(int dir) {
  rebuildAirportList();
  if (apListN == 0) {
    apSel = -1;
    drawInfoTop();
    return;
  }
  int pos = apListPos(apSel);                    // -1 if that airport left the range
  if (pos < 0) pos = (dir > 0) ? 0 : apListN - 1;
  else         pos = (pos + dir + apListN) % apListN;
  apSel = apListIdx[pos];
  apRecenter = 0;
  apRequestWeather(apSel);
  drawInfoTop();
}

// CENTER button: move the whole radar to this airport (same place + time zone lookup as the custom location screen)
void recenterOnAirport() {
  if (apSel < 0 || apSel >= NUM_AIRPORTS || geoBusy()) return;
  const Airport& ap = AIRPORTS[apSel];
  geoReqLat = ap.lat;
  geoReqLon = ap.lon;
  geoTzOnly = false;
  geoSunOnly = false;
  geoRecenter = true;
  copyStr(geoRecenterName, sizeof(geoRecenterName), ap.name);
  geoState = GEO_WAITING;
  apRecenter = 1;
  drawInfoTop();
}

void handleAirportTap(int x, int y) {
  int x0 = DETAIL_X;
  if (apSel >= 0 && x >= x0 + 134 && x <= x0 + 228 && y >= DETAIL_Y + 2 && y <= DETAIL_Y + 42) {   // CENTER
    recenterOnAirport();
    return;
  }
  if (y >= NAV_Y - 6 && y <= NAV_Y + NAV_H + 6) {                                                  // prev / next
    if (x >= NAV_PREV_X - 6 && x <= NAV_PREV_X + NAV_W + 6) { stepAirport(-1); return; }
    if (x >= NAV_NEXT_X - 6 && x <= NAV_NEXT_X + NAV_W + 6) { stepAirport(+1); return; }
  }
}

// A little airplane (purple, like the stars on the radar) where the X sits when a panel is open
void drawAirportBtn() {
  int cx = CLOSE_X + CLOSE_W / 2, cy = CLOSE_Y + CLOSE_H / 2;
  uint16_t col = lcd.color565(190, 150, 255);
  lcd.fillRect(CLOSE_X, CLOSE_Y, CLOSE_W, CLOSE_H, TFT_BLACK);
  lcd.fillRect(cx - 2, cy - 12, 5, 26, col);                                  // body
  lcd.fillTriangle(cx - 2, cy - 12, cx + 2, cy - 12, cx, cy - 17, col);       // nose
  lcd.fillTriangle(cx - 2, cy - 3, cx - 16, cy + 8, cx - 2, cy + 5, col);     // wings
  lcd.fillTriangle(cx + 2, cy - 3, cx + 16, cy + 8, cx + 2, cy + 5, col);
  lcd.fillTriangle(cx - 2, cy + 10, cx - 8, cy + 16, cx - 2, cy + 14, col);   // tail
  lcd.fillTriangle(cx + 2, cy + 10, cx + 8, cy + 16, cx + 2, cy + 14, col);
}

void metarRun() {
  int ap = mtRunAirport;
  static uint8_t* buf = nullptr;
  const size_t CAP = 8191;
  if (!buf) buf = (uint8_t*)ps_malloc(CAP + 1);

  int st = 4;
  char cat[6] = "";
  float tempC = NAN;
  int wdir = -1, wspd = -1, wgst = 0;
  char vis[10] = "";
  char sky[24] = "";
  char raw[130] = "";
  long obs = 0;

  if (buf && ap >= 0 && ap < NUM_AIRPORTS) {
    size_t got = 0;
    String url = String("https://aviationweather.gov/api/data/metar?ids=") + AIRPORTS[ap].icao + "&format=json";
    int code = httpGetBuffered(url, buf, CAP, &got);
    Serial.printf("Weather report %s -> HTTP %d (%u bytes)\n", AIRPORTS[ap].icao, code, (unsigned)got);

    if (code == 204) {
      st = 3;                                                  // no report for this station right now
    } else if (code == 200) {
      buf[got] = '\0';
      DynamicJsonDocument filter(512);
      JsonObject f = filter.createNestedObject();
      f["fltCat"] = true;  f["temp"] = true;   f["wdir"] = true;  f["wspd"] = true;  f["wgst"] = true;
      f["visib"] = true;   f["rawOb"] = true;  f["obsTime"] = true;  f["clouds"] = true;  f["cover"] = true;

      DynamicJsonDocument doc(4096);
      DeserializationError err = deserializeJson(doc, (const char*)buf, got, DeserializationOption::Filter(filter));
      if (!err) {
        JsonObject o = doc[0].as<JsonObject>();
        if (o.isNull()) {
          st = 3;
        } else {
          copyStr(cat, sizeof(cat), o["fltCat"] | "");
          if (o["temp"].is<float>()) tempC = o["temp"].as<float>();
          if (o["wdir"].is<int>()) wdir = o["wdir"].as<int>();            // "VRB" arrives as text
          wspd = o["wspd"] | -1;
          wgst = o["wgst"] | 0;
          if (o["visib"].is<const char*>()) copyStr(vis, sizeof(vis), o["visib"].as<const char*>());
          else if (o["visib"].is<float>()) snprintf(vis, sizeof(vis), "%g", o["visib"].as<float>());
          copyStr(raw, sizeof(raw), o["rawOb"] | "");
          obs = o["obsTime"] | 0L;

          // sky: the lowest broken / overcast layer, else the first layer reported
          int bestBase = 1000000;
          const char* bestCover = nullptr;
          const char* firstCover = nullptr;
          int firstBase = 0;
          JsonArray cl = o["clouds"].as<JsonArray>();
          for (JsonObject c : cl) {
            const char* cv = c["cover"] | "";
            int base = c["base"] | 0;
            if (!firstCover) { firstCover = cv; firstBase = base; }
            bool ceiling = (strcmp(cv, "BKN") == 0 || strcmp(cv, "OVC") == 0 || strcmp(cv, "OVX") == 0);
            if (ceiling && base < bestBase) { bestBase = base; bestCover = cv; }
          }
          if (bestCover) snprintf(sky, sizeof(sky), "%s %d FT", bestCover, bestBase);
          else if (firstCover && firstCover[0] && firstBase > 0) snprintf(sky, sizeof(sky), "%s %d FT", firstCover, firstBase);
          else if (firstCover && firstCover[0]) copyStr(sky, sizeof(sky), firstCover);
          else {
            const char* cv = o["cover"] | "";
            copyStr(sky, sizeof(sky), cv[0] ? cv : "CLEAR");
          }
          st = 2;
        }
      }
    }
  }

  if (mtAirport == ap) {                                       // still the airport on screen
    copyStr(mtCat, sizeof(mtCat), cat);
    mtTempC = tempC;
    mtWdir = wdir;
    mtWspd = wspd;
    mtWgst = wgst;
    copyStr(mtVis, sizeof(mtVis), vis);
    copyStr(mtSky, sizeof(mtSky), sky);
    copyStr(mtRaw, sizeof(mtRaw), raw);
    mtObs = obs;
    mtWhen = millis();
    mtState = st;
  }
}

void metarTask(void* parameter) {
  metarRun();
  metarTaskHandle = NULL;
  metarInProgress = false;
  metarDone = true;
  vTaskDelete(NULL);
}

void maybeStartMetar() {
  if (!metarWanted || metarInProgress) return;
  if (fetchInProgress || lookupInProgress || geoBusy() || wxInProgress || photoInProgress || terrainInProgress) return;
  metarWanted = false;
  if (WiFi.status() != WL_CONNECTED) {
    mtState = 4;
    metarDone = true;
    return;
  }
  mtRunAirport = mtAirport;
  metarInProgress = true;
  metarStartMs = millis();
  BaseType_t ok = xTaskCreatePinnedToCore(metarTask, "metarTask", 16384, NULL, 1, &metarTaskHandle, 0);
  if (ok != pdPASS) {
    metarTaskHandle = NULL;
    metarInProgress = false;
    mtState = 4;
    metarDone = true;
  }
}

void drawAirportPanel() {
  int x0 = DETAIL_X, y0 = DETAIL_Y, w = DETAIL_W;
  uint16_t red = lcd.color565(255, 80, 80);
  uint16_t purple = lcd.color565(190, 150, 255);                          // same purple as the star on the radar

  rebuildAirportList();
  lcd.fillRect(x0, y0, w, DETAIL_H, TFT_BLACK);
  drawCloseX();

  if (apSel < 0 || apSel >= NUM_AIRPORTS) {                               // the airport button, but nothing in range
    lcd.setTextDatum(top_left);
    lcd.setTextSize(3);
    lcd.setTextColor(purple, TFT_BLACK);
    lcd.drawString("AIRPORTS", x0 + 2, y0 + 8);
    lcd.setTextSize(2);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString("NONE IN RANGE", x0 + 2, y0 + 60);
    lcd.setTextSize(1);
    lcd.drawString("ZOOM OUT FROM THE RANGE AC MENU", x0 + 2, y0 + 86);
    return;
  }

  const Airport& ap = AIRPORTS[apSel];

  lcd.setTextDatum(top_left);
  lcd.setTextSize(4);
  lcd.setTextColor(purple, TFT_BLACK);
  lcd.drawString(ap.code, x0 + 2, y0 + 4);
  lcd.setTextSize(2);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString(ap.icao, x0 + 84, y0 + 14);

  // CENTER: move the radar to this airport
  int bx = x0 + 140, by = y0 + 8, bw = 88, bh = 28;
  lcd.drawRect(bx, by, bw, bh, apRecenter == 2 ? red : green);
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(2);
  lcd.setTextColor(apRecenter == 2 ? red : greenBright, TFT_BLACK);
  lcd.drawString(apRecenter == 1 ? "WAIT.." : (apRecenter == 2 ? "FAILED" : "CENTER"), bx + bw / 2, by + bh / 2);
  lcd.setTextDatum(top_left);

  lcd.setTextSize(2);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(String(ap.name).substring(0, 23), x0 + 2, y0 + 42);
  lcd.setTextSize(1);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString(ap.city, x0 + 2, y0 + 62);
  lcd.drawFastHLine(x0, y0 + 76, w, greenDark);

  float d = distanceKm(homeLat, homeLon, ap.lat, ap.lon);
  float b = bearingDeg(homeLat, homeLon, ap.lat, ap.lon);
  lcd.setTextSize(2);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString("DIST", x0 + 2, y0 + 84);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(String(d, 1) + " km @ " + String((int)b) + " " + compass8(b), x0 + 78, y0 + 84);

  lcd.drawFastHLine(x0, y0 + 106, w, greenDark);
  lcd.setTextSize(1);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString("LIVE WEATHER", x0 + 2, y0 + 111);

  bool mine = (mtAirport == apSel);
  if (!mine || mtState == 0 || mtState == 1) {
    lcd.setTextSize(2);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString("LOADING...", x0 + 2, y0 + 130);
  } else if (mtState == 3) {
    lcd.setTextSize(2);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString("NO REPORT RIGHT NOW", x0 + 2, y0 + 130);
  } else if (mtState == 4) {
    lcd.setTextSize(2);
    lcd.setTextColor(red, TFT_BLACK);
    lcd.drawString("WEATHER UNAVAILABLE", x0 + 2, y0 + 130);
  } else {
    // flight category (VFR / MVFR / IFR / LIFR) in its usual colors
    uint16_t catCol = TFT_WHITE;
    if (strcmp(mtCat, "VFR") == 0) catCol = greenBright;
    else if (strcmp(mtCat, "MVFR") == 0) catCol = lcd.color565(80, 150, 255);
    else if (strcmp(mtCat, "IFR") == 0) catCol = red;
    else if (strcmp(mtCat, "LIFR") == 0) catCol = lcd.color565(255, 70, 255);
    lcd.setTextSize(3);
    lcd.setTextColor(catCol, TFT_BLACK);
    lcd.drawString(mtCat[0] ? mtCat : "---", x0 + 2, y0 + 122);

    auto row = [&](int r, const char* label, const String& val) {
      int ry = y0 + 152 + r * 20;
      lcd.setTextSize(2);
      lcd.setTextColor(greenDim, TFT_BLACK);
      lcd.drawString(label, x0 + 2, ry);
      lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      lcd.drawString(val, x0 + 78, ry);
    };

    String temp = "---";
    if (!isnan(mtTempC)) temp = String(mtTempC, 0) + " C / " + String(mtTempC * 9.0f / 5.0f + 32.0f, 0) + " F";
    String wind = "---";
    if (mtWspd == 0) {
      wind = "CALM";
    } else if (mtWspd > 0) {
      wind = (mtWdir >= 0 ? String(mtWdir) : String("VRB")) + " @ " + String(mtWspd);
      if (mtWgst > 0) wind += " G" + String(mtWgst);
      wind += " KT";
    }
    row(0, "TEMP", temp);
    row(1, "WIND", wind);
    row(2, "VIS",  mtVis[0] ? String(mtVis) + " SM" : String("---"));
    row(3, "SKY",  mtSky[0] ? String(mtSky) : String("---"));
  }

  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString("WEATHER: AVIATIONWEATHER.GOV", x0 + 2, y0 + 236);

  // prev / next bar like the plane panel: arrows, position in the nearest-first list
  drawArrowButton(NAV_PREV_X, NAV_Y, NAV_W, NAV_H, true);
  drawArrowButton(NAV_NEXT_X, NAV_Y, NAV_W, NAV_H, false);
  int pos = apListPos(apSel);
  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString((pos >= 0 ? String(pos + 1) : String("-")) + "/" + String(apListN), x0 + w / 2, NAV_Y + 1);
  lcd.setTextSize(1);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString("TAP X TO CLOSE", x0 + w / 2, NAV_Y + 20);
  lcd.setTextDatum(top_left);
}

// A task that hangs (stuck TLS connection) is killed so it can't block the other network jobs for good
void checkNewJobWatchdogs() {
  if (photoInProgress && millis() - photoStartMs > 40000UL) {
    Serial.println("Photo task hung - killing it");
    if (photoTaskHandle) vTaskDelete(photoTaskHandle);
    photoTaskHandle = NULL;
    photoInProgress = false;
    photoState = 4;
    photoDone = true;
  }
  if (metarInProgress && millis() - metarStartMs > 30000UL) {
    Serial.println("Weather report task hung - killing it");
    if (metarTaskHandle) vTaskDelete(metarTaskHandle);
    metarTaskHandle = NULL;
    metarInProgress = false;
    mtState = 4;
    metarDone = true;
  }
  if (terrainInProgress && millis() - terrainStartMs > 60000UL) {
    Serial.println("Terrain task hung - killing it");
    if (terrainTaskHandle) vTaskDelete(terrainTaskHandle);
    terrainTaskHandle = NULL;
    terrainInProgress = false;
    terrainFails = terrainFails + 1;
    if (terrainFails >= 3) terrainGaveUp = true;
    else terrainRetryAt = millis() + 10000UL;
  }
}

// ---------- v10: PLANE TYPES (dot shape) ----------
// airliner = filled circle, small plane = hollow ring, helicopter = X, military = diamond.
// (the dot color still shows altitude)
// (takes plain values, not an AircraftPoint, because of the Arduino prototype rule)
bool airlineCallsign(const char* f) {                  // three letters then a digit: UAL123, DAL45
  while (*f == ' ') f++;
  return isalpha((unsigned char)f[0]) && isalpha((unsigned char)f[1]) && isalpha((unsigned char)f[2]) && isdigit((unsigned char)f[3]);
}

int classify(const char* cat, const char* typ, int dbFlags, const char* call) {
  if (dbFlags & 1) return PC_MIL;
  if (strcmp(cat, "A7") == 0) return PC_HELI;
  static const char* const heli[] = {"R22", "R44", "R66", "EC35", "EC45", "B06", "B407", "B412", "B429", "S76", "S92", "A109", "A139", "AS50", "AS55"};
  for (int k = 0; k < (int)(sizeof(heli) / sizeof(heli[0])); k++) {
    if (strcmp(typ, heli[k]) == 0) return PC_HELI;
  }
  if (strcmp(cat, "A1") == 0 || strcmp(cat, "B1") == 0 || strcmp(cat, "B4") == 0) return PC_SMALL;   // light, glider, ultralight
  if (strcmp(cat, "A3") == 0 || strcmp(cat, "A4") == 0 || strcmp(cat, "A5") == 0) return PC_AIRLINER;
  if (strcmp(cat, "A2") == 0) return airlineCallsign(call) ? PC_AIRLINER : PC_SMALL;
  // no usable category: go by the callsign
  if (airlineCallsign(call)) return PC_AIRLINER;
  while (*call == ' ') call++;
  if (call[0] == 'N' && isdigit((unsigned char)call[1])) return PC_SMALL;      // US tail number
  return PC_AIRLINER;
}

int planeClass(int i) {
  return classify(planes[i].category, planes[i].typeCode, planes[i].dbFlags, planes[i].flight);
}

void drawTypeShapeCanvas(int cls, int x, int y, int s, uint16_t col) {
  if (cls == PC_SMALL) {
    radarCanvas.drawCircle(x, y, s, col);
    radarCanvas.drawCircle(x, y, s - 1, col);
  } else if (cls == PC_HELI) {
    radarCanvas.drawLine(x - s, y - s, x + s, y + s, col);
    radarCanvas.drawLine(x - s + 1, y - s, x + s + 1, y + s, col);
    radarCanvas.drawLine(x - s, y + s, x + s, y - s, col);
    radarCanvas.drawLine(x - s + 1, y + s, x + s + 1, y - s, col);
  } else if (cls == PC_MIL) {
    int d = s + 2;
    radarCanvas.fillTriangle(x, y - d, x + d, y, x, y + d, col);
    radarCanvas.fillTriangle(x, y - d, x - d, y, x, y + d, col);
  } else {
    radarCanvas.fillCircle(x, y, s, col);
  }
}

void drawTypeShapeLcd(int cls, int x, int y, int s, uint16_t col) {
  if (cls == PC_SMALL) {
    lcd.drawCircle(x, y, s, col);
    lcd.drawCircle(x, y, s - 1, col);
  } else if (cls == PC_HELI) {
    lcd.drawLine(x - s, y - s, x + s, y + s, col);
    lcd.drawLine(x - s + 1, y - s, x + s + 1, y + s, col);
    lcd.drawLine(x - s, y + s, x + s, y - s, col);
    lcd.drawLine(x - s + 1, y + s, x + s + 1, y - s, col);
  } else if (cls == PC_MIL) {
    int d = s + 2;
    lcd.fillTriangle(x, y - d, x + d, y, x, y + d, col);
    lcd.fillTriangle(x, y - d, x - d, y, x, y + d, col);
  } else {
    lcd.fillCircle(x, y, s, col);
  }
}

// ---------- v10: SQUAWK BANNER ----------
// Flashing red strip at the bottom of the radar while a plane in range squawks 7500 / 7600 / 7700. Tap it to select the plane.
void drawSquawkBanner() {
  bannerShown = false;
  int idx[8];
  int n = 0;
  for (int i = 0; i < planeCount && n < 8; i++) {
    if (planes[i].valid && planes[i].distanceKm <= rangeKm && isEmergencySquawk(planes[i].squawk)) idx[n++] = i;
  }
  if (n == 0) { bannerHex[0] = '\0'; return; }

  int k = (int)((millis() / 3500UL) % (unsigned long)n);       // several at once: take turns
  int i = idx[k];
  bool on = ((millis() / 450UL) & 1);
  uint16_t bg = on ? radarCanvas.color565(210, 0, 0) : radarCanvas.color565(95, 0, 0);

  radarCanvas.fillRect(BN_X, BN_Y, BN_W, BN_H, bg);
  radarCanvas.drawRect(BN_X, BN_Y, BN_W, BN_H, TFT_WHITE);

  const char* what = strcmp(planes[i].squawk, "7500") == 0 ? "HIJACK" :
                     strcmp(planes[i].squawk, "7600") == 0 ? "NO RADIO" : "EMERGENCY";
  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(2);
  radarCanvas.setTextColor(TFT_WHITE, bg);
  radarCanvas.drawString(String(planes[i].squawk) + " " + what, BN_X + 8, BN_Y + 6);

  String call = String(planes[i].flight);
  call.trim();
  if (call.length() == 0) { call = String(planes[i].hex); call.toUpperCase(); }
  String line = call + "  " + String(planes[i].distanceKm, 1) + " km";
  if (n > 1) line += "   " + String(k + 1) + "/" + String(n);
  radarCanvas.setTextSize(1);
  radarCanvas.drawString(line, BN_X + 8, BN_Y + 28);

  bannerShown = true;
  copyStr(bannerHex, sizeof(bannerHex), planes[i].hex);
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

    drawTypeShapeCanvas(planeClass(i), x, y, size, col);                 // v10: the shape shows the plane type
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

  if (!swShown) {            // v7: the switcher number takes over this corner while it is showing
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
  if (terrainLoading()) {                                  // v11
    radarCanvas.setTextSize(2);
    radarCanvas.setTextColor(greenDim, TFT_BLACK);
    radarCanvas.drawString("TERRAIN", 10, 72);
  }
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
  drawAirportsToCanvas();                    // v10
  drawAircraftTargetsToCanvas();
  drawRangeBarsToCanvas();
  drawSweepHeadingIndicatorToCanvas();
  drawSquawkBanner();                        // v10
  drawSwitcher(radarCanvas, greenDim, greenBright, false);   // v7
}

void drawTimePanel() {
  if (photoMode) return;                     // v11: the aircraft photo covers this part of the sidebar
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
  if (photoMode) return;                     // v11: the aircraft photo covers this part of the sidebar
  int boxX = INFO_X + 18;
  int boxY = INFO_Y + 305;   
  int boxW = INFO_W - 36;
  int boxH = 70;

  lcd.fillRect(boxX, boxY, boxW, boxH, TFT_BLACK);
  lcd.drawFastHLine(INFO_X + 10, boxY - 10, INFO_W - 20, greenDark);

  // place name (PHILLY by default, or whatever the compass screen found)
  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString(placeName, boxX + boxW / 2, boxY);

  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

  String coord = String((float)homeLat, 4) + " / " + String((float)homeLon, 4);
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
                   DETAIL_X + 119, topY + 7);
  } else {
    lcd.drawString("DETECTED AIRCRAFTS", DETAIL_X + 122, topY + 2);
  }

  drawAirportBtn();                              // v11

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

    drawTypeShapeLcd(classify(p.category, p.typeCode, p.dbFlags, p.flight), boxX + 272, y + 5, 4, dotCol);
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

  int pcl = classify(p.category, p.typeCode, p.dbFlags, p.flight);      // v10: type tag
  if (pcl != PC_AIRLINER) {
    lcd.setTextColor(amber, TFT_BLACK);
    if (pcl == PC_MIL)       lcd.drawString("MIL",   x0 + w - 24, y0 + 62);
    else if (pcl == PC_HELI) lcd.drawString("HELI",  x0 + w - 30, y0 + 62);
    else                     lcd.drawString("SMALL", x0 + w - 36, y0 + 62);
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

  if (p.hex[0]) drawCamIcon(photoMode);                  // v11: tap to show a photo of this aircraft

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
  lcd.drawString(label, x + 14, y + 8);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(sub, x + 14, y + 36);
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
  drawMenuButton(x0, MENU_FAV_Y,    w, MENU_BTN_H, "PLACES", favCount() ? String(favCount()) + " SAVED" : String("NONE SAVED"));

  // v11: terrain on / off
  lcd.fillRect(x0, MENU_TERR_Y, w, MENU_TERR_H, TFT_BLACK);
  lcd.drawRect(x0, MENU_TERR_Y, w, MENU_TERR_H, terrainOn ? green : greenDark);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString("TERRAIN", x0 + 14, MENU_TERR_Y + 9);
  lcd.setTextDatum(top_right);
  lcd.setTextColor(terrainOn ? greenBright : greenDim, TFT_BLACK);
  lcd.drawString(!terrainOn ? "OFF" : (terrainKeyIsBuilt() ? "ON" : (terrainGaveUp ? "NO DATA" : "LOADING")), x0 + w - 14, MENU_TERR_Y + 9);

  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString("TAP X OR RADAR TO CLOSE", x0 + 2, y0 + 273);
  lcd.setTextDatum(top_right);
  lcd.drawString("TERRAIN: AWS/MAPZEN", x0 + w - 2, y0 + 273);
  lcd.setTextDatum(top_left);
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
  lcd.drawString("POOL " + String(poolCount()) + " AIRCRAFT", x0 + 2, y0 + 238);
  lcd.setTextSize(1);
  lcd.drawString("10 KM STEPS - A NEW POOL IS FETCHED", x0 + 2, y0 + 266);
}

// ---------- LOCATION SCREEN (tap the compass): latitude / longitude entry with a number pad ----------
const char* LOC_KEYS[15] = {"1", "2", "3", "DEL", "4", "5", "6", "+/-", "7", "8", "9", ".", "0", "NEXT", "GO"};

void drawLocField(int f) {
  int y = LOC_FIELD_Y0 + f * LOC_FIELD_STEP;
  int bw = DETAIL_X + DETAIL_W - LOC_FIELD_X;
  bool focus = (locField == f);
  bool sel = focus && locReplace[f];                       // whole value highlighted: next key replaces it

  lcd.fillRect(DETAIL_X, y, DETAIL_W, LOC_FIELD_H, TFT_BLACK);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(focus ? greenBright : greenDim, TFT_BLACK);
  lcd.drawString(f == 0 ? "LAT" : "LON", DETAIL_X + 4, y + 9);

  lcd.drawRect(LOC_FIELD_X, y, bw, LOC_FIELD_H, focus ? greenBright : green);
  if (focus) lcd.drawRect(LOC_FIELD_X + 1, y + 1, bw - 2, LOC_FIELD_H - 2, greenBright);
  if (sel) lcd.fillRect(LOC_FIELD_X + 3, y + 3, bw - 6, LOC_FIELD_H - 6, greenDark);

  String v = String(locBuf[f]);
  if (focus && !sel && !geoBusy()) v += "_";
  lcd.setTextColor(focus ? TFT_YELLOW : TFT_WHITE, sel ? greenDark : TFT_BLACK);
  lcd.drawString(v, LOC_FIELD_X + 8, y + 9);
}

void drawLocStatus() {
  lcd.fillRect(DETAIL_X, LOC_STATUS_Y, DETAIL_W, 10, TFT_BLACK);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(locStatusRed ? lcd.color565(255, 80, 80) : greenDim, TFT_BLACK);
  lcd.drawString(locStatus, DETAIL_X + 2, LOC_STATUS_Y);
}

void drawLocKeypad() {
  bool busy = geoBusy();
  for (int k = 0; k < 15; k++) {
    int row = k / 4;
    int col = k % 4;
    int x = DETAIL_X + col * LOC_KP_COL_W;
    int y = LOC_KP_Y0 + row * LOC_KP_ROW_H;
    int w = (k == 14) ? LOC_KP_COL_W * 2 : LOC_KP_COL_W;     // GO is two keys wide

    lcd.fillRect(x, y, w, LOC_KP_ROW_H, TFT_BLACK);
    lcd.drawRect(x, y, w, LOC_KP_ROW_H, busy ? greenDark : green);
    lcd.setTextDatum(middle_center);
    lcd.setTextSize(2);
    uint16_t col565 = busy ? greenDim : (k == 14 ? TFT_YELLOW : TFT_WHITE);
    lcd.setTextColor(col565, TFT_BLACK);
    lcd.drawString(LOC_KEYS[k], x + w / 2, y + LOC_KP_ROW_H / 2);
  }
  lcd.setTextDatum(top_left);
}

void drawLocEntryPanel() {
  int x0 = DETAIL_X, y0 = DETAIL_Y, w = DETAIL_W;
  lcd.fillRect(x0, y0, w, DETAIL_H, TFT_BLACK);
  drawCloseX();

  // PHILLY: jump back to the original coordinates
  lcd.drawRect(x0, y0, LOC_HOME_W, LOC_HOME_H, green);
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString("PHILLY", x0 + LOC_HOME_W / 2, y0 + LOC_HOME_H / 2);

  lcd.setTextDatum(top_left);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("LOCATION", x0 + 96, y0 + 12);

  drawLocField(0);
  drawLocField(1);
  drawLocStatus();
  drawLocKeypad();

  // credit for the two lookup services
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(greenDim, TFT_BLACK);
  lcd.drawString("PLACES (C) OPENSTREETMAP CONTRIBUTORS", x0 + 2, y0 + 268);
  lcd.drawString("TIME ZONES: OPEN-METEO.COM", x0 + 2, y0 + 277);
}

// Top half of the sidebar: menu pages, detail panel when a plane is selected, plane list otherwise
void drawInfoTop() {
  if (menuMode == MENU_MAIN) drawMenuPanel();
  else if (menuMode == MENU_LOC) drawLocEntryPanel();
  else if (menuMode == MENU_FILTER) drawFilterPanel();
  else if (menuMode == MENU_ZOOM) drawZoomPanel();
  else if (menuMode == MENU_FAV) drawFavPanel();
  else if (menuMode == MENU_AIRPORT) drawAirportPanel();   // v11
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
  photoHide();                                   // v11

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
  photoHide();                                   // v11
  hasSelection = false;
  selectedHex[0] = '\0';
  drawInfoTop();
}

void openMenu() {
  photoHide();                                   // v11
  hasSelection = false;                          // the menu takes over the detail panel's spot
  selectedHex[0] = '\0';
  menuMode = MENU_MAIN;
  drawInfoTop();
}

void closeMenu() {
  menuMode = MENU_NONE;
  favMode = 0;
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

// ---------- v10: SUNRISE / SUNSET, AUTO-DIM, LAST SCREEN ----------
// Sunrise and sunset come back with the place / time zone lookup (Open-Meteo, free) and with every weather fetch.
void applyGeoSun() {
  if (geoSunriseMin >= 0 && geoSunsetMin > geoSunriseMin) {
    sunriseMin = geoSunriseMin;
    sunsetMin  = geoSunsetMin;
    sunKnown   = true;
  }
  sunNextCheckMs = millis() + 6UL * 3600UL * 1000UL;
}

// The default location never runs the time zone check, so this keeps sunrise / sunset fresh for every location
void maybeScheduleSunCheck() {
  if (geoState != GEO_IDLE) return;
  if (millis() < sunNextCheckMs || WiFi.status() != WL_CONNECTED) return;
  geoReqLat = homeLat;
  geoReqLon = homeLon;
  geoTzOnly = true;
  geoSunOnly = true;
  geoState = GEO_WAITING;
}

int targetBrightness() {
  if (lastTouchMs != 0 && millis() - lastTouchMs < WAKE_MS) return BRIGHT_DAY;   // just touched: full brightness
  if (!sunKnown) return BRIGHT_DAY;
  struct tm ti;
  if (!getLocalTime(&ti, 0)) return BRIGHT_DAY;

  int now = ti.tm_hour * 60 + ti.tm_min;
  int sr = sunriseMin;
  int ss = sunsetMin;
  float w;                                                   // 0 = day, 1 = night
  if (now >= ss)                      w = (float)(now - ss) / DIM_FADE_MIN;          // fading down after sunset
  else if (now < sr - DIM_FADE_MIN)   w = 1.0f;                                      // night
  else if (now < sr)                  w = (float)(sr - now) / DIM_FADE_MIN;          // fading up before sunrise
  else                                w = 0.0f;                                      // day
  if (w > 1.0f) w = 1.0f;
  return BRIGHT_DAY + (int)((BRIGHT_NIGHT - BRIGHT_DAY) * w);
}

// Runs every loop on every screen. Dims slowly, brightens quickly.
void serviceBrightness() {
  static unsigned long last = 0;
  if (millis() - last < 100) return;
  last = millis();

  int target = targetBrightness();
  int next = curBright;
  if (target > curBright)      next = min(curBright + 24, target);
  else if (target < curBright) next = max(curBright - 2, target);
  if (next != curBright) {
    curBright = next;
    lcd.setBrightness(curBright);
  }
}

void saveLastScreen(int s) {
  prefs.begin("radar", false);
  prefs.putUChar("screen", (uint8_t)s);
  prefs.end();
}

int loadLastScreen() {
  prefs.begin("radar", true);
  int s = prefs.getUChar("screen", 0);
  prefs.end();
  return (s == SCREEN_WEATHER) ? SCREEN_WEATHER : SCREEN_RADAR;
}

// ---------- LOCATION: apply, open, type, go ----------

// Move the whole radar to a new center: API query, plane pool, place name, clock. Saved for the next boot.
void commitLocation(float lat, float lon, const char* name, const char* tz) {
  photoMode = false;                             // v11: the panels below are redrawn at the end
  homeLat = lat;
  homeLon = lon;
  strncpy(placeName, name, sizeof(placeName) - 1);
  placeName[sizeof(placeName) - 1] = '\0';
  applyTimezone(tz);
  saveSettings();
  sunKnown = false;                              // v10: sunrise / sunset belong to the old place
  sunNextCheckMs = 0;

  // everything on the radar belongs to the old center: drop it and fetch the new area right away
  planeCount = 0;
  listPage = 0;
  hasSelection = false;
  selectedHex[0] = '\0';
  pendingDataReady = false;
  refetchNow = true;
  rangeApplyWanted = true;
  Serial.printf("Location -> %s (%.4f, %.4f) tz %s\n", placeName, (double)lat, (double)lon, tzString);

  drawLocationPanel();
  drawTimePanel();
}

void openLocPanel() {
  photoHide();                                   // v11
  hasSelection = false;                          // the location screen takes over the detail panel's spot
  selectedHex[0] = '\0';
  snprintf(locBuf[0], sizeof(locBuf[0]), "%.4f", (double)homeLat);
  snprintf(locBuf[1], sizeof(locBuf[1]), "%.4f", (double)homeLon);
  locField = 0;
  locReplace[0] = locReplace[1] = true;
  locStatus = geoBusy() ? "LOOKING UP LOCATION..." : "TAP A FIELD, TYPE, THEN GO";
  locStatusRed = false;
  menuMode = MENU_LOC;
  drawInfoTop();
}

bool parseCoord(const char* txt, double lo, double hi, double* out) {
  char* end = NULL;
  double v = strtod(txt, &end);
  if (txt[0] == '\0' || end == txt || *end != '\0') return false;   // empty, "-", "." ...
  if (v < lo || v > hi) return false;
  *out = v;
  return true;
}

void locGo() {
  double la, lo;
  if (!parseCoord(locBuf[0], -90.0, 90.0, &la) || !parseCoord(locBuf[1], -180.0, 180.0, &lo)) {
    locStatus = "BAD COORDINATES (LAT +-90, LON +-180)";
    locStatusRed = true;
    drawLocStatus();
    return;
  }
  geoReqLat = (float)la;
  geoReqLon = (float)lo;
  geoTzOnly = false;
  geoSunOnly = false;
  geoState = GEO_WAITING;
  locStatus = "LOOKING UP LOCATION...";
  locStatusRed = false;
  drawLocField(0);
  drawLocField(1);
  drawLocStatus();
  drawLocKeypad();
}

void locKey(int k) {
  char* b = locBuf[locField];
  size_t len = strlen(b);

  if (k == 14) { locGo(); return; }                              // GO
  if (k == 13) {                                                 // NEXT: switch field
    locField = 1 - locField;
    drawLocField(0);
    drawLocField(1);
    return;
  }

  if (k == 3) {                                                  // DEL
    if (locReplace[locField]) { b[0] = '\0'; locReplace[locField] = false; }
    else if (len) b[len - 1] = '\0';
  } else if (k == 7) {                                           // +/-
    if (b[0] == '-') memmove(b, b + 1, len);
    else if (len < 12) { memmove(b + 1, b, len + 1); b[0] = '-'; }
    locReplace[locField] = false;
  } else {                                                       // digit or decimal point
    if (locReplace[locField]) { b[0] = '\0'; len = 0; locReplace[locField] = false; }
    char key = LOC_KEYS[k][0];
    if (key == '.') {
      if (strchr(b, '.')) return;                                // only one decimal point
      if (len == 0 || (len == 1 && b[0] == '-')) {               // ".5" -> "0.5"
        b[len] = '0';
        b[len + 1] = '\0';
        len++;
      }
    }
    if (len < 11) { b[len] = key; b[len + 1] = '\0'; }
  }

  locStatus = "TAP A FIELD, TYPE, THEN GO";
  locStatusRed = false;
  drawLocField(locField);
  drawLocStatus();
}

void handleLocTap(int x, int y) {
  if (geoBusy()) return;                         // wait for the lookup to finish

  // PHILLY (top-left corner): back to the original coordinates
  if (x >= DETAIL_X && x <= DETAIL_X + LOC_HOME_W + 6 && y >= DETAIL_Y && y <= DETAIL_Y + LOC_HOME_H + 6) {
    if (!atDefaultLocation()) commitLocation((float)DEFAULT_LAT, (float)DEFAULT_LON, DEFAULT_NAME, DEFAULT_TZ);
    menuMode = MENU_NONE;
    drawInfoTop();
    return;
  }

  // tap a field to type into it
  for (int f = 0; f < 2; f++) {
    int fy = LOC_FIELD_Y0 + f * LOC_FIELD_STEP;
    if (y >= fy && y <= fy + LOC_FIELD_H) {
      locField = f;
      drawLocField(0);
      drawLocField(1);
      return;
    }
  }

  // number pad
  if (y >= LOC_KP_Y0 && y < LOC_KP_Y0 + 4 * LOC_KP_ROW_H && x >= DETAIL_X && x < DETAIL_X + 4 * LOC_KP_COL_W) {
    int row = (y - LOC_KP_Y0) / LOC_KP_ROW_H;
    int col = (x - DETAIL_X) / LOC_KP_COL_W;
    int k = row * 4 + col;
    if (k == 15) k = 14;                         // GO covers the last two cells
    locKey(k);
  }
}

// The lookup task finished (or failed)
void handleGeoResult() {
  int st = geoState;
  geoState = GEO_IDLE;

  if (geoSunOnly) {                              // v10: quiet sunrise / sunset refresh (the time zone is left alone)
    geoSunOnly = false;
    geoTzOnly = false;
    if (st == GEO_OK) applyGeoSun();
    else sunNextCheckMs = millis() + 600000UL;   // try again in 10 minutes
    return;
  }

  if (geoRecenter) {                             // v11: the CENTER button on the airport page
    geoRecenter = false;
    if (st == GEO_OK) {
      commitLocation(geoReqLat, geoReqLon, geoRecenterName, geoTz);
      applyGeoSun();
      tzNextCheckMs = millis() + 6UL * 3600UL * 1000UL;
      apRecenter = 0;
      menuMode = MENU_NONE;
    } else {
      apRecenter = 2;                            // stay where we are; the button says FAILED
    }
    drawInfoTop();
    return;
  }

  if (geoTzOnly) {                               // quiet daylight-saving re-check
    geoTzOnly = false;
    if (st == GEO_OK) {
      applyTimezone(geoTz);
      saveSettings();
      applyGeoSun();
      drawTimePanel();
      tzNextCheckMs = millis() + 6UL * 3600UL * 1000UL;
    } else {
      tzNextCheckMs = millis() + 600000UL;       // try again in 10 minutes
    }
    return;
  }

  if (st == GEO_OK) {
    commitLocation(geoReqLat, geoReqLon, geoName, geoTz);
    applyGeoSun();
    tzNextCheckMs = millis() + 6UL * 3600UL * 1000UL;
    menuMode = MENU_NONE;
    drawInfoTop();
  } else {
    // lookup failed: say so and go back to the original coordinates
    if (!atDefaultLocation()) commitLocation((float)DEFAULT_LAT, (float)DEFAULT_LON, DEFAULT_NAME, DEFAULT_TZ);
    locStatus = "LOOKUP FAILED - USING PHILLY";
    locStatusRed = true;
    if (menuMode == MENU_LOC) {
      drawLocField(0);
      drawLocField(1);
      drawLocStatus();
      drawLocKeypad();
    }
  }
}

// ---------- v10: FAVORITE PLACES ----------
const int FAV_ROW_Y0 = DETAIL_Y + 48;
const int FAV_ROW_H = 42;
const int FAV_ROW_STEP = 46;
const int FAV_BTN_Y = DETAIL_Y + 238;
const int FAV_BTN_H = 30;
const int FAV_BTN_W = 140;

void saveFavs() {
  prefs.begin("radar", false);
  for (int i = 0; i < FAV_SLOTS; i++) {
    char k[8];
    snprintf(k, sizeof(k), "f%du", i);  prefs.putUChar(k, favs[i].used ? 1 : 0);
    snprintf(k, sizeof(k), "f%dla", i); prefs.putFloat(k, favs[i].lat);
    snprintf(k, sizeof(k), "f%dlo", i); prefs.putFloat(k, favs[i].lon);
    snprintf(k, sizeof(k), "f%dn", i);  prefs.putString(k, favs[i].name);
    snprintf(k, sizeof(k), "f%dt", i);  prefs.putString(k, favs[i].tz);
  }
  prefs.end();
}

void loadFavs() {
  prefs.begin("radar", true);
  for (int i = 0; i < FAV_SLOTS; i++) {
    char k[8];
    favs[i].used = false;
    snprintf(k, sizeof(k), "f%du", i);  uint8_t u = prefs.getUChar(k, 0);
    snprintf(k, sizeof(k), "f%dla", i); float la = prefs.getFloat(k, NAN);
    snprintf(k, sizeof(k), "f%dlo", i); float lo = prefs.getFloat(k, NAN);
    snprintf(k, sizeof(k), "f%dn", i);  String n = prefs.getString(k, "");
    snprintf(k, sizeof(k), "f%dt", i);  String t = prefs.getString(k, "");
    if (u == 1 && !isnan(la) && !isnan(lo) && la >= -90 && la <= 90 && lo >= -180 && lo <= 180 &&
        n.length() > 0 && n.length() < sizeof(favs[i].name) && t.length() > 0 && t.length() < sizeof(favs[i].tz)) {
      favs[i].used = true;
      favs[i].lat = la;
      favs[i].lon = lo;
      copyStr(favs[i].name, sizeof(favs[i].name), n.c_str());
      copyStr(favs[i].tz, sizeof(favs[i].tz), t.c_str());
    }
  }
  prefs.end();
}

int favCount() {
  int n = 0;
  for (int i = 0; i < FAV_SLOTS; i++) if (favs[i].used) n++;
  return n;
}

void saveCurrentToSlot(int i) {
  favs[i].used = true;
  favs[i].lat = homeLat;
  favs[i].lon = homeLon;
  copyStr(favs[i].name, sizeof(favs[i].name), placeName);
  copyStr(favs[i].tz, sizeof(favs[i].tz), tzString);
  saveFavs();
}

void drawFavPanel() {
  int x0 = DETAIL_X, y0 = DETAIL_Y, w = DETAIL_W;
  uint16_t amber = lcd.color565(255, 200, 0);
  uint16_t red   = lcd.color565(255, 80, 80);

  lcd.fillRect(x0, y0, w, DETAIL_H, TFT_BLACK);
  drawCloseX();
  drawArrowButton(BACK_X, BACK_Y, BACK_W, BACK_H, true);

  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("PLACES", x0 + 62, y0 + 12);

  for (int i = 0; i < FAV_SLOTS; i++) {
    int ry = FAV_ROW_Y0 + i * FAV_ROW_STEP;
    uint16_t edge = greenDark;
    if (favMode == 1) edge = amber;
    else if (favMode == 2) edge = favs[i].used ? red : greenDark;
    else if (favs[i].used) edge = green;
    lcd.drawRect(x0, ry, w, FAV_ROW_H, edge);

    lcd.setTextDatum(top_left);
    if (favs[i].used) {
      lcd.setTextSize(2);
      lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
      lcd.drawString(favs[i].name, x0 + 8, ry + 5);
      lcd.setTextSize(1);
      lcd.setTextColor(greenDim, TFT_BLACK);
      lcd.drawString(String(favs[i].lat, 4) + " / " + String(favs[i].lon, 4), x0 + 8, ry + 28);
    } else {
      lcd.setTextSize(2);
      lcd.setTextColor(greenDim, TFT_BLACK);
      lcd.drawString("EMPTY", x0 + 8, ry + 5);
      lcd.setTextSize(1);
      lcd.drawString(favMode == 1 ? "TAP TO SAVE HERE" : "TAP TO SAVE THE CURRENT PLACE", x0 + 8, ry + 28);
    }
  }

  // SAVE HERE / CLEAR: press one, then tap a slot
  int bx2 = x0 + FAV_BTN_W + 10;
  lcd.drawRect(x0, FAV_BTN_Y, FAV_BTN_W, FAV_BTN_H, favMode == 1 ? amber : green);
  lcd.drawRect(bx2, FAV_BTN_Y, FAV_BTN_W, FAV_BTN_H, favMode == 2 ? red : green);
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(2);
  lcd.setTextColor(favMode == 1 ? amber : TFT_WHITE, TFT_BLACK);
  lcd.drawString("SAVE HERE", x0 + FAV_BTN_W / 2, FAV_BTN_Y + FAV_BTN_H / 2);
  lcd.setTextColor(favMode == 2 ? red : TFT_WHITE, TFT_BLACK);
  lcd.drawString("CLEAR", bx2 + FAV_BTN_W / 2, FAV_BTN_Y + FAV_BTN_H / 2);
  lcd.setTextDatum(top_left);
}

void goToFav(int i) {
  commitLocation(favs[i].lat, favs[i].lon, favs[i].name, favs[i].tz);
  tzNextCheckMs = millis() + 3000UL;               // re-check the time zone offset soon (daylight saving)
  menuMode = MENU_NONE;
  drawInfoTop();
}

void handleFavTap(int x, int y) {
  int x0 = DETAIL_X;
  int bx2 = x0 + FAV_BTN_W + 10;

  if (y >= FAV_BTN_Y - 4 && y <= FAV_BTN_Y + FAV_BTN_H + 4) {
    if (x >= x0 && x <= x0 + FAV_BTN_W)       favMode = (favMode == 1) ? 0 : 1;
    else if (x >= bx2 && x <= bx2 + FAV_BTN_W) favMode = (favMode == 2) ? 0 : 2;
    else return;
    drawInfoTop();
    return;
  }

  for (int i = 0; i < FAV_SLOTS; i++) {
    int ry = FAV_ROW_Y0 + i * FAV_ROW_STEP;
    if (y < ry || y > ry + FAV_ROW_H || x < x0 || x > x0 + DETAIL_W) continue;

    if (favMode == 1)      { saveCurrentToSlot(i); favMode = 0; }
    else if (favMode == 2) { favs[i].used = false; saveFavs(); favMode = 0; }
    else if (favs[i].used) { goToFav(i); return; }
    else                   { saveCurrentToSlot(i); }
    drawInfoTop();
    return;
  }
}

// Taps inside the sidebar while a menu page is open
void handleMenuTap(int x, int y) {
  int x0 = DETAIL_X, w = DETAIL_W;
  if (menuMode == MENU_AIRPORT) { handleAirportTap(x, y); return; }   // v11
  if (menuMode == MENU_LOC) {                   // the location screen has its own layout
    handleLocTap(x, y);
    return;
  }

  // back arrow (filter / zoom pages)
  if (menuMode != MENU_MAIN && x >= BACK_X && x <= BACK_X + BACK_W + 10 &&
      y >= BACK_Y && y <= BACK_Y + BACK_H + 10) {
    menuMode = MENU_MAIN;
    favMode = 0;
    drawInfoTop();
    return;
  }

  if (menuMode == MENU_FAV) {
    handleFavTap(x, y);
    return;
  }

  if (menuMode == MENU_MAIN) {
    if (x < x0 || x > x0 + w) return;
    if (y >= MENU_FILTER_Y && y <= MENU_FILTER_Y + MENU_BTN_H) { menuMode = MENU_FILTER; drawInfoTop(); }
    else if (y >= MENU_ZOOM_Y && y <= MENU_ZOOM_Y + MENU_BTN_H) { menuMode = MENU_ZOOM; drawInfoTop(); }
    else if (y >= MENU_FAV_Y && y <= MENU_FAV_Y + MENU_BTN_H) { menuMode = MENU_FAV; favMode = 0; drawInfoTop(); }
    else if (y >= MENU_TERR_Y && y <= MENU_TERR_Y + MENU_TERR_H + 4) {        // v11
      terrainOn = !terrainOn;
      saveSettings();
      drawInfoTop();
    }
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

void radarOnTap(int x, int y) {
  lastInteractMs = millis();
  Serial.printf("TAP %d,%d\n", x, y);

  // 00) the squawk banner selects the plane that is squawking
  if (bannerShown && x >= BN_X && x <= BN_X + BN_W && y >= BN_Y && y <= BN_Y + BN_H) {
    for (int i = 0; i < planeCount; i++) {
      if (planes[i].valid && strcmp(planes[i].hex, bannerHex) == 0) { selectPlane(i); break; }
    }
    return;
  }

  // 0) the RANGE AC box on the radar opens the filter / zoom menu
  if (x >= RB_X1 && x <= RB_X2 && y >= RB_Y1 && y <= RB_Y2) {
    openMenu();
    return;
  }

  // 0b) the compass (bottom-left of the radar) opens the location screen
  if (x >= CMP_X1 && x <= CMP_X2 && y >= CMP_Y1 && y <= CMP_Y2) {
    openLocPanel();
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

  // 1a2) v11: the camera icon on the detail panel, and a tap on the photo itself to put it away
  if (hasSelection && menuMode == MENU_NONE) {
    if (photoMode && x >= INFO_X && y >= INFO_Y + 298) { photoHide(); drawInfoTop(); return; }
    if (selSnap.hex[0] && x >= camX() - 8 && x <= camX() + CAM_W + 4 && y >= camY() - 6 && y <= camY() + CAM_H + 6) {
      photoToggle();
      return;
    }
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
    if (best >= 0) {
      selectPlane(best);
    } else {
      int ap = airportAt(x, y);                    // v11: an airport star (planes win when both are under the finger)
      if (ap >= 0) openAirportPanel(ap);
      else if (hasSelection) clearSelection();
      else if (menuMode != MENU_NONE) closeMenu();
    }
    return;
  }

  // 3a0) v11: the airport button in the top right corner of the list screen
  if (!hasSelection && menuMode == MENU_NONE && x >= CLOSE_X - 2 && y >= CLOSE_Y && y <= CLOSE_Y + CLOSE_H + 6) {
    openAirportBrowse();
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
      } else if (x >= PAGER_RIGHT_X - 6 && x <= CLOSE_X - 3) {
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

// =====================================================================
// ---------- v7: SCREEN MANAGER + WEATHER SCREEN ----------
// =====================================================================

// ---------- weather: fetch (Open-Meteo) ----------
bool fetchWeatherData() {
  if (WiFi.status() != WL_CONNECTED) return false;

  float la = homeLat;
  float lo = homeLon;

  String url = "https://api.open-meteo.com/v1/forecast?latitude=";
  url += String(la, 4);
  url += "&longitude=";
  url += String(lo, 4);
  url += "&current=temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,weather_code,wind_speed_10m,wind_direction_10m,is_day";
  url += "&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max,precipitation_sum,sunrise,sunset";
  url += "&hourly=temperature_2m,precipitation_probability,precipitation,weather_code";
  url += "&wind_speed_unit=mph&precipitation_unit=inch&timezone=auto&forecast_days=7";

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(10000);
  http.setUserAgent(GEO_USER_AGENT);
  http.useHTTP10(true);

  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != 200) {
    Serial.printf("Weather HTTP %d\n", code);
    http.end();
    return false;
  }
  String body = http.getString();             // about 1.5 KB
  http.end();

  StaticJsonDocument<1024> filter;
  JsonObject fc = filter["current"].to<JsonObject>();
  fc["time"] = true;
  fc["temperature_2m"] = true;
  fc["relative_humidity_2m"] = true;
  fc["apparent_temperature"] = true;
  fc["precipitation"] = true;
  fc["weather_code"] = true;
  fc["wind_speed_10m"] = true;
  fc["wind_direction_10m"] = true;
  fc["is_day"] = true;
  JsonObject fd = filter["daily"].to<JsonObject>();
  fd["time"] = true;
  fd["weather_code"] = true;
  fd["temperature_2m_max"] = true;
  fd["temperature_2m_min"] = true;
  fd["precipitation_probability_max"] = true;
  fd["precipitation_sum"] = true;
  fd["sunrise"] = true;
  fd["sunset"] = true;
  JsonObject fh = filter["hourly"].to<JsonObject>();
  fh["temperature_2m"] = true;
  fh["precipitation_probability"] = true;
  fh["precipitation"] = true;
  fh["weather_code"] = true;

  DynamicJsonDocument doc(32768);          // 7 days x 24 hours x 4 values is about 13 KB
  DeserializationError err = deserializeJson(doc, body, DeserializationOption::Filter(filter));
  if (err) {
    Serial.print("Weather JSON error: ");
    Serial.println(err.c_str());
    return false;
  }

  JsonObject cur = doc["current"];
  JsonObject dly = doc["daily"];
  if (cur.isNull() || dly.isNull()) return false;

  JsonArray aTime = dly["time"];
  JsonArray aCode = dly["weather_code"];
  JsonArray aHi   = dly["temperature_2m_max"];
  JsonArray aLo   = dly["temperature_2m_min"];
  JsonArray aPop  = dly["precipitation_probability_max"];
  JsonArray aPre  = dly["precipitation_sum"];

  int n = (int)aTime.size();
  if (n > 7) n = 7;
  if (n < 1) return false;

  static WeatherData t;                       // static: keep it off the task stack
  memset(&t, 0, sizeof(t));
  t.valid    = true;
  t.lat      = la;
  t.lon      = lo;
  t.tempC    = cur["temperature_2m"].as<float>();
  t.feelsC   = cur["apparent_temperature"].as<float>();
  t.windMph  = cur["wind_speed_10m"].as<float>();
  t.precipIn = cur["precipitation"].as<float>();
  t.hum      = cur["relative_humidity_2m"].as<int>();
  t.windDir  = cur["wind_direction_10m"].as<int>();
  t.code     = cur["weather_code"].as<int>();
  t.isDay    = cur["is_day"].as<int>() != 0;
  t.dayCount = n;

  t.sunriseMin = -1;                                              // today's sunrise / sunset ("2026-10-25T07:12")
  t.sunsetMin = -1;
  const char* sr = dly["sunrise"][0] | "";
  const char* ss = dly["sunset"][0] | "";
  if (strlen(sr) >= 16 && strlen(ss) >= 16) {
    t.sunriseMin = atoi(sr + 11) * 60 + atoi(sr + 14);
    t.sunsetMin  = atoi(ss + 11) * 60 + atoi(ss + 14);
  }

  const char* ct = cur["time"].as<const char*>();                 // "2026-10-24T16:15"
  t.nowHour = (ct && strlen(ct) >= 13) ? atoi(ct + 11) : -1;

  // hourly: the arrays start at local midnight of day 0, 24 values per day
  JsonObject hly = doc["hourly"];
  JsonArray hT = hly["temperature_2m"];
  JsonArray hP = hly["precipitation_probability"];
  JsonArray hR = hly["precipitation"];
  JsonArray hC = hly["weather_code"];
  int hn = (int)hT.size();
  if ((int)hP.size() < hn) hn = (int)hP.size();
  if ((int)hR.size() < hn) hn = (int)hR.size();
  if ((int)hC.size() < hn) hn = (int)hC.size();
  if (hn > 168) hn = 168;
  for (int i = 0; i < hn; i++) {
    t.hr[i].tC10    = (int16_t)lroundf(hT[i].as<float>() * 10.0f);
    t.hr[i].pop     = (uint8_t)constrain(hP[i].as<int>(), 0, 100);
    t.hr[i].code    = (uint8_t)hC[i].as<int>();
    t.hr[i].pre1000 = (uint16_t)constrain((int)lroundf(hR[i].as<float>() * 1000.0f), 0, 65535);
  }
  t.hrCount = hn;

  for (int i = 0; i < n; i++) {
    const char* ds = aTime[i].as<const char*>();
    copyStr(t.day[i].date, sizeof(t.day[i].date), ds ? ds : "");
    t.day[i].code     = aCode[i].as<int>();
    t.day[i].hiC      = aHi[i].as<float>();
    t.day[i].loC      = aLo[i].as<float>();
    t.day[i].popMax   = aPop[i].as<int>();
    t.day[i].precipIn = aPre[i].as<float>();
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  wx = t;
  xSemaphoreGive(dataMutex);

  if (t.sunriseMin >= 0 && t.sunsetMin > t.sunriseMin &&
      fabsf(la - (float)homeLat) < 0.0001f && fabsf(lo - (float)homeLon) < 0.0001f) {   // v10: keeps auto-dim current
    sunriseMin = t.sunriseMin;
    sunsetMin = t.sunsetMin;
    sunKnown = true;
    sunNextCheckMs = millis() + 6UL * 3600UL * 1000UL;
  }

  Serial.printf("Weather ok: %.1fC, %d%% humidity, %d days, %d hourly\n", (double)t.tempC, t.hum, n, hn);
  return true;
}

void wxFetchTask(void* pv) {
  bool ok = fetchWeatherData();
  wxFails = ok ? 0 : (int)wxFails + 1;
  wxDirty = true;
  wxTaskHandle = NULL;
  wxInProgress = false;
  vTaskDelete(NULL);
}

void startWxFetch() {
  if (wxInProgress) return;
  wxInProgress = true;
  wxStartMs = millis();
  lastWxStartMs = wxStartMs;
  BaseType_t ok = xTaskCreatePinnedToCore(wxFetchTask, "wxFetchTask", 16384, NULL, 1, &wxTaskHandle, 0);
  if (ok != pdPASS) {
    wxTaskHandle = NULL;
    wxInProgress = false;
    wxFails = (int)wxFails + 1;
    wxDirty = true;
  }
}

// One network job at a time. Refresh every 15 min; after a failure retry after 15 s, 30 s, 45 s, then every 60 s.
void maybeStartWx() {
  if (wxInProgress || fetchInProgress || lookupInProgress || geoBusy() || extraNetBusy()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long gap = WX_REFRESH_MS;
  if (wxFails > 0) {
    gap = 15000UL * (unsigned long)wxFails;
    if (gap > 60000UL) gap = 60000UL;
  }
  if (lastWxStartMs != 0 && millis() - lastWxStartMs < gap) return;
  startWxFetch();
}

void checkWxWatchdog() {
  if (wxInProgress && millis() - wxStartMs > WX_HARD_TIMEOUT_MS) {
    Serial.println("Weather task hung - killing it");
    if (wxTaskHandle) vTaskDelete(wxTaskHandle);
    wxTaskHandle = NULL;
    wxInProgress = false;
    wxFails = (int)wxFails + 1;
    wxDirty = true;
  }
}

// ---------- weather: small helpers ----------
uint16_t wxCol(int r, int g, int b) { return lcd.color565(r, g, b); }

int cToF(float c) { return (int)lroundf(c * 9.0f / 5.0f + 32.0f); }

// Smooth fonts (anti-aliased). DejaVu72 holds digits only, so the big temperature is drawn as digits + a drawn degree ring.
void wxFont(LGFX_Sprite& g, int px) {
  g.setTextSize(1);
  if (px >= 72)      g.setFont(&fonts::DejaVu72);
  else if (px >= 40) g.setFont(&fonts::DejaVu40);
  else if (px >= 24) g.setFont(&fonts::DejaVu24);
  else if (px >= 18) g.setFont(&fonts::DejaVu18);
  else               g.setFont(&fonts::DejaVu12);
}

// "72" + degree ring + optional unit letter. Returns the x where it ends.
int drawTempStr(LGFX_Sprite& g, int x, int y, int val, char unit, int fontPx, uint16_t col) {
  char b[8];
  snprintf(b, sizeof(b), "%d", val);
  wxFont(g, fontPx);
  g.setTextColor(col);
  g.setTextDatum(top_left);
  g.drawString(b, x, y);
  int w  = g.textWidth(b);
  int fh = g.fontHeight();
  int rr = (fontPx >= 72) ? 8 : (fontPx >= 40 ? 5 : (fontPx >= 24 ? 3 : 2));
  int rx = x + w + rr + 3;
  int ry = y + (fh * 28) / 100;
  g.drawCircle(rx, ry, rr, col);
  g.drawCircle(rx, ry, rr - 1, col);
  int ex = rx + rr + 3;
  if (unit) {
    int uPx = (fontPx >= 72) ? 40 : (fontPx >= 40 ? 24 : (fontPx >= 24 ? 18 : 12));
    wxFont(g, uPx);
    char u[2] = {unit, 0};
    g.drawString(u, ex, y + (fh * 12) / 100);
    ex += g.textWidth(u);
  }
  return ex;
}

// "H 75°F  L 61°F" style line
void wxHiLo(LGFX_Sprite& g, int x, int y, int hi, int lo, char unit, uint16_t col) {
  wxFont(g, 18);
  g.setTextColor(col);
  g.setTextDatum(top_left);
  g.drawString("H", x, y);
  x += g.textWidth("H") + 8;
  x = drawTempStr(g, x, y, hi, 0, 18, col) + 22;
  wxFont(g, 18);
  g.setTextColor(col);
  g.drawString("L", x, y);
  x += g.textWidth("L") + 8;
  drawTempStr(g, x, y, lo, unit, 18, col);
}

int wxGroup(int code) {            // 0 clear, 1 partly cloudy, 2 cloudy, 3 fog, 4 rain, 5 snow, 6 storm
  if (code == 0 || code == 1) return 0;
  if (code == 2) return 1;
  if (code == 3) return 2;
  if (code == 45 || code == 48) return 3;
  if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) return 4;
  if ((code >= 71 && code <= 77) || code == 85 || code == 86) return 5;
  if (code >= 95) return 6;
  return 2;
}

const char* wxName(int code) {
  switch (code) {
    case 0:  return "Clear";
    case 1:  return "Mostly Clear";
    case 2:  return "Partly Cloudy";
    case 3:  return "Overcast";
    case 45: return "Fog";
    case 48: return "Freezing Fog";
    case 51: return "Light Drizzle";
    case 53: return "Drizzle";
    case 55: return "Heavy Drizzle";
    case 56: case 57: return "Freezing Drizzle";
    case 61: return "Light Rain";
    case 63: return "Rain";
    case 65: return "Heavy Rain";
    case 66: case 67: return "Freezing Rain";
    case 71: return "Light Snow";
    case 73: return "Snow";
    case 75: return "Heavy Snow";
    case 77: return "Snow Grains";
    case 80: return "Light Showers";
    case 81: return "Showers";
    case 82: return "Heavy Showers";
    case 85: return "Snow Showers";
    case 86: return "Heavy Snow Showers";
    case 95: return "Thunderstorm";
    case 96: case 99: return "Thunderstorm, Hail";
  }
  return "Unknown";
}

// day of week from "YYYY-MM-DD" (0 = Sunday), -1 if the text is bad
int dowFromDate(const char* s) {
  if (strlen(s) < 10) return -1;
  int y = atoi(s), m = atoi(s + 5), d = atoi(s + 8);
  static const int tt[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 1 || m > 12) return -1;
  if (m < 3) y -= 1;
  return (y + y / 4 - y / 100 + y / 400 + tt[m - 1] + d) % 7;
}

const char* const DOW_SHORT[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char* const COMPASS_8[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
const char* const DOW_FULL[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char* const MON_SHORT[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

// ---------- weather: icons drawn from shapes ----------
void wxSun(LGFX_Sprite& g, int cx, int cy, int u) {
  uint16_t col = wxCol(255, 200, 40);
  g.fillCircle(cx, cy, (u * 45) / 100, col);
  for (int a = 0; a < 8; a++) {
    float r = a * 45 * DEG_TO_RAD;
    int x1 = cx + cosf(r) * u * 0.64f, y1 = cy + sinf(r) * u * 0.64f;
    int x2 = cx + cosf(r) * u * 0.95f, y2 = cy + sinf(r) * u * 0.95f;
    g.drawLine(x1, y1, x2, y2, col);
    g.drawLine(x1 + 1, y1, x2 + 1, y2, col);
    g.drawLine(x1, y1 + 1, x2, y2 + 1, col);
  }
}

void wxMoon(LGFX_Sprite& g, int cx, int cy, int u) {
  g.fillCircle(cx, cy, (u * 60) / 100, wxCol(232, 234, 248));
  g.fillCircle(cx - u / 5, cy - u / 8, u / 9, wxCol(200, 204, 226));
  g.fillCircle(cx + u / 4, cy + u / 6, u / 7, wxCol(200, 204, 226));
}

void wxCloud(LGFX_Sprite& g, int cx, int cy, int u, uint16_t col) {
  g.fillRoundRect(cx - (u * 85) / 100, cy - (u * 5) / 100, (u * 170) / 100, (u * 55) / 100, (u * 27) / 100, col);
  g.fillCircle(cx - (u * 35) / 100, cy - (u * 10) / 100, (u * 38) / 100, col);
  g.fillCircle(cx + (u * 10) / 100, cy - (u * 32) / 100, (u * 52) / 100, col);
  g.fillCircle(cx + (u * 50) / 100, cy - (u * 2) / 100, (u * 32) / 100, col);
}

// grp: 0 clear, 1 partly cloudy, 2 cloudy, 3 fog, 4 rain, 5 snow, 6 storm
void drawWxIcon(LGFX_Sprite& g, int cx, int cy, int u, int grp, bool day) {
  uint16_t cloudCol = wxCol(228, 234, 247);
  uint16_t grayCol  = wxCol(176, 188, 214);
  uint16_t rainCol  = wxCol(96, 170, 255);

  if (grp == 0) {
    if (day) wxSun(g, cx, cy, u); else wxMoon(g, cx, cy, u);
  } else if (grp == 1) {
    if (day) wxSun(g, cx - (u * 28) / 100, cy - (u * 28) / 100, (u * 70) / 100);
    else     wxMoon(g, cx - (u * 28) / 100, cy - (u * 28) / 100, (u * 70) / 100);
    wxCloud(g, cx + (u * 8) / 100, cy + (u * 18) / 100, (u * 82) / 100, cloudCol);
  } else if (grp == 2) {
    wxCloud(g, cx + (u * 14) / 100, cy - (u * 14) / 100, (u * 62) / 100, grayCol);
    wxCloud(g, cx - (u * 6) / 100, cy + (u * 10) / 100, (u * 80) / 100, cloudCol);
  } else if (grp == 3) {
    wxCloud(g, cx, cy - (u * 28) / 100, (u * 72) / 100, grayCol);
    int t = (u * 8) / 100 + 2;
    g.fillRoundRect(cx - (u * 80) / 100, cy + (u * 22) / 100, (u * 160) / 100, t, t / 2, cloudCol);
    g.fillRoundRect(cx - (u * 55) / 100, cy + (u * 50) / 100, (u * 130) / 100, t, t / 2, cloudCol);
    g.fillRoundRect(cx - (u * 80) / 100, cy + (u * 78) / 100, (u * 100) / 100, t, t / 2, cloudCol);
  } else if (grp == 4) {
    wxCloud(g, cx, cy - (u * 22) / 100, (u * 84) / 100, grayCol);
    for (int i = 0; i < 3; i++) {
      int dx = cx - (u * 42) / 100 + i * (u * 42) / 100;
      int y0 = cy + (u * 40) / 100;
      g.drawLine(dx, y0, dx - (u * 12) / 100, y0 + (u * 34) / 100, rainCol);
      g.drawLine(dx + 1, y0, dx - (u * 12) / 100 + 1, y0 + (u * 34) / 100, rainCol);
      g.drawLine(dx + 2, y0, dx - (u * 12) / 100 + 2, y0 + (u * 34) / 100, rainCol);
    }
  } else if (grp == 5) {
    wxCloud(g, cx, cy - (u * 22) / 100, (u * 84) / 100, grayCol);
    for (int i = 0; i < 3; i++) {
      int dx = cx - (u * 40) / 100 + i * (u * 40) / 100;
      int dy = cy + (u * 52) / 100 + ((i & 1) ? (u * 14) / 100 : 0);
      g.fillCircle(dx, dy, (u * 8) / 100 + 1, TFT_WHITE);
    }
  } else {
    wxCloud(g, cx, cy - (u * 24) / 100, (u * 84) / 100, wxCol(128, 140, 170));
    uint16_t bolt = wxCol(255, 210, 50);
    int bx = cx - (u * 4) / 100;
    int by = cy + (u * 22) / 100;
    g.fillTriangle(bx + (u * 14) / 100, by, bx - (u * 16) / 100, by + (u * 30) / 100, bx + (u * 4) / 100, by + (u * 30) / 100, bolt);
    g.fillTriangle(bx + (u * 4) / 100, by + (u * 22) / 100, bx - (u * 12) / 100, by + (u * 62) / 100, bx + (u * 24) / 100, by + (u * 26) / 100, bolt);
  }
}

// ---------- weather: screen drawing (navy, iPhone-weather style) ----------
void drawWxBackground(LGFX_Sprite& g) {
  for (int y = 0; y < 480; y++) {
    int r = 14 - (9 * y) / 479;
    int gg = 30 - (19 * y) / 479;
    int b = 66 - (36 * y) / 479;
    g.drawFastHLine(0, y, 800, wxCol(r, gg, b));
  }
}

// local time as minutes since midnight, -1 while the clock has not been set yet (never waits for NTP)
int wxClockMinute() {
  struct tm ti;
  if (!getLocalTime(&ti, 0)) return -1;
  return ti.tm_hour * 60 + ti.tm_min;
}

// top-right corner, 24-hour clock
void drawWxClock(LGFX_Sprite& g) {
  char b[8];
  int m = wxClockMinute();
  if (m < 0) snprintf(b, sizeof(b), "--:--");
  else       snprintf(b, sizeof(b), "%02d:%02d", m / 60, m % 60);
  wxFont(g, 24);
  g.setTextDatum(middle_right);
  g.setTextColor(wxCol(245, 248, 255));
  g.drawString(b, 780, 34);
}

void drawWxTopBar(LGFX_Sprite& g) {
  const int tx = 270, ty = 14, tw = 260, th = 40;
  g.fillRoundRect(tx, ty, tw, th, 20, wxCol(22, 38, 82));
  g.drawRoundRect(tx, ty, tw, th, 20, wxCol(60, 84, 140));
  int ax = (wxTab == WX_DAILY) ? tx + 3 : tx + tw / 2;
  g.fillRoundRect(ax, ty + 3, tw / 2 - 3, th - 6, 17, wxCol(66, 104, 184));

  wxFont(g, 18);
  g.setTextDatum(middle_center);
  g.setTextColor(wxTab == WX_DAILY ? wxCol(245, 248, 255) : wxCol(150, 168, 205));
  g.drawString("DAILY", tx + tw / 4, ty + th / 2);
  g.setTextColor(wxTab == WX_WEEKLY ? wxCol(245, 248, 255) : wxCol(150, 168, 205));
  g.drawString("WEEKLY", tx + (tw * 3) / 4, ty + th / 2);
}

void wxCard(LGFX_Sprite& g, int x, int y, int w, int h, const char* title) {
  g.fillRoundRect(x, y, w, h, 16, wxCol(26, 44, 92));
  g.drawRoundRect(x, y, w, h, 16, wxCol(52, 76, 132));
  wxFont(g, 12);
  g.setTextDatum(top_left);
  g.setTextColor(wxCol(150, 168, 205));
  g.drawString(title, x + 14, y + 12);
}

void wxWindArrow(LGFX_Sprite& g, int ccx, int ccy, int r, int dirDeg, uint16_t col) {
  g.drawCircle(ccx, ccy, r, wxCol(70, 98, 160));
  g.drawFastVLine(ccx, ccy - r, 4, wxCol(150, 168, 205));
  float a = (dirDeg + 180) * DEG_TO_RAD;                       // arrow shows where the wind is going
  int tx = ccx + sinf(a) * (r - 6), ty = ccy - cosf(a) * (r - 6);
  int bx = ccx - sinf(a) * (r - 10), by = ccy + cosf(a) * (r - 10);
  g.drawLine(bx, by, tx, ty, col);
  g.drawLine(bx + 1, by, tx + 1, ty, col);
  float pa = a + 2.5f, pb = a - 2.5f;
  int hl = r / 3 + 2;
  g.fillTriangle(tx, ty,
                 tx + sinf(pa) * hl, ty - cosf(pa) * hl,
                 tx + sinf(pb) * hl, ty - cosf(pb) * hl, col);
}

void hourLabel(char* out, size_t n, int h) {                   // 24-hour clock: "15:00"
  snprintf(out, n, "%02d:00", ((h % 24) + 24) % 24);
}

void hourLabelShort(char* out, size_t n, int h) {              // same, for the 3-hour marks of the day view
  hourLabel(out, n, h);
}

// One line about the rain on day d, worked out from the hourly numbers ("Rain 2 PM - 6 PM, 0.31 in total")
void wxRainSummary(int d, char* out, size_t n) {
  int first = -1, last = -1, runs = 0, total = 0, maxPop = 0;
  bool inRun = false;
  for (int h = 0; h < 24; h++) {
    int pop = wxView.hr[d * 24 + h].pop;
    if (pop > maxPop) maxPop = pop;
    bool wet = wxView.hr[d * 24 + h].pre1000 >= 5;
    if (wet) {
      if (first < 0) first = h;
      last = h;
      total += wxView.hr[d * 24 + h].pre1000;
      if (!inRun) { runs++; inRun = true; }
    } else {
      inRun = false;
    }
  }
  if (first < 0) {
    if (maxPop >= 30) snprintf(out, n, "No measurable rain (up to %d%% chance)", maxPop);
    else              snprintf(out, n, "No rain expected");
    return;
  }
  char a[10], b[10];
  hourLabel(a, sizeof(a), first);
  hourLabel(b, sizeof(b), (last + 1) % 24);
  snprintf(out, n, "%s %s - %s, %.2f in total", runs > 1 ? "Rain on and off" : "Rain", a, b, (double)total / 1000.0);
}

// Next hours of today, in order, from now until the end of the day (at most 12)
// day or night at a given hour of today, from the real sunrise / sunset (falls back to 06:00-19:00)
bool wxHourIsDay(int h) {
  if (wxView.sunriseMin >= 0 && wxView.sunsetMin > wxView.sunriseMin) {
    int m = h * 60 + 30;
    return m >= wxView.sunriseMin && m < wxView.sunsetMin;
  }
  return h >= 6 && h < 19;
}

void drawWxHourlyStrip(LGFX_Sprite& g) {
  const int cx0 = 20, cy0 = 282, cw0 = 760, ch0 = 172;
  uint16_t white = wxCol(245, 248, 255);
  uint16_t dim   = wxCol(150, 168, 205);
  uint16_t blue  = wxCol(96, 170, 255);

  g.fillRoundRect(cx0, cy0, cw0, ch0, 16, wxCol(26, 44, 92));
  g.drawRoundRect(cx0, cy0, cw0, ch0, 16, wxCol(52, 76, 132));
  wxFont(g, 12);
  g.setTextDatum(top_left);
  g.setTextColor(dim);
  g.drawString("HOURLY", cx0 + 14, cy0 + 10);

  if (wxView.hrCount < 24 || wxView.nowHour < 0) {
    wxFont(g, 18);
    g.setTextDatum(middle_center);
    g.drawString("Hourly data unavailable", cx0 + cw0 / 2, cy0 + ch0 / 2);
    return;
  }

  const int slotW = 61;
  int first = wxView.nowHour;
  int n = 24 - first;
  if (n > 12) n = 12;

  for (int i = 0; i < n; i++) {
    int h = first + i;
    int sx = cx0 + 14 + i * slotW + slotW / 2;

    if (i == 0) g.fillRoundRect(sx - 29, cy0 + 26, 58, 140, 10, wxCol(38, 62, 118));   // the hour it is now

    char lb[10];
    hourLabel(lb, sizeof(lb), h);
    wxFont(g, 12);
    g.setTextDatum(top_center);
    g.setTextColor(i == 0 ? white : dim);
    g.drawString(lb, sx, cy0 + 30);

    drawWxIcon(g, sx, cy0 + 76, 17, wxGroup(wxView.hr[h].code), wxHourIsDay(h));

    drawTempStr(g, sx - 14, cy0 + 102, cToF(wxView.hr[h].tC10 / 10.0f), 0, 18, white);

    int pop = wxView.hr[h].pop;
    bool wet = wxView.hr[h].pre1000 >= 5;
    if (pop >= 10) {
      char pb[8];
      snprintf(pb, sizeof(pb), "%d%%", pop);
      wxFont(g, 12);
      g.setTextDatum(top_center);
      g.setTextColor(wet ? blue : dim);
      g.drawString(pb, sx, cy0 + 128);
    }
    int bh = (pop * 16) / 100;                       // little chance-of-rain bar, bright when rain is measurable
    if (wet && bh < 4) bh = 4;
    if (bh > 0) g.fillRect(sx - 5, cy0 + 164 - bh, 10, bh, wet ? blue : wxCol(52, 80, 140));
  }
}

void drawWxDaily(LGFX_Sprite& g) {
  uint16_t white = wxCol(245, 248, 255);
  uint16_t dim   = wxCol(150, 168, 205);

  int tF = cToF(wxView.tempC);
  int tC = (int)lroundf(wxView.tempC);

  // hero block
  wxFont(g, 24);
  g.setTextDatum(top_left);
  g.setTextColor(white);
  g.drawString(placeName, 34, 74);

  drawTempStr(g, 30, 100, tF, 'F', 72, white);
  drawTempStr(g, 232, 128, tC, 'C', 24, dim);

  wxFont(g, 24);
  g.setTextColor(wxCol(205, 216, 240));
  g.setTextDatum(top_left);
  g.drawString(wxName(wxView.code), 34, 188);

  if (wxView.dayCount > 0) {
    wxHiLo(g, 34, 224, cToF(wxView.day[0].hiC), cToF(wxView.day[0].loC), 'F', wxCol(205, 216, 240));
    wxHiLo(g, 34, 248, (int)lroundf(wxView.day[0].hiC), (int)lroundf(wxView.day[0].loC), 'C', dim);
  }

  drawWxIcon(g, 358, 150, 46, wxGroup(wxView.code), wxView.isDay);

  // compact tiles (2 x 2)
  const int cw = 172, ch = 96;
  const int x1 = 420, x2 = 608, ya = 72, yb = 176;
  char b[24];

  wxCard(g, x1, ya, cw, ch, "FEELS LIKE");
  drawTempStr(g, x1 + 14, ya + 28, cToF(wxView.feelsC), 'F', 40, white);
  drawTempStr(g, x1 + 14, ya + 76, (int)lroundf(wxView.feelsC), 'C', 12, dim);

  wxCard(g, x2, ya, cw, ch, "WIND");
  snprintf(b, sizeof(b), "%d", (int)lroundf(wxView.windMph));
  wxFont(g, 40);
  g.setTextColor(white);
  g.setTextDatum(top_left);
  g.drawString(b, x2 + 14, ya + 28);
  int ww = g.textWidth(b);
  wxFont(g, 18);
  g.setTextColor(dim);
  g.drawString("mph", x2 + 14 + ww + 6, ya + 50);
  int di = ((wxView.windDir + 22) / 45) % 8;
  snprintf(b, sizeof(b), "from %s", COMPASS_8[di]);
  wxFont(g, 12);
  g.drawString(b, x2 + 14, ya + 76);
  wxWindArrow(g, x2 + 138, ya + 60, 20, wxView.windDir, white);

  wxCard(g, x1, yb, cw, ch, "HUMIDITY");
  snprintf(b, sizeof(b), "%d%%", wxView.hum);
  wxFont(g, 40);
  g.setTextColor(white);
  g.setTextDatum(top_left);
  g.drawString(b, x1 + 14, yb + 28);
  g.fillRoundRect(x1 + 14, yb + 80, 144, 8, 4, wxCol(40, 62, 115));
  int hw = (144 * constrain(wxView.hum, 0, 100)) / 100;
  if (hw > 0) g.fillRoundRect(x1 + 14, yb + 80, hw < 8 ? 8 : hw, 8, 4, wxCol(96, 170, 255));

  wxCard(g, x2, yb, cw, ch, "PRECIPITATION");
  int pop = (wxView.dayCount > 0) ? wxView.day[0].popMax : 0;
  snprintf(b, sizeof(b), "%d%%", pop);
  wxFont(g, 40);
  g.setTextColor(white);
  g.setTextDatum(top_left);
  g.drawString(b, x2 + 14, yb + 28);
  float expected = (wxView.dayCount > 0) ? wxView.day[0].precipIn : 0;
  snprintf(b, sizeof(b), "%.2f in expected", (double)expected);
  wxFont(g, 12);
  g.setTextColor(dim);
  g.drawString(b, x2 + 14, yb + 76);

  drawWxHourlyStrip(g);
}

void drawWxWeekly(LGFX_Sprite& g) {
  uint16_t white = wxCol(245, 248, 255);
  uint16_t dim   = wxCol(150, 168, 205);

  g.fillRoundRect(20, 70, 760, 384, 18, wxCol(26, 44, 92));
  g.drawRoundRect(20, 70, 760, 384, 18, wxCol(52, 76, 132));

  float wMin = 1000, wMax = -1000;
  for (int i = 0; i < wxView.dayCount; i++) {
    if (wxView.day[i].loC < wMin) wMin = wxView.day[i].loC;
    if (wxView.day[i].hiC > wMax) wMax = wxView.day[i].hiC;
  }
  if (wMax - wMin < 1) wMax = wMin + 1;

  const int barX = 372, barW = 210;

  for (int i = 0; i < wxView.dayCount; i++) {
    int ry = 76 + i * 54;
    int rc = ry + 27;
    const WxDay& d = wxView.day[i];

    if (i > 0) g.drawFastHLine(44, ry, 712, wxCol(44, 66, 118));

    // day name
    wxFont(g, 24);
    g.setTextDatum(middle_left);
    g.setTextColor(white);
    int dw = dowFromDate(d.date);
    g.drawString(i == 0 ? "Today" : (dw >= 0 ? DOW_SHORT[dw] : "---"), 44, rc);

    // icon + chance of rain
    drawWxIcon(g, 186, rc, 19, wxGroup(d.code), true);
    if (d.popMax >= 20) {
      char pb[8];
      snprintf(pb, sizeof(pb), "%d%%", d.popMax);
      wxFont(g, 18);
      g.setTextDatum(middle_left);
      g.setTextColor(wxCol(96, 170, 255));
      g.drawString(pb, 222, rc);
    }

    // low  [range bar]  high
    drawTempStr(g, 288, rc - 14, cToF(d.loC), 0, 24, dim);
    g.fillRoundRect(barX, rc - 4, barW, 8, 4, wxCol(40, 62, 115));
    int sx = barX + (int)(((d.loC - wMin) / (wMax - wMin)) * barW);
    int ex = barX + (int)(((d.hiC - wMin) / (wMax - wMin)) * barW);
    if (ex - sx < 8) ex = sx + 8;
    if (ex > barX + barW) { ex = barX + barW; sx = ex - 8; }
    for (int px = sx; px < ex; px++) {
      float t = (float)(px - barX) / (float)barW;               // cold = blue, warm = amber
      g.drawFastVLine(px, rc - 4, 8, wxCol(90 + (int)(165 * t), 170 + (int)(20 * t), 255 - (int)(215 * t)));
    }
    drawTempStr(g, 602, rc - 14, cToF(d.hiC), 0, 24, white);

    // same range in Celsius
    char cb[16];
    snprintf(cb, sizeof(cb), "%d-%dC", (int)lroundf(d.loC), (int)lroundf(d.hiC));
    wxFont(g, 18);
    g.setTextDatum(middle_left);
    g.setTextColor(dim);
    g.drawString(cb, 676, rc);

    g.drawLine(764, rc - 6, 770, rc, dim);               // tap a day for its hours
    g.drawLine(770, rc, 764, rc + 6, dim);
  }
}

// WEEKLY tab, after tapping a day: all 24 hours of that day
void drawWxDayDetail(LGFX_Sprite& g) {
  uint16_t white = wxCol(245, 248, 255);
  uint16_t dim   = wxCol(150, 168, 205);
  uint16_t blue  = wxCol(96, 170, 255);
  int d = wxSelDay;

  g.fillRoundRect(20, 70, 760, 384, 18, wxCol(26, 44, 92));
  g.drawRoundRect(20, 70, 760, 384, 18, wxCol(52, 76, 132));

  // back to the week list
  g.drawLine(46, 88, 38, 96, blue);
  g.drawLine(47, 88, 39, 96, blue);
  g.drawLine(38, 96, 46, 104, blue);
  g.drawLine(39, 96, 47, 104, blue);
  wxFont(g, 18);
  g.setTextDatum(middle_left);
  g.setTextColor(blue);
  g.drawString("Week", 56, 96);

  // title + the day's high / low
  int dw = dowFromDate(wxView.day[d].date);
  int mo = atoi(wxView.day[d].date + 5);
  int dd = atoi(wxView.day[d].date + 8);
  char title[40];
  snprintf(title, sizeof(title), "%s, %s %d", d == 0 ? "Today" : (dw >= 0 ? DOW_FULL[dw] : "---"),
           (mo >= 1 && mo <= 12) ? MON_SHORT[mo - 1] : "", dd);
  wxFont(g, 24);
  g.setTextDatum(middle_center);
  g.setTextColor(white);
  g.drawString(title, 400, 96);
  wxHiLo(g, 590, 86, cToF(wxView.day[d].hiC), cToF(wxView.day[d].loC), 'F', dim);

  if (wxView.hrCount < (d + 1) * 24) {
    wxFont(g, 18);
    g.setTextDatum(middle_center);
    g.setTextColor(dim);
    g.drawString("Hourly data unavailable", 400, 250);
    return;
  }

  // one-line rain summary
  char sm[64];
  wxRainSummary(d, sm, sizeof(sm));
  wxFont(g, 18);
  g.setTextDatum(middle_center);
  g.setTextColor(strstr(sm, "Rain") == sm ? blue : dim);
  g.drawString(sm, 400, 124);

  const int X0 = 52, CW = 29;
  const int yB = 392, BH = 90;

  // icons every 3 hours
  for (int h = 0; h < 24; h += 3)
    drawWxIcon(g, X0 + h * CW + CW / 2, 160, 12, wxGroup(wxView.hr[d * 24 + h].code), wxHourIsDay(h));

  // temperature line (F)
  float tf[24];
  float tmn = 1000, tmx = -1000;
  for (int h = 0; h < 24; h++) {
    tf[h] = wxView.hr[d * 24 + h].tC10 * 0.18f + 32.0f;
    if (tf[h] < tmn) tmn = tf[h];
    if (tf[h] > tmx) tmx = tf[h];
  }
  if (tmx - tmn < 4) { float mid = (tmx + tmn) / 2; tmn = mid - 2; tmx = mid + 2; }
  uint16_t lineCol = wxCol(170, 205, 255);
  int px0 = 0, py0 = 0;
  for (int h = 0; h < 24; h++) {
    int px = X0 + h * CW + CW / 2;
    int py = 274 - (int)(((tf[h] - tmn) / (tmx - tmn)) * 60.0f);
    if (h > 0) {
      g.drawLine(px0, py0, px, py, lineCol);
      g.drawLine(px0, py0 + 1, px, py + 1, lineCol);
    }
    px0 = px;
    py0 = py;
  }
  for (int h = 0; h < 24; h++) {
    int px = X0 + h * CW + CW / 2;
    int py = 274 - (int)(((tf[h] - tmn) / (tmx - tmn)) * 60.0f);
    g.fillCircle(px, py, 3, white);
    if (h % 3 == 0) drawTempStr(g, px - 8, py - 22, (int)lroundf(tf[h]), 0, 12, white);
  }

  // chance-of-rain bars; bright blue + amount when rain is measurable
  g.drawFastHLine(X0, yB, 24 * CW, wxCol(70, 98, 160));
  for (int h = 0; h < 24; h++) {
    int cx = X0 + h * CW + CW / 2;
    int pop = wxView.hr[d * 24 + h].pop;
    int pre = wxView.hr[d * 24 + h].pre1000;
    bool wet = pre >= 5;
    int bh = (pop * BH) / 100;
    if (wet && bh < 14) bh = 14;
    if (bh > 0) {
      uint16_t bc = wet ? blue : wxCol(52, 80, 140);
      if (bh >= 8) g.fillRoundRect(cx - 10, yB - bh, 20, bh, 3, bc);
      else         g.fillRect(cx - 10, yB - bh, 20, bh, bc);
    }
    if (wet) {
      char lb[8];
      int hund = (pre + 5) / 10;
      if (hund > 99) hund = 99;
      snprintf(lb, sizeof(lb), ".%02d", hund);
      wxFont(g, 12);
      g.setTextDatum(top_center);
      g.setTextColor(blue);
      g.drawString(lb, cx, yB - bh - 15);
    }
    g.drawFastVLine(cx, yB, 4, dim);
    if (h % 3 == 0) {
      char hb[8];
      hourLabelShort(hb, sizeof(hb), h);
      wxFont(g, 12);
      g.setTextDatum(top_center);
      g.setTextColor(dim);
      g.drawString(hb, cx, yB + 5);
    }
  }

  // "now" marker on today
  if (d == 0 && wxView.nowHour >= 0 && wxView.nowHour < 24) {
    int nx = X0 + wxView.nowHour * CW + CW / 2;
    uint16_t amber = wxCol(255, 190, 60);
    for (int y = 140; y < yB; y += 8) g.drawFastVLine(nx, y, 4, amber);
    wxFont(g, 12);
    g.setTextDatum(top_center);
    g.setTextColor(amber);
    g.drawString("NOW", nx, yB + 22);
  }

  wxFont(g, 12);
  g.setTextDatum(top_center);
  g.setTextColor(wxCol(110, 128, 168));
  g.drawString("Bar height = chance of rain.  Bright blue = measurable rain, amount in inches.", 400, 430);
}


void drawWeatherScreen() {
  if (!wxCanvasOk) return;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  wxView = wx;
  xSemaphoreGive(dataMutex);

  LGFX_Sprite& g = wxCanvas;
  drawWxBackground(g);
  drawWxTopBar(g);
  drawWxClock(g);

  bool wifiUp = (WiFi.status() == WL_CONNECTED);

  if (!wxView.valid) {
    wxFont(g, 24);
    g.setTextDatum(middle_center);
    g.setTextColor(wxCol(245, 248, 255));
    const char* msg = "Loading weather...";
    const char* sub = "";
    if (!wifiUp)          { msg = "No WiFi";             sub = "Reconnecting..."; }
    else if (wxFails > 0) { msg = "Weather unavailable"; sub = "Retrying..."; }
    g.drawString(msg, 400, 236);
    wxFont(g, 18);
    g.setTextColor(wxCol(150, 168, 205));
    g.drawString(sub, 400, 274);
  } else if (wxTab == WX_DAILY) {
    drawWxDaily(g);
  } else if (wxSelDay >= 0 && wxSelDay < wxView.dayCount) {
    drawWxDayDetail(g);
  } else {
    drawWxWeekly(g);
  }

  // footer: data credit, and a note when the latest refresh failed (the old numbers stay up)
  wxFont(g, 12);
  g.setTextDatum(top_center);
  g.setTextColor(wxCol(110, 128, 168));
  g.drawString("Weather data by Open-Meteo.com", 400, 462);
  if (wxView.valid && (wxFails > 0 || !wifiUp)) {
    g.setTextDatum(top_left);
    g.setTextColor(wxCol(255, 190, 60));
    g.drawString(wifiUp ? "UPDATE FAILED - RETRYING" : "NO WIFI - SHOWING LAST DATA", 20, 462);
  }

  drawSwitcher(g, wxCol(90, 112, 170), wxCol(245, 248, 255), true);
  g.pushSprite(0, 0);
}

// ---------- screen switcher (top-left corner) ----------
// The radar draws its own corner text (REAL ADS-B / AC count), so it only shows the number while switching.
void drawSwitcher(LGFX_Sprite& g, uint16_t idleCol, uint16_t numCol, bool smooth) {
  if (swShown) {
    char b[4];
    snprintf(b, sizeof(b), "%d", swNum);
    g.setTextDatum(top_left);
    g.setTextColor(numCol);
    if (smooth) {
      g.setFont(&fonts::DejaVu40);
      g.setTextSize(1);
      g.drawString(b, 24, 14);
    } else {
      g.setFont(&fonts::Font0);
      g.setTextSize(6);
      g.drawString(b, 18, 12);
      g.setTextSize(1);
    }
  } else if (smooth) {
    g.drawRoundRect(14, 14, 24, 16, 3, idleCol);        // faint "screens" mark (weather screen only)
    g.drawRoundRect(22, 22, 24, 16, 3, idleCol);
  }
}

bool netJobRunning() {
  return fetchInProgress || lookupInProgress || geoState == GEO_RUNNING || wxInProgress || extraNetBusy();
}

// Leaving the radar: close whatever panel was open and cancel anything that hasn't started yet
void exitRadar() {
  photoMode = false;                      // v11
  photoWanted = false;
  metarWanted = false;
  menuMode = MENU_NONE;
  hasSelection = false;
  selectedHex[0] = '\0';
  if (pendingSlot >= 0) { infoCache[pendingSlot].state = 0; pendingSlot = -1; }
  if (geoState == GEO_WAITING) { geoState = GEO_IDLE; geoSunOnly = false; geoTzOnly = false; }
  favMode = 0;
  refetchNow = false;
  rangeApplyWanted = false;
}

// Entering the radar: start clean, like a fresh boot (new plane pool, immediate API call)
void enterRadar() {
  lcd.fillScreen(TFT_BLACK);
  radarEnteredMs = millis();
  planeCount = 0;
  pendingPlaneCount = 0;
  pendingDataReady = false;
  consecutiveFails = 0;
  lastGoodFetchMs = 0;
  lastFetchStartMs = 0;               // 0 = the next maybeStartFetch() fires right away
  refetchNow = false;
  rangeApplyWanted = true;            // show the first pool as soon as it lands
  lookupDone = false;
  listPage = 0;
  menuMode = MENU_NONE;
  hasSelection = false;
  selectedHex[0] = '\0';
  lastClockUpdate = 0;
  photoMode = false;                  // v11
  drawRightInfoPanel();
}

void enterWeather() {
  if (!wxCanvasOk) {
    wxCanvas.setPsram(true);
    wxCanvas.setColorDepth(16);
    wxCanvasOk = wxCanvas.createSprite(800, 480);
    if (!wxCanvasOk) Serial.println("Weather sprite create failed!");
  }
  // data for a different place is no use: drop it so the screen says "Loading" instead of showing the wrong city
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (wx.valid && (fabsf(wx.lat - (float)homeLat) > 0.0001f || fabsf(wx.lon - (float)homeLon) > 0.0001f)) wx.valid = false;
  xSemaphoreGive(dataMutex);

  wxTab = WX_DAILY;
  wxSelDay = -1;
  wxFails = 0;
  lastWxStartMs = 0;                  // 0 = fetch right away
  wxDirty = false;
  if (wxCanvasOk) drawWeatherScreen();
  else {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setTextSize(2);
    lcd.setTextDatum(middle_center);
    lcd.drawString("Not enough memory for the weather screen", 400, 240);
  }
}

void doSwitch(int target) {
  if (curScreen == SCREEN_RADAR) exitRadar();
  curScreen = target;
  swShown = false;
  swLeaving = false;
  Serial.printf("Screen -> %d\n", target + 1);
  saveLastScreen(target);                   // v10: the next boot opens on this screen
  if (target == SCREEN_RADAR) enterRadar();
  else                        enterWeather();
}

void switcherTap() {
  if (swLeaving) return;
  if (!swShown) { swShown = true; swNum = curScreen + 1; }
  else          { swNum = (swNum % NUM_SCREENS) + 1; }
  swLastTapMs = millis();
  wxDirty = true;
}

// Runs every loop: after SW_HOLD_MS with no taps, go to the number on display (once the leaving screen's network job is done)
void serviceSwitcher() {
  if (!swShown) return;
  if (!swLeaving) {
    if (millis() - swLastTapMs < SW_HOLD_MS) return;
    if (swNum - 1 == curScreen) {          // tapped once and waited: stay here
      swShown = false;
      wxDirty = true;
      return;
    }
    swLeaving = true;
    swTarget = swNum - 1;
  }
  if (netJobRunning()) return;             // let the in-flight request finish (the watchdogs still cover hangs)
  doSwitch(swTarget);
}

void weatherOnTap(int x, int y) {
  if (y <= 70 && x >= 270 && x <= 530) {
    int t = (x < 400) ? WX_DAILY : WX_WEEKLY;
    if (t != wxTab) { wxTab = t; wxSelDay = -1; wxDirty = true; }
    else if (t == WX_WEEKLY && wxSelDay >= 0) { wxSelDay = -1; wxDirty = true; }   // WEEKLY tab again = back to the week
    return;
  }
  if (wxTab != WX_WEEKLY || !wxView.valid) return;

  if (wxSelDay >= 0) {                                                 // in a day's hours: top-left area goes back
    if (x >= 20 && x <= 220 && y >= 82 && y <= 118) { wxSelDay = -1; wxDirty = true; }
  } else if (x >= 20 && x <= 780 && y >= 76) {                         // week list: tap a row
    int i = (y - 76) / 54;
    if (i >= 0 && i < wxView.dayCount && i < 7) { wxSelDay = i; wxDirty = true; }
  }
}

// Every tap comes here first: the top-left corner switches screens, everything else goes to the active screen
void onTap(int x, int y) {
  if (x <= SW_ZONE_W && y <= SW_ZONE_H) {
    lastInteractMs = millis();
    switcherTap();
    return;
  }
  if (curScreen == SCREEN_RADAR) radarOnTap(x, y);
  else weatherOnTap(x, y);
}

void weatherLoop(bool allowNet) {
  if (allowNet) maybeStartWx();

  static bool lastWifiUp = true;
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  if (wifiUp != lastWifiUp) { lastWifiUp = wifiUp; wxDirty = true; }

  static unsigned long lastClkCheck = 0;
  static int lastClkMin = -2;
  if (millis() - lastClkCheck >= 1000) {            // the clock only needs a redraw when the minute changes
    lastClkCheck = millis();
    int m = wxClockMinute();
    if (m != lastClkMin) { lastClkMin = m; wxDirty = true; }
  }

  if (wxDirty) {
    wxDirty = false;
    drawWeatherScreen();
  }
}

void handleTouch() {
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll < 20) return;
  lastPoll = millis();

  int x, y;
  if (touchTapStarted(&x, &y)) {
    lastTouchMs = millis();                 // v10: wakes the display from auto-dim
    onTap(x, y);
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  Serial.println("Radar " RADAR_VERSION);

  loadSettings();
  loadFavs();
  Serial.printf("Range %d km, altitude bands 0x%02X, center %s (%.4f, %.4f)\n",
                (int)rangeKm, bandMask(), placeName, (double)homeLat, (double)homeLon);

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
    configTzTime(tzString, "pool.ntp.org", "time.nist.gov");
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

  if (loadLastScreen() == SCREEN_WEATHER) doSwitch(SCREEN_WEATHER);    // v10: reopen on the screen it was left on
}

// ---------- LOOP ----------
// The radar screen's own work (this is the whole v6 loop, minus the pieces every screen shares).
// allowNet = false while the radar is on its way out, so no new network job starts.
void radarLoop(bool allowNet) {
  if (allowNet) maybeStartFetch();
  checkSafetyNet();
  if (allowNet) maybeStartLookup();
  if (allowNet) { maybeStartPhoto(); maybeStartMetar(); }                       // v11
  if (photoDone) {
    photoDone = false;
    if (photoMode && hasSelection && strcmp(photoHex, selSnap.hex) == 0) drawPhotoZone();
  }
  if (metarDone) {
    metarDone = false;
    if (menuMode == MENU_AIRPORT) drawInfoTop();
  }
  if (lookupDone) {
    lookupDone = false;
    if (hasSelection) drawInfoTop();   // route/model just arrived
  }

  if ((hasSelection || menuMode != MENU_NONE) && millis() - lastInteractMs > SELECTION_TIMEOUT_MS) {
    if (menuMode != MENU_NONE) closeMenu();
    else clearSelection();
  }

  // Location lookup (place name + time zone) for the compass screen
  if (allowNet) maybeStartGeo();
  if (geoState == GEO_OK || geoState == GEO_FAIL) handleGeoResult();
  if (allowNet) maybeScheduleSunCheck();     // v10
  if (allowNet) maybeScheduleTzCheck();

  serviceTerrain(allowNet);                  // v11
  {
    static int lastTerrSt = -1;              // keep the menu's TERRAIN bar in step with the map
    int st = !terrainOn ? 0 : (terrainKeyIsBuilt() ? 2 : (terrainGaveUp ? 3 : 1));
    if (st != lastTerrSt) {
      lastTerrSt = st;
      if (menuMode == MENU_MAIN) drawInfoTop();
    }
  }

  // After a zoom or location change, show the fresh pool as soon as it lands instead of waiting for the sweep to wrap
  if (rangeApplyWanted && pendingDataReady && pendingRangeKm == rangeKm &&
      pendingLat == homeLat && pendingLon == homeLon) {
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

void loop() {
  // shared by every screen
  ensureWifi();
  checkFetchWatchdog();
  checkLookupWatchdog();
  checkGeoWatchdog();
  checkWxWatchdog();
  checkNewJobWatchdogs();                    // v11
  handleTouch();
  serviceBrightness();                       // v10: auto-dim (every screen)

  // only the active screen runs
  bool allowNet = !swLeaving;
  if (curScreen == SCREEN_RADAR) radarLoop(allowNet);
  else                           weatherLoop(allowNet);

  serviceSwitcher();
}
