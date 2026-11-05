/*
  ClockCaster - radio-controlled clock time-signal emulator
  for M5Stack Cardputer-Adv + Cap LoRa-1262 (GPS time source)
  -----------------------------------------------------------
  Signals:   WWVB 60 kHz (USA)   DCF77 77.5 kHz (Europe)   MSF 60 kHz (UK)
             JJY 40 kHz (Japan E)   JJY 60 kHz (Japan W)

  The speaker plays a tone at 1/3 (DCF77: 1/5) of the carrier frequency;
  the speaker coil radiates the harmonic at the real carrier frequency.
  A true carrier is also keyed on Grove pin G2 (wire loop + ~330 ohm to GND).

  Keys:  ; .  (up / down)      move between rows
         , /  (left / right)   change the selected setting
         Enter / Space / G0    start or stop the sound
         -  and  =/+           volume down / up
  Settings are saved. Sound always starts OFF at power-up.

  Libraries: M5Cardputer (1.1+), M5Unified, M5GFX, TinyGPSPlus
  Board: M5Cardputer (Arduino-ESP32 3.x)
*/

#include <M5Cardputer.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>

// ===================== Settings =====================
static constexpr uint32_t SAMPLE_RATE     = 48000;
static constexpr int      NMEA_LATENCY_MS = 100;   // NMEA arrives this long after the second starts
static constexpr int      AUDIO_LEAD_MS   = 40;    // compensates for audio buffering delay
static constexpr float    TONE_AMPLITUDE  = 0.85f; // headroom so the amp does not clip

static constexpr int      GPS_RX_PIN      = 15;    // ESP32 RX <- GPS TX (swap with TX if no data)
static constexpr int      GPS_TX_PIN      = 13;    // ESP32 TX -> GPS RX
static constexpr uint32_t GPS_BAUD        = 115200;

static constexpr bool     GPIO_FALLBACK   = true;  // also key the real carrier on a GPIO
static constexpr int      GPIO_PIN        = 2;     // Grove port G2

static constexpr int      DEFAULT_VOLUME  = 30;    // percent
// =====================================================

// ---------------- Stations ----------------
enum Station : uint8_t { ST_WWVB, ST_DCF77, ST_MSF, ST_JJY40, ST_JJY60, ST_COUNT };

struct StationInfo {
  const char *name;
  const char *region;
  uint32_t    carrierHz;
  float       lowLevel;     // carrier level during "reduced" periods
  uint8_t     nativeZone;   // index into ZONES used by "Auto"
};

// ---------------- Time zones ----------------
enum DstRule : uint8_t { R_NONE, R_EU, R_US, R_AU, R_NZ };

struct Zone {
  const char *name;
  int16_t     stdOffset;    // minutes east of UTC
  DstRule     rule;
};

static const Zone ZONES[] = {
  {"Auto",        0,    R_NONE},   // 0: station's own zone
  {"UTC",         0,    R_NONE},   // 1
  {"London",      0,    R_EU},     // 2
  {"Paris/Berlin",60,   R_EU},     // 3
  {"Athens",      120,  R_EU},     // 4
  {"Moscow",      180,  R_NONE},
  {"Dubai",       240,  R_NONE},
  {"India",       330,  R_NONE},
  {"Bangkok",     420,  R_NONE},
  {"China/SG",    480,  R_NONE},
  {"Tokyo",       540,  R_NONE},   // 10
  {"Brisbane",    600,  R_NONE},
  {"Sydney",      600,  R_AU},
  {"Auckland",    720,  R_NZ},
  {"Honolulu",    -600, R_NONE},
  {"Anchorage",   -540, R_US},
  {"Los Angeles", -480, R_US},
  {"Denver",      -420, R_US},
  {"Phoenix",     -420, R_NONE},
  {"Chicago",     -360, R_US},
  {"New York",    -300, R_US},
  {"Halifax",     -240, R_US},
  {"Sao Paulo",   -180, R_NONE},
};
static constexpr int ZONE_COUNT = sizeof(ZONES) / sizeof(ZONES[0]);
static constexpr int Z_UTC = 1, Z_LONDON = 2, Z_CET = 3, Z_TOKYO = 10;

static const StationInfo STATIONS[ST_COUNT] = {
  {"WWVB",  "US",     60000, 0.14f, Z_UTC},     // -17 dB
  {"DCF77", "EU",  77500, 0.15f, Z_CET},     // ~15 %
  {"MSF",   "UK",      60000, 0.00f, Z_LONDON},  // on/off keying
  {"JJY",   "JP-E", 40000, 0.10f, Z_TOKYO},   // ~10 %
  {"JJY",   "JP-W", 60000, 0.10f, Z_TOKYO},
};

// ---------------- State ----------------
static constexpr int BLOCK_MS      = 50;
static constexpr int SPMS          = SAMPLE_RATE / 1000;     // samples per ms
static constexpr int BLOCK_SAMPLES = BLOCK_MS * SPMS;
static constexpr int NBUF          = 3;
static constexpr float ENV_K       = 0.02f;                  // ~1 ms edge smoothing

static int16_t ring[NBUF][BLOCK_SAMPLES];
static int     ringIdx  = 0;
static float   phase    = 0.0f;
static float   envLevel = 0.0f;

TinyGPSPlus    gps;
HardwareSerial GPSSerial(1);
Preferences    prefs;
M5Canvas       cv(&M5Cardputer.Display);

static bool     haveTime      = false;
static int64_t  anchorUtcMs   = 0;
static uint32_t anchorMillis  = 0;
static int      lastGpsSecond = -1;

static Station  station   = ST_WWVB;
static int      zone      = 0;
static int      volume    = DEFAULT_VOLUME;
static bool     soundOn   = false;
static int      selRow    = 0;           // 0 signal, 1 zone, 2 sound
static bool     streaming = false;
static int64_t  nextBlockMs = 0;
static int      lastDuty  = -1;
static uint32_t volShownUntil = 0;

static int64_t  frameMinute  = INT64_MIN;
static Station  frameStation = ST_COUNT;
static int      frameZone    = -1;
static uint8_t  fa[60], fb[60];

// ---------------- Date helpers ----------------
static int64_t floorDiv(int64_t a, int64_t b) {
  int64_t q = a / b;
  if ((a % b) != 0 && ((a < 0) != (b < 0))) q--;
  return q;
}

static int64_t daysFromCivil(int y, int m, int d) {   // days since 1970-01-01
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const int yoe = (int)(y - era * 400);
  const int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

static void civilFromDays(int64_t z, int &y, int &m, int &d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const int doe = (int)(z - era * 146097);
  const int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const int mp  = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y = (int)(yoe + era * 400) + (m <= 2);
}

static int  weekday(int64_t days) { return (int)((days % 7 + 11) % 7); }   // 0 = Sunday
static bool isLeapYear(int y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }

static int64_t nthSunday(int y, int m, int n) {
  const int64_t first = daysFromCivil(y, m, 1);
  return first + (7 - weekday(first)) % 7 + 7 * (n - 1);
}

static int64_t lastSunday(int y, int m) {
  const int64_t last = daysFromCivil(y, m + 1, 1) - 1;
  return last - weekday(last);
}

// Is daylight saving time in effect at the given UTC minute?
static bool dstActive(DstRule rule, int off, int64_t t) {
  if (rule == R_NONE) return false;
  int y, m, d;
  civilFromDays(floorDiv(t + off, 1440), y, m, d);
  int64_t start, end;
  switch (rule) {
    case R_EU:   // last Sun Mar 01:00 UTC .. last Sun Oct 01:00 UTC
      start = lastSunday(y, 3) * 1440 + 60;
      end   = lastSunday(y, 10) * 1440 + 60;
      return t >= start && t < end;
    case R_US:   // 2nd Sun Mar 02:00 local .. 1st Sun Nov 02:00 local DST
      start = nthSunday(y, 3, 2) * 1440 + 120 - off;
      end   = nthSunday(y, 11, 1) * 1440 + 60 - off;
      return t >= start && t < end;
    case R_AU:   // 1st Sun Oct 02:00 std .. 1st Sun Apr 03:00 DST
      start = nthSunday(y, 10, 1) * 1440 + 120 - off;
      end   = nthSunday(y, 4, 1) * 1440 + 120 - off;
      return t < end || t >= start;
    case R_NZ:   // last Sun Sep 02:00 std .. 1st Sun Apr 03:00 DST
      start = lastSunday(y, 9) * 1440 + 120 - off;
      end   = nthSunday(y, 4, 1) * 1440 + 120 - off;
      return t < end || t >= start;
    default:
      return false;
  }
}

static int effectiveZone() { return zone == 0 ? STATIONS[station].nativeZone : zone; }

static bool zoneDst(int z, int64_t t)   { return dstActive(ZONES[z].rule, ZONES[z].stdOffset, t); }
static int  zoneOffset(int z, int64_t t){ return ZONES[z].stdOffset + (zoneDst(z, t) ? 60 : 0); }

struct Fields { int y, mo, d, h, mi, doy, wday; };

static Fields fieldsOf(int64_t localMinute) {
  Fields f;
  const int64_t day = floorDiv(localMinute, 1440);
  const int mod = (int)(localMinute - day * 1440);
  f.h = mod / 60;
  f.mi = mod % 60;
  civilFromDays(day, f.y, f.mo, f.d);
  f.doy  = (int)(day - daysFromCivil(f.y, 1, 1)) + 1;
  f.wday = weekday(day);
  return f;
}

// ---------------- Bit helpers ----------------
static int bcd(int v) { return ((v / 10) << 4) | (v % 10); }

static void setWeighted(uint8_t *f, int value, const int *pos, const int *w, int n) {
  for (int i = 0; i < n; i++)
    if (value >= w[i]) { f[pos[i]] = 1; value -= w[i]; }
}
static void setLsbFirst(uint8_t *f, int pos, int n, int v) { for (int i = 0; i < n; i++) f[pos + i] = (v >> i) & 1; }
static void setMsbFirst(uint8_t *f, int pos, int n, int v) { for (int i = 0; i < n; i++) f[pos + i] = (v >> (n - 1 - i)) & 1; }
static int  ones(const uint8_t *f, int from, int to) { int c = 0; for (int i = from; i <= to; i++) c += f[i] & 1; return c; }

static const int MIN_P[] = {1, 2, 3, 5, 6, 7, 8},  MIN_W[] = {40, 20, 10, 8, 4, 2, 1};
static const int HR_P[]  = {12, 13, 15, 16, 17, 18}, HR_W[] = {20, 10, 8, 4, 2, 1};
static const int DOY_P[] = {22, 23, 25, 26, 27, 28, 30, 31, 32, 33};
static const int DOY_W[] = {200, 100, 80, 40, 20, 10, 8, 4, 2, 1};
static const int YR_P_WWVB[] = {45, 46, 47, 48, 50, 51, 52, 53};
static const int YR_P_JJY[]  = {41, 42, 43, 44, 45, 46, 47, 48};
static const int YR_W[]      = {80, 40, 20, 10, 8, 4, 2, 1};

// ---------------- Frame builders ----------------
// WWVB always carries UTC; the zone only decides the DST bits.
static void buildWWVB(int64_t mi) {
  const Fields t = fieldsOf(mi);
  const int64_t day = floorDiv(mi, 1440);
  DstRule rule = R_US;
  int off = -300;
  if (zone != 0) { rule = ZONES[zone].rule; off = ZONES[zone].stdOffset; }
  for (int p : {0, 9, 19, 29, 39, 49, 59}) fa[p] = 2;
  setWeighted(fa, t.mi, MIN_P, MIN_W, 7);
  setWeighted(fa, t.h, HR_P, HR_W, 6);
  setWeighted(fa, t.doy, DOY_P, DOY_W, 10);
  fa[36] = 1; fa[38] = 1;                               // DUT1 "+0.0"
  setWeighted(fa, t.y % 100, YR_P_WWVB, YR_W, 8);
  fa[55] = isLeapYear(t.y);
  fa[57] = dstActive(rule, off, (day + 1) * 1440);      // DST at 24:00Z
  fa[58] = dstActive(rule, off, day * 1440);            // DST at 00:00Z
}

// DCF77: local time of the NEXT minute.
static void buildDCF77(int64_t mi) {
  const int z = effectiveZone();
  const int64_t enc = mi + 1;
  const bool summer = zoneDst(z, enc);
  const Fields t = fieldsOf(enc + zoneOffset(z, enc));
  fa[16] = zoneDst(z, enc) != zoneDst(z, enc + 60);     // change within the hour
  fa[17] = summer;
  fa[18] = !summer;
  fa[20] = 1;
  setLsbFirst(fa, 21, 7, bcd(t.mi));  fa[28] = ones(fa, 21, 27) & 1;
  setLsbFirst(fa, 29, 6, bcd(t.h));   fa[35] = ones(fa, 29, 34) & 1;
  setLsbFirst(fa, 36, 6, bcd(t.d));
  setLsbFirst(fa, 42, 3, t.wday == 0 ? 7 : t.wday);
  setLsbFirst(fa, 45, 5, bcd(t.mo));
  setLsbFirst(fa, 50, 8, bcd(t.y % 100));
  fa[58] = ones(fa, 36, 57) & 1;
}

// MSF: local time of the NEXT minute.
static void buildMSF(int64_t mi) {
  const int z = effectiveZone();
  const int64_t enc = mi + 1;
  const Fields t = fieldsOf(enc + zoneOffset(z, enc));
  setMsbFirst(fa, 17, 8, bcd(t.y % 100));
  setMsbFirst(fa, 25, 5, bcd(t.mo));
  setMsbFirst(fa, 30, 6, bcd(t.d));
  setMsbFirst(fa, 36, 3, t.wday);
  setMsbFirst(fa, 39, 6, bcd(t.h));
  setMsbFirst(fa, 45, 7, bcd(t.mi));
  for (int i = 53; i <= 58; i++) fa[i] = 1;             // 01111110 minute identifier
  fb[53] = zoneDst(z, enc) != zoneDst(z, enc + 61);
  fb[54] = !(ones(fa, 17, 24) & 1);
  fb[55] = !(ones(fa, 25, 35) & 1);
  fb[56] = !(ones(fa, 36, 38) & 1);
  fb[57] = !(ones(fa, 39, 51) & 1);
  fb[58] = zoneDst(z, enc);
}

// JJY: local time of the current minute.
static void buildJJY(int64_t mi) {
  const int z = effectiveZone();
  const Fields t = fieldsOf(mi + zoneOffset(z, mi));
  for (int p : {0, 9, 19, 29, 39, 49, 59}) fa[p] = 2;
  setWeighted(fa, t.mi, MIN_P, MIN_W, 7);
  setWeighted(fa, t.h, HR_P, HR_W, 6);
  setWeighted(fa, t.doy, DOY_P, DOY_W, 10);
  fa[36] = ones(fa, 12, 18) & 1;
  fa[37] = ones(fa, 1, 8) & 1;
  setWeighted(fa, t.y % 100, YR_P_JJY, YR_W, 8);
  setMsbFirst(fa, 50, 3, t.wday);
}

static void ensureFrame(int64_t mi) {
  if (mi == frameMinute && station == frameStation && zone == frameZone) return;
  memset(fa, 0, sizeof(fa));
  memset(fb, 0, sizeof(fb));
  switch (station) {
    case ST_WWVB:  buildWWVB(mi);  break;
    case ST_DCF77: buildDCF77(mi); break;
    case ST_MSF:   buildMSF(mi);   break;
    default:       buildJJY(mi);   break;
  }
  frameMinute = mi; frameStation = station; frameZone = zone;
}

static bool isHigh(int64_t utcMs) {
  const int64_t mi = floorDiv(utcMs, 60000);
  ensureFrame(mi);
  const int64_t secIdx = floorDiv(utcMs, 1000);
  const int sec = (int)(secIdx - mi * 60);
  const int ms  = (int)(utcMs - secIdx * 1000);
  switch (station) {
    case ST_WWVB: { static const int lowMs[] = {200, 500, 800}; return ms >= lowMs[fa[sec]]; }
    case ST_DCF77:
      if (sec == 59) return true;
      return ms >= (fa[sec] ? 200 : 100);
    case ST_MSF:
      if (sec == 0) return ms >= 500;
      if (ms < 100) return false;
      if (ms < 200) return !fa[sec];
      if (ms < 300) return !fb[sec];
      return true;
    default: { static const int highMs[] = {800, 500, 200}; return ms < highMs[fa[sec]]; }
  }
}

static char symbolChar(int64_t utcMs) {
  const int64_t mi = floorDiv(utcMs, 60000);
  ensureFrame(mi);
  const int sec = (int)(floorDiv(utcMs, 1000) - mi * 60);
  if (station == ST_MSF) return "0123"[fa[sec] | (fb[sec] << 1)];
  if (station == ST_DCF77 && sec == 59) return '-';
  return "01M"[fa[sec]];
}

static float toneHz() {
  const float carrier = (float)STATIONS[station].carrierHz;
  const float limit = 0.45f * SAMPLE_RATE;
  return (carrier / 3.0f < limit) ? carrier / 3.0f : carrier / 5.0f;
}

// ---------------- Time keeping ----------------
static int64_t nowUtcMs() { return anchorUtcMs + (int64_t)(uint32_t)(millis() - anchorMillis); }

static void readGps() {
  while (GPSSerial.available()) {
    if (!gps.encode(GPSSerial.read())) continue;
    if (!gps.time.isUpdated()) continue;
    const uint32_t rxMillis = millis();
    const int s = gps.time.second();
    if (!(gps.time.isValid() && gps.date.isValid() && gps.location.isValid() &&
          gps.location.age() < 3000 && gps.date.year() >= 2024)) continue;
    if (s == lastGpsSecond) continue;
    lastGpsSecond = s;
    const int64_t days = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
    const int64_t sec  = days * 86400 + gps.time.hour() * 3600 + gps.time.minute() * 60 + s;
    anchorUtcMs  = sec * 1000 + NMEA_LATENCY_MS;
    anchorMillis = rxMillis;
    haveTime = true;
  }
}

static bool gpsFix() { return gps.location.isValid() && gps.location.age() < 3000; }

// ---------------- Audio / carrier output ----------------
static void applyVolume() {
  const int v = volume * volume * 255 / 10000;          // perceptual curve
  M5Cardputer.Speaker.setVolume(v);
}

static void fillBlock(int16_t *buf, int64_t startMs) {
  const float inc = 2.0f * PI * toneHz() / SAMPLE_RATE;
  const float low = STATIONS[station].lowLevel;
  const float amp = 32767.0f * TONE_AMPLITUDE;
  int idx = 0;
  for (int j = 0; j < BLOCK_MS; j++) {
    const float target = isHigh(startMs + j) ? 1.0f : low;
    for (int k = 0; k < SPMS; k++) {
      envLevel += (target - envLevel) * ENV_K;           // soft edges, no clicks
      buf[idx++] = (int16_t)(sinf(phase) * amp * envLevel);
      phase += inc;
      if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    }
  }
}

static void feedAudio() {
  if (!soundOn || !haveTime || volume == 0) {
    if (streaming) { M5Cardputer.Speaker.stop(0); streaming = false; envLevel = 0.0f; }
    return;
  }
  const int64_t now = nowUtcMs() + AUDIO_LEAD_MS;
  if (!streaming || nextBlockMs < now - 150 || nextBlockMs > now + 250) {
    nextBlockMs = floorDiv(now, BLOCK_MS) * BLOCK_MS;
    streaming = true;
  }
  while (M5Cardputer.Speaker.isPlaying(0) < 2) {
    int16_t *buf = ring[ringIdx];
    ringIdx = (ringIdx + 1) % NBUF;
    fillBlock(buf, nextBlockMs);
    M5Cardputer.Speaker.playRaw(buf, BLOCK_SAMPLES, SAMPLE_RATE, false, 1, 0, false);
    nextBlockMs += BLOCK_MS;
  }
}

static void applyCarrier() {
  if (!GPIO_FALLBACK) return;
  ledcChangeFrequency(GPIO_PIN, STATIONS[station].carrierHz, 2);
  lastDuty = -1;
}

static void updateGpio() {
  if (!GPIO_FALLBACK) return;
  int duty = 0;
  if (soundOn && haveTime) duty = isHigh(nowUtcMs()) ? 2 : 0;
  if (duty != lastDuty) { ledcWrite(GPIO_PIN, duty); lastDuty = duty; }
}

// ---------------- Settings ----------------
static void save() {
  prefs.putUChar("station", station);
  prefs.putUChar("zone", (uint8_t)zone);
  prefs.putUChar("volume", (uint8_t)volume);
}

static void changeStation(int dir) {
  station = (Station)((station + dir + ST_COUNT) % ST_COUNT);
  applyCarrier();
  streaming = false;                 // resync the tone at the new pitch
  M5Cardputer.Speaker.stop(0);
  save();
}

static void changeZone(int dir) {
  zone = (zone + dir + ZONE_COUNT) % ZONE_COUNT;
  save();
}

static void changeVolume(int dir) {
  volume = constrain(volume + dir * 10, 0, 100);
  applyVolume();
  volShownUntil = millis() + 1500;
  save();
}

static void toggleSound() { soundOn = !soundOn; }

// ---------------- Drawing ----------------
static uint16_t C_BG, C_PANEL, C_SEL, C_TEXT, C_DIM, C_ACCENT, C_ON, C_WARN, C_OFF;

static const char *DAYS[]   = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static const char *MONTHS[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                               "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

static void fmtOffset(char *out, size_t n, int off) {
  const char sign = off < 0 ? '-' : '+';
  off = abs(off);
  if (off % 60) snprintf(out, n, "UTC%c%d:%02d", sign, off / 60, off % 60);
  else if (off) snprintf(out, n, "UTC%c%d", sign, off / 60);
  else          snprintf(out, n, "UTC");
}

static void drawBattery(int x, int y) {
  const int lvl = M5Cardputer.Power.getBatteryLevel();
  if (lvl < 0) return;
  cv.drawRoundRect(x, y, 18, 9, 2, C_DIM);
  cv.fillRect(x + 18, y + 3, 2, 3, C_DIM);
  const int w = constrain(lvl, 0, 100) * 14 / 100;
  cv.fillRect(x + 2, y + 2, w, 5, lvl < 20 ? C_WARN : C_TEXT);
}

static void drawRow(int idx, int y, const char *label, const char *value, uint16_t valColor) {
  const bool sel = (idx == selRow);
  if (sel) cv.fillRoundRect(3, y, 234, 19, 4, C_SEL);
  cv.setFont(&fonts::Font2);
  cv.setTextDatum(middle_left);
  cv.setTextColor(sel ? C_ACCENT : C_DIM);
  cv.drawString(label, 10, y + 10);
  cv.setTextDatum(middle_center);
  cv.setTextColor(valColor);
  cv.drawString(value, 152, y + 10);
  if (sel) {
    cv.fillTriangle(72, y + 9, 78, y + 5, 78, y + 13, C_ACCENT);
    cv.fillTriangle(231, y + 9, 225, y + 5, 225, y + 13, C_ACCENT);
  }
}

static void draw() {
  const StationInfo &si = STATIONS[station];
  const int z = effectiveZone();
  cv.fillScreen(C_BG);

  // Header
  cv.fillRect(0, 0, 240, 16, C_PANEL);
  cv.setFont(&fonts::Font2);
  cv.setTextDatum(middle_left);
  cv.setTextColor(C_ACCENT);
  cv.drawString("CLOCKCASTER", 5, 8);
  cv.setTextDatum(middle_right);
  char buf[40];
  const bool fix = gpsFix();
  snprintf(buf, sizeof(buf), "GPS %u", (unsigned)gps.satellites.value());
  cv.setTextColor(fix ? C_ON : C_WARN);
  cv.drawString(buf, 210, 8);
  cv.fillCircle(150, 8, 3, fix ? C_ON : C_WARN);
  drawBattery(216, 4);

  // Clock
  if (haveTime) {
    const int64_t t   = nowUtcMs();
    const int64_t um  = floorDiv(t, 60000);
    const int     off = zoneOffset(z, um);
    const int64_t lt  = t + (int64_t)off * 60000;
    const int64_t day = floorDiv(lt, 86400000LL);
    const int     sod = (int)((lt - day * 86400000LL) / 1000);
    int y, m, d;
    civilFromDays(day, y, m, d);

    cv.setFont(&fonts::Font4);
    cv.setTextDatum(top_left);
    cv.setTextColor(C_TEXT);
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", sod / 3600, (sod / 60) % 60, sod % 60);
    cv.drawString(buf, 6, 21);

    cv.setFont(&fonts::Font2);
    cv.setTextDatum(top_right);
    cv.setTextColor(C_TEXT);
    snprintf(buf, sizeof(buf), "%s %d %s", DAYS[weekday(day)], d, MONTHS[m - 1]);
    cv.drawString(buf, 234, 19);
    char ofs[16];
    fmtOffset(ofs, sizeof(ofs), off);
    snprintf(buf, sizeof(buf), "%s%s", ofs, zoneDst(z, um) ? " DST" : "");
    cv.setFont(&fonts::Font0);
    cv.setTextColor(C_DIM);
    cv.drawString(buf, 234, 37);

    // Minute progress: one tick per second of the time-code frame
    const int sec = (int)(floorDiv(t, 1000) - um * 60);
    for (int i = 0; i < 60; i++) {
      const uint16_t c = i < sec ? (soundOn ? C_ON : C_DIM) : (i == sec ? C_ACCENT : C_PANEL);
      cv.fillRect(6 + i * 38 / 10, 49, 3, 3, c);
    }
  } else {
    cv.setFont(&fonts::Font2);
    cv.setTextDatum(top_left);
    cv.setTextColor(C_WARN);
    cv.drawString(fix ? "Getting GPS time..." : "Searching for GPS...", 6, 22);
    cv.setTextColor(C_DIM);
    cv.setFont(&fonts::Font0);
    cv.drawString("Go near a window or outside", 6, 40);
  }

  // Rows
  snprintf(buf, sizeof(buf), "%s %g kHz %s", si.name, si.carrierHz / 1000.0f, si.region);
  drawRow(0, 56, "Signal", buf, C_TEXT);

  if (zone == 0) snprintf(buf, sizeof(buf), "Auto (%s)", ZONES[z].name);
  else           snprintf(buf, sizeof(buf), "%s", ZONES[zone].name);
  drawRow(1, 76, "Zone", buf, C_TEXT);

  uint16_t sc = C_OFF;
  if (!soundOn)                   snprintf(buf, sizeof(buf), "OFF");
  else if (!haveTime)           { snprintf(buf, sizeof(buf), "ON - waiting GPS"); sc = C_WARN; }
  else if (volume == 0)         { snprintf(buf, sizeof(buf), "ON - muted"); sc = C_WARN; }
  else {
    snprintf(buf, sizeof(buf), "ON  %.1f kHz  [%c]", toneHz() / 1000.0f, symbolChar(nowUtcMs()));
    sc = C_ON;
  }
  drawRow(2, 96, "Sound", buf, sc);

  // Volume bar
  const bool volHot = millis() < volShownUntil;
  cv.setFont(&fonts::Font0);
  cv.setTextDatum(middle_left);
  cv.setTextColor(volHot ? C_ACCENT : C_DIM);
  cv.drawString("VOL", 10, 121);
  for (int i = 0; i < 10; i++) {
    const bool lit = i < volume / 10;
    cv.fillRoundRect(34 + i * 15, 117, 12, 8, 2, lit ? (volHot ? C_ACCENT : C_TEXT) : C_PANEL);
  }
  snprintf(buf, sizeof(buf), "%d%%", volume);
  cv.setTextDatum(middle_right);
  cv.drawString(buf, 234, 121);

  // Footer hints
  cv.setTextDatum(bottom_left);
  cv.setTextColor(C_DIM);
  const int fy = 131;                                    // icon centre line
  cv.fillTriangle(12, fy - 1, 15, fy - 4, 18, fy - 1, C_DIM);   // up
  cv.fillTriangle(20, fy - 2, 23, fy + 1, 26, fy - 2, C_DIM);   // down
  cv.drawString("row", 30, 135);
  cv.fillTriangle(64, fy - 1, 67, fy - 4, 67, fy + 2, C_DIM);   // left
  cv.fillTriangle(74, fy - 4, 77, fy - 1, 74, fy + 2, C_DIM);   // right
  cv.drawString("set", 81, 135);
  cv.drawString("OK on/off", 120, 135);
  cv.drawString("-+ vol", 200, 135);

  cv.pushSprite(0, 0);
}

// ---------------- Input ----------------
static void handleKeys() {
  if (M5Cardputer.BtnA.wasClicked()) toggleSound();

  if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) return;
  auto &ks = M5Cardputer.Keyboard.keysState();

  bool up = ks.up, down = ks.down, left = ks.left, right = ks.right;
  int vol = 0;
  for (char c : ks.word) {
    switch (c) {
      case ';': up = true; break;
      case '.': down = true; break;
      case ',': left = true; break;
      case '/': right = true; break;
      case '-': case '_': vol = -1; break;
      case '=': case '+': vol = +1; break;
    }
  }

  if (ks.enter || ks.space) toggleSound();
  if (vol) changeVolume(vol);
  if (up)   selRow = (selRow + 2) % 3;
  if (down) selRow = (selRow + 1) % 3;
  if (left || right) {
    const int dir = right ? 1 : -1;
    if (selRow == 0)      changeStation(dir);
    else if (selRow == 1) changeZone(dir);
    else                  toggleSound();
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);

  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.fillScreen(TFT_BLACK);
  cv.setColorDepth(16);
  cv.createSprite(240, 135);

  C_BG     = cv.color565(10, 12, 16);
  C_PANEL  = cv.color565(34, 38, 48);
  C_SEL    = cv.color565(44, 50, 64);
  C_TEXT   = cv.color565(236, 238, 242);
  C_DIM    = cv.color565(130, 136, 150);
  C_ACCENT = cv.color565(255, 176, 32);
  C_ON     = cv.color565(64, 210, 120);
  C_WARN   = cv.color565(255, 120, 64);
  C_OFF    = cv.color565(150, 150, 150);

  prefs.begin("rcclock", false);
  const uint8_t s = prefs.getUChar("station", ST_WWVB);
  station = s < ST_COUNT ? (Station)s : ST_WWVB;
  const uint8_t zz = prefs.getUChar("zone", 0);
  zone = zz < ZONE_COUNT ? zz : 0;
  volume = constrain((int)prefs.getUChar("volume", DEFAULT_VOLUME), 0, 100);

  M5Cardputer.Mic.end();
  auto spk = M5Cardputer.Speaker.config();
  spk.sample_rate = SAMPLE_RATE;
  M5Cardputer.Speaker.config(spk);
  M5Cardputer.Speaker.begin();
  M5Cardputer.Speaker.setChannelVolume(0, 255);
  applyVolume();

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (GPIO_FALLBACK) {
    ledcAttach(GPIO_PIN, STATIONS[station].carrierHz, 2);
    ledcWrite(GPIO_PIN, 0);
    lastDuty = 0;
  }
}

void loop() {
  static uint32_t lastDraw = 0;
  M5Cardputer.update();
  handleKeys();
  readGps();
  feedAudio();
  updateGpio();
  if (millis() - lastDraw >= 100) {
    draw();
    lastDraw = millis();
  }
  delay(1);
}
