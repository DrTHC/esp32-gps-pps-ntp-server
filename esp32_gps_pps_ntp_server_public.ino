/*
  ESP32 GPS PPS NTP Server for Arduino IDE

  Required external Arduino library:
    - TinyGPSPlus by Mikal Hart

  Uses only Arduino ESP32 core libraries otherwise:
    WiFi, WebServer, WiFiUdp, ArduinoOTA, HardwareSerial.

  Timing model:
    - The PPS interrupt records only the ESP32 monotonic timestamp of each pulse.
    - TinyGPSPlus parses UTC date/time from NMEA in the main loop.
    - When a valid UTC second is seen shortly after a PPS pulse, that UTC second
      is paired with the captured PPS timestamp after applying the configurable
      PPS_EPOCH_OFFSET_SECONDS association correction. The corrected epoch
      becomes the disciplined reference: unix_us = accepted_epoch * 1,000,000
      at pps_mono_us.
    - Current NTP time is always computed from that accepted reference plus
      elapsed microseconds from esp_timer_get_time().
    - If PPS/GPS later becomes stale, the server enters holdover and continues
      from the ESP32 monotonic timer while exposing degraded status.

  Adjustable settings are near the top of the file.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <TinyGPSPlus.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"

// ----------------------------- User Settings -----------------------------

// Set these for your own network before flashing.
// Keep real Wi-Fi credentials out of public repositories.
static const char WIFI_SSID[] = "YOUR_WIFI_SSID";
static const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

static const uint8_t GPS_RX_PIN = 16;     // ESP32 RX2 receives GPS TX
static const uint8_t GPS_TX_PIN = 17;     // ESP32 TX2 sends to GPS RX, if used
static const uint32_t GPS_BAUD = 9600;
static const uint8_t PPS_PIN = 27;

static const uint16_t WEB_PORT = 80;
static const uint16_t NTP_PORT = 123;
static const uint16_t GPS_BRIDGE_PORT = 5000;

// Named calibration offsets. Leave at 0 unless measured with suitable tools.
static const int64_t PPS_CALIBRATION_US = 0;
static const int64_t GPS_NMEA_RECEIVE_CAL_US = 0;

// Whole-second PPS epoch association correction.
// This value is subtracted from the decoded NMEA UTC epoch when assigning that
// epoch to a captured PPS edge. For this receiver, RMC/ZDA sentences arrive
// after the PPS edge for the same UTC second, so the decoded epoch is assigned
// directly to the most recent PPS edge. Try +1 or -1 only if logs and external
// NTP tests prove your GPS module associates NMEA and PPS differently.
static const int32_t PPS_EPOCH_OFFSET_SECONDS = 0;

// Robustness thresholds.
static const uint32_t GPS_STALE_MS = 15000UL;
static const uint32_t PPS_STALE_MS = 5000UL;
static const uint32_t PPS_WARNING_MS = 30UL * 60UL * 1000UL;
static const int64_t PPS_PAIR_WINDOW_US = 1500000LL;
static const int64_t PPS_UTC_AFTER_PPS_MIN_US = 0LL;
static const int64_t PPS_UTC_AFTER_PPS_MAX_US = 450000LL;
static const int64_t LATE_CONSISTENT_ACCEPT_US = 50000LL;
static const int64_t PPS_UTC_BEFORE_NEXT_PPS_MAX_US = 1200000LL;
static const int64_t PPS_DISCIPLINE_REJECT_STEP_US = 500000LL;
static const int64_t PPS_WRONG_EDGE_NEAR_US = 1000000LL;
static const int64_t PPS_WRONG_EDGE_TOLERANCE_US = 200000LL;
static const int64_t PPS_NEXT_EDGE_DELTA_MIN_US = 750000LL;
static const int64_t PPS_NEXT_EDGE_DELTA_MAX_US = 1250000LL;
static const uint32_t LARGE_STEP_UNLOCK_HOLDOVER_MS = 30UL * 60UL * 1000UL;
static const int64_t MAX_SYNC_STEP_US = 2000000LL;
static const uint8_t MAX_BAD_SAMPLES_BEFORE_HOLDOVER = 4;
static const uint32_t WIFI_RECONNECT_MS = 10000UL;
static const uint32_t STATUS_LOG_RATE_MS = 10000UL;
static const uint32_t WRONG_EDGE_LOG_RATE_MS = 2000UL;
static const uint32_t GPS_UTC_STALE_SYNC_LOG_MS = 60000UL;
static const uint32_t CANDIDATE_HISTORY_STALE_MS = 30000UL;
static const uint32_t GPS_BRIDGE_NO_TX_LOG_MS = 5000UL;
static const uint32_t GPS_MAINTENANCE_MODE_MS = 300000UL;
static const uint16_t GPS_BYTES_PER_LOOP = 96;
static const uint16_t GPS_BYTES_PER_LOOP_OTA = 16;
static const uint16_t GPS_BRIDGE_BYTES_PER_LOOP = 64;
static const uint16_t GPS_BRIDGE_TX_RING_SIZE = 512;
static const uint16_t GPS_BRIDGE_TX_BYTES_PER_LOOP = 64;
static const uint8_t POST_MAINTENANCE_REQUIRED_SAMPLES = 3;
static const int64_t POST_MAINTENANCE_SANE_DELTA_US = 50000LL;
static const uint32_t OTA_SAFE_MODE_MS = 120000UL;

// ----------------------------- Global Objects -----------------------------

TinyGPSPlus gps;
TinyGPSCustom gpzdaTime(gps, "GPZDA", 1);
TinyGPSCustom gpzdaDay(gps, "GPZDA", 2);
TinyGPSCustom gpzdaMonth(gps, "GPZDA", 3);
TinyGPSCustom gpzdaYear(gps, "GPZDA", 4);
TinyGPSCustom gnzdaTime(gps, "GNZDA", 1);
TinyGPSCustom gnzdaDay(gps, "GNZDA", 2);
TinyGPSCustom gnzdaMonth(gps, "GNZDA", 3);
TinyGPSCustom gnzdaYear(gps, "GNZDA", 4);
TinyGPSCustom gprmcTime(gps, "GPRMC", 1);
TinyGPSCustom gprmcStatus(gps, "GPRMC", 2);
TinyGPSCustom gprmcDate(gps, "GPRMC", 9);
TinyGPSCustom gnrmcTime(gps, "GNRMC", 1);
TinyGPSCustom gnrmcStatus(gps, "GNRMC", 2);
TinyGPSCustom gnrmcDate(gps, "GNRMC", 9);
TinyGPSCustom gpggaFixQuality(gps, "GPGGA", 6);
TinyGPSCustom gpggaSatellites(gps, "GPGGA", 7);
TinyGPSCustom gpggaHdop(gps, "GPGGA", 8);
TinyGPSCustom gnggaFixQuality(gps, "GNGGA", 6);
TinyGPSCustom gnggaSatellites(gps, "GNGGA", 7);
TinyGPSCustom gnggaHdop(gps, "GNGGA", 8);
TinyGPSCustom gpgsaFixType(gps, "GPGSA", 2);
TinyGPSCustom gngsaFixType(gps, "GNGSA", 2);
HardwareSerial GPSSerial(2);
WebServer server(WEB_PORT);
WiFiUDP ntpUdp;
WiFiServer gpsBridgeServer(GPS_BRIDGE_PORT);
WiFiClient gpsBridgeClient;

// ----------------------------- Sync State -----------------------------

enum SyncState : uint8_t {
  SYNC_UNSYNCHRONISED = 0,
  SYNC_ACQUIRING = 1,
  SYNC_SYNCED = 2,
  SYNC_HOLDOVER = 3
};

enum TimebaseSource : uint8_t {
  TIMEBASE_INVALID = 0,
  TIMEBASE_GPS_ONLY = 1,
  TIMEBASE_PPS_DISCIPLINED = 2
};

static SyncState syncState = SYNC_UNSYNCHRONISED;
static SyncState previousSyncState = SYNC_UNSYNCHRONISED;
static TimebaseSource timebaseSource = TIMEBASE_INVALID;

static bool timeEverValid = false;
static bool everPpsDisciplined = false;
static bool servicesStarted = false;
static bool gpsBridgeStarted = false;
static bool gpsBridgeClientWasConnected = false;
static bool gpsMaintenanceMode = false;
static bool postMaintenanceRevalidate = false;

static int64_t referenceMonoUs = 0;
static int64_t referenceUnixUs = 0;
static int64_t lastAcceptedUnixUs = 0;
static int64_t lastAcceptedMonoUs = 0;
static uint32_t lastAcceptedEpoch = 0;
static uint32_t lastAcceptedPpsSeq = 0;

static uint32_t lastGpsEpoch = 0;
static uint32_t lastGpsDataMs = 0;
static uint32_t lastGpsSentenceMs = 0;
static uint32_t lastGpsSentenceSeq = 0;
static int64_t lastGpsSentenceMonoUs = 0;
static uint32_t lastGpsValidMs = 0;
static uint32_t lastPpsSeenMs = 0;
static uint32_t lastRejectedSampleMs = 0;
static uint32_t holdoverSinceMs = 0;
static uint32_t badSampleCount = 0;
static bool ppsWarningActive = false;
static int64_t lastRejectedDeltaUs = 0;
static char lastRejectedReason[72] = "none";

static char lastCandidateSource[12] = "none";
static char lastCandidateReason[80] = "none";
static uint32_t lastCandidateDecodedEpoch = 0;
static uint32_t lastCandidateAssignedEpoch = 0;
static uint32_t lastCandidatePpsSeq = 0;
static int64_t lastCandidatePpsMonoUs = 0;
static int64_t lastCandidateGpsArrivalUs = 0;
static int32_t lastCandidateGapMs = 0;
static int64_t lastCandidateDeltaUs = 0;
static uint32_t lastCandidateMs = 0;
static int64_t lastGoodPpsDeltaUs = 0;
static int32_t lastPairingGapMs = 0;
static char lastPairingAcceptReason[64] = "none";
static bool lastCandidatePpsAlreadyConsumed = false;
static bool lastCandidateUtcAlreadyConsumed = false;
static uint32_t lastConsumedGpsEpoch = 0;
static char lastConsumedGpsSource[12] = "";
static uint32_t lastConsumedAssignedEpoch = 0;
static uint32_t lastHandledGpsSentenceSeq = 0;
static uint32_t lastHandledGpsEpoch = 0;
static char lastHandledGpsSource[12] = "";
static uint32_t lastRejectedPpsSeq = 0;

static bool pendingUtcValid = false;
static uint32_t pendingUtcEpoch = 0;
static int64_t pendingUtcAssignedEpoch = 0;
static int64_t pendingUtcArrivalUs = 0;
static uint32_t pendingUtcSentenceSeq = 0;
static uint32_t pendingUtcAfterPpsSeq = 0;
static char pendingUtcSource[12] = "";

static bool otaInProgress = false;
static uint32_t otaSafeUntilMs = 0;
static uint8_t otaLastProgressPct = 255;

static uint32_t ppsEventsProcessed = 0;
static uint32_t rejectedSamples = 0;
static uint32_t gpsOnlySamples = 0;

// GPS dashboard values.
static bool gpsFixValid = false;
static bool gpsPositionKnown = false;
static bool gpsSatellitesValid = false;
static bool gpsHdopValid = false;
static bool gpsUtcValid = false;
static uint32_t gpsSatellites = 0;
static double gpsHdop = 0.0;
static uint8_t gpsFixType = 0;  // 0 unavailable, 1 none, 2 2D, 3 3D
static uint8_t gpsFixQuality = 0;  // GGA fix quality, 0 means invalid/unavailable
static uint32_t gpsTelemetryMs = 0;
static char gpsTelemetrySource[12] = "Unavailable";

// Diagnostic edge logging state.
static bool ppsWasFresh = false;
static bool gpsWasFresh = false;
static bool utcWasValid = false;

// NTP stats.
static uint64_t ntpRequestCount = 0;
static uint64_t gpsBridgeBytesToClient = 0;
static uint64_t gpsBridgeBytesFromClient = 0;
static uint64_t gpsBridgeBytesDropped = 0;
static uint64_t gpsBridgeRxDiscarded = 0;
static uint64_t gpsBridgeRxForwarded = 0;
static uint64_t gpsSerialBytesSeen = 0;
static uint64_t gpsBridgeMirrorBytesQueued = 0;
static uint8_t gpsBridgeTxRing[GPS_BRIDGE_TX_RING_SIZE];
static uint16_t gpsBridgeTxHead = 0;
static uint16_t gpsBridgeTxTail = 0;
static uint16_t gpsBridgeTxCount = 0;
static char gpsBridgeFirstBytesHex[3 * 32 + 1] = "";
static uint8_t gpsBridgeFirstBytesCount = 0;
static bool gpsBridgeFirstBytesLogged = false;
static IPAddress lastNtpClient;
static uint32_t lastNtpRequestMs = 0;
static int64_t lastNtpTxUnixUs = 0;
static uint32_t lastNtpTxMs = 0;
static uint32_t lastGpsBridgeTxMs = 0;
static uint32_t gpsParserBytesPerSecond = 0;
static uint32_t gpsParserBytesThisSecond = 0;
static uint32_t gpsParserRateWindowMs = 0;
static uint32_t lastGpsRmcMs = 0;
static uint32_t gpsMaintenanceUntilMs = 0;
static uint8_t postMaintenanceGoodSamples = 0;
static uint32_t postMaintenanceLastPpsSeq = 0;
static uint32_t postMaintenanceLastAssignedEpoch = 0;
static char gpsMaintenanceDisableReason[64] = "none";
static char postMaintenanceLastRejectReason[96] = "none";

// PPS ISR state.
static portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t isrLastPpsUs = 0;
static volatile uint32_t isrPpsSeq = 0;
static volatile bool isrPpsFlag = false;

// Logging ring.
static const uint8_t LOG_LINES = 60;
static const uint8_t LOG_LINE_LEN = 192;
static char logRing[LOG_LINES][LOG_LINE_LEN];
static uint8_t logHead = 0;
static uint8_t logCount = 0;
static uint32_t logSequence = 0;

// Rate-limit markers.
static uint32_t lastWifiAttemptMs = 0;
static uint32_t lastGpsStaleLogMs = 0;
static uint32_t lastPpsStaleLogMs = 0;
static uint32_t lastNtpRefuseLogMs = 0;
static uint32_t lastConsumedPpsWaitLogMs = 0;
static uint32_t lastStalePairLogMs = 0;
static uint32_t lastWrongEdgePairLogMs = 0;
static uint32_t lastGpsUtcStaleSyncedLogMs = 0;
static uint32_t lastGpsBridgeNoTxLogMs = 0;
static uint32_t lastGpsBridgeRxDiscardLogMs = 0;
static uint32_t lastGpsBridgeRxForwardLogMs = 0;

// ----------------------------- Utility -----------------------------

static const char *syncStateName(SyncState s) {
  switch (s) {
    case SYNC_UNSYNCHRONISED: return "Unsynchronised";
    case SYNC_ACQUIRING: return "Acquiring";
    case SYNC_SYNCED: return "Synced";
    case SYNC_HOLDOVER: return "Holdover";
    default: return "Unknown";
  }
}

static const char *timebaseSourceName() {
  if (syncState == SYNC_HOLDOVER && timebaseSource == TIMEBASE_PPS_DISCIPLINED) {
    return "Holdover";
  }
  switch (timebaseSource) {
    case TIMEBASE_GPS_ONLY: return "GPS-only";
    case TIMEBASE_PPS_DISCIPLINED: return "PPS-disciplined";
    default: return "Invalid";
  }
}

static uint32_t unixFromDateTime(uint16_t year, uint8_t month, uint8_t day,
                                 uint8_t hour, uint8_t minute, uint8_t second) {
  if (year < 2024 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 60) {
    return 0;
  }

  static const uint16_t daysBeforeMonth[] =
      {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

  uint32_t days = 0;
  for (uint16_t y = 1970; y < year; y++) {
    days += 365;
    if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) days++;
  }

  days += daysBeforeMonth[month - 1];
  if (month > 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
    days++;
  }
  days += day - 1;

  return days * 86400UL + hour * 3600UL + minute * 60UL + second;
}

static void formatDuration(uint32_t seconds, char *out, size_t outLen) {
  uint32_t d = seconds / 86400UL;
  seconds %= 86400UL;
  uint8_t h = seconds / 3600UL;
  seconds %= 3600UL;
  uint8_t m = seconds / 60UL;
  uint8_t s = seconds % 60UL;

  if (d > 0) {
    snprintf(out, outLen, "%lud %02u:%02u:%02u", (unsigned long)d, h, m, s);
  } else {
    snprintf(out, outLen, "%02u:%02u:%02u", h, m, s);
  }
}

static void formatIsoUtc(int64_t unixUs, char *out, size_t outLen) {
  if (unixUs <= 0) {
    strlcpy(out, "not valid", outLen);
    return;
  }

  time_t seconds = (time_t)(unixUs / 1000000LL);
  uint32_t micros = (uint32_t)(unixUs % 1000000LL);
  struct tm tmUtc;
  gmtime_r(&seconds, &tmUtc);
  snprintf(out, outLen, "%04d-%02d-%02d %02d:%02d:%02d.%06lu UTC",
           tmUtc.tm_year + 1900, tmUtc.tm_mon + 1, tmUtc.tm_mday,
           tmUtc.tm_hour, tmUtc.tm_min, tmUtc.tm_sec,
           (unsigned long)micros);
}

static void addLog(const char *fmt, ...) {
  char msg[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  if (otaInProgress && strncmp(msg, "OTA", 3) != 0) {
    return;
  }

  uint32_t up = millis() / 1000UL;
  char dur[18];
  formatDuration(up, dur, sizeof(dur));
  snprintf(logRing[logHead], LOG_LINE_LEN, "[%s] %s", dur, msg);
  Serial.println(logRing[logHead]);

  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
  logSequence++;
}

static int64_t currentUnixUs() {
  if (!timeEverValid) return 0;
  return referenceUnixUs + (esp_timer_get_time() - referenceMonoUs);
}

static bool otaSafeActive() {
  return otaInProgress || (otaSafeUntilMs != 0 && (int32_t)(otaSafeUntilMs - millis()) > 0);
}

static void enableGpsMaintenanceMode(const char *reason) {
  gpsMaintenanceMode = true;
  gpsMaintenanceUntilMs = millis() + GPS_MAINTENANCE_MODE_MS;
  strlcpy(gpsMaintenanceDisableReason, "active", sizeof(gpsMaintenanceDisableReason));
  addLog("GPS maintenance mode enabled: %s", reason ? reason : "dashboard");
}

static void disableGpsMaintenanceMode(const char *reason) {
  if (!gpsMaintenanceMode) {
    return;
  }
  addLog("GPS maintenance mode disabled: %s", reason ? reason : "disabled");
  gpsMaintenanceMode = false;
  gpsMaintenanceUntilMs = 0;
  addLog("GPS bridge read-only: u-center commands discarded");
  strlcpy(gpsMaintenanceDisableReason, reason ? reason : "disabled",
          sizeof(gpsMaintenanceDisableReason));
  postMaintenanceRevalidate = true;
  postMaintenanceGoodSamples = 0;
  postMaintenanceLastPpsSeq = 0;
  postMaintenanceLastAssignedEpoch = 0;
  strlcpy(postMaintenanceLastRejectReason, "waiting for sane GPS/PPS samples",
          sizeof(postMaintenanceLastRejectReason));
  addLog("Post-maintenance revalidation started");
}

static void recordRejectedSample(const char *source, const char *reason, int64_t deltaUs,
                                 bool logBasic = true) {
  rejectedSamples++;
  badSampleCount++;
  lastRejectedDeltaUs = deltaUs;
  lastRejectedSampleMs = millis();
  snprintf(lastRejectedReason, sizeof(lastRejectedReason), "%s: %s", source, reason);
  if (logBasic) {
    addLog("Rejected %s sample. %s. Candidate delta %+lld us",
           source, reason, (long long)deltaUs);
  }
}

static void updateCandidateDiagnostics(const char *source, uint32_t decodedEpoch,
                                       int64_t assignedEpoch, uint32_t ppsSeq,
                                       int64_t ppsMonoUs, int64_t gpsArrivalUs,
                                       int64_t gapUs, int64_t deltaUs,
                                       const char *reason) {
  strlcpy(lastCandidateSource, source, sizeof(lastCandidateSource));
  strlcpy(lastCandidateReason, reason, sizeof(lastCandidateReason));
  lastCandidateDecodedEpoch = decodedEpoch;
  lastCandidateAssignedEpoch = (assignedEpoch > 0) ? (uint32_t)assignedEpoch : 0;
  lastCandidatePpsSeq = ppsSeq;
  lastCandidatePpsMonoUs = ppsMonoUs;
  lastCandidateGpsArrivalUs = gpsArrivalUs;
  lastCandidateGapMs = (int32_t)(gapUs / 1000LL);
  lastCandidateDeltaUs = deltaUs;
  lastCandidateMs = millis();
  lastCandidatePpsAlreadyConsumed = (ppsSeq != 0 && ppsSeq == lastAcceptedPpsSeq);
  lastCandidateUtcAlreadyConsumed = (assignedEpoch > 0 && lastConsumedAssignedEpoch != 0 &&
                                     (uint32_t)assignedEpoch == lastConsumedAssignedEpoch);
}

static bool isWrongEdgeDelta(int64_t deltaUs) {
  int64_t distanceFromOneSecond = llabs(llabs(deltaUs) - PPS_WRONG_EDGE_NEAR_US);
  return distanceFromOneSecond <= PPS_WRONG_EDGE_TOLERANCE_US;
}

static void ignoreWrongEdgeCandidate(const char *source, uint32_t decodedEpoch,
                                     int64_t assignedEpoch, uint32_t ppsSeq,
                                     int64_t ppsUs, int64_t gpsArrivalUs,
                                     int64_t gapUs, int64_t deltaUs) {
  const char *reason = "ignored wrong-edge UTC/PPS candidate";
  updateCandidateDiagnostics(source, decodedEpoch, assignedEpoch, ppsSeq, ppsUs,
                             gpsArrivalUs, gapUs, deltaUs, reason);
  uint32_t nowMs = millis();
  if (nowMs - lastWrongEdgePairLogMs >= WRONG_EDGE_LOG_RATE_MS) {
    lastWrongEdgePairLogMs = nowMs;
    addLog("Ignored wrong-edge %s/PPS candidate: decoded %lu assigned %lu pps_seq %lu gap %ld ms delta %+lld us reason %s",
           source, (unsigned long)decodedEpoch,
           (unsigned long)((assignedEpoch > 0) ? assignedEpoch : 0),
           (unsigned long)ppsSeq, (long)(gapUs / 1000LL),
           (long long)deltaUs, reason);
  }
}

static uint32_t ageOrMax(uint32_t lastMs) {
  if (lastMs == 0) return 0xFFFFFFFFUL;
  return millis() - lastMs;
}

static void setSyncState(SyncState newState, const char *reason = nullptr) {
  if (newState == syncState) return;
  previousSyncState = syncState;
  syncState = newState;

  if (syncState == SYNC_HOLDOVER && previousSyncState != SYNC_HOLDOVER) {
    holdoverSinceMs = millis();
    addLog("Entered holdover. Reason: %s", reason ? reason : "not specified");
  }
  if (previousSyncState == SYNC_HOLDOVER && syncState != SYNC_HOLDOVER) {
    addLog("Exited holdover. Reason: %s", reason ? reason : "not specified");
    holdoverSinceMs = 0;
  }
  addLog("State changed to %s%s%s", syncStateName(syncState),
         reason ? ". Reason: " : "", reason ? reason : "");
}

static void processPendingUtcWithPps(int64_t ppsUs, uint32_t ppsSeq);
static bool postMaintenanceAcceptAllowed(const char *source, uint32_t decodedEpoch,
                                         int64_t assignedEpoch, uint32_t ppsSeq,
                                         int64_t ppsUs, int64_t gpsArrivalUs,
                                         int64_t deltaUs);

// ----------------------------- PPS ISR -----------------------------

void IRAM_ATTR ppsISR() {
  int64_t nowUs = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&ppsMux);
  isrLastPpsUs = nowUs;
  isrPpsSeq++;
  isrPpsFlag = true;
  portEXIT_CRITICAL_ISR(&ppsMux);
}

static bool snapshotLatestPps(int64_t *ppsUs, uint32_t *seq) {
  bool seen;
  portENTER_CRITICAL(&ppsMux);
  *ppsUs = isrLastPpsUs;
  *seq = isrPpsSeq;
  seen = (*seq != 0);
  portEXIT_CRITICAL(&ppsMux);
  return seen;
}

static void servicePpsFlag() {
  bool flag;
  int64_t ppsUs;
  uint32_t seq;

  portENTER_CRITICAL(&ppsMux);
  flag = isrPpsFlag;
  if (flag) {
    isrPpsFlag = false;
    ppsUs = isrLastPpsUs;
    seq = isrPpsSeq;
  }
  portEXIT_CRITICAL(&ppsMux);

  if (!flag) return;

  (void)ppsUs;
  (void)seq;
  bool wasStale = ageOrMax(lastPpsSeenMs) > PPS_STALE_MS;
  ppsEventsProcessed++;
  lastPpsSeenMs = millis();
  if (ppsEventsProcessed == 1) {
    addLog("PPS detected on GPIO%u", PPS_PIN);
  } else if (wasStale) {
    addLog("PPS recovered on GPIO%u", PPS_PIN);
  }

  processPendingUtcWithPps(ppsUs, seq);
}

static bool postMaintenanceAcceptAllowed(const char *source, uint32_t decodedEpoch,
                                         int64_t assignedEpoch, uint32_t ppsSeq,
                                         int64_t ppsUs, int64_t gpsArrivalUs,
                                         int64_t deltaUs) {
  if (!postMaintenanceRevalidate) return true;

  int64_t gapUs = gpsArrivalUs - ppsUs;
  bool ppsFresh = ageOrMax(lastPpsSeenMs) <= PPS_STALE_MS;
  bool duplicatePps = (ppsSeq == 0 || ppsSeq == lastAcceptedPpsSeq ||
                       ppsSeq == postMaintenanceLastPpsSeq);
  bool duplicateUtc = (assignedEpoch <= 0 ||
                       (lastConsumedAssignedEpoch != 0 &&
                        (uint32_t)assignedEpoch == lastConsumedAssignedEpoch) ||
                       (postMaintenanceLastAssignedEpoch != 0 &&
                        (uint32_t)assignedEpoch == postMaintenanceLastAssignedEpoch));
  bool sane = ppsFresh && !duplicatePps && !duplicateUtc &&
              llabs(deltaUs) <= POST_MAINTENANCE_SANE_DELTA_US;

  updateCandidateDiagnostics(source, decodedEpoch, assignedEpoch, ppsSeq, ppsUs,
                             gpsArrivalUs, gapUs, deltaUs,
                             sane ? "post-maintenance sane sample"
                                  : "post-maintenance sample rejected");

  if (!sane) {
    postMaintenanceGoodSamples = 0;
    if (!ppsFresh) {
      strlcpy(postMaintenanceLastRejectReason,
              "Post-maintenance sample rejected: PPS stale",
              sizeof(postMaintenanceLastRejectReason));
    } else if (duplicatePps || duplicateUtc) {
      strlcpy(postMaintenanceLastRejectReason,
              "Post-maintenance sample rejected: stale/ambiguous UTC/PPS pairing",
              sizeof(postMaintenanceLastRejectReason));
    } else {
      snprintf(postMaintenanceLastRejectReason, sizeof(postMaintenanceLastRejectReason),
               "Post-maintenance sample rejected: would step by %+lld us",
               (long long)deltaUs);
    }
    addLog("%s", postMaintenanceLastRejectReason);
    return false;
  }

  postMaintenanceLastPpsSeq = ppsSeq;
  postMaintenanceLastAssignedEpoch = (uint32_t)assignedEpoch;
  postMaintenanceGoodSamples++;
  snprintf(postMaintenanceLastRejectReason, sizeof(postMaintenanceLastRejectReason),
           "sane GPS/PPS samples %u/%u",
           postMaintenanceGoodSamples, POST_MAINTENANCE_REQUIRED_SAMPLES);

  if (postMaintenanceGoodSamples < POST_MAINTENANCE_REQUIRED_SAMPLES) {
    addLog("Post-maintenance sample accepted %u/%u: %s delta %+lld us pps_seq %lu",
           postMaintenanceGoodSamples, POST_MAINTENANCE_REQUIRED_SAMPLES,
           source, (long long)deltaUs, (unsigned long)ppsSeq);
    return false;
  }

  postMaintenanceRevalidate = false;
  postMaintenanceGoodSamples = 0;
  postMaintenanceLastPpsSeq = 0;
  postMaintenanceLastAssignedEpoch = 0;
  strlcpy(postMaintenanceLastRejectReason, "revalidated",
          sizeof(postMaintenanceLastRejectReason));
  addLog("Post-maintenance revalidated; normal discipline resumed");
  return true;
}

// ----------------------------- Time Discipline -----------------------------

static void acceptTimeSample(uint32_t epoch, int64_t monoUs, bool ppsDisciplined,
                             const char *source, uint32_t pairedPpsSeq = 0,
                             int64_t gpsArrivalUs = 0,
                             const char *acceptReason = "accepted") {
  int64_t assignedEpoch = (int64_t)epoch;
  if (ppsDisciplined) {
    assignedEpoch -= PPS_EPOCH_OFFSET_SECONDS;
  }

  if (assignedEpoch <= 0) {
    updateCandidateDiagnostics(source, epoch, assignedEpoch, pairedPpsSeq, monoUs,
                               gpsArrivalUs, gpsArrivalUs - monoUs, 0,
                               "corrected epoch invalid");
    recordRejectedSample(source, "corrected epoch invalid", 0);
    return;
  }

  int64_t candidateUnixUs = assignedEpoch * 1000000LL;
  if (ppsDisciplined) {
    candidateUnixUs += PPS_CALIBRATION_US;
  } else {
    candidateUnixUs += GPS_NMEA_RECEIVE_CAL_US;
  }

  int64_t acceptDeltaUs = 0;
  bool rebasedGpsOnlyToPps = false;
  if (timeEverValid) {
    int64_t predicted = referenceUnixUs + (monoUs - referenceMonoUs);
    int64_t error = candidateUnixUs - predicted;
    acceptDeltaUs = error;
    if (ppsDisciplined &&
        !postMaintenanceAcceptAllowed(source, epoch, assignedEpoch, pairedPpsSeq,
                                      monoUs, gpsArrivalUs, acceptDeltaUs)) {
      return;
    }
    bool prolongedHoldover = (syncState == SYNC_HOLDOVER && holdoverSinceMs != 0 &&
                              (millis() - holdoverSinceMs) > LARGE_STEP_UNLOCK_HOLDOVER_MS);
    if (ppsDisciplined && timebaseSource == TIMEBASE_GPS_ONLY &&
        llabs(error) > PPS_DISCIPLINE_REJECT_STEP_US) {
      rebasedGpsOnlyToPps = true;
      addLog("Rebased GPS-only timebase to PPS-disciplined time by %+lld us",
             (long long)error);
    }
    if (ppsDisciplined && everPpsDisciplined &&
        timebaseSource == TIMEBASE_PPS_DISCIPLINED &&
        syncState == SYNC_SYNCED &&
        !postMaintenanceRevalidate && isWrongEdgeDelta(error)) {
      ignoreWrongEdgeCandidate(source, epoch, assignedEpoch, pairedPpsSeq, monoUs,
                               gpsArrivalUs, gpsArrivalUs - monoUs, error);
      return;
    }
    if (ppsDisciplined && everPpsDisciplined &&
        llabs(error) > PPS_DISCIPLINE_REJECT_STEP_US && !prolongedHoldover) {
      updateCandidateDiagnostics(source, epoch, assignedEpoch, pairedPpsSeq, monoUs,
                                 gpsArrivalUs, gpsArrivalUs - monoUs, error,
                                 "would step disciplined clock too far");
      recordRejectedSample(source, "would step disciplined clock too far", error);
      return;
    }
    if (ppsDisciplined && prolongedHoldover && llabs(error) > PPS_DISCIPLINE_REJECT_STEP_US) {
      addLog("Allowing large PPS step after prolonged holdover: %+lld us", (long long)error);
    }
    if (!ppsDisciplined && syncState == SYNC_SYNCED && llabs(error) > MAX_SYNC_STEP_US) {
      updateCandidateDiagnostics(source, epoch, assignedEpoch, pairedPpsSeq, monoUs,
                                 gpsArrivalUs, gpsArrivalUs - monoUs, error,
                                 "GPS-only step too large while synced");
      recordRejectedSample(source, "GPS-only step too large while synced", error);
      return;
    }
  }

  referenceMonoUs = monoUs;
  referenceUnixUs = candidateUnixUs;
  lastAcceptedUnixUs = candidateUnixUs;
  lastAcceptedMonoUs = monoUs;
  lastAcceptedEpoch = (uint32_t)assignedEpoch;
  timeEverValid = true;
  timebaseSource = ppsDisciplined ? TIMEBASE_PPS_DISCIPLINED : TIMEBASE_GPS_ONLY;
  badSampleCount = 0;

  if (ppsDisciplined) {
    everPpsDisciplined = true;
    lastAcceptedPpsSeq = pairedPpsSeq;
    lastConsumedGpsEpoch = epoch;
    lastConsumedAssignedEpoch = (uint32_t)assignedEpoch;
    strlcpy(lastConsumedGpsSource, source, sizeof(lastConsumedGpsSource));
    int64_t gapUs = gpsArrivalUs - monoUs;
    lastGoodPpsDeltaUs = acceptDeltaUs;
    lastPairingGapMs = (int32_t)(gapUs / 1000LL);
    strlcpy(lastPairingAcceptReason,
            rebasedGpsOnlyToPps ? "rebased GPS-only to PPS" : acceptReason,
            sizeof(lastPairingAcceptReason));
    updateCandidateDiagnostics(source, epoch, assignedEpoch, pairedPpsSeq, monoUs,
                               gpsArrivalUs, gapUs, acceptDeltaUs,
                               rebasedGpsOnlyToPps ? "rebased GPS-only to PPS" : acceptReason);
    addLog("%s GPS/PPS %s: decoded %lu assigned %lu pps_seq %lu pps_us %lld gps_us %lld gap %ld ms delta %+lld us offset %+ld s",
           rebasedGpsOnlyToPps ? "Rebased" : "Accepted",
           source,
           (unsigned long)epoch, (unsigned long)assignedEpoch,
           (unsigned long)pairedPpsSeq, (long long)monoUs,
           (long long)gpsArrivalUs, (long)(gapUs / 1000LL),
           (long long)acceptDeltaUs,
           (long)PPS_EPOCH_OFFSET_SECONDS);
  } else {
    gpsOnlySamples++;
    addLog("Accepted GPS-only time: epoch %lu", (unsigned long)epoch);
  }
}

static void rejectGpsPpsCandidate(const char *source, const char *reason,
                                  uint32_t decodedEpoch, int64_t assignedEpoch,
                                  uint32_t ppsSeq, int64_t ppsUs,
                                  int64_t gpsArrivalUs, int64_t gapUs,
                                  int64_t deltaUs) {
  updateCandidateDiagnostics(source, decodedEpoch, assignedEpoch, ppsSeq, ppsUs,
                             gpsArrivalUs, gapUs, deltaUs, reason);
  lastRejectedPpsSeq = ppsSeq;
  addLog("Rejected GPS/PPS %s: dec %lu asg %lu seq %lu pps_us %lld gps_us %lld pps_used %u utc_used %u gap %ldms delta %+lld reason %s",
         source, (unsigned long)decodedEpoch,
         (unsigned long)((assignedEpoch > 0) ? assignedEpoch : 0),
         (unsigned long)ppsSeq, (long long)ppsUs, (long long)gpsArrivalUs,
         lastCandidatePpsAlreadyConsumed ? 1 : 0,
         lastCandidateUtcAlreadyConsumed ? 1 : 0,
         (long)(gapUs / 1000LL), (long long)deltaUs, reason);
  recordRejectedSample(source, reason, deltaUs, false);
}

static void rejectSample(const char *source, const char *reason) {
  recordRejectedSample(source, reason, 0);
}

static void queueUtcForNextPps(const char *source, uint32_t epoch,
                               int64_t assignedEpoch, int64_t arrivalUs,
                               uint32_t sentenceSeq, int64_t deltaUs,
                               uint32_t latestPpsSeq, int64_t latestPpsUs,
                               int64_t gapUs, const char *reason) {
  if (pendingUtcValid && pendingUtcEpoch == epoch &&
      strcmp(pendingUtcSource, source) == 0) {
    updateCandidateDiagnostics(source, epoch, assignedEpoch, latestPpsSeq,
                               latestPpsUs, arrivalUs, gapUs, deltaUs,
                               "already pending for next PPS");
    return;
  }

  if (pendingUtcValid && lastConsumedAssignedEpoch != 0 &&
      pendingUtcAssignedEpoch > 0 &&
      (uint32_t)pendingUtcAssignedEpoch == lastConsumedAssignedEpoch) {
    pendingUtcValid = false;
  }

  if (pendingUtcValid && latestPpsSeq > pendingUtcAfterPpsSeq + 1) {
    updateCandidateDiagnostics(pendingUtcSource, pendingUtcEpoch, pendingUtcAssignedEpoch,
                               latestPpsSeq, latestPpsUs, pendingUtcArrivalUs,
                               pendingUtcArrivalUs - latestPpsUs, 0,
                               "Discarded pending UTC: missed next PPS");
    addLog("Discarded pending UTC: missed next PPS. %s decoded %lu after pps_seq %lu now pps_seq %lu",
           pendingUtcSource, (unsigned long)pendingUtcEpoch,
           (unsigned long)pendingUtcAfterPpsSeq, (unsigned long)latestPpsSeq);
    pendingUtcValid = false;
  }

  if (pendingUtcValid && latestPpsSeq == pendingUtcAfterPpsSeq) {
    updateCandidateDiagnostics(source, epoch, assignedEpoch, latestPpsSeq,
                               latestPpsUs, arrivalUs, gapUs, deltaUs,
                               "already waiting for next PPS with earlier UTC");
    if (millis() - lastConsumedPpsWaitLogMs > STATUS_LOG_RATE_MS) {
      lastConsumedPpsWaitLogMs = millis();
      addLog("UTC %s decoded; already waiting for next PPS with %s decoded %lu",
             source, pendingUtcSource, (unsigned long)pendingUtcEpoch);
    }
    return;
  }

  pendingUtcValid = true;
  pendingUtcEpoch = epoch;
  pendingUtcAssignedEpoch = assignedEpoch;
  pendingUtcArrivalUs = arrivalUs;
  pendingUtcSentenceSeq = sentenceSeq;
  pendingUtcAfterPpsSeq = latestPpsSeq;
  strlcpy(pendingUtcSource, source, sizeof(pendingUtcSource));

  updateCandidateDiagnostics(source, epoch, assignedEpoch, latestPpsSeq,
                             latestPpsUs, arrivalUs, gapUs, deltaUs, reason);
  if (strcmp(reason, "UTC candidate waiting for next unconsumed PPS") == 0) {
    if (millis() - lastConsumedPpsWaitLogMs > STATUS_LOG_RATE_MS) {
      lastConsumedPpsWaitLogMs = millis();
      addLog("UTC %s decoded; waiting for next PPS because latest PPS already consumed",
             source);
    }
  } else {
    addLog("Queued %s for next PPS edge: decoded %lu assigned %lu after pps_seq %lu gap %ld ms delta %+lld us reason %s",
           source, (unsigned long)epoch,
           (unsigned long)((assignedEpoch > 0) ? assignedEpoch : 0),
           (unsigned long)latestPpsSeq, (long)(gapUs / 1000LL),
           (long long)deltaUs, reason);
  }
}

static void processPendingUtcWithPps(int64_t ppsUs, uint32_t ppsSeq) {
  if (!pendingUtcValid) return;

  if (pendingUtcAfterPpsSeq != 0 && ppsSeq <= pendingUtcAfterPpsSeq) {
    return;
  }

  if (pendingUtcAfterPpsSeq != 0 && ppsSeq > pendingUtcAfterPpsSeq + 1) {
    updateCandidateDiagnostics(pendingUtcSource, pendingUtcEpoch, pendingUtcAssignedEpoch,
                               ppsSeq, ppsUs, pendingUtcArrivalUs,
                               pendingUtcArrivalUs - ppsUs, 0,
                               "Discarded pending UTC: missed next PPS");
    addLog("Discarded pending UTC: missed next PPS. %s decoded %lu after pps_seq %lu now pps_seq %lu",
           pendingUtcSource, (unsigned long)pendingUtcEpoch,
           (unsigned long)pendingUtcAfterPpsSeq, (unsigned long)ppsSeq);
    pendingUtcValid = false;
    return;
  }

  int64_t leadUs = ppsUs - pendingUtcArrivalUs;
  int64_t deltaUs = 0;

  if (ppsSeq == lastAcceptedPpsSeq) {
    updateCandidateDiagnostics(pendingUtcSource, pendingUtcEpoch, pendingUtcAssignedEpoch,
                               ppsSeq, ppsUs, pendingUtcArrivalUs,
                               pendingUtcArrivalUs - ppsUs, 0,
                               "pending waiting for next unconsumed PPS");
    return;
  }

  if (timeEverValid && pendingUtcAssignedEpoch > 0) {
    int64_t candidateUnixUs = pendingUtcAssignedEpoch * 1000000LL + PPS_CALIBRATION_US;
    deltaUs = candidateUnixUs - (referenceUnixUs + (ppsUs - referenceMonoUs));
  }

  if (lastConsumedAssignedEpoch != 0 &&
      (uint32_t)pendingUtcAssignedEpoch == lastConsumedAssignedEpoch) {
    updateCandidateDiagnostics(pendingUtcSource, pendingUtcEpoch, pendingUtcAssignedEpoch,
                               ppsSeq, ppsUs, pendingUtcArrivalUs,
                               pendingUtcArrivalUs - ppsUs, deltaUs,
                               "pending UTC second already consumed");
    pendingUtcValid = false;
    return;
  }

  if (leadUs < 0 || leadUs > PPS_UTC_BEFORE_NEXT_PPS_MAX_US) {
    updateCandidateDiagnostics(pendingUtcSource, pendingUtcEpoch, pendingUtcAssignedEpoch,
                               ppsSeq, ppsUs, pendingUtcArrivalUs,
                               pendingUtcArrivalUs - ppsUs, deltaUs,
                               "Discarded pending UTC: missed next PPS");
    addLog("Discarded pending UTC: missed next PPS. %s decoded %lu lead %ld ms",
           pendingUtcSource, (unsigned long)pendingUtcEpoch, (long)(leadUs / 1000LL));
    pendingUtcValid = false;
    return;
  }

  acceptTimeSample(pendingUtcEpoch, ppsUs, true, pendingUtcSource, ppsSeq,
                   pendingUtcArrivalUs);
  pendingUtcValid = false;
}

static void handleGpsUtc(uint32_t epoch, const char *source,
                         int64_t gpsArrivalUs, uint32_t sentenceSeq) {
  if (epoch == 0) {
    rejectSample(source, "invalid UTC");
    return;
  }

  if (sentenceSeq != 0 && sentenceSeq == lastHandledGpsSentenceSeq &&
      epoch == lastHandledGpsEpoch && strcmp(source, lastHandledGpsSource) == 0) {
    return;
  }
  if (epoch == lastHandledGpsEpoch && strcmp(source, lastHandledGpsSource) == 0) {
    return;
  }
  lastHandledGpsSentenceSeq = sentenceSeq;
  lastHandledGpsEpoch = epoch;
  strlcpy(lastHandledGpsSource, source, sizeof(lastHandledGpsSource));

  lastGpsEpoch = epoch;
  lastGpsValidMs = millis();

  int64_t nowUs = (gpsArrivalUs > 0) ? gpsArrivalUs : esp_timer_get_time();
  int64_t ppsUs = 0;
  uint32_t ppsSeq = 0;
  bool hasPps = snapshotLatestPps(&ppsUs, &ppsSeq);
  int64_t assignedEpoch = (int64_t)epoch - PPS_EPOCH_OFFSET_SECONDS;
  int64_t gapUs = hasPps ? (nowUs - ppsUs) : 0;
  int64_t deltaUs = 0;

  if (!hasPps) {
    if (!timeEverValid) {
      acceptTimeSample(epoch, nowUs, false, source);
      return;
    }
    rejectGpsPpsCandidate(source, "valid UTC but no PPS seen yet", epoch,
                          assignedEpoch, 0, 0, nowUs, 0, deltaUs);
    return;
  }

  if (ppsSeq == lastAcceptedPpsSeq) {
    queueUtcForNextPps(source, epoch, assignedEpoch, nowUs, sentenceSeq,
                       0, ppsSeq, ppsUs, gapUs,
                       "UTC candidate waiting for next unconsumed PPS");
    return;
  }

  if (assignedEpoch > 0 && timeEverValid) {
    int64_t candidateUnixUs = assignedEpoch * 1000000LL + PPS_CALIBRATION_US;
    deltaUs = candidateUnixUs - (referenceUnixUs + (ppsUs - referenceMonoUs));
  }

  if (ppsSeq == lastCandidatePpsSeq && strcmp(source, lastCandidateSource) != 0 &&
      lastCandidateDecodedEpoch != 0 &&
      llabs((int64_t)epoch - (int64_t)lastCandidateDecodedEpoch) == 1) {
    rejectGpsPpsCandidate(source, "ambiguous UTC source disagreement by one second",
                          epoch, assignedEpoch, ppsSeq, ppsUs, nowUs, gapUs, deltaUs);
    return;
  }

  if (lastConsumedAssignedEpoch != 0 &&
      assignedEpoch > 0 && (uint32_t)assignedEpoch == lastConsumedAssignedEpoch) {
    updateCandidateDiagnostics(source, epoch, assignedEpoch, ppsSeq, ppsUs,
                               nowUs, gapUs, deltaUs,
                               "UTC second already consumed");
    return;
  }

  if (timeEverValid && deltaUs >= PPS_NEXT_EDGE_DELTA_MIN_US &&
      deltaUs <= PPS_NEXT_EDGE_DELTA_MAX_US) {
    queueUtcForNextPps(source, epoch, assignedEpoch, nowUs, sentenceSeq,
                       deltaUs, ppsSeq, ppsUs, gapUs,
                       "candidate belongs to next PPS edge");
    return;
  }

  if (everPpsDisciplined && timebaseSource == TIMEBASE_PPS_DISCIPLINED &&
      syncState == SYNC_SYNCED && !postMaintenanceRevalidate &&
      isWrongEdgeDelta(deltaUs)) {
    ignoreWrongEdgeCandidate(source, epoch, assignedEpoch, ppsSeq, ppsUs,
                             nowUs, gapUs, deltaUs);
    return;
  }

  if (everPpsDisciplined && llabs(deltaUs) > PPS_NEXT_EDGE_DELTA_MAX_US) {
    const char *reason = (timebaseSource == TIMEBASE_PPS_DISCIPLINED)
                             ? "Discarded stale UTC/PPS pairing; already disciplined"
                             : "Discarded stale UTC/PPS pairing before discipline";
    updateCandidateDiagnostics(source, epoch, assignedEpoch, ppsSeq, ppsUs,
                               nowUs, gapUs, deltaUs,
                               reason);
    if (millis() - lastStalePairLogMs > STATUS_LOG_RATE_MS) {
      lastStalePairLogMs = millis();
      if (timebaseSource == TIMEBASE_PPS_DISCIPLINED) {
        addLog("Discarded stale UTC/PPS pairing; already disciplined. %s decoded %lu assigned %lu pps_seq %lu gap %ld ms would step %+lld us",
               source, (unsigned long)epoch,
               (unsigned long)((assignedEpoch > 0) ? assignedEpoch : 0),
               (unsigned long)ppsSeq, (long)(gapUs / 1000LL),
               (long long)deltaUs);
      } else {
        addLog("Discarded stale UTC/PPS pairing before discipline %s: decoded %lu assigned %lu pps_seq %lu gap %ld ms delta %+lld us",
               source, (unsigned long)epoch,
               (unsigned long)((assignedEpoch > 0) ? assignedEpoch : 0),
               (unsigned long)ppsSeq, (long)(gapUs / 1000LL),
               (long long)deltaUs);
      }
    }
    return;
  }

  if (gapUs < PPS_UTC_AFTER_PPS_MIN_US || gapUs > PPS_UTC_AFTER_PPS_MAX_US) {
    bool gpsOnlyCurrentEdgeRebase = (assignedEpoch > 0 && timeEverValid &&
                                     timebaseSource == TIMEBASE_GPS_ONLY &&
                                     !everPpsDisciplined &&
                                     gapUs >= 0 && gapUs <= PPS_PAIR_WINDOW_US);
    bool lateConsistent = (assignedEpoch > 0) &&
                          ((timeEverValid && llabs(deltaUs) <= LATE_CONSISTENT_ACCEPT_US) ||
                           (!timeEverValid && gapUs >= 0 && gapUs <= PPS_PAIR_WINDOW_US) ||
                           gpsOnlyCurrentEdgeRebase);
    if (ppsSeq != lastAcceptedPpsSeq && lateConsistent) {
      addLog("Accepted late-but-consistent GPS/PPS %s: decoded %lu assigned %lu pps_seq %lu gap %ld ms delta %+lld us",
             source, (unsigned long)epoch,
             (unsigned long)assignedEpoch, (unsigned long)ppsSeq,
             (long)(gapUs / 1000LL), (long long)deltaUs);
      acceptTimeSample(epoch, ppsUs, true, source, ppsSeq, nowUs,
                       "accepted late-but-consistent");
      return;
    }
    rejectGpsPpsCandidate(source, "UTC sentence outside deterministic PPS pairing window",
                          epoch, assignedEpoch, ppsSeq, ppsUs, nowUs, gapUs, deltaUs);
    if (!timeEverValid) {
      acceptTimeSample(epoch, nowUs, false, source);
    }
    return;
  }

  acceptTimeSample(epoch, ppsUs, true, source, ppsSeq, nowUs);
}

static bool parseZdaEpoch(TinyGPSCustom &timeField, TinyGPSCustom &dayField,
                          TinyGPSCustom &monthField, TinyGPSCustom &yearField,
                          uint32_t *epochOut) {
  if (!(timeField.isUpdated() || dayField.isUpdated() ||
        monthField.isUpdated() || yearField.isUpdated())) {
    return false;
  }
  if (!timeField.isValid() || !dayField.isValid() ||
      !monthField.isValid() || !yearField.isValid()) {
    return false;
  }

  const char *t = timeField.value();
  if (!t || strlen(t) < 6) return false;
  if (!isdigit((unsigned char)t[0]) || !isdigit((unsigned char)t[1]) ||
      !isdigit((unsigned char)t[2]) || !isdigit((unsigned char)t[3]) ||
      !isdigit((unsigned char)t[4]) || !isdigit((unsigned char)t[5])) {
    return false;
  }

  uint8_t hour = (uint8_t)((t[0] - '0') * 10 + (t[1] - '0'));
  uint8_t minute = (uint8_t)((t[2] - '0') * 10 + (t[3] - '0'));
  uint8_t second = (uint8_t)((t[4] - '0') * 10 + (t[5] - '0'));
  uint8_t day = (uint8_t)atoi(dayField.value());
  uint8_t month = (uint8_t)atoi(monthField.value());
  uint16_t year = (uint16_t)atoi(yearField.value());

  *epochOut = unixFromDateTime(year, month, day, hour, minute, second);
  return *epochOut != 0;
}

static bool parseRmcEpoch(TinyGPSCustom &timeField, TinyGPSCustom &statusField,
                          TinyGPSCustom &dateField, uint32_t *epochOut) {
  (void)statusField;
  if (!(timeField.isUpdated() || dateField.isUpdated() || statusField.isUpdated())) {
    return false;
  }
  if (!timeField.isValid() || !dateField.isValid()) {
    return false;
  }

  const char *t = timeField.value();
  const char *d = dateField.value();
  if (!t || !d || strlen(t) < 6 || strlen(d) < 6) return false;
  for (uint8_t i = 0; i < 6; i++) {
    if (!isdigit((unsigned char)t[i]) || !isdigit((unsigned char)d[i])) return false;
  }

  uint8_t hour = (uint8_t)((t[0] - '0') * 10 + (t[1] - '0'));
  uint8_t minute = (uint8_t)((t[2] - '0') * 10 + (t[3] - '0'));
  uint8_t second = (uint8_t)((t[4] - '0') * 10 + (t[5] - '0'));
  uint8_t day = (uint8_t)((d[0] - '0') * 10 + (d[1] - '0'));
  uint8_t month = (uint8_t)((d[2] - '0') * 10 + (d[3] - '0'));
  uint8_t yy = (uint8_t)((d[4] - '0') * 10 + (d[5] - '0'));
  uint16_t year = (yy >= 80) ? (uint16_t)(1900 + yy) : (uint16_t)(2000 + yy);

  *epochOut = unixFromDateTime(year, month, day, hour, minute, second);
  return *epochOut != 0;
}

static void updateGgaTelemetry(TinyGPSCustom &fixField, TinyGPSCustom &satField,
                               TinyGPSCustom &hdopField, const char *source) {
  bool anyUpdated = false;

  if (fixField.isUpdated() && fixField.isValid()) {
    gpsFixQuality = (uint8_t)atoi(fixField.value());
    gpsFixValid = gpsFixQuality > 0;
    anyUpdated = true;
  }
  if (satField.isUpdated() && satField.isValid()) {
    gpsSatellites = (uint32_t)atoi(satField.value());
    gpsSatellitesValid = true;
    anyUpdated = true;
  }
  if (hdopField.isUpdated() && hdopField.isValid()) {
    gpsHdop = atof(hdopField.value());
    gpsHdopValid = true;
    anyUpdated = true;
  }

  if (anyUpdated) {
    gpsTelemetryMs = millis();
    strlcpy(gpsTelemetrySource, source, sizeof(gpsTelemetrySource));
  }
}

static void enqueueGpsBridgeTx(uint8_t c) {
  if (!gpsBridgeStarted || !gpsBridgeClient.connected()) return;
  if (gpsBridgeFirstBytesCount < 32) {
    static const char hex[] = "0123456789ABCDEF";
    uint8_t pos = gpsBridgeFirstBytesCount * 3;
    gpsBridgeFirstBytesHex[pos] = hex[(c >> 4) & 0x0F];
    gpsBridgeFirstBytesHex[pos + 1] = hex[c & 0x0F];
    gpsBridgeFirstBytesHex[pos + 2] = (gpsBridgeFirstBytesCount == 31) ? '\0' : ' ';
    gpsBridgeFirstBytesCount++;
    if (gpsBridgeFirstBytesCount == 32 && !gpsBridgeFirstBytesLogged) {
      gpsBridgeFirstBytesLogged = true;
      addLog("GPS bridge first raw bytes: %s", gpsBridgeFirstBytesHex);
    }
  }
  if (gpsBridgeTxCount >= GPS_BRIDGE_TX_RING_SIZE) {
    gpsBridgeBytesDropped++;
    return;
  }
  gpsBridgeTxRing[gpsBridgeTxHead] = c;
  gpsBridgeTxHead = (uint16_t)((gpsBridgeTxHead + 1) % GPS_BRIDGE_TX_RING_SIZE);
  gpsBridgeTxCount++;
  gpsBridgeMirrorBytesQueued++;
}

static void dropGpsBridgeTxBytes(uint16_t count) {
  while (count-- > 0 && gpsBridgeTxCount > 0) {
    gpsBridgeTxTail = (uint16_t)((gpsBridgeTxTail + 1) % GPS_BRIDGE_TX_RING_SIZE);
    gpsBridgeTxCount--;
    gpsBridgeBytesDropped++;
  }
}

static void serviceGpsBridgeTx() {
  if (!gpsBridgeStarted || !gpsBridgeClient.connected() || gpsBridgeTxCount == 0) return;

  int writable = gpsBridgeClient.availableForWrite();
  (void)writable;
  uint16_t budget = min((uint16_t)GPS_BRIDGE_TX_BYTES_PER_LOOP, gpsBridgeTxCount);

  uint8_t out[GPS_BRIDGE_TX_BYTES_PER_LOOP];
  uint16_t outLen = 0;
  uint16_t idx = gpsBridgeTxTail;
  while (outLen < budget && outLen < gpsBridgeTxCount) {
    out[outLen++] = gpsBridgeTxRing[idx];
    idx = (uint16_t)((idx + 1) % GPS_BRIDGE_TX_RING_SIZE);
  }
  if (outLen == 0) return;

  size_t written = gpsBridgeClient.write(out, outLen);
  if (written == 0) {
    dropGpsBridgeTxBytes(1);
    return;
  }
  while (written-- > 0 && gpsBridgeTxCount > 0) {
    gpsBridgeTxTail = (uint16_t)((gpsBridgeTxTail + 1) % GPS_BRIDGE_TX_RING_SIZE);
    gpsBridgeTxCount--;
    gpsBridgeBytesToClient++;
    lastGpsBridgeTxMs = millis();
  }
}

static void serviceGpsBridge() {
  if (!gpsBridgeStarted || WiFi.status() != WL_CONNECTED) return;

  if (gpsMaintenanceMode && (int32_t)(gpsMaintenanceUntilMs - millis()) <= 0) {
    disableGpsMaintenanceMode("auto-timeout");
  }

  if (gpsBridgeClientWasConnected && !gpsBridgeClient.connected()) {
    if (gpsMaintenanceMode) {
      disableGpsMaintenanceMode("TCP client disconnected");
    }
    gpsBridgeClient.stop();
    gpsBridgeClientWasConnected = false;
    gpsBridgeTxHead = 0;
    gpsBridgeTxTail = 0;
    gpsBridgeTxCount = 0;
    addLog("GPS TCP bridge client disconnected");
  }

  if (!gpsBridgeClient.connected()) {
    WiFiClient newClient = gpsBridgeServer.available();
    if (newClient) {
      gpsBridgeTxHead = 0;
      gpsBridgeTxTail = 0;
      gpsBridgeTxCount = 0;
      gpsBridgeFirstBytesHex[0] = '\0';
      gpsBridgeFirstBytesCount = 0;
      gpsBridgeFirstBytesLogged = false;
      gpsBridgeClient = newClient;
      gpsBridgeClient.setNoDelay(true);
      gpsBridgeClientWasConnected = true;
      lastGpsBridgeNoTxLogMs = millis();
      addLog("GPS TCP bridge client connected from %s",
             gpsBridgeClient.remoteIP().toString().c_str());
    }
  }

  serviceGpsBridgeTx();

  uint16_t budget = GPS_BRIDGE_BYTES_PER_LOOP;
  while (gpsBridgeClient.connected() && gpsBridgeClient.available() > 0 && budget-- > 0) {
    int b = gpsBridgeClient.read();
    if (b < 0) break;
    gpsBridgeBytesFromClient++;
    if (gpsMaintenanceMode) {
      GPSSerial.write((uint8_t)b);
      gpsBridgeRxForwarded++;
      if (millis() - lastGpsBridgeRxForwardLogMs > STATUS_LOG_RATE_MS) {
        lastGpsBridgeRxForwardLogMs = millis();
        addLog("TCP-to-GPS bytes forwarded");
      }
    } else {
      gpsBridgeRxDiscarded++;
      if (millis() - lastGpsBridgeRxDiscardLogMs > STATUS_LOG_RATE_MS) {
        lastGpsBridgeRxDiscardLogMs = millis();
        addLog("TCP-to-GPS bytes discarded");
      }
    }
  }

  if (gpsBridgeClient.connected() && ageOrMax(lastGpsDataMs) <= GPS_STALE_MS &&
      ageOrMax(lastGpsBridgeTxMs) > GPS_BRIDGE_NO_TX_LOG_MS &&
      millis() - lastGpsBridgeNoTxLogMs > GPS_BRIDGE_NO_TX_LOG_MS) {
    lastGpsBridgeNoTxLogMs = millis();
    addLog("GPS bridge client connected but no bytes sent. serial %llu queued %llu ring %u dropped %llu",
           (unsigned long long)gpsSerialBytesSeen,
           (unsigned long long)gpsBridgeMirrorBytesQueued,
           (unsigned)gpsBridgeTxCount,
           (unsigned long long)gpsBridgeBytesDropped);
  }
}

static void serviceGps() {
  uint16_t budget = otaSafeActive() ? GPS_BYTES_PER_LOOP_OTA : GPS_BYTES_PER_LOOP;

  while (budget-- > 0 && GPSSerial.available() > 0) {
    char c = (char)GPSSerial.read();
    bool sentenceComplete = gps.encode(c);
    gpsSerialBytesSeen++;
    gpsParserBytesThisSecond++;
    uint32_t nowMs = millis();
    if (gpsParserRateWindowMs == 0) gpsParserRateWindowMs = nowMs;
    if (nowMs - gpsParserRateWindowMs >= 1000UL) {
      gpsParserBytesPerSecond = gpsParserBytesThisSecond;
      gpsParserBytesThisSecond = 0;
      gpsParserRateWindowMs = nowMs;
    }
    if (sentenceComplete) {
      lastGpsSentenceMs = millis();
      lastGpsSentenceSeq++;
      lastGpsSentenceMonoUs = esp_timer_get_time();

      uint32_t utcEpoch = 0;
      if (parseRmcEpoch(gprmcTime, gprmcStatus, gprmcDate, &utcEpoch)) {
        gpsUtcValid = true;
        lastGpsRmcMs = millis();
        handleGpsUtc(utcEpoch, "RMC", lastGpsSentenceMonoUs, lastGpsSentenceSeq);
      } else if (parseRmcEpoch(gnrmcTime, gnrmcStatus, gnrmcDate, &utcEpoch)) {
        gpsUtcValid = true;
        lastGpsRmcMs = millis();
        handleGpsUtc(utcEpoch, "RMC", lastGpsSentenceMonoUs, lastGpsSentenceSeq);
      } else if (parseZdaEpoch(gpzdaTime, gpzdaDay, gpzdaMonth, gpzdaYear, &utcEpoch)) {
        gpsUtcValid = true;
        handleGpsUtc(utcEpoch, "GPZDA", lastGpsSentenceMonoUs, lastGpsSentenceSeq);
      } else if (parseZdaEpoch(gnzdaTime, gnzdaDay, gnzdaMonth, gnzdaYear, &utcEpoch)) {
        gpsUtcValid = true;
        handleGpsUtc(utcEpoch, "GNZDA", lastGpsSentenceMonoUs, lastGpsSentenceSeq);
      }
    }
    enqueueGpsBridgeTx((uint8_t)c);
    lastGpsDataMs = millis();
  }

  if (gps.satellites.isUpdated() && gps.satellites.isValid()) {
    gpsSatellitesValid = true;
    gpsSatellites = gps.satellites.value();
    gpsTelemetryMs = millis();
    strlcpy(gpsTelemetrySource, "TinyGPS", sizeof(gpsTelemetrySource));
  }
  if (gps.hdop.isUpdated() && gps.hdop.isValid()) {
    gpsHdopValid = true;
    gpsHdop = gps.hdop.hdop();
    gpsTelemetryMs = millis();
    strlcpy(gpsTelemetrySource, "TinyGPS", sizeof(gpsTelemetrySource));
  }
  if (gps.location.isUpdated()) {
    gpsPositionKnown = true;
    gpsFixValid = gps.location.isValid();
    gpsTelemetryMs = millis();
    strlcpy(gpsTelemetrySource, "TinyGPS", sizeof(gpsTelemetrySource));
  }
  updateGgaTelemetry(gpggaFixQuality, gpggaSatellites, gpggaHdop, "GPGGA");
  updateGgaTelemetry(gnggaFixQuality, gnggaSatellites, gnggaHdop, "GNGGA");
  if (gpgsaFixType.isUpdated() && gpgsaFixType.isValid()) {
    gpsFixType = (uint8_t)atoi(gpgsaFixType.value());
    if (gpsFixType >= 2) gpsFixValid = true;
    gpsTelemetryMs = millis();
    strlcpy(gpsTelemetrySource, "GPGSA", sizeof(gpsTelemetrySource));
  }
  if (gngsaFixType.isUpdated() && gngsaFixType.isValid()) {
    gpsFixType = (uint8_t)atoi(gngsaFixType.value());
    if (gpsFixType >= 2) gpsFixValid = true;
    gpsTelemetryMs = millis();
    strlcpy(gpsTelemetrySource, "GNGSA", sizeof(gpsTelemetrySource));
  }
  if ((gprmcStatus.isUpdated() && gprmcStatus.isValid()) ||
      (gnrmcStatus.isUpdated() && gnrmcStatus.isValid())) {
    const char *status = (gprmcStatus.isUpdated() && gprmcStatus.isValid()) ?
                         gprmcStatus.value() : gnrmcStatus.value();
    if (status && status[0] == 'A') {
      gpsFixValid = true;
      gpsTelemetryMs = millis();
      strlcpy(gpsTelemetrySource, "RMC", sizeof(gpsTelemetrySource));
    }
  }

}

static void serviceSyncState() {
  uint32_t nowMs = millis();
  bool gpsFresh = ageOrMax(lastGpsDataMs) <= GPS_STALE_MS;
  bool utcFresh = ageOrMax(lastGpsValidMs) <= GPS_STALE_MS;
  bool ppsFresh = ageOrMax(lastPpsSeenMs) <= PPS_STALE_MS;

  if (ppsFresh != ppsWasFresh) {
    addLog(ppsFresh ? "PPS detected. Remaining %s" : "PPS loss detected",
           syncStateName(syncState));
    ppsWasFresh = ppsFresh;
  }
  if (gpsFresh != gpsWasFresh) {
    addLog(gpsFresh ? "GPS receiver recovered" : "GPS receiver loss detected");
    gpsWasFresh = gpsFresh;
  }
  if (utcFresh != utcWasValid) {
    addLog(utcFresh ? "GPS UTC valid/recent" : "GPS UTC stale or invalid");
    utcWasValid = utcFresh;
  }

  if (!timeEverValid) {
    setSyncState(gpsFresh ? SYNC_ACQUIRING : SYNC_UNSYNCHRONISED,
                 gpsFresh ? "GPS receiver active; waiting for valid time" : "no valid time and no GPS data");
  } else if (!everPpsDisciplined) {
    setSyncState(SYNC_ACQUIRING, "GPS-only time valid; waiting for PPS-disciplined sample");
  } else if (ppsFresh) {
    setSyncState(SYNC_SYNCED, "PPS fresh and disciplined clock valid");
    if (!utcFresh && nowMs - lastGpsUtcStaleSyncedLogMs > GPS_UTC_STALE_SYNC_LOG_MS) {
      lastGpsUtcStaleSyncedLogMs = nowMs;
      addLog("GPS UTC stale; PPS still fresh, staying synced");
    }
  } else {
    char reason[48];
    uint32_t ppsAge = ageOrMax(lastPpsSeenMs);
    snprintf(reason, sizeof(reason), "PPS missing for %lu ms", (unsigned long)ppsAge);
    setSyncState(SYNC_HOLDOVER, reason);
  }

  bool warnNow = (lastPpsSeenMs == 0) ? (nowMs > PPS_WARNING_MS)
                                      : (ageOrMax(lastPpsSeenMs) > PPS_WARNING_MS);
  if (warnNow && !ppsWarningActive) {
    addLog("PPS warning: no recent PPS; timing accuracy may be degraded");
  }
  ppsWarningActive = warnNow;

  if (!gpsFresh && nowMs - lastGpsStaleLogMs > STATUS_LOG_RATE_MS) {
    lastGpsStaleLogMs = nowMs;
    addLog("GPS receiver data stale");
  }

  if (!ppsFresh && nowMs - lastPpsStaleLogMs > STATUS_LOG_RATE_MS) {
    lastPpsStaleLogMs = nowMs;
    addLog("PPS missing or stale");
  }
}

// ----------------------------- Wi-Fi and OTA -----------------------------

static void startServicesIfNeeded() {
  if (servicesStarted || WiFi.status() != WL_CONNECTED) return;

  WiFi.setSleep(false);
  server.begin();
  ntpUdp.begin(NTP_PORT);
  gpsBridgeServer.begin();
  gpsBridgeStarted = true;
  ArduinoOTA.begin();
  servicesStarted = true;

  addLog("DHCP IP assigned: %s", WiFi.localIP().toString().c_str());
  addLog("Dashboard ready on http://%s/", WiFi.localIP().toString().c_str());
  addLog("NTP server listening on UDP port %u", NTP_PORT);
  addLog("GPS TCP bridge listening on port %u", GPS_BRIDGE_PORT);
  addLog("GPS bridge read-only: u-center commands discarded");
  addLog("OTA ready");
}

static void serviceWifi() {
  uint32_t nowMs = millis();

  if (WiFi.status() == WL_CONNECTED) {
    startServicesIfNeeded();
    return;
  }

  if (nowMs - lastWifiAttemptMs < WIFI_RECONNECT_MS) return;
  lastWifiAttemptMs = nowMs;

  addLog("Connecting to Wi-Fi SSID %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ----------------------------- JSON and Web Dashboard -----------------------------

static void jsonEscapeAppend(String &out, const char *s) {
  while (*s) {
    char c = *s++;
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if ((uint8_t)c < 0x20) {
      out += ' ';
    } else {
      out += c;
    }
  }
}

static void appendJsonStringField(String &out, const char *name, const char *value, bool comma = true) {
  out += '"';
  out += name;
  out += "\":\"";
  jsonEscapeAppend(out, value);
  out += '"';
  if (comma) out += ',';
}

static void appendJsonU64Field(String &out, const char *name, uint64_t value, bool comma = true) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%llu", (unsigned long long)value);
  out += '"';
  out += name;
  out += "\":";
  out += buf;
  if (comma) out += ',';
}

static void appendJsonI64Field(String &out, const char *name, int64_t value, bool comma = true) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%lld", (long long)value);
  out += '"';
  out += name;
  out += "\":";
  out += buf;
  if (comma) out += ',';
}

static void appendJsonI32Field(String &out, const char *name, int32_t value, bool comma = true) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%ld", (long)value);
  out += '"';
  out += name;
  out += "\":";
  out += buf;
  if (comma) out += ',';
}

static void appendJsonU32Field(String &out, const char *name, uint32_t value, bool comma = true) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)value);
  out += '"';
  out += name;
  out += "\":";
  out += buf;
  if (comma) out += ',';
}

static void appendJsonBoolField(String &out, const char *name, bool value, bool comma = true) {
  out += '"';
  out += name;
  out += "\":";
  out += value ? "true" : "false";
  if (comma) out += ',';
}

static const char *positionStatusText() {
  if (gpsFixType == 2) return "2D";
  if (gpsFixType == 3) return "3D";
  if (gpsFixType == 1) return "No Fix";
  return "Unavailable";
}

static const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 GPS/PPS NTP Server</title>
<style>
:root{color-scheme:dark;--bg:#080b10;--panel:#101722;--panel2:#141e2b;--panel3:#0d131c;--text:#eef5fb;--muted:#93a4b7;--line:#263444;--ok:#36d184;--warn:#ffc857;--bad:#ff6678;--blue:#67b7ff;--shadow:0 18px 45px rgba(0,0,0,.32)}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#0c121b 0,#080b10 38%,#07090d 100%);color:var(--text);font:14px/1.45 system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}
main{max-width:1680px;margin:0 auto;padding:24px}.top{display:grid;grid-template-columns:minmax(520px,1fr) minmax(360px,.8fr);gap:22px;align-items:start;margin-bottom:16px}
h1{font-size:26px;margin:0 0 8px}.muted{color:var(--muted)}.small{font-size:12px}.mono,.clock,.metric,.kv b{font-variant-numeric:tabular-nums;font-family:ui-monospace,SFMono-Regular,Consolas,monospace}.clock{font-weight:800;font-size:clamp(38px,5vw,72px);line-height:.98;color:#fff;letter-spacing:0;white-space:nowrap}
.badges{display:flex;gap:7px;flex-wrap:wrap;justify-content:flex-end}.badge{display:inline-flex;align-items:center;gap:7px;border:1px solid var(--line);border-radius:6px;padding:5px 8px;background:rgba(20,30,43,.88);font-weight:700;font-size:12px;color:var(--muted)}.badge.on{color:var(--text);border-color:#39516a}.badge.ok.on{color:var(--ok)}.badge.warn.on{color:var(--warn)}.badge.bad.on{color:var(--bad)}.dot{width:7px;height:7px;border-radius:50%;background:currentColor}
.warning{border:1px solid var(--line);border-left-width:4px;border-radius:8px;padding:11px 13px;margin:14px 0;background:rgba(16,23,34,.96);box-shadow:var(--shadow)}.warning.ok{border-left-color:var(--ok)}.warning.warn{border-left-color:var(--warn)}.warning.bad{border-left-color:var(--bad)}
.grid{display:grid;grid-template-columns:repeat(12,minmax(0,1fr));gap:14px;align-items:stretch}.card{grid-column:span 4;background:linear-gradient(180deg,var(--panel),var(--panel3));border:1px solid var(--line);border-radius:8px;padding:16px;min-width:0;box-shadow:var(--shadow)}.card.timebase,.card.log{grid-column:span 8}.card.full{grid-column:1/-1}
.card h2{font-size:13px;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin:0 0 10px}.statusline{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:10px}.metric{font-size:28px;font-weight:800;overflow-wrap:anywhere}.kv{display:grid;grid-template-columns:minmax(118px,.68fr) minmax(0,1.55fr);gap:7px 14px}.kv span{color:var(--muted)}.kv b{font-weight:700;overflow-wrap:anywhere;min-height:20px}.kv b.long{font-size:12px;line-height:1.35;white-space:pre-line;word-break:normal}.note{margin-top:10px;color:var(--muted);font-size:13px}.maplink{display:inline-block;margin-top:8px;color:var(--blue);text-decoration:none;font-weight:700}.maplink:hover{text-decoration:underline}
.actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}.actions button,.card button{border:1px solid #35506a;border-radius:6px;background:#162335;color:var(--text);padding:8px 10px;font:700 13px system-ui;cursor:pointer}.actions button:hover,.card button:hover{border-color:var(--blue)}
.console{height:min(48vh,520px);min-height:340px;overflow:auto;background:#05080c;border:1px solid #1e2a38;border-radius:8px;padding:12px;font:12px/1.5 ui-monospace,SFMono-Regular,Consolas,monospace;white-space:pre-wrap;color:#cbd8e7}
@media(max-width:1180px){.top{grid-template-columns:1fr}.badges{justify-content:flex-start}.card,.card.timebase,.card.log{grid-column:span 6}}
@media(max-width:720px){main{padding:12px}.grid{grid-template-columns:1fr}.card,.card.timebase,.card.log{grid-column:1}.kv{grid-template-columns:1fr}.clock{white-space:normal;font-size:clamp(34px,13vw,54px)}}
</style>
</head>
<body>
<main>
<header class="top">
<div><h1>ESP32 GPS/PPS NTP Server</h1><div id="utcClock" class="clock">--</div><div class="muted">Visual clock interpolated from ESP32 PPS-disciplined timebase. NTP replies use the ESP32 internal disciplined clock.</div></div>
<div class="badges">
<div id="badgePps" class="badge"><span class="dot"></span>PPS disciplined</div><div id="badgeHoldover" class="badge"><span class="dot"></span>Holdover</div><div id="badgeAcquiring" class="badge"><span class="dot"></span>Acquiring</div><div id="badgeUnsync" class="badge"><span class="dot"></span>Unsynchronised</div><div id="badgeUtcStale" class="badge"><span class="dot"></span>GPS UTC stale</div><div id="badgeMaint" class="badge"><span class="dot"></span>Maintenance</div><div id="badgeReval" class="badge"><span class="dot"></span>Revalidating GPS/PPS</div><div id="badgeOta" class="badge ok on"><span class="dot"></span>OTA Ready</div>
</div>
</header>
<section class="actions"><button id="otaSafeBtn" type="button">OTA Safe</button><button id="rebootBtn" type="button">Reboot</button></section>
<section id="warningBox" class="warning bad"><strong id="warningTitle">Waiting for telemetry</strong><div id="warningText" class="small">Fetching /status...</div></section>
<section class="grid">
<div class="card timebase"><div class="statusline"><h2>Timebase</h2><span id="timebase_badge" class="badge">--</span></div><div class="kv"><span>Sync state</span><b id="sync_state">--</b><span>Timebase</span><b id="timebase_type">--</b><span>PPS age</span><b id="pps_age">--</b><span>GPS UTC age</span><b id="utc_age">--</b><span>Last accepted</span><b id="last_accept" class="long">--</b><span>Last good PPS</span><b id="last_good_pps" class="long">--</b><span>Last rejected delta</span><b id="last_reject_delta">--</b><span>Reject reason</span><b id="last_reject_reason" class="long">--</b><span>UTC/PPS alignment</span><b id="alignment" class="long">--</b><span>Revalidation</span><b id="revalidation" class="long">--</b></div></div>
<div class="card"><div class="statusline"><h2>NTP Server</h2><span id="ntp_badge" class="badge">--</span></div><div class="metric mono" id="ntp_count">--</div><div class="kv"><span>UDP port</span><b id="ntp_port">123</b><span>Last client</span><b id="last_ntp_client" class="long">--</b><span>Last request</span><b id="last_ntp_age">--</b><span>TX age</span><b id="last_ntp_tx_age">--</b></div></div>
<div class="card"><div class="statusline"><h2>GPS Receiver</h2><span id="gps_status" class="badge">--</span></div><div class="kv"><span>UTC</span><b id="utc_status" class="long">--</b><span>Parser rate</span><b id="gps_parser_rate">--</b><span>Last RMC</span><b id="gps_rmc_age">--</b><span>Satellites</span><b id="satellites">--</b><span>HDOP</span><b id="hdop">--</b><span>Fix telemetry</span><b id="position_status" class="long">--</b><span>Telemetry source</span><b id="telemetry_source" class="long">--</b><span>Last UTC source</span><b id="last_utc_source">--</b><span>Candidate</span><b id="candidate_summary" class="long">--</b></div></div>
<div class="card"><div class="statusline"><h2>GPS TCP Bridge</h2><span id="bridge_badge" class="badge">--</span></div><div class="kv"><span>Port</span><b id="bridge_port">5000</b><span>Mode</span><b id="bridge_mode">--</b><span>Client</span><b id="bridge_client">--</b><span>TX to client</span><b id="bridge_tx">--</b><span>TX dropped</span><b id="bridge_tx_dropped">--</b><span>TX queued</span><b id="bridge_tx_queued">--</b><span>RX discarded</span><b id="bridge_discarded">--</b><span>RX forwarded</span><b id="bridge_forwarded">--</b><span>Last TX</span><b id="bridge_last_tx">--</b><span>Maintenance</span><b id="bridge_maint_remaining">--</b></div><div id="bridge_note" class="note">--</div><div class="actions"><button id="gpsMaintBtn" type="button">GPS Maintenance</button></div></div>
<div class="card"><div class="statusline"><h2>System</h2><span id="ota_status" class="badge ok">--</span></div><div class="kv"><span>Uptime</span><b id="uptime">--</b><span>Wi-Fi RSSI</span><b id="wifi">--</b><span>IP address</span><b id="ip">--</b><span>Free heap</span><b id="heap">--</b><span>Min heap</span><b id="min_heap">--</b><span>CPU / Flash</span><b id="cpu_flash">--</b></div></div>
<div class="card"><div class="statusline"><h2>Position / Map</h2><span id="position_badge" class="badge">--</span></div><div class="kv"><span>Position</span><b id="map_position">--</b><span>Quality gate</span><b id="map_quality">--</b></div><div id="map_note" class="note">Position fix unavailable. Timing can still be valid without position.</div><a id="osm_link" class="maplink" href="#" target="_blank" rel="noopener" style="display:none">Open in OpenStreetMap</a><div class="note">Map uses browser access to OpenStreetMap when position fix is valid.</div></div>
<div class="card log"><div class="statusline"><h2>Live Log</h2><span id="counters" class="muted mono">--</span></div><div id="console" class="console"></div></div>
</section>
</main>
<script>
let anchorUnixSeconds=0,anchorFracUs=0,browserPerfAnchor=0,lastLogSeq=-1;
const $=id=>document.getElementById(id);
function setText(id,v){const e=$(id);if(e&&e.textContent!==String(v))e.textContent=v;}
function postAction(path){fetch(path,{method:'POST',cache:'no-store'}).then(()=>poll()).catch(()=>{});}
function fmtAge(ms){if(ms===4294967295||ms<0)return'never';let s=Math.floor(ms/1000);if(s<60)return s+'s ago';let m=Math.floor(s/60);s%=60;if(m<60)return m+'m '+s+'s ago';let h=Math.floor(m/60);m%=60;return h+'h '+m+'m ago';}
function pad(n,w){return String(n).padStart(w,'0');}
function fmtBytes(v){v=Number(v||0);if(v<1000)return v+' B';if(v<1000000)return Math.round(v/10)/100+' KB';return Math.round(v/10000)/100+' MB';}
function fmtUs(v){v=Number(v||0);return(v>0?'+':'')+v+' us';}
function fmtUtc(sec,fracUs){if(sec<=0)return'not valid';let d=new Date(sec*1000),frac4=Math.floor(fracUs/100);return pad(d.getUTCHours(),2)+':'+pad(d.getUTCMinutes(),2)+':'+pad(d.getUTCSeconds(),2)+'.'+pad(frac4,4)+' UTC';}
function animate(){if(anchorUnixSeconds>0){let elapsedUs=Math.floor((performance.now()-browserPerfAnchor)*1000),totalFracUs=anchorFracUs+elapsedUs,displayUnixSeconds=anchorUnixSeconds+Math.floor(totalFracUs/1000000),displayFracUs=totalFracUs%1000000;setText('utcClock',fmtUtc(displayUnixSeconds,displayFracUs));}requestAnimationFrame(animate);}
function levelFor(s,valid){if(!valid)return'bad';if(s==='Synced')return'ok';return'warn';}
function badge(id,on,kind){const e=$(id);if(!e)return;e.className='badge '+(kind||'')+(on?' on':'');}
async function poll(){try{let r=await fetch('/status',{cache:'no-store'});let j=await r.json();if(j.time_valid){let currentUnixUs=Number(j.current_unix_us||j.unix_us);anchorUnixSeconds=Math.floor(currentUnixUs/1000000);anchorFracUs=Math.floor(currentUnixUs-(anchorUnixSeconds*1000000));browserPerfAnchor=performance.now();}else{anchorUnixSeconds=0;anchorFracUs=0;setText('utcClock','time not valid');}
let lvl=levelFor(j.sync_state,j.time_valid);
badge('badgePps',j.timebase_type==='PPS-disciplined'&&j.sync_state==='Synced','ok');badge('badgeHoldover',j.sync_state==='Holdover','warn');badge('badgeAcquiring',j.sync_state==='Acquiring','warn');badge('badgeUnsync',!j.time_valid||j.sync_state==='Unsynchronised','bad');badge('badgeUtcStale',!j.utc_fresh&&j.time_valid,'warn');badge('badgeMaint',j.gps_bridge_allow_client_to_gps,'warn');badge('badgeReval',j.post_maintenance_revalidate,'warn');badge('badgeOta',j.ota_status==='Ready','ok');
$('warningBox').className='warning '+j.warning_level;setText('warningTitle',j.warning_title);setText('warningText',j.warning);
setText('sync_state',j.sync_state);setText('timebase_type',j.timebase_type);$('timebase_badge').className='badge '+lvl+' on';setText('timebase_badge',j.sync_state);
setText('pps_age',fmtAge(j.pps_age_ms));setText('utc_age',fmtAge(j.utc_age_ms));setText('last_accept',j.last_accepted_utc);setText('last_good_pps','delta '+fmtUs(j.last_good_pps_delta_us)+' | gap '+j.last_pairing_gap_ms+' ms\n'+j.last_pairing_accept_reason);setText('last_reject_delta',fmtUs(j.last_rejected_delta_us));setText('last_reject_reason',j.last_rejected_reason||'none');setText('alignment',(j.pps_epoch_alignment_mode||'--')+'\noffset '+j.pps_epoch_offset_s+' s');setText('revalidation',j.post_maintenance_revalidate?(j.post_maintenance_good_samples+'/'+j.post_maintenance_required_samples+' good\n'+j.post_maintenance_last_reject_reason):'not active');
$('ntp_badge').className='badge '+(j.time_valid?'ok on':'bad on');setText('ntp_badge',j.time_valid?'Serving':'Not serving');setText('ntp_count',j.ntp_requests);setText('ntp_port',j.ntp_port||123);setText('last_ntp_client',j.last_ntp_client);setText('last_ntp_age',fmtAge(j.last_ntp_age_ms));setText('last_ntp_tx_age',fmtAge(j.last_ntp_tx_age_ms));
$('gps_status').className='badge '+(j.gps_fresh?'ok on':'warn on');setText('gps_status',j.gps_receiver_status);let utcSource=j.last_utc_source||j.last_candidate_source||'Unavailable';setText('utc_status',j.utc_fresh?'Timing valid via '+utcSource+'/PPS\n'+fmtAge(j.utc_age_ms):j.utc_status+' / '+fmtAge(j.utc_age_ms));setText('gps_parser_rate',(j.gps_parser_bytes_per_second||0)+' B/s');setText('gps_rmc_age',fmtAge(j.gps_last_rmc_age_ms));setText('satellites',j.gps_satellites_valid?j.gps_satellites:'Unavailable');setText('hdop',j.gps_hdop_valid?j.gps_hdop:'Unavailable');let q=(j.gps_fix_quality>0)?j.gps_fix_quality:'Unavailable',fixTelemetry=(j.gps_satellites_valid||j.gps_hdop_valid||j.gps_position_valid||j.gps_fix_quality>0)?(j.position_status+' / quality '+q):'Fix telemetry unavailable';setText('position_status',fixTelemetry);setText('telemetry_source',(j.gps_telemetry_source||'Unavailable')+'\n'+fmtAge(j.gps_telemetry_age_ms));setText('last_utc_source',utcSource);let cand=j.last_candidate_stale?'historical '+fmtAge(j.last_candidate_age_ms):'Source '+j.last_candidate_source+' | Seq '+j.last_candidate_pps_seq+'\nGap '+j.last_candidate_gap_ms+' ms | Delta '+fmtUs(j.last_candidate_delta_us)+'\n'+j.last_candidate_reason;setText('candidate_summary',cand);
$('bridge_badge').className='badge '+(j.gps_bridge_client_connected?'ok on':'warn on');setText('bridge_badge',j.gps_bridge_client_connected?'Client connected':'Listening');setText('bridge_port',j.gps_bridge_port);setText('bridge_mode',j.gps_bridge_mode);setText('bridge_client',j.gps_bridge_client_connected?'connected':'none');setText('bridge_tx',fmtBytes(j.gps_bridge_bytes_to_client));setText('bridge_tx_dropped',fmtBytes(j.gps_bridge_tx_dropped));setText('bridge_tx_queued',fmtBytes(j.gps_bridge_tx_queued));setText('bridge_discarded',fmtBytes(j.gps_bridge_rx_discarded));setText('bridge_forwarded',fmtBytes(j.gps_bridge_rx_forwarded));setText('bridge_last_tx',fmtAge(j.gps_bridge_tx_last_age_ms));setText('bridge_maint_remaining',j.gps_bridge_allow_client_to_gps?fmtAge(j.gps_maintenance_remaining_ms).replace(' ago',' remaining'):'off');setText('bridge_note',j.gps_bridge_allow_client_to_gps?'Maintenance mode active: u-center commands are forwarded to the GPS receiver. NTP may degrade until revalidation completes.':'Bridge read-only: u-center can observe GPS data; commands discarded.');setText('gpsMaintBtn',j.gps_bridge_allow_client_to_gps?'Disable GPS Maintenance':'GPS Maintenance');
setText('ota_status',j.ota_status);$('ota_status').className='badge '+(j.ota_status==='Ready'?'ok on':'warn on');setText('wifi',j.rssi+' dBm');setText('ip',j.ip);setText('uptime',j.uptime);setText('cpu_flash',j.cpu_mhz+' MHz / '+j.flash_used);setText('heap',fmtBytes(j.free_heap));setText('min_heap',fmtBytes(j.min_free_heap));
let posRecent=j.gps_position_age_ms!==4294967295&&j.gps_position_age_ms<30000,hdopOk=!j.gps_hdop_valid||Number(j.gps_hdop)<=3.0,satsOk=!j.gps_satellites_valid||Number(j.gps_satellites)>=4,posOk=j.gps_position_valid&&posRecent&&hdopOk&&satsOk,lat=Number(j.gps_latitude),lon=Number(j.gps_longitude),osm=$('osm_link');$('position_badge').className='badge '+(posOk?'ok on':'warn on');setText('position_badge',posOk?'Position valid':'No map');setText('map_position',posOk?(lat.toFixed(5)+', '+lon.toFixed(5)):'Position fix unavailable');setText('map_quality',(j.gps_hdop_valid?'HDOP '+j.gps_hdop:'HDOP unavailable')+' / '+(j.gps_satellites_valid?j.gps_satellites+' sats':'sats unavailable')+' / '+fmtAge(j.gps_position_age_ms));setText('map_note',posOk?'Position valid and recent. Timing remains independent of map display.':'Position fix unavailable. Timing can still be valid without position.');if(osm){if(posOk){osm.style.display='inline-block';osm.href='https://www.openstreetmap.org/?mlat='+encodeURIComponent(lat.toFixed(6))+'&mlon='+encodeURIComponent(lon.toFixed(6))+'#map=16/'+encodeURIComponent(lat.toFixed(6))+'/'+encodeURIComponent(lon.toFixed(6));}else{osm.style.display='none';osm.href='#';}}
setText('counters','PPS '+j.pps_count+' / rejected '+j.rejected_samples+' / GPS-only '+j.gps_only_samples+' / accepted seq '+j.last_accepted_pps_seq+' / rejected seq '+j.last_rejected_pps_seq+' / PPS offset '+j.pps_epoch_offset_s+' s');
if(j.log_seq!==lastLogSeq){lastLogSeq=j.log_seq;const c=$('console');c.textContent=(j.logs||[]).join('\n');c.scrollTop=c.scrollHeight;}
}catch(e){$('warningBox').className='warning bad';setText('warningTitle','Telemetry unavailable');setText('warningText','Could not fetch /status');}}
requestAnimationFrame(animate);poll();setInterval(poll,1000);
$('otaSafeBtn').onclick=()=>postAction('/ota-safe');$('gpsMaintBtn').onclick=()=>{let on=$('gpsMaintBtn').textContent.indexOf('Disable')===0;if(on){postAction('/gps-maintenance-off');}else if(confirm('Enable GPS maintenance mode? u-center can change receiver output and NTP may degrade.')){postAction('/gps-maintenance');}};$('rebootBtn').onclick=()=>{if(confirm('Reboot ESP32 now?'))postAction('/reboot');};
</script>
</body>
</html>
)HTML";

static void buildWarning(char *level, size_t levelLen, char *title, size_t titleLen,
                         char *message, size_t msgLen) {
  bool gpsFresh = ageOrMax(lastGpsDataMs) <= GPS_STALE_MS;
  bool utcFresh = ageOrMax(lastGpsValidMs) <= GPS_STALE_MS;
  bool ppsFresh = ageOrMax(lastPpsSeenMs) <= PPS_STALE_MS;

  if (gpsMaintenanceMode) {
    strlcpy(level, "warn", levelLen);
    strlcpy(title, "GPS maintenance active", titleLen);
    strlcpy(message, "GPS maintenance mode active: u-center can change receiver output; NTP may degrade.", msgLen);
  } else if (postMaintenanceRevalidate) {
    strlcpy(level, "warn", levelLen);
    strlcpy(title, "GPS/PPS revalidation", titleLen);
    snprintf(message, msgLen, "GPS maintenance ended; verifying GPS/PPS timing before normal discipline resumes. Revalidation %u/%u good samples.",
             postMaintenanceGoodSamples, POST_MAINTENANCE_REQUIRED_SAMPLES);
  } else if (!timeEverValid) {
    strlcpy(level, "bad", levelLen);
    strlcpy(title, "Time not valid", titleLen);
    strlcpy(message, "No valid GPS UTC time has been accepted; NTP requests are refused.", msgLen);
  } else if (ppsWarningActive) {
    strlcpy(level, "warn", levelLen);
    strlcpy(title, "PPS missing for 30 minutes", titleLen);
    strlcpy(message, "PPS has not been obtained recently and timing accuracy may be degraded.", msgLen);
  } else if (syncState == SYNC_HOLDOVER) {
    strlcpy(level, "warn", levelLen);
    strlcpy(title, "Holdover active", titleLen);
    snprintf(message, msgLen, "PPS is missing or stale for %lu ms; time is being served from the ESP32 monotonic timer.",
             (unsigned long)ageOrMax(lastPpsSeenMs));
  } else if (!gpsFresh) {
    strlcpy(level, "warn", levelLen);
    strlcpy(title, "GPS receiver data stale", titleLen);
    strlcpy(message, "No recent GPS NMEA bytes have been received; PPS is still tracked independently.", msgLen);
  } else if (!utcFresh) {
    strlcpy(level, "warn", levelLen);
    strlcpy(title, "GPS UTC stale", titleLen);
    strlcpy(message, "No recent valid UTC epoch has been accepted; PPS is still fresh so the disciplined clock remains active.", msgLen);
  } else if (!ppsFresh) {
    strlcpy(level, "warn", levelLen);
    strlcpy(title, "PPS stale", titleLen);
    strlcpy(message, "No recent PPS pulse has been captured.", msgLen);
  } else if (syncState == SYNC_ACQUIRING) {
    strlcpy(level, "warn", levelLen);
    strlcpy(title, "Acquiring PPS lock", titleLen);
    strlcpy(message, "Valid UTC has been seen; waiting for stable GPS/PPS pairing.", msgLen);
  } else {
    strlcpy(level, "ok", levelLen);
    strlcpy(title, "Healthy", titleLen);
    strlcpy(message, "GPS UTC and PPS are fresh; NTP is serving disciplined time.", msgLen);
  }
}

static void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
}

static void handleOtaSafe() {
  otaSafeUntilMs = millis() + OTA_SAFE_MODE_MS;
  addLog("OTA safe mode enabled for %lu ms", (unsigned long)OTA_SAFE_MODE_MS);
  server.send(200, "application/json", "{\"ok\":true,\"ota_safe_ms\":120000}");
}

static void handleGpsMaintenance() {
  enableGpsMaintenanceMode("dashboard");
  server.send(200, "application/json", "{\"ok\":true,\"gps_maintenance\":true}");
}

static void handleGpsMaintenanceOff() {
  bool wasActive = gpsMaintenanceMode;
  disableGpsMaintenanceMode("manual off");
  server.send(200, "application/json",
              wasActive ? "{\"ok\":true,\"gps_maintenance\":false,\"revalidating\":true}"
                        : "{\"ok\":true,\"gps_maintenance\":false,\"revalidating\":false}");
}

static void handleReboot() {
  addLog("Reboot requested from dashboard");
  server.send(200, "application/json", "{\"ok\":true,\"rebooting\":true}");
  delay(20);
  ESP.restart();
}

static void handleStatus() {
  char level[8], title[40], warning[120], uptimeText[20], acceptedText[40];
  char satellitesText[16], hdopText[16];
  char latitudeText[20], longitudeText[20];
  buildWarning(level, sizeof(level), title, sizeof(title), warning, sizeof(warning));
  formatDuration(millis() / 1000UL, uptimeText, sizeof(uptimeText));
  formatIsoUtc(lastAcceptedUnixUs, acceptedText, sizeof(acceptedText));

  uint32_t ppsAge = ageOrMax(lastPpsSeenMs);
  uint32_t gpsAge = ageOrMax(lastGpsDataMs);
  uint32_t gpsSentenceAge = ageOrMax(lastGpsSentenceMs);
  uint32_t utcAge = ageOrMax(lastGpsValidMs);
  uint32_t acceptedAge = (lastAcceptedMonoUs > 0) ?
      (uint32_t)((esp_timer_get_time() - lastAcceptedMonoUs) / 1000LL) : 0xFFFFFFFFUL;
  uint32_t lastNtpAge = ageOrMax(lastNtpRequestMs);
  uint32_t lastNtpTxAge = ageOrMax(lastNtpTxMs);
  uint32_t gpsBridgeLastTxAge = ageOrMax(lastGpsBridgeTxMs);
  uint32_t gpsMaintenanceRemaining = gpsMaintenanceMode && gpsMaintenanceUntilMs != 0 ?
      (uint32_t)(gpsMaintenanceUntilMs - millis()) : 0;
  uint32_t lastCandidateAge = ageOrMax(lastCandidateMs);
  bool lastCandidateStale = lastCandidateAge > CANDIDATE_HISTORY_STALE_MS;
  uint32_t holdoverAge = (syncState == SYNC_HOLDOVER && holdoverSinceMs != 0) ? millis() - holdoverSinceMs : 0xFFFFFFFFUL;
  uint32_t otaSafeRemaining = otaSafeActive() && !otaInProgress && otaSafeUntilMs != 0 ?
      (uint32_t)(otaSafeUntilMs - millis()) : 0;
  bool ppsFresh = ppsAge <= PPS_STALE_MS;
  bool gpsFresh = gpsAge <= GPS_STALE_MS;
  bool utcFresh = utcAge <= GPS_STALE_MS;
  if (gpsSatellitesValid) {
    snprintf(satellitesText, sizeof(satellitesText), "%lu", (unsigned long)gpsSatellites);
  } else {
    strlcpy(satellitesText, "Unavailable", sizeof(satellitesText));
  }
  if (gpsHdopValid) {
    snprintf(hdopText, sizeof(hdopText), "%.1f", gpsHdop);
  } else {
    strlcpy(hdopText, "Unavailable", sizeof(hdopText));
  }
  bool positionValid = gpsPositionKnown && gps.location.isValid();
  uint32_t positionAge = positionValid ? (uint32_t)gps.location.age() : 0xFFFFFFFFUL;
  if (positionValid) {
    snprintf(latitudeText, sizeof(latitudeText), "%.6f", gps.location.lat());
    snprintf(longitudeText, sizeof(longitudeText), "%.6f", gps.location.lng());
  } else {
    strlcpy(latitudeText, "Unavailable", sizeof(latitudeText));
    strlcpy(longitudeText, "Unavailable", sizeof(longitudeText));
  }

  String json;
  json.reserve(7200);
  int64_t statusUnixUs = currentUnixUs();
  json += '{';
  appendJsonStringField(json, "sync_state", syncStateName(syncState));
  appendJsonStringField(json, "timebase_source", timebaseSourceName());
  appendJsonStringField(json, "timebase_type", timebaseSourceName());
  appendJsonBoolField(json, "time_valid", timeEverValid);
  appendJsonI64Field(json, "current_unix_us", statusUnixUs);
  appendJsonI64Field(json, "unix_us", statusUnixUs);
  appendJsonStringField(json, "warning_level", level);
  appendJsonStringField(json, "warning_title", title);
  appendJsonStringField(json, "warning", warning);
  appendJsonBoolField(json, "pps_seen", lastPpsSeenMs != 0);
  appendJsonBoolField(json, "pps_fresh", ppsFresh);
  appendJsonU32Field(json, "pps_age_ms", ppsAge);
  appendJsonBoolField(json, "gps_seen", lastGpsDataMs != 0);
  appendJsonBoolField(json, "gps_fresh", gpsFresh);
  appendJsonStringField(json, "gps_receiver_status", gpsFresh ? "Receiving" : ((lastGpsDataMs != 0) ? "Stale" : "No data"));
  appendJsonU32Field(json, "gps_age_ms", gpsAge);
  appendJsonU32Field(json, "gps_sentence_age_ms", gpsSentenceAge);
  appendJsonBoolField(json, "utc_fresh", utcFresh);
  appendJsonStringField(json, "utc_status",
                        gpsMaintenanceMode ? "Maintenance" :
                        (postMaintenanceRevalidate ? "Revalidating" :
                         (utcFresh ? "Valid" : "Invalid")));
  appendJsonU32Field(json, "utc_age_ms", utcAge);
  appendJsonStringField(json, "position_status", positionStatusText());
  appendJsonStringField(json, "satellites", satellitesText);
  appendJsonBoolField(json, "fix_valid", gpsFixValid);
  appendJsonStringField(json, "hdop", hdopText);
  appendJsonBoolField(json, "gps_satellites_valid", gpsSatellitesValid);
  appendJsonU32Field(json, "gps_satellites", gpsSatellites);
  appendJsonBoolField(json, "gps_hdop_valid", gpsHdopValid);
  appendJsonStringField(json, "gps_hdop", hdopText);
  appendJsonBoolField(json, "gps_position_valid", positionValid);
  appendJsonStringField(json, "gps_latitude", latitudeText);
  appendJsonStringField(json, "gps_longitude", longitudeText);
  appendJsonU32Field(json, "gps_position_age_ms", positionAge);
  appendJsonU32Field(json, "gps_fix_quality", gpsFixQuality);
  appendJsonStringField(json, "gps_telemetry_source", gpsTelemetrySource);
  appendJsonU32Field(json, "gps_telemetry_age_ms", ageOrMax(gpsTelemetryMs));
  appendJsonStringField(json, "last_utc_source", lastHandledGpsSource[0] ? lastHandledGpsSource : "Unavailable");
  appendJsonU32Field(json, "ntp_port", NTP_PORT);
  appendJsonU64Field(json, "ntp_requests", ntpRequestCount);
  appendJsonStringField(json, "last_ntp_client", lastNtpClient.toString().c_str());
  appendJsonU32Field(json, "last_ntp_age_ms", lastNtpAge);
  appendJsonI64Field(json, "last_ntp_tx_unix_us", lastNtpTxUnixUs);
  appendJsonU32Field(json, "last_ntp_tx_age_ms", lastNtpTxAge);
  appendJsonBoolField(json, "gps_bridge_enabled", gpsBridgeStarted);
  appendJsonU32Field(json, "gps_bridge_port", GPS_BRIDGE_PORT);
  appendJsonBoolField(json, "gps_bridge_client_connected", gpsBridgeClient.connected());
  appendJsonStringField(json, "gps_bridge_mode", gpsMaintenanceMode ? "maintenance" : "read_only");
  appendJsonBoolField(json, "gps_bridge_allow_client_to_gps", gpsMaintenanceMode);
  appendJsonU64Field(json, "gps_bridge_bytes_to_client", gpsBridgeBytesToClient);
  appendJsonU64Field(json, "gps_bridge_bytes_from_client", gpsBridgeBytesFromClient);
  appendJsonU64Field(json, "gps_bridge_bytes_dropped", gpsBridgeBytesDropped);
  appendJsonU64Field(json, "gps_bridge_tx_dropped", gpsBridgeBytesDropped);
  appendJsonU64Field(json, "gps_serial_bytes_seen", gpsSerialBytesSeen);
  appendJsonU64Field(json, "gps_bridge_mirror_bytes_queued", gpsBridgeMirrorBytesQueued);
  appendJsonU64Field(json, "gps_bridge_tx_bytes_written", gpsBridgeBytesToClient);
  appendJsonStringField(json, "gps_bridge_first_bytes_hex", gpsBridgeFirstBytesHex);
  appendJsonU64Field(json, "gps_bridge_rx_discarded", gpsBridgeRxDiscarded);
  appendJsonU64Field(json, "gps_bridge_rx_forwarded", gpsBridgeRxForwarded);
  appendJsonU32Field(json, "gps_bridge_last_tx_age_ms", gpsBridgeLastTxAge);
  appendJsonU32Field(json, "gps_bridge_tx_last_age_ms", gpsBridgeLastTxAge);
  appendJsonU32Field(json, "gps_bridge_tx_queued", gpsBridgeTxCount);
  appendJsonU32Field(json, "gps_parser_bytes_per_second", gpsParserBytesPerSecond);
  appendJsonU32Field(json, "gps_last_byte_age_ms", gpsAge);
  appendJsonU32Field(json, "gps_last_rmc_age_ms", ageOrMax(lastGpsRmcMs));
  appendJsonU32Field(json, "gps_last_sentence_age_ms", gpsSentenceAge);
  appendJsonU32Field(json, "gps_maintenance_remaining_ms", gpsMaintenanceRemaining);
  appendJsonBoolField(json, "gps_maintenance_warning", gpsMaintenanceMode);
  appendJsonStringField(json, "gps_maintenance_disable_reason", gpsMaintenanceDisableReason);
  appendJsonBoolField(json, "post_maintenance_revalidate", postMaintenanceRevalidate);
  appendJsonU32Field(json, "post_maintenance_good_samples", postMaintenanceGoodSamples);
  appendJsonU32Field(json, "post_maintenance_required_samples", POST_MAINTENANCE_REQUIRED_SAMPLES);
  appendJsonStringField(json, "post_maintenance_last_reject_reason", postMaintenanceLastRejectReason);
  appendJsonStringField(json, "ip", WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "not connected");
  appendJsonI32Field(json, "rssi", WiFi.status() == WL_CONNECTED ? (int32_t)WiFi.RSSI() : 0);
  appendJsonStringField(json, "uptime", uptimeText);
  appendJsonU32Field(json, "cpu_mhz", getCpuFrequencyMhz());
  json += "\"flash_used\":\"";
  json += String(ESP.getSketchSize() / 1024);
  json += " KB / ";
  json += String(ESP.getFlashChipSize() / 1024);
  json += " KB\",";
  appendJsonU32Field(json, "free_heap", ESP.getFreeHeap());
  appendJsonU32Field(json, "min_free_heap", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
  appendJsonStringField(json, "last_accepted_utc", acceptedText);
  appendJsonU32Field(json, "accepted_age_ms", acceptedAge);
  appendJsonU32Field(json, "holdover_age_ms", holdoverAge);
  appendJsonU32Field(json, "pps_count", ppsEventsProcessed);
  appendJsonU32Field(json, "rejected_samples", rejectedSamples);
  appendJsonU32Field(json, "gps_only_samples", gpsOnlySamples);
  appendJsonI32Field(json, "pps_epoch_offset_s", PPS_EPOCH_OFFSET_SECONDS);
  appendJsonStringField(json, "pps_epoch_alignment_mode",
                        (PPS_EPOCH_OFFSET_SECONDS == 0)
                            ? "decoded_epoch_to_latest_pps"
                            : "decoded_epoch_minus_offset_to_pps");
  appendJsonI64Field(json, "last_rejected_delta_us", lastRejectedDeltaUs);
  appendJsonStringField(json, "last_rejected_reason", lastRejectedReason);
  appendJsonStringField(json, "ota_status", otaInProgress ? "In Progress" : "Ready");
  appendJsonU32Field(json, "ota_safe_remaining_ms", otaSafeRemaining);
  appendJsonU32Field(json, "last_accepted_pps_seq", lastAcceptedPpsSeq);
  appendJsonU32Field(json, "last_rejected_pps_seq", lastRejectedPpsSeq);
  appendJsonStringField(json, "last_candidate_source", lastCandidateSource);
  appendJsonStringField(json, "last_candidate_reason", lastCandidateReason);
  appendJsonU32Field(json, "last_candidate_decoded_epoch", lastCandidateDecodedEpoch);
  appendJsonU32Field(json, "last_candidate_assigned_epoch", lastCandidateAssignedEpoch);
  appendJsonU32Field(json, "last_candidate_pps_seq", lastCandidatePpsSeq);
  appendJsonI64Field(json, "last_candidate_pps_mono_us", lastCandidatePpsMonoUs);
  appendJsonI64Field(json, "last_candidate_gps_arrival_us", lastCandidateGpsArrivalUs);
  appendJsonI32Field(json, "last_candidate_gap_ms", lastCandidateGapMs);
  appendJsonI64Field(json, "last_candidate_delta_us", lastCandidateDeltaUs);
  appendJsonU32Field(json, "last_candidate_age_ms", lastCandidateAge);
  appendJsonBoolField(json, "last_candidate_stale", lastCandidateStale);
  appendJsonI64Field(json, "last_good_pps_delta_us", lastGoodPpsDeltaUs);
  appendJsonI32Field(json, "last_pairing_gap_ms", lastPairingGapMs);
  appendJsonStringField(json, "last_pairing_accept_reason", lastPairingAcceptReason);
  appendJsonBoolField(json, "last_candidate_pps_consumed", lastCandidatePpsAlreadyConsumed);
  appendJsonBoolField(json, "last_candidate_utc_consumed", lastCandidateUtcAlreadyConsumed);
  appendJsonU32Field(json, "log_seq", logSequence);
  json += "\"logs\":[";
  for (uint8_t i = 0; i < logCount; i++) {
    uint8_t idx = (logHead + LOG_LINES - logCount + i) % LOG_LINES;
    if (i) json += ',';
    json += '"';
    jsonEscapeAppend(json, logRing[idx]);
    json += '"';
  }
  json += "]}";

  server.send(200, "application/json", json);
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ----------------------------- NTP Server -----------------------------

static void writeU32(uint8_t *buf, uint8_t offset, uint32_t value) {
  buf[offset] = (uint8_t)(value >> 24);
  buf[offset + 1] = (uint8_t)(value >> 16);
  buf[offset + 2] = (uint8_t)(value >> 8);
  buf[offset + 3] = (uint8_t)value;
}

static void writeNtpTimestamp(uint8_t *buf, uint8_t offset, int64_t unixUs) {
  const uint64_t NTP_UNIX_OFFSET = 2208988800ULL;
  uint64_t seconds = (uint64_t)(unixUs / 1000000LL) + NTP_UNIX_OFFSET;
  uint64_t micros = (uint64_t)(unixUs % 1000000LL);
  uint32_t fraction = (uint32_t)((micros << 32) / 1000000ULL);
  writeU32(buf, offset, (uint32_t)seconds);
  writeU32(buf, offset + 4, fraction);
}

static void serviceNtp() {
  int packetSize = ntpUdp.parsePacket();
  if (packetSize <= 0) return;

  uint8_t request[48];
  memset(request, 0, sizeof(request));
  int readLen = ntpUdp.read(request, sizeof(request));
  while (ntpUdp.available()) ntpUdp.read();

  IPAddress client = ntpUdp.remoteIP();
  uint16_t clientPort = ntpUdp.remotePort();
  ntpRequestCount++;
  lastNtpClient = client;
  lastNtpRequestMs = millis();

  if (readLen < 48) {
    addLog("Ignored short NTP packet from %s", client.toString().c_str());
    return;
  }

  if (!timeEverValid) {
    if (millis() - lastNtpRefuseLogMs > STATUS_LOG_RATE_MS) {
      lastNtpRefuseLogMs = millis();
      addLog("Refused NTP request from %s: time not valid", client.toString().c_str());
    }
    return;
  }

  int64_t receiveUnixUs = currentUnixUs();
  uint8_t reply[48];
  memset(reply, 0, sizeof(reply));

  uint8_t leap = 0;
  uint8_t version = (request[0] >> 3) & 0x07;
  if (version == 0) version = 4;
  reply[0] = (uint8_t)((leap << 6) | (version << 3) | 4);
  reply[1] = (syncState == SYNC_SYNCED) ? 1 : 2;
  reply[2] = request[2];
  reply[3] = 0xEC;  // roughly 2^-20 seconds precision

  writeU32(reply, 4, (syncState == SYNC_SYNCED) ? 0x00000001UL : 0x00010000UL);
  writeU32(reply, 8, (syncState == SYNC_SYNCED) ? 0x00000001UL : 0x00020000UL);

  if (syncState == SYNC_SYNCED) {
    reply[12] = 'G'; reply[13] = 'P'; reply[14] = 'P'; reply[15] = 'S';
  } else {
    reply[12] = 'H'; reply[13] = 'O'; reply[14] = 'L'; reply[15] = 'D';
  }

  writeNtpTimestamp(reply, 16, lastAcceptedUnixUs > 0 ? lastAcceptedUnixUs : receiveUnixUs);
  memcpy(reply + 24, request + 40, 8);
  writeNtpTimestamp(reply, 32, receiveUnixUs);
  int64_t transmitUnixUs = currentUnixUs();
  lastNtpTxUnixUs = transmitUnixUs;
  lastNtpTxMs = millis();
  writeNtpTimestamp(reply, 40, transmitUnixUs);

  ntpUdp.beginPacket(client, clientPort);
  ntpUdp.write(reply, sizeof(reply));
  ntpUdp.endPacket();

  if ((ntpRequestCount <= 5) || (ntpRequestCount % 100 == 0)) {
    addLog("Served NTP request #%llu to %s", (unsigned long long)ntpRequestCount,
           client.toString().c_str());
  }
}

// ----------------------------- Setup and Loop -----------------------------

void setup() {
  Serial.begin(115200);
  delay(50);
  addLog("Booting ESP32 GPS PPS NTP firmware");
  addLog("PPS epoch alignment: assigned PPS epoch = decoded NMEA epoch - (%+ld s)",
         (long)PPS_EPOCH_OFFSET_SECONDS);
  if (PPS_EPOCH_OFFSET_SECONDS == 0) {
    addLog("PPS epoch alignment mode: decoded UTC second is assigned to the latest PPS edge");
  }

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  addLog("GPS Serial2 started: RX GPIO%u TX GPIO%u at %lu baud",
         GPS_RX_PIN, GPS_TX_PIN, (unsigned long)GPS_BAUD);

  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsISR, RISING);
  addLog("PPS interrupt attached to GPIO%u", PPS_PIN);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    (void)info;
    switch (event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        addLog("Wi-Fi connected, IP %s", WiFi.localIP().toString().c_str());
        break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        addLog("Wi-Fi disconnected");
        if (gpsMaintenanceMode) {
          disableGpsMaintenanceMode("Wi-Fi disconnected");
        }
        servicesStarted = false;
        gpsBridgeClient.stop();
        gpsBridgeStarted = false;
        gpsBridgeClientWasConnected = false;
        break;
      default:
        break;
    }
  });

  ArduinoOTA.setHostname("esp32-gps-ntp");
  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    otaLastProgressPct = 255;
    addLog("OTA start");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total == 0) return;
    uint8_t pct = (uint8_t)((progress * 100U) / total);
    if (pct != otaLastProgressPct && (pct % 10 == 0 || pct == 100)) {
      otaLastProgressPct = pct;
      addLog("OTA progress %u%%", pct);
    }
  });
  ArduinoOTA.onEnd([]() {
    addLog("OTA complete");
    otaInProgress = false;
  });
  ArduinoOTA.onError([](ota_error_t error) {
    addLog("OTA error %u", (unsigned)error);
    otaInProgress = false;
  });

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/ota-safe", HTTP_GET, handleOtaSafe);
  server.on("/ota-safe", HTTP_POST, handleOtaSafe);
  server.on("/gps-maintenance", HTTP_GET, handleGpsMaintenance);
  server.on("/gps-maintenance", HTTP_POST, handleGpsMaintenance);
  server.on("/gps-maintenance-off", HTTP_GET, handleGpsMaintenanceOff);
  server.on("/gps-maintenance-off", HTTP_POST, handleGpsMaintenanceOff);
  server.on("/reboot", HTTP_GET, handleReboot);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.onNotFound(handleNotFound);

  serviceWifi();
}

void loop() {
  if (servicesStarted) ArduinoOTA.handle();

  servicePpsFlag();
  if (servicesStarted) ArduinoOTA.handle();

  serviceGps();
  if (servicesStarted) ArduinoOTA.handle();

  if (servicesStarted) {
    serviceGpsBridge();
    ArduinoOTA.handle();
  }

  serviceSyncState();
  serviceWifi();
  if (servicesStarted) ArduinoOTA.handle();

  if (servicesStarted) {
    if (!otaInProgress) {
      server.handleClient();
    }
    ArduinoOTA.handle();
    if (!otaSafeActive()) {
      serviceNtp();
    }
    ArduinoOTA.handle();
  }

  delay(1);
}
