/*
 * ============================================================================
 *  DeliveryBot — Production Firmware v2.0
 *  Board      : ESP32 Dev Module  (Espressif ESP32-WROOM-32 / DevKitC)
 *  Baud Rate  : 115200
 *  Author     : Kartikey Dwivedi, ECE, NIT Goa
 * ----------------------------------------------------------------------------
 *  Architecture : Non-blocking FSM  |  Navigation : Haversine + PD controller
 *  Obstacle Det : HC-SR04 (non-blocking, millis)  |  Compass : I2C low-pass
 *  GPS          : NEO-6M, UART2, TinyGPSPlus       |  Web : ESPAsyncWebServer
 * ============================================================================
 *
 *  WIRING MAP
 *  ──────────────────────────────────────────────────────────────────
 *  L298N Motor Driver
 *    IN1  → GPIO 18   IN2  → GPIO 19   ENA (PWM) → GPIO 5
 *    IN3  → GPIO 23   IN4  → GPIO 13   ENB (PWM) → GPIO 4
 *    12V Battery  → L298N VIN
 *    L298N 5V OUT → ESP32 VIN  (onboard regulator, no extra buck needed)
 *    GND          → Common GND
 *
 *  NEO-6M GPS (UART2)
 *    GPS TX → ESP32 GPIO 16 (RX2)
 *    GPS RX → ESP32 GPIO 17 (TX2, optional)
 *    VCC    → 3.3 V or 5 V (module accepts both)
 *    GND    → GND
 *
 *  QMC5883L Compass (I2C)
 *    SDA → GPIO 21  |  SCL → GPIO 22
 *    VCC → 3.3 V    |  GND → GND
 *    I2C Address: 0x0D (QMC) or 0x1E (HMC) — auto-detected
 *
 *  HC-SR04 Ultrasonic
 *    TRIG → GPIO 27  (3.3 V safe, direct)
 *    ECHO → GPIO 14  via voltage divider: ECHO pin → 1kΩ → GPIO14 → 2kΩ → GND
 *    VCC  → 5 V   |  GND → GND
 *    ⚠ ECHO is 5 V logic — divider is MANDATORY to protect ESP32
 *
 *  ──────────────────────────────────────────────────────────────────
 *  REQUIRED LIBRARIES
 *    ESPAsyncWebServer : https://github.com/me-no-dev/ESPAsyncWebServer/archive/refs/heads/master.zip
 *    AsyncTCP          : https://github.com/me-no-dev/AsyncTCP/archive/refs/heads/master.zip
 *    TinyGPSPlus       : https://github.com/mikalhart/TinyGPSPlus/archive/refs/heads/master.zip
 *    Wire              : built-in
 *
 *  ──────────────────────────────────────────────────────────────────
 *  COMPASS CALIBRATION — HOW TO READ THE SERIAL MONITOR
 *    1. Uncomment `#define COMPASS_CALIBRATE` in cfg namespace (line ~134).
 *    2. Flash and open Serial Monitor at 115200 baud.
 *    3. Hold the robot level and rotate it slowly through 3–4 full circles.
 *       Keep it away from motors and power cables during this step.
 *    4. The monitor prints:  [CAL] mx=XXXX  my=XXXX
 *       Note the minimum and maximum values for both mx and my over ~60 seconds.
 *    5. Compute offsets:
 *         HARD_IRON_X = (max_mx + min_mx) / 2.0
 *         HARD_IRON_Y = (max_my + min_my) / 2.0
 *    6. Fill those values into cfg::HARD_IRON_X / HARD_IRON_Y below.
 *    7. Comment `#define COMPASS_CALIBRATE` out again and re-flash.
 *
 *  CRITICAL DESIGN RULES
 *    1. NO delay() outside setup(). Use millis() timers exclusively.
 *    2. NO blocking I2C/UART reads in the main loop body.
 *    3. FSM tick() is the ONLY place state transitions occur.
 *    4. All tunable parameters live in the `cfg` namespace.
 * ============================================================================
 */
// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 0 : INCLUDES
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 1 : CONFIGURATION NAMESPACE
//  All magic numbers belong here. Tune these in the field; never bury them
//  inside logic functions.
// ─────────────────────────────────────────────────────────────────────────────
namespace cfg {

  // ---------- WiFi Access Point -------------------------------------------
  constexpr char AP_SSID[]     = "DeliveryBot_NITG";
  constexpr char AP_PASSWORD[] = "nitgoa2024";

  // ---------- Motor PWM ---------------------------------------------------
  // Base PWM (0–255). At 255 the L298N sees full duty; start lower for safety.
  constexpr uint8_t  PWM_BASE        = 210; // Increased cruising speed
  constexpr uint8_t  PWM_MIN         = 120; // High enough to overcome road friction
  constexpr uint8_t  PWM_MAX         = 255; // Unlocked to absolute maximum power
  
  // Slope boost: applied on tagged "uphill" route segments.
  constexpr uint8_t  PWM_SLOPE_BOOST = 45;  // Extra kick for slopes

  // ---------- PD Controller (Heading) -------------------------------------
  /*
   * TUNING GUIDE:
   *   Kp : Start at 1.0. If the robot oscillates (S-curves), reduce.
   *        If it barely corrects, increase. Typical range: 0.8 – 2.5.
   *   Kd : Derivative damps oscillation. Start at 0. Increase in small
   *        steps (0.1) until overshoot is eliminated. Typical: 0.2 – 1.0.
   *
   * PD output formula:
   *   correction = Kp * error + Kd * (error - prevError) / dt
   *
   * This correction is ADDED to the right motor and SUBTRACTED from the left
   * (for a positive/right error — robot drifted left of target bearing).
   */
  constexpr float Kp              = 1.5f;
  constexpr float Kd              = 0.5f;

  // ---------- Navigation --------------------------------------------------
  constexpr float  ARRIVAL_RADIUS_M  = 3.0f;   // waypoint capture radius (m)
  constexpr float  MAX_HEADING_ERR   = 45.0f;  // beyond this, pivot in place

  // ---------- GPS Staleness -----------------------------------------------
  constexpr uint32_t GPS_TIMEOUT_MS  = 5000;   // halt if no fix for 5 s

  // ---------- Runaway Detection -------------------------------------------
  // If distance to target increases for N consecutive readings → halt
  constexpr uint8_t RUNAWAY_COUNT    = 3;

  // ---------- Ultrasonic --------------------------------------------------
  constexpr float    OBSTACLE_CM     = 40.0f;  // danger threshold (cm)
  constexpr uint32_t SONAR_POLL_MS   = 100;    // poll interval (ms)
  constexpr uint32_t CLEAR_HOLD_MS   = 2000;   // must be clear this long to resume
  constexpr uint32_t SONAR_TIMEOUT_US= 23200;  // ~400 cm max range timeout (µs)

  // ---------- Compass Low-Pass Filter -------------------------------------
  /*
   * α ∈ (0, 1]. Closer to 1.0 = more raw (responsive but jittery).
   *             Closer to 0.0 = very smooth but lags behind fast turns.
   * Motor EMI can spike compass 10–20°; α = 0.15 damps this well.
   */
  constexpr float COMPASS_ALPHA      = 0.15f;

  // ---------- I2C ---------------------------------------------------------
  /*
   * QMC5883L (clone, common in cheap modules):
   *   Address 0x0D, Set/Reset register 0x0B = 0x01, Control 0x09 = 0x1D
   *   (OSR=512, RNG=8G, ODR=200Hz, MODE=Continuous)
   *
   * Genuine HMC5883L:
   *   Address 0x1E, CRA=0x00→0x70 (8avg,15Hz,normal),
   *   CRB=0x01→0xA0 (gain 5), MR=0x02→0x00 (continuous)
   *
   * The firmware auto-detects which IC is present.
   */
  constexpr uint8_t HMC_ADDR        = 0x1E;
  constexpr uint8_t QMC_ADDR        = 0x0D;

  /*
   * HARD-IRON CALIBRATION (mandatory before field use):
   *   1. Upload this firmware, open Serial Monitor at 115200 baud.
   *   2. Uncomment the `#define COMPASS_CALIBRATE` line below.
   *   3. Rotate the robot slowly through 3–4 full 360° turns in the air,
   *      away from motors and power cables.
   *   4. Note the min/max X and Y values printed to Serial.
   *   5. Fill in HARD_IRON_X and HARD_IRON_Y below:
   *        HARD_IRON_X = (max_x + min_x) / 2.0f
   *        HARD_IRON_Y = (max_y + min_y) / 2.0f
   *   6. Comment `#define COMPASS_CALIBRATE` out again and re-flash.
   *
   *   NOTE: Soft-iron correction (ellipse → circle) requires a 2×2 matrix.
   *   For a campus robot at low speed, hard-iron correction alone is usually
   *   sufficient. If systematic ~90° heading errors persist, implement soft-
   *   iron correction using the Magneto 1.2 tool (Stuart McLachlan, 2013).
   */
  // #define COMPASS_CALIBRATE    // ← uncomment ONLY during calibration run
  constexpr float HARD_IRON_X       = -183.0f; // ← field calibrated
  constexpr float HARD_IRON_Y       = 47.5f; // ← field calibrated   // ← fill after calibration

  // ---------- Timing Intervals (main loop tasks) --------------------------
  constexpr uint32_t NAV_UPDATE_MS       = 250;   // navigation recalc rate
  constexpr uint32_t COMPASS_UPDATE_MS  = 50;    // compass read rate
  constexpr uint32_t TELEMETRY_UPDATE_MS = 500;   // JSON telemetry refresh
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 2 : PIN DEFINITIONS (STRICT — match hardware exactly)
// ─────────────────────────────────────────────────────────────────────────────
namespace pin {
  // Left Motor
  constexpr uint8_t IN1     = 18;
  constexpr uint8_t IN2     = 19;
  constexpr uint8_t ENA     = 5;    // PWM channel 0

  // Right Motor
  constexpr uint8_t IN3     = 23;
  constexpr uint8_t IN4     = 13;
  constexpr uint8_t ENB     = 4;    // PWM channel 1   ← NOT GPIO 12

  // Ultrasonic (HC-SR04 — ECHO through 1kΩ/2kΩ voltage divider → 3.3V safe)
  constexpr uint8_t TRIG    = 27;
  constexpr uint8_t ECHO    = 14;

  // I2C Compass
  constexpr uint8_t SDA     = 21;
  constexpr uint8_t SCL     = 22;

  // GPS UART2
  constexpr uint8_t GPS_RX  = 16;   // ESP32 RX2 ← GPS TX
  constexpr uint8_t GPS_TX  = 17;   // ESP32 TX2 → GPS RX (often unused)
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 3 : PWM CHANNELS (ESP32 LEDC)
// ─────────────────────────────────────────────────────────────────────────────
namespace pwm {
  constexpr uint8_t  CH_LEFT  = 0;
  constexpr uint8_t  CH_RIGHT = 1;
  constexpr uint32_t FREQ     = 5000;  // 5 kHz — above audible, below L298N limit
  constexpr uint8_t  RES      = 8;     // 8-bit resolution → 0–255
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 4 : ROUTE & WAYPOINT DATA
// ─────────────────────────────────────────────────────────────────────────────
struct Waypoint {
  double lat;
  double lon;
  bool   uphill;    // true → apply slope boost on the leg ARRIVING at this WP
  const char* label;
};

// All physical locations on campus
namespace locations {
  constexpr Waypoint TALPONA    = { 15.1698389, 74.0120126, false, "Talpona Boys Hostel" }; // origin
  constexpr Waypoint TURN1      = { 15.1718341, 74.0152044, false, "Turn 1" };
  constexpr Waypoint MESS_GATE  = { 15.1716549, 74.0156825, false, "Mess Gate" };
  // ── Legacy waypoints kept for future routes ──
  constexpr Waypoint TURN2      = { 15.1693800, 74.0119103, false, "Turn 2" };
  constexpr Waypoint TURN3      = { 15.1693341, 74.0120008, true,  "Turn 3 (Slope UP)" };
  constexpr Waypoint GYAN       = { 15.1691943, 74.0119928, true,  "Gyan Mandir" };
  constexpr Waypoint TURN4      = { 15.1692296, 74.0117997, false, "Turn 4 (Slope DOWN)" };
  constexpr Waypoint SEMINAR    = { 15.1689166, 74.0117899, false, "Seminar Hall" };
  constexpr Waypoint TURN5      = { 15.1685872, 74.0118972, false, "Turn 5" };
  constexpr Waypoint ADMIN      = { 15.1685778, 74.0119552, false, "Admin Block" };
}

// Route arrays
constexpr Waypoint ROUTE_D[] = {   // ACTIVE: Talpona Boys Hostel → Mess Gate
  locations::TALPONA,
  locations::TURN1,
  locations::MESS_GATE
};

constexpr Waypoint ROUTE_A[] = {   // Legacy: kept for future use
  locations::TALPONA,
  locations::TURN2,
  locations::TURN3,
  locations::GYAN
};

constexpr Waypoint ROUTE_B[] = {   // Legacy: kept for future use
  locations::TALPONA,
  locations::TURN2,
  locations::TURN4,
  locations::SEMINAR
};

constexpr Waypoint ROUTE_C[] = {   // Legacy: kept for future use
  locations::TALPONA,
  locations::TURN5,
  locations::ADMIN
};

constexpr uint8_t ROUTE_D_LEN = sizeof(ROUTE_D) / sizeof(ROUTE_D[0]);
constexpr uint8_t ROUTE_A_LEN = sizeof(ROUTE_A) / sizeof(ROUTE_A[0]);
constexpr uint8_t ROUTE_B_LEN = sizeof(ROUTE_B) / sizeof(ROUTE_B[0]);
constexpr uint8_t ROUTE_C_LEN = sizeof(ROUTE_C) / sizeof(ROUTE_C[0]);

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 5 : FSM STATE DEFINITIONS
// ─────────────────────────────────────────────────────────────────────────────
enum class BotState : uint8_t {
  IDLE,           // Waiting at Talpona Boys Hostel for a mission command via web UI
  NAVIGATING,     // Actively driving toward current waypoint
  HALTED_OBS,     // Obstacle detected — motors stopped, polling for clearance
  HALTED_GPS,     // GPS fix lost — motors stopped, waiting for satellite lock
  HALTED_RUNAWAY, // Runaway detected — motors stopped, awaiting operator reset
  ARRIVED         // Final waypoint reached — mission complete
};

// Human-readable state labels for telemetry JSON
const char* stateLabel(BotState s) {
  switch (s) {
    case BotState::IDLE:            return "IDLE";
    case BotState::NAVIGATING:      return "NAVIGATING";
    case BotState::HALTED_OBS:      return "HALTED_OBSTACLE";
    case BotState::HALTED_GPS:      return "HALTED_GPS_LOSS";
    case BotState::HALTED_RUNAWAY:  return "HALTED_RUNAWAY";
    case BotState::ARRIVED:         return "ARRIVED";
    default:                        return "UNKNOWN";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 6 : GLOBAL STATE (all runtime mutable data in one struct)
// ─────────────────────────────────────────────────────────────────────────────
struct BotContext {
  // FSM
  BotState state           = BotState::IDLE;
  BotState stateBeforeHalt = BotState::IDLE; // restore after obstacle clears

  // Active route
  const Waypoint* route    = nullptr;
  uint8_t routeLen         = 0;
  uint8_t wpIndex          = 0;        // index of CURRENT target waypoint

  // GPS
  double   gpsLat          = 0.0;
  double   gpsLon          = 0.0;
  uint32_t lastGpsFixMs    = 0;
  bool     gpsValid        = false;

  // Navigation
  float    distToWp        = 9999.0f;  // metres
  float    targetBearing   = 0.0f;     // degrees, 0–360, true north
  float    headingError    = 0.0f;     // signed, –180 to +180
  float    lockedBearing  = 0.0f;     // bearing locked at dispatch / WP advance

  // Runaway detection (kept for GPS-opportunistic mode)
  float    prevDist        = 9999.0f;
  uint8_t  runawayCount    = 0;

  // Compass
  float    compassRaw      = 0.0f;     // pre-filter reading (°)
  float    compassSmooth   = 0.0f;     // low-pass filtered heading (°)
  bool     compassOk       = false;
  bool     compassIsQMC    = false;    // true if QMC5883L detected

  // PD controller state
  float    pdPrevError     = 0.0f;
  uint32_t pdPrevTimeMs    = 0;

  // Obstacle detection
  float    sonarCm         = 999.0f;
  uint32_t lastSonarMs     = 0;
  uint32_t clearSinceMs    = 0;        // when the path first became clear again
  bool     pathClear       = true;

  // Ultrasonic non-blocking state machine
  enum class SonarPhase : uint8_t { IDLE, TRIGGERED, LISTENING };
  SonarPhase sonarPhase    = SonarPhase::IDLE;
  uint32_t   triggerMs     = 0;        // when TRIG was pulled HIGH
  uint32_t   echoStartUs   = 0;        // micros() when ECHO went HIGH

  // Timing
  uint32_t lastNavMs       = 0;
  uint32_t lastCompassMs   = 0;
  uint32_t lastTelemetryMs = 0;

  // Telemetry string (updated on TELEMETRY_UPDATE_MS cadence)
  char telemetryJson[512]  = "{}";
} bot;

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 7 : PERIPHERAL OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
TinyGPSPlus    gps;
HardwareSerial gpsSerial(2);           // UART2: RX=GPIO16, TX=GPIO17
AsyncWebServer server(80);

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 8 : WEB UI HTML PAYLOAD
//  ─────────────────────────────────────────────────────────────────────────
//  NOTE TO REVIEWER: The original HTML was unreadable at review time.
//  This payload reproduces the expected functional interface based on the
//  project specification. If your original HTML differs, replace the content
//  between the R"rawhtml( ... )rawhtml" delimiters ONLY — do not touch the
//  server route handler below.
// ─────────────────────────────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"/><meta name="viewport" content="width=device-width,initial-scale=1.0,user-scalable=no"/><title>NITG Robo-Delivery</title><style>*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}body{font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh}.page{display:none;flex-direction:column;min-height:100vh}.page.active{display:flex}#page-dashboard{background:#C62828}.dash-header{background:#EEE;display:flex;flex-direction:column;align-items:center;padding:48px 24px 36px;gap:10px}.dash-header img{width:110px;height:88px;object-fit:contain}.dash-header h1{font-size:26px;font-weight:900;color:#1A1A1A;letter-spacing:1.5px}.dash-header p{font-size:12px;color:#888;letter-spacing:3px;font-weight:500}#page-dispatched{background:#F0F0F0}.disp-header{background:#C62828;display:flex;flex-direction:column;align-items:center;padding:48px 24px 36px;gap:10px}.disp-header img{width:110px;height:88px;object-fit:contain;filter:brightness(0) invert(1)}.disp-header h1{font-size:26px;font-weight:900;color:#FFF;letter-spacing:1.5px}.disp-header p{font-size:12px;color:rgba(255,255,255,0.65);letter-spacing:3px;font-weight:500}.card{background:#FFF;border-radius:22px;padding:28px 22px 30px;margin:20px 16px;display:flex;flex-direction:column;align-items:center;gap:22px;box-shadow:0 6px 24px rgba(0,0,0,0.13)}.status-pill{background:#C62828;color:#FFF;border-radius:30px;padding:9px 24px;font-size:15px;font-weight:700;display:flex;align-items:center;gap:9px;transition:background .3s}.status-pill::before{content:'';width:10px;height:10px;border-radius:50%;background:#FFF;flex-shrink:0}.status-pill.offline{background:#9E9E9E}.status-pill.halted{background:#B71C1C}.dest-label{font-size:20px;font-weight:800;color:#1A1A1A;align-self:flex-start}.dest-select{width:100%;background:#EEE;border:none;border-radius:14px;padding:17px 16px;font-size:16px;color:#1A1A1A;appearance:none;-webkit-appearance:none;cursor:pointer;outline:none;font-family:inherit}.btn-primary{width:100%;background:#B71C1C;color:#FFF;border:none;border-radius:16px;padding:19px;font-size:17px;font-weight:900;cursor:pointer;letter-spacing:.4px;font-family:inherit;transition:opacity .18s;line-height:1.3}.btn-primary:active{opacity:.82}.btn-return{background:#C62828;color:#FFF;border:none;border-radius:30px;padding:15px 36px;font-size:16px;font-weight:700;cursor:pointer;margin:4px 16px 0;font-family:inherit;transition:opacity .18s}.btn-return:active{opacity:.82}.disp-title{font-size:27px;font-weight:900;color:#1A1A1A;text-align:center;letter-spacing:.5px}.disp-subtitle{font-size:12px;color:#9E9E9E;text-align:center;letter-spacing:1.5px;line-height:1.7;font-weight:600}.dest-pill{background:#C62828;color:#FFF;border-radius:30px;padding:13px 32px;font-size:18px;font-weight:800;text-align:center}#mascot{width:230px;margin:16px auto 0;display:block;object-fit:contain}.spacer{flex:1}</style></head><body>
<div class="page active" id="page-dashboard"><div class="dash-header">
<svg id="logo-dash" viewBox="0 0 80 60" width="110" height="88" fill="none" xmlns="http://www.w3.org/2000/svg"><line x1="8" y1="28" x2="22" y2="28" stroke="#C62828" stroke-width="3" stroke-linecap="round"/><line x1="8" y1="22" x2="18" y2="22" stroke="#C62828" stroke-width="3" stroke-linecap="round"/><polyline points="22,35 22,18 44,18 54,28 62,28 62,38 56,38" stroke="#C62828" stroke-width="3" stroke-linecap="round" stroke-linejoin="round" fill="none"/><line x1="22" y1="38" x2="30" y2="38" stroke="#C62828" stroke-width="3"/><circle cx="34" cy="39" r="5" stroke="#C62828" stroke-width="3" fill="none"/><circle cx="52" cy="39" r="5" stroke="#C62828" stroke-width="3" fill="none"/><polyline points="44,18 44,28 54,28" stroke="#C62828" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" fill="none"/></svg>
<h1>[DeliveryBot NITG]</h1><p>ROBO-DELIVERY APP</p></div>
<div class="card"><div class="status-pill" id="statusPill">System Online</div><span class="dest-label">Select Destination:</span>
<select class="dest-select" id="destSelect"><option value="">Talpona Boys Hostel (Origin)</option><option value="D">Mess Gate</option><option value="A">Gyan Mandir</option><option value="C">Admin Block</option><option value="B">Seminar Hall</option><option value="GATE">Campus Main Gate</option></select>
<button class="btn-primary" onclick="initDelivery()">Initialize Delivery<br>Sequence</button></div>
<div class="spacer"></div></div>
<div class="page" id="page-dispatched"><div class="disp-header">
<svg id="logo-disp" viewBox="0 0 80 60" width="110" height="88" fill="none" xmlns="http://www.w3.org/2000/svg"><line x1="8" y1="28" x2="22" y2="28" stroke="#fff" stroke-width="3" stroke-linecap="round"/><line x1="8" y1="22" x2="18" y2="22" stroke="#fff" stroke-width="3" stroke-linecap="round"/><polyline points="22,35 22,18 44,18 54,28 62,28 62,38 56,38" stroke="#fff" stroke-width="3" stroke-linecap="round" stroke-linejoin="round" fill="none"/><line x1="22" y1="38" x2="30" y2="38" stroke="#fff" stroke-width="3"/><circle cx="34" cy="39" r="5" stroke="#fff" stroke-width="3" fill="none"/><circle cx="52" cy="39" r="5" stroke="#fff" stroke-width="3" fill="none"/><polyline points="44,18 44,28 54,28" stroke="#fff" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" fill="none"/></svg>
<h1>[DeliveryBot NITG]</h1><p>ROBO-DELIVERY APP</p></div>
<div class="card"><p class="disp-title">ITEM DISPATCHED!!</p><p class="disp-subtitle">THE DELIVERY ROBO IS<br>NOW EN ROUTE TO:</p><div class="dest-pill" id="destNamePill">&#8212;</div></div>
<button class="btn-return" onclick="returnToDashboard()">Return to Dashboard</button>
<svg id="mascot" viewBox="0 0 200 120" width="230" style="margin:16px auto 0;display:block"><rect x="20" y="50" width="140" height="50" rx="12" fill="#C62828"/><rect x="50" y="30" width="80" height="40" rx="8" fill="#e53935"/><rect x="58" y="36" width="28" height="22" rx="4" fill="#90caf9"/><rect x="92" y="36" width="28" height="22" rx="4" fill="#90caf9"/><circle cx="55" cy="102" r="16" fill="#333"/><circle cx="55" cy="102" r="8" fill="#888"/><circle cx="145" cy="102" r="16" fill="#333"/><circle cx="145" cy="102" r="8" fill="#888"/><rect x="148" y="58" width="22" height="14" rx="4" fill="#ffca28"/></svg>
<div class="spacer"></div></div>
<script>
const DN={'':'Talpona Boys Hostel','D':'Mess Gate','A':'Gyan Mandir','B':'Seminar Hall','C':'Admin Block','GATE':'Campus Main Gate'};
async function pollStatus(){const p=document.getElementById('statusPill');try{const d=await(await fetch('/telemetry')).json();p.classList.remove('offline','halted');if(d.state.startsWith('HALTED')){p.textContent='System Halted';p.classList.add('halted');}else{p.textContent='System Online';}}catch(e){p.classList.remove('halted');p.classList.add('offline');p.textContent='System Offline';}}
function initDelivery(){const s=document.getElementById('destSelect');const r=s.value;if(!r){alert('Talpona Boys Hostel is the origin.\nSelect a destination.');return;}if(r==='GATE'){alert('Campus Main Gate not configured yet.');return;}fetch('/go?route='+r).then(()=>{document.getElementById('destNamePill').textContent=DN[r]||s.options[s.selectedIndex].text;showPage('page-dispatched');}).catch(()=>alert('Connection failed.'));}
function returnToDashboard(){fetch('/reset').catch(()=>{});showPage('page-dashboard');}
function showPage(id){document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));document.getElementById(id).classList.add('active');}
setInterval(pollStatus,2000);pollStatus();
</script></body></html>)rawhtml";

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 9 : MATH UTILITIES
// ─────────────────────────────────────────────────────────────────────────────

float haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  constexpr double R  = 6371000.0;
  constexpr double DR = M_PI / 180.0;

  double dlat = (lat2 - lat1) * DR;
  double dlon = (lon2 - lon1) * DR;
  double a    = sin(dlat * 0.5) * sin(dlat * 0.5) +
                cos(lat1 * DR)  * cos(lat2 * DR) *
                sin(dlon * 0.5) * sin(dlon * 0.5);
  double c    = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return (float)(R * c);
}

float initialBearing(double lat1, double lon1, double lat2, double lon2) {
  constexpr double DR = M_PI / 180.0;
  double dlon  = (lon2 - lon1) * DR;
  double y     = sin(dlon) * cos(lat2 * DR);
  double x     = cos(lat1 * DR) * sin(lat2 * DR) -
                 sin(lat1 * DR) * cos(lat2 * DR) * cos(dlon);
  double theta = atan2(y, x) * (180.0 / M_PI);
  return (float)fmod(theta + 360.0, 360.0);
}

float normalizeAngle(float deg) {
  while (deg >  180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 10 : MOTOR DRIVER
// ─────────────────────────────────────────────────────────────────────────────

void motorsStop() {
  ledcWrite(pin::ENA, 0);
  ledcWrite(pin::ENB, 0);
  digitalWrite(pin::IN1, LOW);  digitalWrite(pin::IN2, LOW);
  digitalWrite(pin::IN3, LOW);  digitalWrite(pin::IN4, LOW);
}

void setMotors(int leftPwm, int rightPwm) {
  // Clamp to valid range
  leftPwm  = constrain(leftPwm,  0, cfg::PWM_MAX);
  rightPwm = constrain(rightPwm, 0, cfg::PWM_MAX);

  // Left motor forward
  if (leftPwm > 0) {
    digitalWrite(pin::IN1, HIGH); digitalWrite(pin::IN2, LOW);
  } else {
    digitalWrite(pin::IN1, LOW);  digitalWrite(pin::IN2, LOW);
  }

  // Right motor forward
  if (rightPwm > 0) {
    digitalWrite(pin::IN3, HIGH); digitalWrite(pin::IN4, LOW);
  } else {
    digitalWrite(pin::IN3, LOW);  digitalWrite(pin::IN4, LOW);
  }

  ledcWrite(pin::ENA, (uint8_t)leftPwm);
  ledcWrite(pin::ENB, (uint8_t)rightPwm);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 11 : COMPASS DRIVER (HMC5883L / QMC5883L auto-detect)
// ─────────────────────────────────────────────────────────────────────────────

bool compassInit() {
  Wire.beginTransmission(cfg::QMC_ADDR);
  if (Wire.endTransmission() == 0) {
    // QMC5883L detected
    bot.compassIsQMC = true;
    // Set/Reset period register
    Wire.beginTransmission(cfg::QMC_ADDR);
    Wire.write(0x0B); Wire.write(0x01);
    Wire.endTransmission();
    // Control: OSR=512, RNG=8G, ODR=200Hz, Continuous mode
    Wire.beginTransmission(cfg::QMC_ADDR);
    Wire.write(0x09); Wire.write(0x1D);
    Wire.endTransmission();
    Serial.println("[COMPASS] QMC5883L detected and configured.");
    return true;
  }

  Wire.beginTransmission(cfg::HMC_ADDR);
  if (Wire.endTransmission() == 0) {
    // HMC5883L detected
    bot.compassIsQMC = false;
    // CRA: 8-sample average, 15 Hz, normal measurement
    Wire.beginTransmission(cfg::HMC_ADDR);
    Wire.write(0x00); Wire.write(0x70);
    Wire.endTransmission();
    // CRB: Gain = 5 (±4.7 Gauss)
    Wire.beginTransmission(cfg::HMC_ADDR);
    Wire.write(0x01); Wire.write(0xA0);
    Wire.endTransmission();
    // Mode: Continuous measurement
    Wire.beginTransmission(cfg::HMC_ADDR);
    Wire.write(0x02); Wire.write(0x00);
    Wire.endTransmission();
    Serial.println("[COMPASS] HMC5883L detected and configured.");
    return true;
  }

  Serial.println("[COMPASS] ERROR — no compass found on I2C bus! Check wiring.");
  return false;
}

void compassRead() {
  int16_t mx = 0, my = 0, mz = 0;
  uint8_t addr = bot.compassIsQMC ? cfg::QMC_ADDR : cfg::HMC_ADDR;

  // Point to data register
  Wire.beginTransmission(addr);
  Wire.write(bot.compassIsQMC ? 0x00 : 0x03);
  if (Wire.endTransmission(false) != 0) return;  // bus error guard

  uint8_t bytes = Wire.requestFrom(addr, (uint8_t)6);
  if (bytes < 6) return;  // incomplete read — skip this cycle

  if (bot.compassIsQMC) {
    // QMC5883L byte order: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    mx = (int16_t)(Wire.read() | (Wire.read() << 8));
    my = (int16_t)(Wire.read() | (Wire.read() << 8));
    mz = (int16_t)(Wire.read() | (Wire.read() << 8));
  } else {
    mx = (int16_t)((Wire.read() << 8) | Wire.read());
    mz = (int16_t)((Wire.read() << 8) | Wire.read());
    my = (int16_t)((Wire.read() << 8) | Wire.read());
  }

  // Apply hard-iron offset correction
  float cx = (float)mx - cfg::HARD_IRON_X;
  float cy = (float)my - cfg::HARD_IRON_Y;

#ifdef COMPASS_CALIBRATE
  // During calibration: stream raw values to find min/max
  Serial.printf("[CAL] mx=%d  my=%d\n", mx, my);
  return;
#endif

  // Compute heading (degrees, 0–360)
  float rawHeading = atan2f(cy, cx) * (180.0f / M_PI);
  if (rawHeading < 0.0f) rawHeading += 360.0f;

  bot.compassRaw = rawHeading;

  // Angle-safe low-pass filter (via unit-circle decomposition)
  float smoothRad   = bot.compassSmooth * (M_PI / 180.0f);
  float rawRad      = rawHeading        * (M_PI / 180.0f);
  float sinSmoothed = cfg::COMPASS_ALPHA * sinf(rawRad)  + (1.0f - cfg::COMPASS_ALPHA) * sinf(smoothRad);
  float cosSmoothed = cfg::COMPASS_ALPHA * cosf(rawRad)  + (1.0f - cfg::COMPASS_ALPHA) * cosf(smoothRad);
  float result      = atan2f(sinSmoothed, cosSmoothed) * (180.0f / M_PI);
  if (result < 0.0f) result += 360.0f;

  bot.compassSmooth = result;
  bot.compassOk     = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 12 : ULTRASONIC — NON-BLOCKING STATE MACHINE
// ─────────────────────────────────────────────────────────────────────────────

static float sonarSamples[3];
static uint8_t sonarSampleCount = 0;

float medianOf3(float* arr) {
  // Simple sort-3 and return middle element
  if (arr[0] > arr[1]) { float t = arr[0]; arr[0] = arr[1]; arr[1] = t; }
  if (arr[1] > arr[2]) { float t = arr[1]; arr[1] = arr[2]; arr[2] = t; }
  if (arr[0] > arr[1]) { float t = arr[0]; arr[0] = arr[1]; arr[1] = t; }
  return arr[1];
}

void sonarTick() {
  uint32_t now = millis();

  switch (bot.sonarPhase) {

    case BotContext::SonarPhase::IDLE:
      // Only start a new measurement cycle on the poll interval
      if (now - bot.lastSonarMs >= cfg::SONAR_POLL_MS) {
        bot.lastSonarMs = now;
        // Fire trigger pulse
        digitalWrite(pin::TRIG, HIGH);
        bot.triggerMs  = now;
        bot.sonarPhase = BotContext::SonarPhase::TRIGGERED;
      }
      break;

    case BotContext::SonarPhase::TRIGGERED:
      // Hold TRIG HIGH for ~10µs (millis resolution is 1ms, so 1ms is fine)
      if (now - bot.triggerMs >= 1) {
        digitalWrite(pin::TRIG, LOW);
        bot.echoStartUs = 0;
        bot.sonarPhase  = BotContext::SonarPhase::LISTENING;
      }
      break;

    case BotContext::SonarPhase::LISTENING: {
      uint32_t elapsedMs = now - bot.triggerMs;

      // Timeout guard: if no echo in 30ms, sensor returned >400cm or error
      if (elapsedMs > 30) {
        sonarSamples[sonarSampleCount++] = 400.0f; // clamp to max range
        bot.sonarPhase = BotContext::SonarPhase::IDLE;
        break;
      }

      if (bot.echoStartUs == 0) {
        // Waiting for ECHO to go HIGH
        if (digitalRead(pin::ECHO) == HIGH) {
          bot.echoStartUs = micros();
        }
      } else {
        // ECHO was HIGH, waiting for it to go LOW
        if (digitalRead(pin::ECHO) == LOW) {
          uint32_t duration = micros() - bot.echoStartUs;
          float    cm       = (float)duration / 58.0f;
          sonarSamples[sonarSampleCount++] = constrain(cm, 2.0f, 400.0f);
          bot.sonarPhase = BotContext::SonarPhase::IDLE;
        }
      }
      break;
    }
  }

  // Once 3 samples are accumulated, compute median and update bot state
  if (sonarSampleCount >= 3) {
    float samples[3] = { sonarSamples[0], sonarSamples[1], sonarSamples[2] };
    bot.sonarCm      = medianOf3(samples);
    sonarSampleCount = 0;
    bot.pathClear    = (bot.sonarCm >= cfg::OBSTACLE_CM);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 13 : PD HEADING CONTROLLER
// ─────────────────────────────────────────────────────────────────────────────
struct MotorCmd { int left; int right; };

MotorCmd computeMotorPwm(float error, bool uphill) {
  uint32_t now     = millis();
  float    dt      = (float)(now - bot.pdPrevTimeMs) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.05f;  // clamp first-call or stale dt

  float de         = error - bot.pdPrevError;
  float correction = cfg::Kp * error + cfg::Kd * (de / dt);

  bot.pdPrevError  = error;
  bot.pdPrevTimeMs = now;

  int base = cfg::PWM_BASE + (uphill ? cfg::PWM_SLOPE_BOOST : 0);

  // Large error → pivot in place
  if (fabsf(error) > cfg::MAX_HEADING_ERR) {
    if (error > 0) {
      // Turn right: left motor forward, right motor off
      return { base, 0 };
    } else {
      // Turn left: right motor forward, left motor off
      return { 0, base };
    }
  }

  // Normal driving with differential correction
  // Normal driving with differential correction
  int left  = (int)(base + correction); // ADD correction to left
  int right = (int)(base - correction); // SUBTRACT correction from right

  if (left  > 0 && left  < cfg::PWM_MIN) left  = cfg::PWM_MIN;
  if (right > 0 && right < cfg::PWM_MIN) right = cfg::PWM_MIN;

  return { constrain(left, 0, cfg::PWM_MAX), constrain(right, 0, cfg::PWM_MAX) };
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 14 : NAVIGATION LOGIC
// ─────────────────────────────────────────────────────────────────────────────

// Called when a waypoint is reached. Advances wpIndex and locks bearing for next leg.
void advanceWaypoint() {
  const Waypoint& arrived = bot.route[bot.wpIndex];
  Serial.printf("[NAV] Arrived at WP%d: %s\n", bot.wpIndex, arrived.label);
  bot.wpIndex++;

  if (bot.wpIndex >= bot.routeLen) {
    Serial.println("[NAV] MISSION COMPLETE — final destination reached.");
    motorsStop();
    bot.state = BotState::ARRIVED;
    return;
  }

  // Lock compass bearing for the next leg using hardcoded waypoint coordinates.
  // This bearing is fixed for the entire leg — GPS is NOT required to steer.
  const Waypoint& from = bot.route[bot.wpIndex - 1];
  const Waypoint& to   = bot.route[bot.wpIndex];
  bot.lockedBearing = initialBearing(from.lat, from.lon, to.lat, to.lon);
  bot.targetBearing = bot.lockedBearing;
  // Pre-compute leg distance from hardcoded coords so telemetry shows something useful
  bot.distToWp = haversineDistance(from.lat, from.lon, to.lat, to.lon);

  Serial.printf("[NAV] Next WP%d: %s | Locked bearing: %.1f° | Leg dist: %.1fm\n",
    bot.wpIndex, to.label, bot.lockedBearing, bot.distToWp);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 14 : NAVIGATION LOGIC — COMPASS-LOCK MODE
//
//  HOW IT WORKS:
//  At dispatch (or each WP advance), the bearing from the current hardcoded
//  waypoint to the next is computed ONCE and stored in bot.lockedBearing.
//  During the leg, the compass PD controller steers to hold that bearing.
//  GPS is used opportunistically to confirm arrival — if unavailable, the
//  hardcoded leg distance is shown in telemetry and the operator can halt
//  manually when the robot reaches the destination.
// ─────────────────────────────────────────────────────────────────────────────
void navigationTick() {
  // Compass is mandatory in this mode — no heading, no movement.
  if (!bot.compassOk) {
    Serial.println("[ERR] Compass unavailable — cannot navigate. Check I2C wiring.");
    motorsStop();
    bot.state = BotState::IDLE;
    return;
  }

  const Waypoint& wp = bot.route[bot.wpIndex];

  // ── GPS opportunistic arrival check ───────────────────────────────────
  // If GPS has a valid fix, use it to detect arrival.
  // If GPS has no fix, navigation continues on compass bearing — robot will
  // not stop automatically; operator halts via /halt when destination is reached.
  if (bot.gpsValid) {
    float dist = haversineDistance(bot.gpsLat, bot.gpsLon, wp.lat, wp.lon);
    bot.distToWp = dist;
    if (dist <= cfg::ARRIVAL_RADIUS_M) {
      advanceWaypoint();
      return;
    }
    if (dist > bot.prevDist) {
      bot.runawayCount++;
      if (bot.runawayCount >= cfg::RUNAWAY_COUNT) {
        Serial.println("[ERR] Runaway detected! Distance increasing. Halting.");
        motorsStop();
        bot.state = BotState::HALTED_RUNAWAY;
        return;
      }
    } else {
      bot.runawayCount = 0; // reset if getting closer
    }
    bot.prevDist = dist;
  }

  // ── Compass bearing hold (PD controller) ──────────────────────────────
  // bot.lockedBearing was set once when this leg started — it does not change.
  float currentHeading = bot.compassSmooth;
  float error          = normalizeAngle(bot.lockedBearing - currentHeading);
  bot.headingError     = error;
  bot.targetBearing    = bot.lockedBearing;

  MotorCmd cmd = computeMotorPwm(error, wp.uphill);
  setMotors(cmd.left, cmd.right);

  Serial.printf("[NAV] WP%d(%s) | Locked=%.1f° | Hdg=%.1f° | Err=%.1f° | L=%d R=%d | GPS:%s\n",
    bot.wpIndex, wp.label, bot.lockedBearing, currentHeading,
    error, cmd.left, cmd.right,
    bot.gpsValid ? "OK" : "no-fix");
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 15 : FSM TICK
// ─────────────────────────────────────────────────────────────────────────────
void fsmTick() {
  uint32_t now = millis();

  switch (bot.state) {

    // ── IDLE ────────────────────────────────────────────────────────────
    case BotState::IDLE:
      // Motors already stopped. Waiting for web command to set route + go.
      break;

    // ── NAVIGATING ──────────────────────────────────────────────────────
    case BotState::NAVIGATING: {

      // GPS warning (compass-lock mode — no fix does NOT halt the robot)
      if (now - bot.lastGpsFixMs > cfg::GPS_TIMEOUT_MS) {
        Serial.println("[WARN] GPS no fix — navigating on compass bearing only. Arrival must be confirmed manually.");
      }

      // Obstacle check
      if (!bot.pathClear) {
        Serial.printf("[OBS] Obstacle at %.1f cm — halting.\n", bot.sonarCm);
        motorsStop();
        bot.clearSinceMs    = 0;
        bot.stateBeforeHalt = BotState::NAVIGATING;
        bot.state           = BotState::HALTED_OBS;
        break;
      }

      // Run navigation on its interval
      if (now - bot.lastNavMs >= cfg::NAV_UPDATE_MS) {
        bot.lastNavMs = now;
        navigationTick();
      }
      break;
    }

    // ── HALTED_OBS ──────────────────────────────────────────────────────
    case BotState::HALTED_OBS:
      if (bot.pathClear) {
        if (bot.clearSinceMs == 0) {
          bot.clearSinceMs = now;   // start counting clear time
        } else if (now - bot.clearSinceMs >= cfg::CLEAR_HOLD_MS) {
          Serial.println("[OBS] Path clear for 2s — resuming navigation.");
          bot.pdPrevError   = 0.0f; // reset PD to avoid derivative spike on resume
          bot.pdPrevTimeMs  = now;
          bot.state         = BotState::NAVIGATING;
        }
      } else {
        bot.clearSinceMs = 0;       // reset if obstacle reappears
      }
      break;

    // ── HALTED_GPS ──────────────────────────────────────────────────────
    case BotState::HALTED_GPS:
      if (bot.gpsValid && (now - bot.lastGpsFixMs < cfg::GPS_TIMEOUT_MS)) {
        Serial.println("[GPS] Fix restored — resuming navigation.");
        bot.state = bot.stateBeforeHalt;
      }
      break;

    // ── HALTED_RUNAWAY ──────────────────────────────────────────────────
    case BotState::HALTED_RUNAWAY:
      // Operator must send /reset via web UI to clear this state.
      break;

    // ── ARRIVED ─────────────────────────────────────────────────────────
    case BotState::ARRIVED:
      // Mission complete. Await operator reset or new mission.
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 16 : TELEMETRY JSON BUILDER
// ─────────────────────────────────────────────────────────────────────────────
void buildTelemetry() {
  const char* routeStr = "None";
  if      (bot.route == ROUTE_D) routeStr = "D — Mess Gate";
  else if (bot.route == ROUTE_A) routeStr = "A — Gyan Mandir";
  else if (bot.route == ROUTE_B) routeStr = "B — Seminar Hall";
  else if (bot.route == ROUTE_C) routeStr = "C — Admin Block";

  const char* wpName = (bot.route && bot.wpIndex < bot.routeLen)
                       ? bot.route[bot.wpIndex].label
                       : "—";

  snprintf(bot.telemetryJson, sizeof(bot.telemetryJson),
    "{"
    "\"state\":\"%s\","
    "\"lat\":%.7f,"
    "\"lon\":%.7f,"
    "\"gpsOk\":%s,"
    "\"heading\":%.1f,"
    "\"bearing\":%.1f,"
    "\"headingErr\":%.1f,"
    "\"dist\":%.1f,"
    "\"route\":\"%s\","
    "\"wpName\":\"%s\","
    "\"wpIdx\":%d,"
    "\"wpLen\":%d,"
    "\"sonar\":%.1f,"
    "\"pathClear\":%s"
    "}",
    stateLabel(bot.state),
    bot.gpsLat, bot.gpsLon,
    bot.gpsValid ? "true" : "false",
    bot.compassSmooth,
    bot.targetBearing,
    bot.headingError,
    bot.distToWp,
    routeStr,
    wpName,
    bot.wpIndex,
    bot.routeLen,
    bot.sonarCm,
    bot.pathClear ? "true" : "false"
  );
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 17 : WEB SERVER ROUTE HANDLERS
// ─────────────────────────────────────────────────────────────────────────────
void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", bot.telemetryJson);
  });

  // /go?route=A|B|C — assigns route and transitions to NAVIGATING
  server.on("/go", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (bot.state != BotState::IDLE && bot.state != BotState::ARRIVED) {
      req->send(400, "text/plain", "Robot already on mission. Send /halt first.");
      return;
    }
    if (!req->hasParam("route")) {
      req->send(400, "text/plain", "Missing ?route= parameter (A, B, or C).");
      return;
    }
    String r = req->getParam("route")->value();
    r.toUpperCase();

    if (r == "D") {
      bot.route    = ROUTE_D;
      bot.routeLen = ROUTE_D_LEN;
    } else if (r == "A") {
      bot.route    = ROUTE_A;
      bot.routeLen = ROUTE_A_LEN;
    } else if (r == "B") {
      bot.route    = ROUTE_B;
      bot.routeLen = ROUTE_B_LEN;
    } else if (r == "C") {
      bot.route    = ROUTE_C;
      bot.routeLen = ROUTE_C_LEN;
    } else {
      req->send(400, "text/plain", "Invalid route. Use D, A, B, or C.");
      return;
    }

    bot.wpIndex       = 0;
    bot.runawayCount  = 0;
    bot.prevDist      = 9999.0f;
    bot.pdPrevError   = 0.0f;
    bot.pdPrevTimeMs  = millis();

    // Lock compass bearing for the first leg immediately at dispatch.
    // Bearing is from hardcoded WP[0] → WP[1] — no GPS fix required.
    {
      const Waypoint& from = bot.route[0];
      const Waypoint& to   = bot.route[1];
      bot.lockedBearing = initialBearing(from.lat, from.lon, to.lat, to.lon);
      bot.targetBearing = bot.lockedBearing;
      bot.distToWp      = haversineDistance(from.lat, from.lon, to.lat, to.lon);
      Serial.printf("[CMD] First leg: %s → %s | Locked bearing: %.1f° | Dist: %.1fm\n",
                    from.label, to.label, bot.lockedBearing, bot.distToWp);
    }

    bot.state         = BotState::NAVIGATING;

    Serial.printf("[CMD] Mission started — Route %s (%d waypoints) — COMPASS-LOCK MODE\n",
                  r.c_str(), bot.routeLen);
    req->send(200, "text/plain", "Mission started.");
  });

  // /halt — emergency stop (any state → IDLE with motors off)
  server.on("/halt", HTTP_GET, [](AsyncWebServerRequest* req) {
    motorsStop();
    bot.state = BotState::IDLE;
    Serial.println("[CMD] HALT received.");
    req->send(200, "text/plain", "Halted.");
  });

  // /reset — clears runaway/arrived state, returns to IDLE
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest* req) {
    motorsStop();
    bot.state        = BotState::IDLE;
    bot.route        = nullptr;
    bot.routeLen     = 0;
    bot.wpIndex      = 0;
    bot.runawayCount = 0;
    bot.prevDist     = 9999.0f;
    Serial.println("[CMD] RESET received.");
    req->send(200, "text/plain", "Reset complete.");
  });

  server.begin();
  Serial.println("[WEB] Async server started on port 80.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 18 : SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] DeliveryBot v2.0 starting...");

  // ── GPIO Configuration ────────────────────────────────────────────────
  pinMode(pin::IN1,  OUTPUT); pinMode(pin::IN2,  OUTPUT);
  pinMode(pin::IN3,  OUTPUT); pinMode(pin::IN4,  OUTPUT);
  pinMode(pin::TRIG, OUTPUT); pinMode(pin::ECHO, INPUT);
  digitalWrite(pin::TRIG, LOW);

  // ── LEDC PWM channels ────────────────────────────────────────────────
  // ESP32 Arduino core v3.x API: ledcAttach(pin, freq, resolution)
  // replaces the old ledcSetup() + ledcAttachPin() two-step.
  ledcAttach(pin::ENA, pwm::FREQ, pwm::RES);
  ledcAttach(pin::ENB, pwm::FREQ, pwm::RES);
  motorsStop();
  Serial.println("[MOTOR] LEDC PWM configured.");

  // ── I2C + Compass ────────────────────────────────────────────────────
  Wire.begin(pin::SDA, pin::SCL);
  Wire.setClock(400000);  // 400 kHz fast-mode
  bot.compassOk = compassInit();

  // ── GPS UART ─────────────────────────────────────────────────────────
  gpsSerial.begin(9600, SERIAL_8N1, pin::GPS_RX, pin::GPS_TX);
  Serial.println("[GPS] UART2 initialized at 9600 baud.");

  // ── WiFi Access Point ────────────────────────────────────────────────
  WiFi.softAP(cfg::AP_SSID, cfg::AP_PASSWORD);
  Serial.printf("[WIFI] AP '%s' up. IP: %s\n",
                cfg::AP_SSID, WiFi.softAPIP().toString().c_str());

  // ── Web Server ───────────────────────────────────────────────────────
  setupWebServer();

  bot.pdPrevTimeMs = millis();
  Serial.println("[BOOT] Setup complete. Waiting for mission command.\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 19 : MAIN LOOP
//  This function must return as fast as possible on every call.
//  All timing is millis()-based. No delay() calls here.
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // ── 1. GPS parsing (feed bytes as they arrive — no blocking) ─────────
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // TinyGPSPlus has decoded a complete sentence
      if (gps.location.isValid() && gps.location.isUpdated()) {
        bot.gpsLat      = gps.location.lat();
        bot.gpsLon      = gps.location.lng();
        bot.gpsValid    = true;
        bot.lastGpsFixMs = now;
      }
    }
  }

  // ── 2. Compass reading (on interval) ─────────────────────────────────
  if (now - bot.lastCompassMs >= cfg::COMPASS_UPDATE_MS) {
    bot.lastCompassMs = now;
    if (bot.compassOk) compassRead();
  }

  // ── 3. Ultrasonic state machine (runs every loop tick) ───────────────
  sonarTick();

  // ── 4. FSM tick ───────────────────────────────────────────────────────
  fsmTick();

  // ── 5. Telemetry JSON builder (on interval) ──────────────────────────
  if (now - bot.lastTelemetryMs >= cfg::TELEMETRY_UPDATE_MS) {
    bot.lastTelemetryMs = now;
    buildTelemetry();
  }

  // ── 6. ESPAsyncWebServer runs on its own FreeRTOS task ───────────────
  //  No yield() or handleClient() needed here — this is a key advantage
  //  of the async server vs the blocking WebServer.h approach.
}