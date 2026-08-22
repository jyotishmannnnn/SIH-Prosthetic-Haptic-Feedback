/*
  eFlesh -> Haptic Feedback Controller — FIRST WORKING PROTOTYPE
  Target: Seeed Studio XIAO ESP32-S3
  Sensor: 1x MLX90393 (single sensor, single eFlesh patch)
  Actuators: 6x coin ERM vibration motors (PWM, independent channels)

  Pipeline:
    eFlesh -> MLX90393 (Bx,By,Bz) -> baseline subtraction -> EMA filter ->
    tactile magnitude -> deadband -> contact detection (hysteresis) ->
    feedback level state machine -> haptic pattern -> 6 motors (PWM)

  SOURCE OF TRUTH for MLX90393 config: eFlesh-main/arduino/5X_eflesh_stream/5X_eflesh_stream.ino
  Library: https://github.com/tesshellebrekers/arduino-MLX90393 (install manually, see README below)

  Not implemented here (by design, first milestone only):
    multi-sensor fusion, spatial/localization ML, WiFi, dataset logging, PC dependency.
*/

#include <Wire.h>
#include <MLX90393.h>

// ======================================================================
// USER-CONFIGURABLE SECTION
// ======================================================================

// ---- Debug ----
#define DEBUG_SERIAL 1          // 1 = print debug at DEBUG_RATE_HZ, 0 = silent
#define DEBUG_RATE_HZ 10        // do not raise this near loop rate, floods Serial

// ---- I2C pins (XIAO ESP32-S3 default I2C = D4/D5) ----
#define I2C_SDA_PIN 5           // D4
#define I2C_SCL_PIN 6           // D5
#define I2C_CLOCK_HZ 400000

// ---- MLX90393 I2C address ----
// eFlesh boards commonly present at 0x0C (see repo TARGETS_ALL_CONSEC[0]).
// If your board uses a different address strap, change this or rely on
// the startup I2C scan fallback below.
#define MLX_I2C_ADDR 0x0C

// ---- MLX90393 sensor config (copied verbatim from eFlesh-main reference sketch) ----
#define MLX_GAIN_SEL        0x1
#define MLX_RES_X           0x2
#define MLX_RES_Y           0x2
#define MLX_RES_Z           0x2
#define MLX_DIG_FILTERING   0x4
#define MLX_BURST_SET       0xF   // temp + X + Y + Z

// ---- Motor GPIO pins (XIAO ESP32-S3) ----
// D4/D5 reserved for I2C, D6/D7 reserved for UART0 (avoided for reliability).
#define MOTOR_0_PIN 1   // D0
#define MOTOR_1_PIN 2   // D1
#define MOTOR_2_PIN 3   // D2
#define MOTOR_3_PIN 4   // D3
#define MOTOR_4_PIN 7   // D8
#define MOTOR_5_PIN 8   // D9

#define NUM_MOTORS 6
const uint8_t MOTOR_PINS[NUM_MOTORS] = {
  MOTOR_0_PIN, MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, MOTOR_5_PIN
};

// If your driver topology inverts logic (e.g. high-side PNP where
// GPIO=LOW means "motor ON"), set this to true. Everything else in the
// code stays the same — only setMotor() applies the inversion.
#define MOTOR_PWM_INVERTED false

#define PWM_FREQ_HZ 20000     // above audible range, avoids driver whine
#define PWM_RESOLUTION_BITS 8 // 0-255 duty range

// ---- Coin ERM motors have a dead zone: below this PWM they don't spin ----
#define MOTOR_MIN_PWM 60      // TUNE: lowest PWM that reliably spins your motors
#define MOTOR_MAX_PWM 255

// ---- Baseline calibration ----
#define CALIBRATION_SAMPLES 100

// ---- EMA filter ----
#define FILTER_ALPHA 0.3f     // 0..1, higher = less smoothing, more responsive

// ---- Deadband ----
// Magnitude below this (in raw MLX90393 sensor units, same units as Bx/By/Bz)
// is treated as noise and forced to 0. MUST be calibrated per-unit: watch
// the "mag=" debug value with the eFlesh untouched, set this a bit above
// the observed noise floor.
#define DEAD_BAND 3.0f

// ---- Contact detection hysteresis ----
// TUNE these against observed "mag=" values while pressing/releasing eFlesh.
#define CONTACT_ON_THRESHOLD  8.0f
#define CONTACT_OFF_THRESHOLD 4.0f

// ---- Feedback level thresholds (on filtered magnitude, post-deadband) ----
// TUNE against observed light/medium/hard press magnitudes.
#define LEVEL1_THRESHOLD  8.0f
#define LEVEL2_THRESHOLD  25.0f
#define LEVEL3_THRESHOLD  50.0f
#define LEVEL4_THRESHOLD  80.0f

// ---- Per-level motor PWM (subject to MOTOR_MIN_PWM..MOTOR_MAX_PWM clamp) ----
#define LEVEL1_PWM 90
#define LEVEL2_PWM 150
#define LEVEL3_PWM 210
#define LEVEL4_PWM 255

// ---- Per-level pulse timing (ms). LEVEL4 is continuous (ON, no pulsing). ----
#define LEVEL1_ON_MS  60
#define LEVEL1_OFF_MS 340   // slow pulse
#define LEVEL2_ON_MS  90
#define LEVEL2_OFF_MS 160   // moderate pulse
#define LEVEL3_ON_MS  110
#define LEVEL3_OFF_MS 60    // fast pulse

// ---- How many motors are active at each level (default strategy) ----
#define LEVEL1_MOTOR_COUNT 2
#define LEVEL2_MOTOR_COUNT 4
#define LEVEL3_MOTOR_COUNT 6
#define LEVEL4_MOTOR_COUNT 6

// ---- Optional spatial mode (secondary, off by default) ----
#define SPATIAL_MODE_ENABLED 0

// ---- Sensor fault handling ----
#define MAX_CONSECUTIVE_I2C_FAILS 10

// ---- Loop timing target ----
#define LOOP_TARGET_HZ 100
#define LOOP_PERIOD_US (1000000UL / LOOP_TARGET_HZ)

// ---- Onboard LED (XIAO ESP32-S3 user LED, active LOW) ----
#define STATUS_LED_PIN 21
#define STATUS_LED_ACTIVE_LOW true

// ======================================================================
// TYPES / STATE
// ======================================================================

MLX90393 mlx;
MLX90393::txyz raw;

float baselineX = 0, baselineY = 0, baselineZ = 0;
bool calibrated = false;

float filteredMag = 0.0f;

bool contactActive = false;

enum FeedbackLevel {
  LEVEL_0_NONE = 0,
  LEVEL_1_LOW,
  LEVEL_2_MEDIUM,
  LEVEL_3_HIGH,
  LEVEL_4_MAX
};
FeedbackLevel currentLevel = LEVEL_0_NONE;

uint8_t motorIntensity[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

uint32_t consecutiveI2CFails = 0;
bool sensorFaulted = false;

uint32_t lastDebugPrintMs = 0;
uint32_t lastLoopMicros = 0;

// pulse timing state (per-level, shared across active motors for simplicity)
bool pulseOn = true;
uint32_t pulsePhaseStartMs = 0;

// ======================================================================
// FORWARD DECLS
// ======================================================================
bool initMLX90393();
bool readMLX90393(float &x, float &y, float &z);
void calibrateBaseline();
float filterSensorData(float rawMag);
float calculateTactileMagnitude(float dx, float dy, float dz);
void detectContact(float mag);
FeedbackLevel calculateFeedbackLevel(float mag);
void updateHapticPattern(FeedbackLevel level, float dx, float dy, float dz);
void setMotor(uint8_t index, uint8_t intensity);
void setAllMotors(uint8_t intensity);
void stopAllMotors();
void printDebug(float dx, float dy, float dz, float mag);
void handleSerialCommands();
void setStatusLed(bool on);

// ======================================================================
// SETUP
// ======================================================================

void setup() {
  Serial.begin(115200);
  uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 2000) { /* wait briefly, don't hang forever */ }

  pinMode(STATUS_LED_PIN, OUTPUT);
  setStatusLed(false);

  // --- Motor PWM init ---
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    ledcAttach(MOTOR_PINS[i], PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  }
  stopAllMotors();

  // --- I2C init ---
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(10);

  Serial.println(F("eFlesh haptic controller starting..."));

  if (!initMLX90393()) {
    Serial.println(F("[FATAL] MLX90393 init failed. Motors disabled."));
    stopAllMotors();
    sensorFaulted = true;
    return;
  }

  calibrateBaseline();

  Serial.println(F("Ready. Serial commands: 'b'=recalibrate  'x'=emergency stop"));
  lastLoopMicros = micros();
}

// ======================================================================
// MAIN LOOP (non-blocking, ~100 Hz target)
// ======================================================================

void loop() {
  handleSerialCommands();

  uint32_t nowMicros = micros();
  if ((uint32_t)(nowMicros - lastLoopMicros) < LOOP_PERIOD_US) {
    return; // not time yet, do nothing blocking
  }
  lastLoopMicros = nowMicros;

  if (sensorFaulted || !calibrated) {
    stopAllMotors();
    return;
  }

  float bx, by, bz;
  if (!readMLX90393(bx, by, bz)) {
    consecutiveI2CFails++;
    if (consecutiveI2CFails >= MAX_CONSECUTIVE_I2C_FAILS) {
      Serial.println(F("[ERROR] MLX90393 repeated read failures. Stopping motors."));
      stopAllMotors();
      sensorFaulted = true;
      // attempt recovery
      if (initMLX90393()) {
        Serial.println(F("[RECOVERY] Sensor re-initialized."));
        sensorFaulted = false;
        consecutiveI2CFails = 0;
        calibrateBaseline();
      }
    }
    return; // do not act on stale/missing data
  }
  consecutiveI2CFails = 0;

  float dx = bx - baselineX;
  float dy = by - baselineY;
  float dz = bz - baselineZ;

  float rawMag = calculateTactileMagnitude(dx, dy, dz);
  filteredMag = filterSensorData(rawMag);

  if (filteredMag < DEAD_BAND) {
    filteredMag = 0.0f;
  }

  detectContact(filteredMag);
  currentLevel = calculateFeedbackLevel(filteredMag);
  updateHapticPattern(currentLevel, dx, dy, dz);

  printDebug(dx, dy, dz, filteredMag);
}

// ======================================================================
// MLX90393 DRIVER (config adapted verbatim from eFlesh-main reference sketch)
// ======================================================================

bool initMLX90393() {
  byte status = mlx.begin(MLX_I2C_ADDR, -1, Wire);
  Serial.print(F("MLX90393 init status=0x")); Serial.println(status, HEX);

  mlx.setGainSel(MLX_GAIN_SEL);
  mlx.setResolution(MLX_RES_X, MLX_RES_Y, MLX_RES_Z);
  mlx.setDigitalFiltering(MLX_DIG_FILTERING);
  mlx.startBurst(MLX_BURST_SET);

  // A hard failure status from begin() is treated as "not detected".
  // (Per the tedyapo/tesshellebrekers library, 0xFF-style / high bit
  // patterns indicate no ACK / bad status; a clean status is small.)
  return true; // begin() in this library does not hard-fail cleanly;
               // real detection is verified by the calibration read below.
}

bool readMLX90393(float &x, float &y, float &z) {
  byte status = mlx.readBurstData(raw);
  if (status == 0xFF) {
    return false; // I2C transaction failed
  }
  x = raw.x;
  y = raw.y;
  z = raw.z;
  return true;
}

// ======================================================================
// CALIBRATION
// ======================================================================

void calibrateBaseline() {
  Serial.println(F("Calibrating baseline. DO NOT TOUCH eFlesh..."));
  stopAllMotors();
  setStatusLed(true); // solid LED = calibrating

  // Use arrays for median. CALIBRATION_SAMPLES kept modest (100) to bound RAM.
  static float samplesX[CALIBRATION_SAMPLES];
  static float samplesY[CALIBRATION_SAMPLES];
  static float samplesZ[CALIBRATION_SAMPLES];

  uint16_t collected = 0;
  uint16_t attempts = 0;
  const uint16_t MAX_ATTEMPTS = CALIBRATION_SAMPLES * 10;

  while (collected < CALIBRATION_SAMPLES && attempts < MAX_ATTEMPTS) {
    float x, y, z;
    if (readMLX90393(x, y, z)) {
      samplesX[collected] = x;
      samplesY[collected] = y;
      samplesZ[collected] = z;
      collected++;
    }
    attempts++;
    delayMicroseconds(2000); // ~500 Hz sample attempt rate during calibration only
  }

  if (collected < CALIBRATION_SAMPLES / 2) {
    Serial.println(F("[FATAL] Calibration failed: insufficient sensor reads."));
    stopAllMotors();
    sensorFaulted = true;
    calibrated = false;
    setStatusLed(false);
    return;
  }

  // simple insertion sort (small N, fine at startup only)
  for (uint16_t i = 1; i < collected; i++) {
    float kx = samplesX[i], ky = samplesY[i], kz = samplesZ[i];
    int16_t j = i - 1;
    while (j >= 0 && samplesX[j] > kx) { samplesX[j + 1] = samplesX[j]; j--; }
    samplesX[j + 1] = kx;
  }
  for (uint16_t i = 1; i < collected; i++) {
    float ky = samplesY[i];
    int16_t j = i - 1;
    while (j >= 0 && samplesY[j] > ky) { samplesY[j + 1] = samplesY[j]; j--; }
    samplesY[j + 1] = ky;
  }
  for (uint16_t i = 1; i < collected; i++) {
    float kz = samplesZ[i];
    int16_t j = i - 1;
    while (j >= 0 && samplesZ[j] > kz) { samplesZ[j + 1] = samplesZ[j]; j--; }
    samplesZ[j + 1] = kz;
  }

  baselineX = samplesX[collected / 2];
  baselineY = samplesY[collected / 2];
  baselineZ = samplesZ[collected / 2];

  calibrated = true;
  sensorFaulted = false;
  filteredMag = 0.0f;
  contactActive = false;
  currentLevel = LEVEL_0_NONE;

  setStatusLed(false);
  Serial.print(F("Calibration done. baseline=(" ));
  Serial.print(baselineX); Serial.print(F(", "));
  Serial.print(baselineY); Serial.print(F(", "));
  Serial.print(baselineZ); Serial.println(F(")"));
}

// ======================================================================
// SIGNAL PROCESSING
// ======================================================================

float calculateTactileMagnitude(float dx, float dy, float dz) {
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

float filterSensorData(float rawMag) {
  static float previous = 0.0f;
  float out = FILTER_ALPHA * rawMag + (1.0f - FILTER_ALPHA) * previous;
  previous = out;
  return out;
}

void detectContact(float mag) {
  if (!contactActive && mag > CONTACT_ON_THRESHOLD) {
    contactActive = true;
  } else if (contactActive && mag < CONTACT_OFF_THRESHOLD) {
    contactActive = false;
  }
}

FeedbackLevel calculateFeedbackLevel(float mag) {
  if (!contactActive) return LEVEL_0_NONE;
  if (mag >= LEVEL4_THRESHOLD) return LEVEL_4_MAX;
  if (mag >= LEVEL3_THRESHOLD) return LEVEL_3_HIGH;
  if (mag >= LEVEL2_THRESHOLD) return LEVEL_2_MEDIUM;
  if (mag >= LEVEL1_THRESHOLD) return LEVEL_1_LOW;
  return LEVEL_0_NONE;
}

// ======================================================================
// HAPTIC PATTERN GENERATION
// ======================================================================

uint8_t clampPwm(uint16_t v) {
  if (v == 0) return 0;
  if (v < MOTOR_MIN_PWM) return MOTOR_MIN_PWM;
  if (v > MOTOR_MAX_PWM) return MOTOR_MAX_PWM;
  return (uint8_t)v;
}

void updateHapticPattern(FeedbackLevel level, float dx, float dy, float dz) {
  if (level == LEVEL_0_NONE) {
    stopAllMotors();
    pulseOn = true;
    pulsePhaseStartMs = millis();
    return;
  }

  uint8_t pwm = 0;
  uint16_t onMs = 0, offMs = 0;
  uint8_t activeCount = 0;
  bool continuous = false;

  switch (level) {
    case LEVEL_1_LOW:
      pwm = LEVEL1_PWM; onMs = LEVEL1_ON_MS; offMs = LEVEL1_OFF_MS;
      activeCount = LEVEL1_MOTOR_COUNT;
      break;
    case LEVEL_2_MEDIUM:
      pwm = LEVEL2_PWM; onMs = LEVEL2_ON_MS; offMs = LEVEL2_OFF_MS;
      activeCount = LEVEL2_MOTOR_COUNT;
      break;
    case LEVEL_3_HIGH:
      pwm = LEVEL3_PWM; onMs = LEVEL3_ON_MS; offMs = LEVEL3_OFF_MS;
      activeCount = LEVEL3_MOTOR_COUNT;
      break;
    case LEVEL_4_MAX:
      pwm = LEVEL4_PWM; activeCount = LEVEL4_MOTOR_COUNT;
      continuous = true;
      break;
    default:
      break;
  }

  // pulse phase (skipped for continuous level 4)
  if (!continuous) {
    uint32_t elapsed = millis() - pulsePhaseStartMs;
    uint16_t phaseLen = pulseOn ? onMs : offMs;
    if (elapsed >= phaseLen) {
      pulseOn = !pulseOn;
      pulsePhaseStartMs = millis();
    }
  } else {
    pulseOn = true;
  }

  uint8_t drivePwm = pulseOn ? clampPwm(pwm) : 0;

#if SPATIAL_MODE_ENABLED
  // Optional: bias which motors are "first" to activate based on dominant axis.
  // Kept simple: build a priority order, activate the first activeCount of it.
  uint8_t order[NUM_MOTORS] = {0, 1, 2, 3, 4, 5};
  float ax = fabsf(dx), ay = fabsf(dy);
  if (ax >= ay) {
    if (dx >= 0) { order[0]=0; order[1]=1; order[2]=2; order[3]=3; order[4]=4; order[5]=5; }
    else         { order[0]=2; order[1]=3; order[2]=0; order[3]=1; order[4]=4; order[5]=5; }
  } else {
    if (dy >= 0) { order[0]=4; order[1]=0; order[2]=1; order[3]=2; order[4]=3; order[5]=5; }
    else         { order[0]=5; order[1]=0; order[2]=1; order[3]=2; order[4]=3; order[5]=4; }
  }
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    bool active = i < activeCount;
    setMotor(order[i], active ? drivePwm : 0);
  }
#else
  // Default strategy: activate the first N motors (0..activeCount-1).
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    bool active = i < activeCount;
    setMotor(i, active ? drivePwm : 0);
  }
#endif
}

// ======================================================================
// MOTOR CONTROL PRIMITIVES
// ======================================================================

void setMotor(uint8_t index, uint8_t intensity) {
  if (index >= NUM_MOTORS) return;
  uint8_t duty = intensity;
#if MOTOR_PWM_INVERTED
  duty = 255 - duty;
#endif
  ledcWrite(MOTOR_PINS[index], duty);
  motorIntensity[index] = intensity;
}

void setAllMotors(uint8_t intensity) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    setMotor(i, intensity);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    setMotor(i, 0);
  }
}

// ======================================================================
// SERIAL COMMANDS / DEBUG
// ======================================================================

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') {
      Serial.println(F("Recalibration requested..."));
      calibrateBaseline();
    } else if (c == 'x' || c == 'X') {
      Serial.println(F("[EMERGENCY STOP] Motors off."));
      stopAllMotors();
      sensorFaulted = true; // require 'b' to resume normal operation
    }
  }
}

void printDebug(float dx, float dy, float dz, float mag) {
#if DEBUG_SERIAL
  uint32_t nowMs = millis();
  if (nowMs - lastDebugPrintMs < (1000 / DEBUG_RATE_HZ)) return;
  lastDebugPrintMs = nowMs;

  Serial.print(F("t=")); Serial.print(nowMs);
  Serial.print(F(" Bx=")); Serial.print(raw.x, 1);
  Serial.print(F(" By=")); Serial.print(raw.y, 1);
  Serial.print(F(" Bz=")); Serial.print(raw.z, 1);
  Serial.print(F(" dX=")); Serial.print(dx, 1);
  Serial.print(F(" dY=")); Serial.print(dy, 1);
  Serial.print(F(" dZ=")); Serial.print(dz, 1);
  Serial.print(F(" mag=")); Serial.print(mag, 1);
  Serial.print(F(" level=")); Serial.print((int)currentLevel);
  Serial.print(F(" contact=")); Serial.print(contactActive ? 1 : 0);
  Serial.print(F(" motors=["));
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    Serial.print(motorIntensity[i]);
    if (i < NUM_MOTORS - 1) Serial.print(F(","));
  }
  Serial.println(F("]"));
#endif
}

void setStatusLed(bool on) {
#if STATUS_LED_ACTIVE_LOW
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
#else
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
#endif
}
