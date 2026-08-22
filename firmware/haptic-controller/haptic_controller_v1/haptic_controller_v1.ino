/*
  HAPTIC NODE — ESP32 #2
  Target: Seeed Studio XIAO ESP32-S3
  Actuators: 6x coin ERM vibration motors, each through its own CPN2222A
  (2N2222-family NPN) transistor stage — SAME topology physically bench-
  verified on Motor 0 / GPIO9 before this file was written:

      ESP32 GPIO --333ohm--> CPN2222A base --> transistor --> coin motor --> motor supply

  RESPONSIBILITY (and ONLY this):
    - receive motor commands from PC over USB serial
    - drive 6 PWM motor outputs
    - watchdog: stop all motors if no valid command received within
      COMM_TIMEOUT_MS (protects against PC/Python crash, USB drop, etc.)

  Does NOT: read MLX90393, compute haptics (that's pc/haptic_algorithm.py),
  use WiFi/BT, or generate its own pulse timing — the PC algorithm already
  computes time-varying PWM (including pulse ON/OFF) and resends the
  current value at a fixed rate (see pc/haptic_engine.py, HAPTIC_SEND_HZ).
  This firmware just executes whatever PWM value it was last told, per
  motor, which is what keeps this loop non-blocking and protocol-simple.

  PWM API: ESP32 Arduino core 3.x pin-addressed LEDC —
    ledcAttach(pin, freq, resolutionBits) + ledcWrite(pin, duty)
  (core 2.x would need ledcSetup/ledcAttachPin/channel numbers instead —
  not used here, this repo targets core 3.x per docs/hardware.md)

  Serial protocol (must match docs/serial-protocol.md and
  pc/haptic_algorithm.py's build_motor_command() exactly):
    M,<m0>,<m1>,<m2>,<m3>,<m4>,<m5>   -- set all 6 PWM values (0-255), from the PC algorithm
    S                                  -- stop all motors immediately
    PING                                -- replies PONG
    STATUS                              -- replies STATUS,<m0..m5>,age_ms=<n>
  Bench-test-only additions (not used by the PC algorithm, for manual
  testing over a serial terminal):
    P,<index>,<pwm>                    -- set one motor's PWM directly
    A,<pwm>                            -- set all motors to the same PWM
*/

// ---- Compile-time modes ----
#define MOTOR_TEST_MODE 0   // 1 = hardware bring-up sequence only (see bottom of file), ignores PC protocol
#define DEBUG_SERIAL 0      // 1 = print "M0=.. M1=.." on every valid M, command received

// ======================================================================
// MOTOR GPIO PINS (XIAO ESP32-S3)
// ======================================================================
// M0 = GPIO9 (D10) is PHYSICALLY BENCH-VERIFIED with a CPN2222A stage
// (333 ohm base resistor) — do not move it without re-verifying.
// M1/M2 continue down the same free SPI-labeled pin group (D9, D8).
// M3/M4/M5 use D3/D2/D1 (plain GPIO, no special function on this board).
// D0 (GPIO1) is left spare. D4/D5 avoided (this board family's default
// I2C pins — not needed here since this board never reads the MLX90393,
// but left clear for consistency with the sensor board's pin docs).
// D6/D7 (GPIO43/44, UART0) avoided — GPIO43 blips for ~200ms at boot
// with the ROM bootloader log, which would glitch a motor on power-up.
#define MOTOR_0_PIN 9   // D10 -- PROVEN: CPN2222A base via 333 ohm, verified with real motor
#define MOTOR_1_PIN 8   // D9
#define MOTOR_2_PIN 7   // D8
#define MOTOR_3_PIN 4   // D3
#define MOTOR_4_PIN 3   // D2
#define MOTOR_5_PIN 2   // D1

#define NUM_MOTORS 6
const uint8_t MOTOR_PINS[NUM_MOTORS] = {
  MOTOR_0_PIN, MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, MOTOR_5_PIN
};

// The proven single-motor test drove the base directly HIGH/LOW (NPN,
// GPIO HIGH = motor ON). If a later driver revision inverts this (e.g. a
// PNP high-side stage), flip this one define — it's the only place
// inversion is applied.
#define MOTOR_PWM_INVERTED false

#define PWM_FREQUENCY 20000       // Hz, above audible range, avoids driver whine
#define PWM_RESOLUTION 8          // bits, 0-255 duty range

// ---- Communication watchdog ----
#define COMM_TIMEOUT_MS 500

// ---- Serial line buffer ----
#define LINE_BUF_SIZE 64

uint8_t motorIntensity[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
uint32_t lastValidCommandMs = 0;

char lineBuf[LINE_BUF_SIZE];
uint8_t lineLen = 0;

void setMotor(uint8_t index, uint8_t intensity);
void setAllMotors(uint8_t intensity);
void stopAllMotors();
void handleLine(char *line);
void handleMotorCommand(char *line);
void handleSingleMotorCommand(char *line);
void handleAllMotorsCommand(char *line);
void sendStatus();
void printPinTable();

#if MOTOR_TEST_MODE
void runMotorTestMode();
#endif

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* brief wait only, never hang */ }

  Serial.println(F("Haptic Controller V1"));
  printPinTable();

#if MOTOR_TEST_MODE
  Serial.println(F("*** MOTOR_TEST_MODE=1: hardware bring-up sequence, PC protocol NOT active ***"));
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    digitalWrite(MOTOR_PINS[i], MOTOR_PWM_INVERTED ? HIGH : LOW); // off before anything else
  }
#else
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    ledcAttach(MOTOR_PINS[i], PWM_FREQUENCY, PWM_RESOLUTION);
  }
  stopAllMotors();
  lastValidCommandMs = millis();
  Serial.println(F("READY"));
#endif
}

void loop() {
#if MOTOR_TEST_MODE
  runMotorTestMode();
#else
  // ---- read serial, one line at a time, non-blocking ----
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < LINE_BUF_SIZE - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0; // overflow, discard malformed line
    }
  }

  // ---- watchdog: no valid command recently -> stop everything ----
  // This is the ONLY place motors turn off due to inactivity. It runs
  // every loop() iteration regardless of serial traffic, so a PC crash,
  // Python crash, or USB drop always results in motors off within
  // COMM_TIMEOUT_MS, never "stuck on."
  if ((uint32_t)(millis() - lastValidCommandMs) > COMM_TIMEOUT_MS) {
    stopAllMotors();
  }
#endif
}

void handleLine(char *line) {
  if (line[0] == 'M' && line[1] == ',') {
    handleMotorCommand(line);
  } else if (line[0] == 'P' && line[1] == ',') {
    handleSingleMotorCommand(line);
  } else if (line[0] == 'A' && line[1] == ',') {
    handleAllMotorsCommand(line);
  } else if (line[0] == 'S' && (line[1] == '\0')) {
    stopAllMotors();
    lastValidCommandMs = millis(); // STOP counts as a valid, intentional command
  } else if (strcmp(line, "PING") == 0) {
    Serial.println(F("PONG"));
  } else if (strcmp(line, "STATUS") == 0) {
    sendStatus();
  }
  // anything else: silently ignored (malformed / unknown command)
}

// Parses the PC algorithm's actual protocol: M,<m0>,<m1>,<m2>,<m3>,<m4>,<m5>
void handleMotorCommand(char *line) {
  int values[NUM_MOTORS];
  char *token = strtok(line, ","); // "M"
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    token = strtok(NULL, ",");
    if (token == NULL) return; // malformed: too few fields, ignore whole command
    long v = atol(token);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    values[i] = (int)v;
  }
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    setMotor(i, (uint8_t)values[i]);
  }
  lastValidCommandMs = millis();

#if DEBUG_SERIAL
  Serial.print(F("M0=")); Serial.print(motorIntensity[0]);
  Serial.print(F(" M1=")); Serial.print(motorIntensity[1]);
  Serial.print(F(" M2=")); Serial.print(motorIntensity[2]);
  Serial.print(F(" M3=")); Serial.print(motorIntensity[3]);
  Serial.print(F(" M4=")); Serial.print(motorIntensity[4]);
  Serial.print(F(" M5=")); Serial.println(motorIntensity[5]);
#endif
}

// Bench-test-only: P,<index>,<pwm> -- e.g. "P,0,150" -> Motor 0 = PWM 150
void handleSingleMotorCommand(char *line) {
  char *token = strtok(line, ","); // "P"
  token = strtok(NULL, ",");
  if (token == NULL) return;
  long idx = atol(token);
  token = strtok(NULL, ",");
  if (token == NULL) return;
  long pwm = atol(token);
  if (idx < 0 || idx >= NUM_MOTORS) return;
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  setMotor((uint8_t)idx, (uint8_t)pwm);
  lastValidCommandMs = millis();
  Serial.print(F("OK M")); Serial.print(idx); Serial.print(F("=")); Serial.println(pwm);
}

// Bench-test-only: A,<pwm> -- e.g. "A,150" -> all motors = PWM 150
void handleAllMotorsCommand(char *line) {
  char *token = strtok(line, ","); // "A"
  token = strtok(NULL, ",");
  if (token == NULL) return;
  long pwm = atol(token);
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  setAllMotors((uint8_t)pwm);
  lastValidCommandMs = millis();
  Serial.print(F("OK ALL=")); Serial.println(pwm);
}

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
  for (uint8_t i = 0; i < NUM_MOTORS; i++) setMotor(i, intensity);
}

void stopAllMotors() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) setMotor(i, 0);
}

void sendStatus() {
  Serial.print(F("STATUS,"));
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    Serial.print(motorIntensity[i]);
    if (i < NUM_MOTORS - 1) Serial.print(',');
  }
  Serial.print(F(",age_ms="));
  Serial.println((uint32_t)(millis() - lastValidCommandMs));
}

void printPinTable() {
  Serial.print(F("Motor 0: GPIO")); Serial.println(MOTOR_0_PIN);
  Serial.print(F("Motor 1: GPIO")); Serial.println(MOTOR_1_PIN);
  Serial.print(F("Motor 2: GPIO")); Serial.println(MOTOR_2_PIN);
  Serial.print(F("Motor 3: GPIO")); Serial.println(MOTOR_3_PIN);
  Serial.print(F("Motor 4: GPIO")); Serial.println(MOTOR_4_PIN);
  Serial.print(F("Motor 5: GPIO")); Serial.println(MOTOR_5_PIN);
}

// ======================================================================
// HARDWARE BRING-UP TEST MODE (MOTOR_TEST_MODE=1)
// ======================================================================
// Digital ON/OFF only (no PWM) — proves GPIO -> CPN2222A -> motor wiring
// for all six channels using the exact same electrical approach already
// bench-verified on Motor 0. Non-blocking (millis()-based), so 'S' typed
// into the serial monitor still stops everything immediately even
// mid-sequence — this is a safety behavior, not part of the PC protocol.
//
// Sequence: M0 ON 1s -> OFF -> M1 ON 1s -> OFF -> ... -> M5 ON 1s -> OFF
//           -> ALL ON 1s -> ALL OFF -> pause 2s -> repeat
#if MOTOR_TEST_MODE

enum TestPhase { TEST_MOTOR_ON, TEST_ALL_ON, TEST_ALL_OFF, TEST_PAUSE };
TestPhase testPhase = TEST_MOTOR_ON;
uint8_t testMotorIndex = 0;
uint32_t testPhaseStart = 0;
bool testPhaseEntered = false; // true once the current phase's entry action has run

void testDigitalWrite(uint8_t index, bool on) {
  bool level = MOTOR_PWM_INVERTED ? !on : on;
  digitalWrite(MOTOR_PINS[index], level ? HIGH : LOW);
}

void enterTestPhase(TestPhase phase) {
  testPhase = phase;
  testPhaseStart = millis();
  testPhaseEntered = false;
}

void runMotorTestMode() {
  // Safety: allow 'S' + Enter to abort and stop everything at any time.
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'S' || c == 's') {
      for (uint8_t i = 0; i < NUM_MOTORS; i++) testDigitalWrite(i, false);
      Serial.println(F("STOP (test mode aborted)"));
    }
  }

  uint32_t elapsed = millis() - testPhaseStart;

  switch (testPhase) {
    case TEST_MOTOR_ON:
      if (!testPhaseEntered) {
        Serial.print(F("Motor ")); Serial.print(testMotorIndex); Serial.println(F(" ON"));
        testDigitalWrite(testMotorIndex, true);
        testPhaseEntered = true;
      }
      if (elapsed >= 1000) {
        testDigitalWrite(testMotorIndex, false);
        Serial.print(F("Motor ")); Serial.print(testMotorIndex); Serial.println(F(" OFF"));
        testMotorIndex++;
        if (testMotorIndex >= NUM_MOTORS) {
          enterTestPhase(TEST_ALL_ON);
        } else {
          enterTestPhase(TEST_MOTOR_ON); // next motor
        }
      }
      break;

    case TEST_ALL_ON:
      if (!testPhaseEntered) {
        Serial.println(F("ALL MOTORS ON"));
        for (uint8_t i = 0; i < NUM_MOTORS; i++) testDigitalWrite(i, true);
        testPhaseEntered = true;
      }
      if (elapsed >= 1000) {
        enterTestPhase(TEST_ALL_OFF);
      }
      break;

    case TEST_ALL_OFF:
      if (!testPhaseEntered) {
        Serial.println(F("ALL MOTORS OFF"));
        for (uint8_t i = 0; i < NUM_MOTORS; i++) testDigitalWrite(i, false);
        testPhaseEntered = true;
      }
      enterTestPhase(TEST_PAUSE);
      break;

    case TEST_PAUSE:
      if (elapsed >= 2000) {
        Serial.println(F("--- repeating test sequence ---"));
        testMotorIndex = 0;
        enterTestPhase(TEST_MOTOR_ON);
      }
      break;

    default:
      break;
  }
}

#endif // MOTOR_TEST_MODE
