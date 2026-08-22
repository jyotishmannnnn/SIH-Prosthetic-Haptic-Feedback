/*
  HAPTIC NODE — ESP32 #2
  Target: Seeed Studio XIAO ESP32-S3
  Actuators: 6x coin ERM vibration motors, PWM-driven through transistor/MOSFET stage.

  RESPONSIBILITY (and ONLY this):
    - receive motor commands from PC over USB serial
    - drive 6 PWM motor outputs
    - watchdog: stop all motors if no valid command received within
      COMM_TIMEOUT_MS (protects against PC/Python crash, USB drop, etc.)

  Does NOT: read MLX90393, compute haptics, use WiFi/BT.

  PWM API: ESP32 Arduino core 3.x pin-addressed LEDC —
    ledcAttach(pin, freq, resolutionBits) + ledcWrite(pin, duty)
  If your installed core is 2.x, swap for ledcSetup/ledcAttachPin/channel API.
*/

// ---- Motor GPIO pins (XIAO ESP32-S3) ----
// D4/D5 avoided (commonly I2C on this board family), D6/D7 avoided (UART0).
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

// If your driver stage inverts logic (GPIO LOW = motor ON), flip this.
// Only place inversion is applied — never scattered elsewhere.
#define MOTOR_PWM_INVERTED false

#define PWM_FREQ_HZ 20000       // above audible range
#define PWM_RESOLUTION_BITS 8   // 0-255 duty

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
void sendStatus();

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* brief wait only */ }

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    ledcAttach(MOTOR_PINS[i], PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  }
  stopAllMotors();

  lastValidCommandMs = millis();
  Serial.println(F("#DBG haptic node ready"));
}

void loop() {
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
  if ((uint32_t)(millis() - lastValidCommandMs) > COMM_TIMEOUT_MS) {
    stopAllMotors();
  }
}

void handleLine(char *line) {
  if (line[0] == 'M' && line[1] == ',') {
    handleMotorCommand(line);
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

// Parses: M,<m0>,<m1>,<m2>,<m3>,<m4>,<m5>
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
