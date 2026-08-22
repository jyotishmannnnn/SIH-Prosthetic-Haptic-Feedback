/*
  eFlesh SENSOR NODE — ESP32 #1
  Target: Seeed Studio XIAO ESP32-S3
  Sensor: 1x MLX90393, 1x eFlesh patch (mounted on 3D-printed robotic hand)

  RESPONSIBILITY (and ONLY this):
    - init MLX90393
    - read Bx,By,Bz
    - timestamp
    - stream to PC over USB serial
    - detect I2C read failures

  Does NOT: control motors, compute haptics, run ML, use WiFi/BT.

  MLX90393 config below is copied verbatim from eFlesh-main reference sketch:
    eFlesh-main/arduino/5X_eflesh_stream/5X_eflesh_stream.ino
  (gain 0x1, resolution 0x2/0x2/0x2, digital filtering 0x4, burst set 0xF = T+X+Y+Z)

  Library required: https://github.com/tesshellebrekers/arduino-MLX90393
  (repo submodule arduino/arduino-MLX90393 — install manually into Arduino libraries/)
*/

#include <Wire.h>
#include <MLX90393.h>

// ---- Debug ----
// When 0 (default): ONLY clean "S,..." packets on Serial. Nothing else.
// When 1: adds "#DBG ..." lines (prefixed so PC parser can ignore them).
#define DEBUG_SERIAL 0

// ---- I2C pins ----
// XIAO ESP32-S3 default I2C break-out: D4=GPIO5 (SDA), D5=GPIO6 (SCL).
// Matches the wiring given in the prompt; repo sketch uses Wire.begin()
// with no explicit pins (relies on board default), which is the SAME
// SDA/SCL pair on the boards the repo targets (QT Py / Trinket M0 Qwiic).
// No discrepancy for XIAO ESP32-S3 — D4/D5 is correct here too.
#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 6
#define I2C_CLOCK_HZ 400000

// ---- MLX90393 I2C address ----
// eFlesh boards commonly present at 0x0C (repo's TARGETS_ALL_CONSEC[0]).
// Verify with an I2C scan if your board doesn't ACK here.
#define MLX_I2C_ADDR 0x0C

// ---- MLX90393 sensor config (verbatim from eFlesh-main reference sketch) ----
#define MLX_GAIN_SEL       0x1
#define MLX_RES_X          0x2
#define MLX_RES_Y          0x2
#define MLX_RES_Z          0x2
#define MLX_DIG_FILTERING  0x4
#define MLX_BURST_SET      0xF   // temp + X + Y + Z

// ---- Sample rate target ----
#define SAMPLE_RATE_HZ 100
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_RATE_HZ)

// ---- Fault handling ----
#define MAX_CONSECUTIVE_I2C_FAILS 10

MLX90393 mlx;
MLX90393::txyz raw;

uint32_t lastSampleMicros = 0;
uint32_t consecutiveFails = 0;
bool sensorFaulted = false;

bool initMLX90393();
bool readMLX90393(float &x, float &y, float &z);
void sendSample(uint32_t ts, float x, float y, float z);
void sendFault(const char *msg);

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* brief wait only, never hang */ }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(10);

  if (!initMLX90393()) {
    sendFault("MLX90393 init failed");
    sensorFaulted = true;
  }

#if DEBUG_SERIAL
  Serial.println(F("#DBG sensor node ready"));
#endif

  lastSampleMicros = micros();
}

void loop() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastSampleMicros) < SAMPLE_PERIOD_US) return;
  lastSampleMicros = now;

  if (sensorFaulted) {
    // try to recover without ever blocking the loop indefinitely
    if (initMLX90393()) {
      sensorFaulted = false;
      consecutiveFails = 0;
#if DEBUG_SERIAL
      Serial.println(F("#DBG sensor recovered"));
#endif
    } else {
      return;
    }
  }

  float x, y, z;
  if (!readMLX90393(x, y, z)) {
    consecutiveFails++;
    if (consecutiveFails >= MAX_CONSECUTIVE_I2C_FAILS) {
      sendFault("MLX90393 repeated read failure");
      sensorFaulted = true;
    }
    return;
  }
  consecutiveFails = 0;

  sendSample(millis(), x, y, z);
}

bool initMLX90393() {
  mlx.begin(MLX_I2C_ADDR, -1, Wire);
  mlx.setGainSel(MLX_GAIN_SEL);
  mlx.setResolution(MLX_RES_X, MLX_RES_Y, MLX_RES_Z);
  mlx.setDigitalFiltering(MLX_DIG_FILTERING);
  mlx.startBurst(MLX_BURST_SET);
  // Library begin() status is not a clean pass/fail; real verification
  // happens via successful burst reads at runtime (see readMLX90393).
  return true;
}

bool readMLX90393(float &x, float &y, float &z) {
  byte status = mlx.readBurstData(raw);
  if (status == 0xFF) return false; // I2C transaction failed
  x = raw.x;
  y = raw.y;
  z = raw.z;
  return true;
}

// Machine-readable data line. Exactly this format, nothing else on this stream.
// S,<millis>,<Bx>,<By>,<Bz>
void sendSample(uint32_t ts, float x, float y, float z) {
  Serial.print('S'); Serial.print(',');
  Serial.print(ts); Serial.print(',');
  Serial.print(x, 2); Serial.print(',');
  Serial.print(y, 2); Serial.print(',');
  Serial.println(z, 2);
}

// Faults go out prefixed so the PC parser can distinguish/ignore them.
void sendFault(const char *msg) {
  Serial.print(F("F,")); Serial.println(msg);
}
