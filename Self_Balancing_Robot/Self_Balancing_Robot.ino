#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ---------------- MPU6050 / DMP ----------------
MPU6050 mpu;

const int MPU_INT_PIN = 3;   // INT on D3

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// ---------------- Motors ----------------
Servo leftMotor;
Servo rightMotor;

const int LEFT_PIN  = 2;
const int RIGHT_PIN = 4;

const int MIN_US  = 1000;
const int STOP_US = 1500;
const int MAX_US  = 2000;

const bool reverseRight = true;

int clampUS(int us) {
  if (us < MIN_US) return MIN_US;
  if (us > MAX_US) return MAX_US;
  return us;
}

void setMotorsUS(int leftUS, int rightUS) {
  leftUS  = clampUS(leftUS);
  rightUS = clampUS(rightUS);

  if (reverseRight) rightUS = 3000 - rightUS;

  leftMotor.writeMicroseconds(leftUS);
  rightMotor.writeMicroseconds(rightUS);
}

void armControllers() {
  setMotorsUS(STOP_US, STOP_US);
  delay(2000);
  for (int i = 0; i < 20; i++) {
    setMotorsUS(STOP_US, STOP_US);
    delay(20);
  }
}

void stopMotorsSafe() {
  setMotorsUS(STOP_US, STOP_US);
}

// ---------------- Control tuning ----------------
float ROLL_OFFSET_DEG = 0.0f;   // set so roll is ~0 when upright

float Kp = 20.0f;
float Ki = 0.0f;
float Kd = 0.8f;

int MAX_CMD_US = 250;

float TILT_CUTOFF_DEG = 35.0f;

float ENABLE_WINDOW_DEG = 10.0f;
uint32_t ENABLE_HOLD_MS = 800;

int steeringUS = 0; // keep 0 for now

float integral = 0.0f;
float lastError = 0.0f;
uint32_t lastMicros = 0;

bool controlEnabled = false;
uint32_t enableStartMs = 0;

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Read one DMP packet; returns true if updated values
bool readDMP(float &yawDeg, float &pitchDeg, float &rollDeg) {
  if (!dmpReady) return false;

  if (!mpuInterrupt) {
    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) return false;
  }
  mpuInterrupt = false;

  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    return false;
  }

  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  yawDeg   = ypr[0] * 180.0f / M_PI;
  pitchDeg = ypr[1] * 180.0f / M_PI;
  rollDeg  = ypr[2] * 180.0f / M_PI;
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  leftMotor.attach(LEFT_PIN, MIN_US, MAX_US);
  rightMotor.attach(RIGHT_PIN, MIN_US, MAX_US);
  armControllers();
  stopMotorsSafe();

  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU OK") : F("MPU FAIL"));

  // Your offsets
  mpu.setXAccelOffset(1015);
  mpu.setYAccelOffset(489);
  mpu.setZAccelOffset(1091);
  mpu.setXGyroOffset(113);
  mpu.setYGyroOffset(-48);
  mpu.setZGyroOffset(-5);

  devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    Serial.print(F("DMP init failed: "));
    Serial.println(devStatus);
    return;
  }

  mpu.setDMPEnabled(true);

  pinMode(MPU_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);

  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady = true;

  lastMicros = micros();

  Serial.println(F("Self-balance (ROLL) starting."));
  Serial.println(F("Tune Kp/Kd and set ROLL_OFFSET_DEG."));
}

void loop() {
  float yawDeg, pitchDeg, rollDeg;
  if (!readDMP(yawDeg, pitchDeg, rollDeg)) return;

  float roll = rollDeg - ROLL_OFFSET_DEG;  // balance angle is roll

  // Hard cutoff if fallen
  if (fabs(roll) > TILT_CUTOFF_DEG) {
    controlEnabled = false;
    integral = 0;
    stopMotorsSafe();
    return;
  }

  // Enable only after staying near upright
  if (!controlEnabled) {
    if (fabs(roll) < ENABLE_WINDOW_DEG) {
      if (enableStartMs == 0) enableStartMs = millis();
      if (millis() - enableStartMs > ENABLE_HOLD_MS) {
        controlEnabled = true;
        integral = 0;
        lastError = roll;
        lastMicros = micros();
      }
    } else {
      enableStartMs = 0;
    }
    stopMotorsSafe();
    return;
  }

  // Control timing
  uint32_t now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;
  lastMicros = now;

  // We want roll -> 0
  float error = roll;

  integral += error * dt;
  integral = clampf(integral, -50.0f, 50.0f);

  float derivative = (error - lastError) / dt;
  lastError = error;

  float u = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Convert to command (microseconds around STOP_US)
  int cmd = (int)(-u);

  if (cmd >  MAX_CMD_US) cmd =  MAX_CMD_US;
  if (cmd < -MAX_CMD_US) cmd = -MAX_CMD_US;

  int leftUS  = STOP_US + cmd + steeringUS;
  int rightUS = STOP_US + cmd - steeringUS;

  setMotorsUS(leftUS, rightUS);

  // Debug
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("roll=")); Serial.print(roll, 2);
    Serial.print(F("\tcmd=")); Serial.print(cmd);
    Serial.print(F("\ten=")); Serial.println(controlEnabled ? 1 : 0);
  }
}