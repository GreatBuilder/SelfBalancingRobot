#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

const int MPU_INT_PIN = 3;   // <-- INT is on D3 now

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

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

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

  Serial.println(F("Yaw\tPitch\tRoll"));
}

void loop() {
  if (!dmpReady) return;

  if (!mpuInterrupt) {
    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) return;
  }
  mpuInterrupt = false;

  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    return;
  }

  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0] * 180.0 / M_PI;
  float pitch = ypr[1] * 180.0 / M_PI;
  float roll  = ypr[2] * 180.0 / M_PI;

  Serial.print(yaw, 2);   Serial.print('\t');
  Serial.print(pitch, 2); Serial.print('\t');
  Serial.println(roll, 2);
}