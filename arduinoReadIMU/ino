// Uduino library code based from https://marcteyssier.com/uduino/projects/connect-a-imu-to-unity

#include "Uduino.h"
Uduino uduino("IMU");

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld; // world-scene accel 
VectorFloat gravity;
float euler[3];
float ypr[3]; // yaw pitch roll

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(54);
  mpu.setYGyroOffset(-21);
  mpu.setZGyroOffset(5);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("err");
  }
}

void loop() {
  uduino.update();

  if (uduino.isInit()) {
    if (!dmpReady) {
      Serial.println("Ino imu connected");
      delay(100);
      return;
    }

    int  mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      SendQuaternion();
      SendEuler();
      //SendYawPitchRoll();  // rotation isnt needed if multiple if tracking position
      //SendRealAccel();
      SendWorldAccel();
    }
  }
}

void SendQuaternion() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  Serial.print("r/");
  Serial.print(q.w, 4); Serial.print("/");
  Serial.print(q.x, 4); Serial.print("/");
  Serial.print(q.y, 4); Serial.print("/");
  Serial.println(q.z, 4);
}

void SendEuler() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  Serial.print(euler[0] * 180 / M_PI); Serial.print("/");
  Serial.print(euler[1] * 180 / M_PI); Serial.print("/");
  Serial.println(euler[2] * 180 / M_PI);
}

void SendYawPitchRoll() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print(ypr[0] * 180 / M_PI); Serial.print("/");
  Serial.print(ypr[1] * 180 / M_PI); Serial.print("/");
  Serial.println(ypr[2] * 180 / M_PI);
}

void SendRealAccel() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  Serial.print("a/");
  Serial.print(aaReal.x); Serial.print("/");
  Serial.print(aaReal.y); Serial.print("/");
  Serial.println(aaReal.z);
}

void SendWorldAccel() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  Serial.print("a/");
  Serial.print(aaWorld.x); Serial.print("/");
  Serial.print(aaWorld.y); Serial.print("/");
  Serial.println(aaWorld.z);
}
