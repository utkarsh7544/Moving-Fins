#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servoX1, servoX2, servoZ1, servoZ2;

int16_t ax, ay, az, gx, gy, gz;
float angleX, angleZ;
float kp = 1.0, ki = 0.05, kd = 0.01;
float dt = 0.02; // Loop time in seconds
float alpha = 0.98; // Complementary filter constant

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
 
  servoX1.attach(9);
  servoX2.attach(6);
  servoZ1.attach(3);
  servoZ2.attach(5);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyro values to angles
  float gyroAngleX = gx / 131.0 * dt;
  float gyroAngleZ = gz / 131.0 * dt;

  // Convert accelerometer values to angles
  float accelAngleX = atan2(ay, az) * 180 / PI;
  float accelAngleZ = atan2(ay, az) * 180 / PI;

  // Complementary filter to combine gyro and accelerometer values
  angleX = alpha * (angleX + gyroAngleX) + (1 - alpha) * accelAngleX;
  angleZ = alpha * (angleZ + gyroAngleZ) + (1 - alpha) * accelAngleZ;

  // Map angles to servo positions
  int servoPosX = map(angleX, -90, 90, 0, 180);
  int servoPosZ = map(angleZ, -90, 90, 0, 180);

  // Control servos for X-axis
  servoX1.write(servoPosX);
  servoX2.write(servoPosX);

  // Control servos for Z-axis
  if (gz == 0) {
    servoZ1.write(90);
    servoZ2.write(90);
  } else {
    servoZ1.write(servoPosZ);
    servoZ2.write(servoPosZ);
  }

  delay(0.000000000001);
}
