#include <ESP32_Servo.h>

//#include <Servo.h>
#include <PID_v1_bc.h>
#include <Kalman.h>
#include <Wire.h>

#define RESTRICT_PITCH 
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

Servo servo1;
Servo servo2;
Servo servo3;

double gyroXangle, gyroYangle, gyroZangle; // Angle calculated using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// PID data X
double SetpointX, InputX, OutputX;
double KpX = 1.05, KiX = 0, KdX = 0;
PID PIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);

// PID data Y
double SetpointY, InputY, OutputY;
double KpY = 1.2, KiY = 0, KdY = 0;
PID PIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

// PID data Z
double SetpointZ, InputZ, OutputZ;
double KpZ = 1.2, KiZ = 0, KdZ = 0;
PID PIDZ(&InputZ, &OutputZ, &SetpointZ, KpZ, KiZ, KdZ, DIRECT);

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
  servo1.attach(8);
  servo2.attach(9);
  servo3.attach(10);

#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to 250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to 2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);


#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw = atan2(accX, accY) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double yaw = atan(accZ / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;
  timer = micros();

  // The desired value
  SetpointX = 90;
  SetpointY = 90;
  SetpointZ = 70;

  servo1.write(0);
  servo2.write(0);
  servo3.write(0);

  // Turn the PID
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);
  PIDZ.SetMode(AUTOMATIC);
  PIDX.SetSampleTime(10);
  PIDY.SetSampleTime(10);
  PIDZ.SetSampleTime(10);
}

void loop() {
  // Read all the values from the MPU6050 sensor, calculates the angles and filters them
  mpuKalman();

  // The desired value
  SetpointX = 90;
  SetpointY = 90;
  SetpointZ = 70;

  InputX = kalAngleX;
  InputY = kalAngleY;
  InputZ = kalAngleZ;

  if (InputX < -90.0) {
    InputX = -90.0;
  } else if (InputX > 90.0) {
    InputX = 90.0;
  }

  if (InputY < -90.0) {
    InputY = -70.0;
  } else if (InputY > 90.0) {
    InputY = 70.0;
  }

  if (InputZ < -90.0) {
    InputZ = -70.0;
  } else if (InputZ > 90.0) {
    InputZ = 70.0;
  }

  // Compute the Output
  PIDX.Compute();
  PIDY.Compute();
  PIDZ.Compute();

  servo1.write(OutputY);
  servo2.write(OutputX);
  servo3.write(OutputZ);

  Serial.print("s1: ");
  Serial.print(OutputX);
  Serial.print("  s2");
  Serial.print(OutputY);
  Serial.print("  s3");
  Serial.print(OutputZ);
  Serial.print("\t");



  /* Print Data */

  String data = "AccX:" + String(accX) + ",";
  data += "AccY:" + String(accY) + ",";
  data += "AccZ:" + String(accZ) + ",";
  data += "GyroX:" + String(gyroX) + ",";
  data += "GyroY:" + String(gyroY) + ",";
  data += "GyroZ:" + String(gyroZ) + ",";

  // Send sensor data over serial
  //Serial.println(data);
}

void mpuKalman() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH 
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw = atan2(accX, accY) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double yaw = atan(accZ / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
#else
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
#endif

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  // compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  // compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  // compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Print Data
  /*Serial.print("X: ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(kalAngleX);
  Serial.print("\t");
  Serial.print(compAngleX);
  Serial.print("\t");
  Serial.print(gyroXangle);
  Serial.print("\tY: ");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(kalAngleY);
  Serial.print("\t");
  Serial.print(compAngleY);
  Serial.print("\t");
  Serial.print(gyroYangle);
  Serial.print("\tZ: ");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(kalAngleZ);
  Serial.print("\t");
  Serial.print(compAngleZ);
  Serial.print("\t");
  Serial.print(gyroZangle);
  Serial.print("\t");
  Serial.print("\n");*/
}

// I2C read/write functions
int i2cRead(uint8_t registerAddr, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(0x68);
  Wire.write(registerAddr);
  if (Wire.endTransmission(false))
    return 1;
  Wire.requestFrom(0x68, length);
  for (uint8_t i = 0; i < length && Wire.available(); i++) {
    data[i] = Wire.read();
  }
  return 0;
}

int i2cWrite(uint8_t registerAddr, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(0x68);
  Wire.write(registerAddr);
  Wire.write(data, length);
  return Wire.endTransmission(sendStop);
}

int i2cWrite(uint8_t registerAddr, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddr, &data, 1, sendStop);
}