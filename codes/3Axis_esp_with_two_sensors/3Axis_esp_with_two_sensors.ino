#include <ESP32_Servo.h>

//#include <Servo.h>
#include <PID_v1_bc.h>
#include <Kalman.h>
#include <Wire.h>

#define RESTRICT_PITCH1
#define RESTRICT_PITCH2

Kalman kalmanX1, kalmanY1; // Kalman instances for the first sensor
Kalman kalmanX2, kalmanY2; // Kalman instances for the second sensor

/* IMU Data */
double accX1, accY1, accZ1;
double gyroX1, gyroY1, gyroZ1;

double accX2, accY2, accZ2;
double gyroX2, gyroY2, gyroZ2;

Servo servo1;
Servo servo2;
Servo servo3;

double gyroXangle1, gyroYangle1; // Angle calculate using the gyro only for sensor 1
double compAngleX1, compAngleY1; // Calculated angle using a complementary filter for sensor 1
double kalAngleX1, kalAngleY1; // Calculated angle using a Kalman filter for sensor 1

double gyroXangle2, gyroYangle2; // Angle calculate using the gyro only for sensor 2
double compAngleX2, compAngleY2; // Calculated angle using a complementary filter for sensor 2
double kalAngleX2, kalAngleY2; // Calculated angle using a Kalman filter for sensor 2

uint32_t timer1, timer2;
uint8_t i2cData[14]; // Buffer for I2C data

// PID data for sensor 1
double SetpointX1, InputX1, OutputX1;
double KpX1 = 1, KiX1 = 0.00000005, KdX1 = 0.0000005;
PID PIDX1(&InputX1, &OutputX1, &SetpointX1, KpX1, KiX1, KdX1, DIRECT);

// PID data for sensor 1
double SetpointY1, InputY1, OutputY1;
double KpY1 = 1, KiY1 = 0.00000005, KdY1 = 0.0000005;
PID PIDY1(&InputY1, &OutputY1, &SetpointY1, KpY1, KiY1, KdY1, DIRECT);

// PID data for sensor 2
double SetpointX2, InputX2, OutputX2;
double KpX2 = 1, KiX2 = 0.00000005, KdX2 = 0.0000005;
PID PIDX2(&InputX2, &OutputX2, &SetpointX2, KpX2, KiX2, KdX2, DIRECT);

const uint8_t IMUAddress1 = 0x68; // Address for the first MPU6050
const uint8_t IMUAddress2 = 0x69; // Address for the second MPU6050
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

void setup() {
  Serial.begin(9600);
  Wire.begin();
  servo1.attach(17);
  servo2.attach(18);
  servo3.attach(19);

#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  // Initialize the first MPU6050
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to 250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to 2g
  while (i2cWrite(IMUAddress1, 0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(IMUAddress1, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(IMUAddress1, 0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor 1"));
    while (1);
  }

  // Initialize the second MPU6050
  while (i2cWrite(IMUAddress2, 0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(IMUAddress2, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(IMUAddress2, 0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor 2"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  // Set kalman and gyro starting angle for the first sensor
  while (i2cRead(IMUAddress1, 0x3B, i2cData, 6));
  accX1 = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY1 = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ1 = (int16_t)((i2cData[4] << 8) | i2cData[5]);

#ifdef RESTRICT_PITCH1 // Eq. 25 and 26
  double roll1 = atan2(accY1, accZ1) * RAD_TO_DEG;
  double pitch1 = atan(-accX1 / sqrt(accY1 * accY1 + accZ1 * accZ1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll1 = atan(accY1 / sqrt(accX1 * accX1 + accZ1 * accZ1)) * RAD_TO_DEG;
  double pitch1 = atan2(-accX1, accZ1) * RAD_TO_DEG;
#endif

  kalmanX1.setAngle(roll1); // Set starting angle
  kalmanY1.setAngle(pitch1);
  gyroXangle1 = roll1;
  gyroYangle1 = pitch1;
  compAngleX1 = roll1;
  compAngleY1 = pitch1;
  timer1 = micros();

  // Set kalman and gyro starting angle for the second sensor
  while (i2cRead(IMUAddress2, 0x3B, i2cData, 6));
  accX2 = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY2 = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ2 = (int16_t)((i2cData[4] << 8) | i2cData[5]);

#ifdef RESTRICT_PITCH2 // Eq. 25 and 26
  double roll2 = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2 = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  kalmanX2.setAngle(roll2); // Set starting angle
  kalmanY2.setAngle(pitch2);
  gyroXangle2 = roll2;
  gyroYangle2 = pitch2;
  compAngleX2 = roll2;
  compAngleY2 = pitch2;
  timer2 = micros();

  // The desired value
  SetpointX1 = 90;
  SetpointY1 = 90;

  SetpointX2 = 90;

  servo1.write(0);
  servo2.write(0);
  servo3.write(0);

  // Turn the PID
  PIDX1.SetMode(AUTOMATIC);
  PIDY1.SetMode(AUTOMATIC);
  PIDX2.SetMode(AUTOMATIC);

  PIDX1.SetSampleTime(1);
  PIDY1.SetSampleTime(1);
  PIDX2.SetSampleTime(1);

  PIDX1.SetOutputLimits(0, 180);
  PIDY1.SetOutputLimits(0, 180);
  PIDX2.SetOutputLimits(0, 180);

  delay(1000);
}

void loop() {
  // Read all the values from the MPU6050 sensor, calculates the angles and filters them
  mpuKalman1();
  mpuKalman2();


   // The desired value
  SetpointX1 = 80;
  SetpointY1 = 90;
  SetpointX2 = 90;

 
  InputX1 = kalAngleX1;
  InputY1 = kalAngleY1;
  InputX2 = kalAngleX2;

  if(InputX1 < -90.0 ) {
    InputX1 = -90.0 ;
  } else if (InputX1 > 90.0) {
    InputX1 = 90.0 ;
  }

  if(InputY1 < -90.0 ) {
    InputY1 = -90.0 ;
  } else if (InputY1 > 90.0) {
    InputY1 = 90.0 ;
  }
  if(InputX2 < -90.0 ) {
    InputX2 = -90.0 ;
  } else if (InputX2 > 90.0) {
    InputX2 = 90.0 ;
  }

  //Compute the Output
  PIDX1.Compute();
  PIDY1.Compute();
  PIDX2.Compute();

  servo1.write(OutputY1);
  servo2.write(OutputX1);
  servo3.write(OutputX2);



  /* Print Data */

  Serial.print(InputX1); 
  Serial.print("\t");
  Serial.print(OutputX1); 
  Serial.print("\t");
  Serial.print(InputY1);
  Serial.print("\t");
  Serial.print(SetpointY1);
  Serial.print("\t");
  Serial.print(InputX2); 
  Serial.print("\t");
  Serial.print(OutputX2); 
  Serial.print("\t");


}

void mpuKalman1() {
  /* Update all the values */
  uint32_t startTime = millis();
  while (i2cRead(IMUAddress1, 0x3B, i2cData, 14)) {
    if (millis() - startTime > I2C_TIMEOUT) {
      Serial.println(F("mpuKalman1: I2C read timeout"));
      return; // Exit the function if the read times out
    }
  }
  accX1 = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY1 = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ1 = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX1 = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY1 = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ1 = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  double dt = (double)(micros() - timer1) / 1000000; // Calculate delta time
  timer1 = micros();

  
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH1 
  double roll1  = atan2(accY1, accZ1) * RAD_TO_DEG;
  double pitch1 = atan(-accX1 / sqrt(accY1 * accY1 + accZ1 * accZ1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll1  = atan(accY1 / sqrt(accX1 * accX1 + accZ1 * accZ1)) * RAD_TO_DEG;
  double pitch1 = atan2(-accX1, accZ1) * RAD_TO_DEG;
#endif

  double gyroXrate1 = gyroX1 / 131.0; // Convert to deg/s
  double gyroYrate1 = gyroY1 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll1 < -90 && kalAngleX1 > 90) || (roll1 > 90 && kalAngleX1 < -90)) {
    kalmanX1.setAngle(roll1);
    compAngleX1 = roll1;
    kalAngleX1 = roll1;
    gyroXangle1 = roll1;
  } else
    kalAngleX1 = kalmanX1.getAngle(roll1, gyroXrate1, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX1) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY1 = kalmanY1.getAngle(pitch1, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch1 < -90 && kalAngleY1 > 90) || (pitch1 > 90 && kalAngleY1 < -90)) {
    kalmanY1.setAngle(pitch1);
    compAngleY1 = pitch1;
    kalAngleY1 = pitch1;
    gyroYangle1 = pitch1;
  } else
    kalAngleY1 = kalmanY1.getAngle(pitch1, gyroYrate1, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY1) > 90)
    gyroXrate1 = -gyroXrate1; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX1 = kalmanX1.getAngle(roll1, gyroXrate1, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle1 += gyroXrate1 * dt; // Calculate gyro angle without any filter
  gyroYangle1 += gyroYrate1 * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX1 = 0.93 * (compAngleX1 + gyroXrate1 * dt) + 0.07 * roll1; // Calculate the angle using a Complimentary filter
  compAngleY1 = 0.93 * (compAngleY1 + gyroYrate1 * dt) + 0.07 * pitch1;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle1 < -180 || gyroXangle1 > 180)
    gyroXangle1 = kalAngleX1;
  if (gyroYangle1 < -180 || gyroYangle1 > 180)
    gyroYangle1 = kalAngleY1;

}

void mpuKalman2() {
  /* Update all the values */
  uint32_t startTime = millis();
  while (i2cRead(IMUAddress2, 0x3B, i2cData, 14)) {
    if (millis() - startTime > I2C_TIMEOUT) {
      Serial.println(F("mpuKalman2: I2C read timeout"));
      return; // Exit the function if the read times out
    }
  }
  accX2 = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY2 = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ2 = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX2 = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY2 = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ2 = (int16_t)((i2cData[12] << 8) | i2cData[13]);
  double dt = (double)(micros() - timer2) / 1000000; // Calculate delta time
  timer2 = micros();

  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH2
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  double gyroXrate2 = gyroX2 / 131.0; // Convert to deg/s
  double gyroYrate2 = gyroY2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH2
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroXangle2 = roll2;
  } else
    kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX2) > 90)
    gyroYrate2 = -gyroYrate2; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch2 < -90 && kalAngleY2 > 90) || (pitch2 > 90 && kalAngleY2 < -90)) {
    kalmanY2.setAngle(pitch2);
    compAngleY2 = pitch2;
    kalAngleY2 = pitch2;
    gyroYangle2 = pitch2;
  } else
    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY2) > 90)
    gyroXrate2 = -gyroXrate2; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle2 += gyroXrate2 * dt; // Calculate gyro angle without any filter
  gyroYangle2 += gyroYrate2 * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX2 = 0.93 * (compAngleX2 + gyroXrate2 * dt) + 0.07 * roll2; // Calculate the angle using a Complimentary filter
  compAngleY2 = 0.93 * (compAngleY2 + gyroYrate2 * dt) + 0.07 * pitch2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle2 < -180 || gyroXangle2 > 180)
    gyroXangle2 = kalAngleX2;
  if (gyroYangle2 < -180 || gyroYangle2 > 180)
    gyroYangle2 = kalAngleY2;
}


// i2cWrite function to accept IMUAddress
uint8_t i2cWrite(uint8_t IMUAddress, uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(IMUAddress, registerAddress, &data, 1, sendStop); // Returns 0 on success
}

// i2cWrite function to accept IMUAddress
uint8_t i2cWrite(uint8_t IMUAddress, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode != 0) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

// i2cRead function to accept IMUAddress
uint8_t i2cRead(uint8_t IMUAddress, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode != 0) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; 
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  uint32_t timeOutTimer = millis(); // Initialize the timeout timer
  for (uint8_t i = 0; i < nbytes; i++) {
    while (Wire.available() == 0) {
      if ((millis() - timeOutTimer) > I2C_TIMEOUT) {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
    data[i] = Wire.read();
  }
  return 0; // Success
}
