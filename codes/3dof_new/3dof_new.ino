#include <MPU6050.h>

//#include <ESP32_Servo.h>
#include <Servo.h>
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



double gyroXangle, gyroYangle,gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY,compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY,kalAngleZ; // Calculated angle using a Kalman filter


uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

//PID data X
double SetpointX ,InputX,OutputX ;
double KpX =1, KiX = 0, KdX = 0.0000005;
PID PIDX(&InputX , &OutputX , &SetpointX , KpX, KiX , KdX, DIRECT) ;

// PID data Y
double SetpointY ,InputY,OutputY ;
double KpY = 1, KiY = 0 , KdY = 0.0000005;
PID PIDY(&InputY , &OutputY , &SetpointY , KpY, KiY , KdY, DIRECT) ;

// PID data Z
double SetpointZ, InputZ, OutputZ;
double KpZ = 1, KiZ = 0, KdZ = 0;
PID PIDZ(&InputZ, &OutputZ, &SetpointZ, KpZ, KiZ, KdZ, DIRECT);

// TODO: Make calibration routine

void setup() {
  Serial.begin(9600);
  Wire.begin();
  servo1.attach(9);
  servo2.attach(8);
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
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw = atan2(accX, accY) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
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
  SetpointZ = 90;


  servo1.write(0);
  servo2.write(0);
  servo3.write(0);


  //Turn the PID
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
  SetpointZ = 90;

 
  InputX = kalAngleX;
  InputY = kalAngleY;
  InputZ = kalAngleZ;


  if(InputX < -90.0 ) {
    InputX = -90.0 ;
  } else if (InputX > 90.0) {
    InputX = 90.0 ;
  }

  if(InputY < -90.0 ) {
    InputY = -90.0 ;
  } else if (InputY > 90.0) {
    InputY = 90.0 ;
  }
  if (InputZ < -90.0) {
    InputZ = -90.0;
  } else if (InputZ > 90.0) {
    InputZ = 90.0;
  }

  //Compute the Output
  PIDX.Compute();
  PIDY.Compute();
  PIDZ.Compute();

  servo1.write(OutputY);
  servo2.write(OutputX);
  servo3.write(OutputZ);



  /* Print Data */

  Serial.print(InputX); Serial.print("\t");
  Serial.print(InputY); Serial.print("\t");
  Serial.print(InputZ); Serial.print("\t");
  //Serial.print(SetpointY); Serial.print("\t");
  Serial.print(OutputX); Serial.print("\t");
  Serial.println(OutputY); Serial.print("\t");
  Serial.println(OutputZ); Serial.print("\t");


}

// Read all the values from the MPU6050 sensor, calculates the angles and filters them
void mpuKalman() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH 
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw = atan2(accX, accY) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double yaw = atan(accZ / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;

  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  //kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  //kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
#endif
//kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);

#ifdef RESTRICT_ROLL
  // Handle roll when RESTRICT_PITCH is defined
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  }
  
  if (abs(kalAngleX) > 90) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  //kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);

  // Handle yaw if needed
  // Uncomment and implement if yaw handling is required here
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
     kalmanZ.setAngle(yaw);
     compAngleZ = yaw;
     kalAngleZ = yaw;
     gyroZangle = yaw;
   } else {
     kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter
   }
  
  if (abs(kalAngleZ) > 90) {
     gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  }
#else
  // Handle pitch when RESTRICT_PITCH is not defined
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else {
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  }

  if (abs(kalAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  }
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);

  // Handle yaw if needed
  // Uncomment and implement if yaw handling is required here
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
     kalmanZ.setAngle(yaw);
     compAngleZ = yaw;
     kalAngleZ = yaw;
     gyroZangle = yaw;
  } else {
     kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter
  }
  
  if (abs(kalAngleZ) > 90) {
     gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
 }
#endif



  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;

}




















const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}
 
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; 
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
