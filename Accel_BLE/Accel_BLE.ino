#include <ArduinoBLE.h>

/* The ArduinoLSM9DS1 library takes care of the sensor initialization and sets its values as follows:
- Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
- Gyroscope range is set at ±2000 dps (degrees per second) with a resolution of 70 mdps.
- Magnetometer range is set at ±400 uT (microTesla) with a resolution of 0.014 uT.
- Accelerometer and gyrospcope output data rate is fixed at 119 Hz.
- Magnetometer output data rate is fixed at 20 Hz. 
- acceleration in g (earth gravity), angular speed from gyro in dps (degrees per second), magnetic field in uT (micro Tesla) */
#include <Arduino_LSM9DS1.h>

#define DEBUGBLE    // send debug information on serial
#define VIEWER3D    // send Euler angles and/or quaternion information on serial for Adafruit 3D Model Viewer

BLEService accelService("19b10000-e8f2-537e-4f6c-d104768a1214");

BLEFloatCharacteristic xCharacteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLENotify);
BLEFloatCharacteristic yCharacteristic("19b10002-e8f2-537e-4f6c-d104768a1214", BLENotify);
BLEFloatCharacteristic zCharacteristic("19b10003-e8f2-537e-4f6c-d104768a1214", BLENotify);

BLEBoolCharacteristic ledCharacteristic("19b10004-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);

BLEFloatCharacteristic xgCharacteristic("19b10005-e8f2-537e-4f6c-d104768a1214", BLENotify);
BLEFloatCharacteristic ygCharacteristic("19b10006-e8f2-537e-4f6c-d104768a1214", BLENotify);
BLEFloatCharacteristic zgCharacteristic("19b10007-e8f2-537e-4f6c-d104768a1214", BLENotify);

BLEFloatCharacteristic xmCharacteristic("19b10008-e8f2-537e-4f6c-d104768a1214", BLENotify);
BLEFloatCharacteristic ymCharacteristic("19b10009-e8f2-537e-4f6c-d104768a1214", BLENotify);
BLEFloatCharacteristic zmCharacteristic("19b10010-e8f2-537e-4f6c-d104768a1214", BLENotify);

BLEDevice central;

// Notes from kriswiner LSM9DS1_BasicAHRS_Nano33.ino :
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float deltat = 0.0f;          // integration interval for filter scheme

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
//float q[4] = {0.7545, 0.2937, 0.5858, -0.0356};

void setup(){

  //Serial.begin(9600);
  Serial.begin(115200); // 115200 speed needed to send to Adafruit_WebSerial_3DModelViewer
  while (!Serial);
  #ifdef DEBUGBLE
    Serial.println("Started");
  #endif
  pinMode(13,OUTPUT);

  if(!BLE.begin()){
    Serial.println("Starting BLE failed!");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  BLE.setLocalName("Accelerometer");
  BLE.setAdvertisedService(accelService);

  accelService.addCharacteristic(xCharacteristic);
  accelService.addCharacteristic(yCharacteristic);
  accelService.addCharacteristic(zCharacteristic);
  
  accelService.addCharacteristic(ledCharacteristic);

  accelService.addCharacteristic(xgCharacteristic);
  accelService.addCharacteristic(ygCharacteristic);
  accelService.addCharacteristic(zgCharacteristic);

  accelService.addCharacteristic(xmCharacteristic);
  accelService.addCharacteristic(ymCharacteristic);
  accelService.addCharacteristic(zmCharacteristic);

  BLE.addService(accelService);

  ledCharacteristic.writeValue(0);

  BLE.setEventHandler(BLEConnected, connectHandler);
  BLE.setEventHandler(BLEDisconnected, disconnectHandler);

  BLE.advertise();

  //delay(2000);    // 2 second
  #ifdef DEBUGBLE
    Serial.println("IMU begin");

    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.print("Magnetometer sample rate = ");
    Serial.print(IMU.magneticFieldSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in g - Gyroscope in degrees/second");
    Serial.println("\taX\taY\taZ\tgX\tgY\tgZ\tmX\tmY\tmZ");
    Serial.print("beta: ");
    Serial.println(beta, 4);
  #endif  
}

unsigned long prevTime = 0;

void loop(){

  BLE.poll();

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }

/* Notes from kriswiner LSM9DS1_BasicAHRS_Nano33.ino :
  Sensors x, y, and z axes of the accelerometer and gyro are aligned. The magnetometer  
  the magnetometer z-axis (+ up) is aligned with the z-axis (+ up) of accelerometer and gyro, but the magnetometer
  x-axis is aligned with the -x axis of the gyro and the magnetometer y axis is aligned with the y axis of the gyro!
  We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  For the LSM9DS1, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  This is ok by aircraft orientation standards!  
  Pass gyro rate as rad/s */
  /* kriswiner use mx, my, mz in Gauss, when Arduino_LSM9DS1 library provide magnetometer data in µT 
     1 Gauss = 100µT */
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx/100.0, my/100.0, mz/100.0);

  unsigned long t = millis();

  if(central.connected()){
    if(t - prevTime > 500){       // every 500ms
      xCharacteristic.writeValue(ax);
      yCharacteristic.writeValue(ay);
      zCharacteristic.writeValue(az);
      xgCharacteristic.writeValue(gx);
      ygCharacteristic.writeValue(gy);
      zgCharacteristic.writeValue(gz);
      xmCharacteristic.writeValue(mx);
      ymCharacteristic.writeValue(my);
      zmCharacteristic.writeValue(mz);
      prevTime = t;
    }
  }
  else{
    if(t - prevTime > 100){     // every 100ms
      central = BLE.central();
      #ifdef DEBUGBLE
        Serial.print('\t');
        Serial.print(ax);
        Serial.print('\t');
        Serial.print(ay);
        Serial.print('\t');
        Serial.print(az);
        Serial.print('\t');
        Serial.print(gx);
        Serial.print('\t');
        Serial.print(gy);
        Serial.print('\t');
        Serial.print(gz);
        Serial.print('\t');
        Serial.print(mx);
        Serial.print('\t');
        Serial.print(my);
        Serial.print('\t');
        Serial.println(mz);
      #endif
      #ifdef VIEWER3D
      // something like: Orientation: 180.82 -1.65 2.48
      // and / or: Quaternion: 0.7545, 0.2937, 0.5858, -0.0356
        Serial.print("Quaternion: ");
        Serial.print(q[0], 4);
        Serial.print(", ");
        Serial.print(q[1], 4);
        Serial.print(", ");
        Serial.print(q[2], 4);
        Serial.print(", ");
        Serial.println(q[3], 4);
      #endif
      prevTime = t;
    }
  }

  if(ledCharacteristic.written()){
    if(ledCharacteristic.value()){
      digitalWrite(13,HIGH);
      #ifdef DEBUGBLE
        Serial.println("LED On");
      #endif
    }
    else{
      digitalWrite(13,LOW);
      #ifdef DEBUGBLE
        Serial.println("LED Off");
      #endif
    }
  }
}

void connectHandler(BLEDevice central){
  #ifdef DEBUGBLE
    Serial.print("Connected to: ");
    Serial.println(central.address());
  #endif
}

void disconnectHandler(BLEDevice central){
  #ifdef DEBUGBLE
    Serial.print("Disconnected from: ");
    Serial.println(central.address());
  #endif
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  #ifdef DEBUGBLE
    Serial.print("norm a: ");
    Serial.println(norm);
  #endif
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  #ifdef DEBUGBLE
    Serial.print("norm m: ");
    Serial.println(norm);
  #endif
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  #ifdef DEBUGBLE
    Serial.print("norm sm: ");
    Serial.println(norm);
  #endif
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  #ifdef DEBUGBLE
    Serial.print("norm q: ");
    Serial.println(norm);
  #endif
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
