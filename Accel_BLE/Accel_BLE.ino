#include <ArduinoBLE.h>

/* The ArduinoLSM9DS1 library takes care of the sensor initialization and sets its values as follows:
- Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
- Gyroscope range is set at ±2000 dps (degrees per second) with a resolution of 70 mdps.
- Magnetometer range is set at ±400 uT (microTesla) with a resolution of 0.014 uT.
- Accelerometer and gyrospcope output data rate is fixed at 119 Hz.
- Magnetometer output data rate is fixed at 20 Hz. */
#include <Arduino_LSM9DS1.h>

#define DEBUGBLE

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

void setup(){

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

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
  Serial.println("Xa\tYa\tZa\tXg\tYg\tZg\tXm\tYm\tZm");
}

unsigned long prevTime = 0;

void loop(){

  BLE.poll();

  float x, y, z;
  float xg, yg, zg;
  float xm, ym, zm;
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(xg, yg, zg);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(xm, ym, zm);
  }

  unsigned long t = millis();

  if(central.connected()){
    if(t - prevTime > 500){
      xCharacteristic.writeValue(x);
      yCharacteristic.writeValue(y);
      zCharacteristic.writeValue(z);
      xgCharacteristic.writeValue(xg);
      ygCharacteristic.writeValue(yg);
      zgCharacteristic.writeValue(zg);
      xmCharacteristic.writeValue(xm);
      ymCharacteristic.writeValue(ym);
      zmCharacteristic.writeValue(zm);
      prevTime = t;
    }
  }
  else{
    if(t - prevTime > 100){
      central = BLE.central();
      #ifdef DEBUGBLE
        Serial.print('\t');
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(z);
        Serial.print('\t');
        Serial.print(xg);
        Serial.print('\t');
        Serial.print(yg);
        Serial.print('\t');
        Serial.print(zg);
        Serial.print('\t');
        Serial.print(xm);
        Serial.print('\t');
        Serial.print(ym);
        Serial.print('\t');
        Serial.println(zm);
      #endif
      prevTime = t;
    }
  }

  if(ledCharacteristic.written()){
    if(ledCharacteristic.value()){
      digitalWrite(13,HIGH);
      Serial.println("LED On");
    }
    else{
      digitalWrite(13,LOW);
      Serial.println("LED Off");
    }
  }
}

void connectHandler(BLEDevice central){
  Serial.print("Connected to: ");
  Serial.println(central.address());
}

void disconnectHandler(BLEDevice central){
  Serial.print("Disconnected from: ");
  Serial.println(central.address());
}

