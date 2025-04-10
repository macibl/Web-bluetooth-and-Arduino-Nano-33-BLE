#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

//#define DEBUGBLE

BLEService accelService("19b10000-e8f2-537e-4f6c-d104768a1214");

BLEFloatCharacteristic xCharacteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLENotify);
BLEFloatCharacteristic yCharacteristic("19b10002-e8f2-537e-4f6c-d104768a1214", BLENotify);
BLEFloatCharacteristic zCharacteristic("19b10003-e8f2-537e-4f6c-d104768a1214", BLENotify);

BLEBoolCharacteristic ledCharacteristic("19b10004-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);

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

  BLE.addService(accelService);

  ledCharacteristic.writeValue(0);

  BLE.setEventHandler(BLEConnected, connectHandler);
  BLE.setEventHandler(BLEDisconnected, disconnectHandler);

  BLE.advertise();

  //delay(2000);    // 2 second
  Serial.println("Accel begin");

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g - Gyroscope in degrees/second");
  Serial.println("X\tY\tZ\tX\tY\tZ");
}

unsigned long prevTime = 0;

void loop(){

  BLE.poll();

  float x, y, z;
  float xg, yg, zg;

  IMU.readAcceleration(x, y, z);

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(xg, yg, zg);
  }

  unsigned long t = millis();

  if(central.connected()){
    if(t - prevTime > 100){
      xCharacteristic.writeValue(x);
      yCharacteristic.writeValue(y);
      zCharacteristic.writeValue(z);
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
        Serial.println(zg);
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
