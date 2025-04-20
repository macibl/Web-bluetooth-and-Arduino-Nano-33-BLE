# Web Bluetooth and Arduino Nano 33 BLE
Use of a web app to connect through Bluetooth Low Energy to Arduino Nano 33 BLE, display IMU data and control internal Led

Based on following repo : https://github.com/ErniW/Web-bluetooth-and-Arduino-Nano-33-BLE to use BLE correctly
Also use work of this repo : https://github.com/kriswiner/LSM9DS1 to calculate quaternon

### How to use it:
- compile/Upload Accel_BLE.ino on Arduino Nano 33 BLE module
- open btwebtest.html in Chrome
- Connect button, a pop-up will display "Accelerometer", select it then Associate button
- data from IMU accelerometer, gyroscope, magnetometer will be refreshed 
- Toggle led button turn internal led on / off

### TODO
- add disconnect button
- add quaternon calculus