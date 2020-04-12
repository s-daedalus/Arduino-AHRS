# Arduino-AHRS
Attitude and heading reference system for XCSoar using an Arduino Nano 33 Ble
# Dependencies
This project uses:
 - ArduinoBLE library (https://github.com/arduino-libraries/ArduinoBLE)
 - The Arduino port (https://github.com/arduino-libraries/MadgwickAHRS) of Sebastian Madgwicks AHRS algorithm (https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
 - and the Arduino library for the LSM9DS1 MIMU (https://github.com/arduino-libraries/Arduino_LSM9DS1).
# Usage
Flash your Arduino Nano 33 Ble with this program, connect XCSoar via Bluetooth LE to the device "Arduino-AHRS" choose the Levil AHRS driver and you are done!
