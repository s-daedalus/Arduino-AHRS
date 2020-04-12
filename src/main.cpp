#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include "BLESerial.h"
#include "MadgwickAHRS.h"
// time interval the characteristic gets updated with [ms]
#define BLE_REFRESH_TIME 125
// Sample rate for the filter update [hz]
#define AHRS_SAMPLE_RATE 60.0f
//#define WHDEBUG
unsigned long last_send_millis = 0;
unsigned long microsPerReading, microsPrevious, microsNow;
int roll, pitch, heading;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
char buff[64];

Madgwick filter;

BLEHelpers::BLESerial bs(BLE);

void setup() {
  // Set up rgb led and run test.
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  delay(250);
  digitalWrite(LEDR, HIGH);
  delay(250);
  digitalWrite(LEDG, HIGH);
  delay(250);
  digitalWrite(LEDB, HIGH);

  #ifdef WHDEBUG
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Start");
  #endif

  if (!IMU.begin()) {
    // light up red on error
    digitalWrite(LEDR, LOW);
    #ifdef WHDEBUG
    Serial.println("IMU init failed");
    #endif
    while (1);
  }
  if (!bs.begin("Arduino-AHRS")) {
    // light up red on error
    digitalWrite(LEDR, LOW);
    #ifdef WHDEBUG
    Serial.println("BLE init failed");
    #endif
    while (1);
  }
  
  filter.begin(AHRS_SAMPLE_RATE);

  microsPerReading = 1000000 / AHRS_SAMPLE_RATE;
  microsPrevious = micros();
  // enable blue led, signalling operational state
  digitalWrite(LEDB, LOW);
}

void loop() { 
  BLEDevice central = bs.hasClient();
  if (central) {
    #ifdef WHDEBUG
    Serial.println("Device connected");
    #endif
    if(central.connected()){
      // disable blue led when connected
      digitalWrite(LEDB, HIGH);
    }
    while (central.connected()) {
      microsNow = micros();
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
      }

      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
      }

      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
      }
      if (microsNow - microsPrevious >= microsPerReading) {
        filter.update(-gx, -gy, -gz, ax, ay, az, mx, my, mz);
        mx = 0;
        my = 0;
        mz = 0;

        if(millis() - last_send_millis > BLE_REFRESH_TIME){
          // flash grenn led when transmitting
          digitalWrite(LEDG, LOW);
          sprintf(buff, "$RPYL,%i,%i,%i,0,0,1,0\n", (int) filter.getRoll()*10, (int) filter.getPitch()*10,(int) filter.getYaw()*10);
          
          #ifdef WHDEBUG
          Serial.println(buff);
          #endif
          bs.write(buff, strlen(buff));
          last_send_millis = millis();
          digitalWrite(LEDG, HIGH);
        }
      microsPrevious = microsPrevious + microsPerReading;
      }
    }
    digitalWrite(LEDB, LOW);
  #ifdef WHDEBUG
  Serial.println("device disconnected");
  #endif
  }
}