// Arduino libraries used
#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

#include "BLESerial.h"
#include "INS.h"

// time interval the characteristic gets updated with [ms]
#define BLE_REFRESH_TIME 60
// Sample rate for the filter update [hz]
#define INS_SAMPLE_FREQ 119.0f
//#define WHDEBUG



unsigned long last_send_millis = 0;
unsigned long microsPerReading, microsPrevious, microsNow;
int roll, pitch, heading;

float ax, ay, az;
float gx, gy, gz;
float wx, wy, wz;
float mx, my, mz;
char buff[20];
float vx = 0;

INS ins(119.0f);
BLEHelpers::BLESerial bs(BLE);

void speedUpdateHandler(BLEDevice dev, BLECharacteristic characteristic){
  #ifdef WHDEBUG
  Serial.print((char)*characteristic.value());
  Serial.print((char)*(characteristic.value()+1));
  Serial.print((int16_t)*(characteristic.value()+2));
  Serial.println();
  #endif
  if((char)*characteristic.value() == '$'){
    switch ((char)*(characteristic.value()+1)){
      case 'V':
      vx = 0.1f * (int16_t)*(characteristic.value()+2);
      ins.provideAirspeed(0.1f * (int16_t)*(characteristic.value()+2));
      break;
      case 'G':
      ins.x(0, 0) = 0.01f * (int16_t) * (characteristic.value()+2);
      ins.x(1, 0) = 0.01f * (int16_t) * (characteristic.value()+4);
      ins.x(2, 0) = 0.01f * (int16_t) * (characteristic.value()+6);
      ins.stepcount = 0;
      break;

    }
    
  }
  
}


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
  bs.RegisterRecieveCallback(speedUpdateHandler);

  microsPerReading = 1000000 / INS_SAMPLE_FREQ;
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
      ins.x(0, 0) = 0;
      ins.x(1, 0) = 0;
      ins.x(2, 0) = 0;
    }
    microsPrevious = micros();
    while (central.connected()) {
      microsNow = micros();
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(wx, wy, wz);
        //convert to right handed ned system
        wz *= -1;
      }

      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        //convert to right-handed ned system
        az += 0.015f;
        az *= -1;
      }

      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
        // calibration
        mx -= 27.5f;
        my -= 7.5f;
        mz += 17.50f;
        // convert to right handed ned system
        mx *= -1;
        mz *= -1;
      }
      if (microsNow - microsPrevious >= microsPerReading) {
        //substract centripetal force, convert radian to int
        ins.predict(wx, wy, wz, ax, ay, az, -mx, -my, -mz);
        mx = 0;
        my = 0;
        mz = 0;

        if(millis() - last_send_millis > BLE_REFRESH_TIME){
          // flash grenn led when transmitting
          digitalWrite(LEDG, LOW);
          buff[0] = '$';
          buff[1] = 'A';
          int16_t roll = (int16_t) ins.ahrs.getRoll()*10;
          int16_t pitch = (int16_t) ins.ahrs.getPitch()*10;
          int16_t yaw = (int16_t) ins.ahrs.getYaw()*10;
          int16_t vx_ins = (int16_t) (sqrtf(ins.x(0,0) * ins.x(0,0) + ins.x(1,0) * ins.x(1,0)) *100);
          int16_t vz_ins = ins.x(2, 0) * 100;
          int16_t v_te = ins.getTE() * -100;
          
          buff[2] = *((char*)&roll);
          buff[3] = *((char*)&roll + 1);

          buff[4] = *((char*)&pitch);
          buff[5] = *((char*)&pitch + 1);

          buff[6] = *((char*)&yaw);
          buff[7] = *((char*)&yaw + 1);

          buff[8] = *((char*)&vx_ins);
          buff[9] = *((char*)&vx_ins + 1);

          buff[10] = *((char*)&vz_ins);
          buff[11] = *((char*)&vz_ins + 1);
          
          buff[12] = *((char*)&v_te);
          buff[13] = *((char*)&v_te + 1);

          bs.write(buff, 14);
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

