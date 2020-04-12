#include "BLESerial.h"
#include <ArduinoBLE.h>
using namespace BLEHelpers;

BLESerial::BLESerial(BLELocalDevice & dev):
    dev(dev),
    rcf_registered(false),
    rx_tx_characteristic(rx_tx_uuid, BLERead|BLEWrite|BLENotify, chunksize),
    service(service_uuid){}


bool BLESerial::begin(const char* name){
    if(!dev.begin()){
        return false;
    }
    dev.setLocalName(name);
    dev.setAdvertisedService(service);
    service.addCharacteristic(rx_tx_characteristic);
    dev.addService(service);
    dev.advertise();
    return true;
}

 bool BLESerial::written(){
     return rx_tx_characteristic.written();
 }
void BLESerial::write(void* data, size_t length){
    int i = 0;
    while(i < length){
            if(length-i >20){
              rx_tx_characteristic.writeValue(data+i, 20);
            }else{
              rx_tx_characteristic.writeValue(data+i, length-i);
            }
            i += chunksize;
          }
}


BLEDevice BLESerial::hasClient(){
    return dev.central();
}