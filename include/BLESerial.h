#include <ArduinoBLE.h>
#define rx_tx_uuid "0000FFE1-0000-1000-8000-00805F9B34FB"
#define service_uuid "00000000-0000-1000-8000-00805F9B34FB"
// TODO: Recieve handling
namespace BLEHelpers{

    
    typedef void (recieveCallback)(char*, int length);

    class BLESerial{
        public:
            BLESerial(BLELocalDevice& BLE);
            bool begin(const char* name);
            bool written();
            void write(void* data, size_t length);
            void RegisterRecieveCallback(BLECharacteristicEventHandler eh);
            BLEDevice hasClient();
        private:
            static constexpr int chunksize = 20;
            static constexpr uint16_t buffsize = 512;
            BLELocalDevice& dev;
            BLEService service;//("00000000-0000-1000-8000-00805F9B34FB");
            BLECharacteristic rx_tx_characteristic;//("0000FFE1-0000-1000-8000-00805F9B34FB", BLEWrite |BLERead |BLENotify, 20);


};
}