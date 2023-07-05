#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int txValue = 0;

#define SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(9600);

  // Create the BLE Device
  BLEDevice::init("ESP32test");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                              CHARACTERISTIC_UUID_TX, 
                              BLECharacteristic::PROPERTY_NOTIFY
                              );

 // BLE2902 needed to notify
 pCharacteristic->addDescriptor(new BLE2902());

 // Start the service
 pService->start();

 // Start advertising
 pServer->getAdvertising()->start();
 Serial.println("Waiting for a client connection to notify...");

 

}

void loop() {
  if(deviceConnected) {
    txValue = random(-10, 20);

    // Conversion of txValue
    char txString[8];
    dtostrf(txValue, 1, 2, txString);

    // Setting the value to the characteristic
    pCharacteristic->setValue(txString);

    // Notify the conncted client
    pCharacteristic->notify();
    Serial.println("Sent value: " + String(txString));
    delay(500);
  }

  

}
