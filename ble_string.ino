/*
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
  Ported to Arduino ESP32 by Evandro Copercini
  updated by chegewara and MoThunderz
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Zanshin_BME680.h"     // BME sensor library

BME680_Class BME680;      // new instance of class BME680

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "6b3fdcd9-e80c-46f3-9e7c-78b6ae7c8400"
#define CHARACTERISTIC_UUID "6b3fdcd9-e80c-46f3-9e7c-78b6ae7c8400"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );                   

  // Create a BLE Descriptor
  
  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("A very interesting variable");
  pCharacteristic->addDescriptor(pDescr);
  
  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");


  Serial.println("Starting BME680 example...");
  Serial.println("Initialising BME680 sensor...");

  while(!BME680.begin(I2C_STANDARD_MODE)) {
    Serial.println("unable to find BME680. Try again in 5 seconds...");
    delay(5000);
  }

  Serial.println("Setting 16x overstamping for all sensors");
  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);

  Serial.println("Setting IIR filter to a value of 4 samples");
  BME680.setIIRFilter(IIR4);

  // ?
  BME680.setGas(320, 150);
}

void loop() {
    // notify changed value
    if (deviceConnected) {
      static int32_t  temp, humidity, pressure, gas;  // BME readings
      static char     buf[16];                        // sprintf text buffer
      std::string     finalbuf;
      static float    alt;                            // Temporary variable
      static uint16_t loopCounter = 0;                // Display iterations
      if (loopCounter % 25 == 0) {                    // Show header @25 loops
        Serial.print(F("\nLoop Temp\xC2\xB0\x43 Humid% Press hPa   Alt m Air m"));
        Serial.print(F("\xE2\x84\xA6\n==== ====== ====== ========= ======= ======\n"));  // "�C" symbol
      }                                                     // if-then time to show headers
      BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
      if (loopCounter++ != 0) {                             // Ignore first reading, might be incorrect
        sprintf(buf, "%4d %3d.%02d", (loopCounter - 1) % 9999,  // Clamp to 9999,
                (int8_t)(temp / 100), (uint8_t)(temp % 100));   // Temp in decidegrees
        finalbuf.append(buf);
        sprintf(buf, "%3d.%03d", (int8_t)(humidity / 1000),
                (uint16_t)(humidity % 1000));  // Humidity milli-pct
        finalbuf.append(buf);
        sprintf(buf, "%7d.%02d", (int16_t)(pressure / 100),
                (uint8_t)(pressure % 100));  // Pressure Pascals
        finalbuf.append(buf);
        sprintf(buf, "%4d.%02d\n", (int16_t)(gas / 100), (uint8_t)(gas % 100));  // Resistance milliohms
        finalbuf.append(buf);
        Serial.println(finalbuf.c_str());
//        if (Serial.available()) {
//          SerialBT.print(finalbuf);
//        }
        pCharacteristic->setValue(finalbuf);
        pCharacteristic->notify();
        value++;
        delay(1000);
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    }
}
