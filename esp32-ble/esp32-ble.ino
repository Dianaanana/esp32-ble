#include "Zanshin_BME680.h"     // BME sensor library

const uint32_t SERIAL_SPEED{115200}; // baud rate for serial I/O


BME680_Class BME680;      // new instance of class BME680


void setup() {
  Serial.begin(SERIAL_SPEED);
  #ifdef __AVR_ATmega32U4__
    delay(3000);
  #endif

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
  
  static int32_t  temp, humidity, pressure, gas;  // BME readings
  static char     buf[16];                        // sprintf text buffer
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
    Serial.print(buf);
    sprintf(buf, "%3d.%03d", (int8_t)(humidity / 1000),
            (uint16_t)(humidity % 1000));  // Humidity milli-pct
    Serial.print(buf);
    sprintf(buf, "%7d.%02d", (int16_t)(pressure / 100),
            (uint8_t)(pressure % 100));  // Pressure Pascals
    Serial.print(buf);
    sprintf(buf, "%4d.%02d\n", (int16_t)(gas / 100), (uint8_t)(gas % 100));  // Resistance milliohms
    Serial.print(buf);
    delay(10000);  // Wait 10s
  }                // of ignore first reading

}
