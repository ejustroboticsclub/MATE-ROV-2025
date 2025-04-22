#include <Wire.h>



void setup() {
  Serial.begin(9600);
  
  // Initialize both I2C buses
  Wire.begin(8);               // Default I2C (PB7 = SDA, PB6 = SCL)

  Serial.println("\nI2C Bus Scanner for STM32");
}

void scanBus(TwoWire &bus, const char* busName) {
  byte error, address;
  int nDevices = 0;

  Serial.print("\nScanning ");
  Serial.println(busName);

  for(address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    error = bus.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("Done.");
}

void loop() {
  scanBus(Wire, "Bus 1 (PB6/PB7)");

  delay(5000);  // wait 5 seconds before scanning again
}
