const int adcPins[5] = {PA0, PA1, PA2, PA3, PA4};  // Define the analog pins for ADC reading
uint16_t readValue[5];
float sensitivity = 0.1; // 0.1 for 20A Model
float rawVoltage[5];
float current[5];
float zeroOffset[5] = {0}; // Offset for each channel

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  analogReadResolution(12); // Set ADC resolution to 12 bits

  // Calibration phase
  Serial.println("Calibrating... Please ensure no current is flowing.");
  
  for (int ch = 0; ch < 5; ch++) {
    float sumVoltage = 0;
    for (int i = 0; i < 350; i++) {
      readValue[ch] = analogRead(adcPins[ch]);
      sumVoltage += readValue[ch] * 3.3 * 2.0 / 4095; // 
      delay(1);  // Small delay for stable readings
    }
    zeroOffset[ch] = sumVoltage / 350; // Average value to zero out

    Serial.print("Calibration complete for PA");
    Serial.print(ch);
    Serial.print(". Zero Offset: ");
    Serial.println(zeroOffset[ch], 4);  
  }
}

void loop() {
  
  for (int ch = 0; ch < 5; ch++) {
    readValue[ch] = analogRead(adcPins[ch]);  // Read ADC value
    rawVoltage[ch] = readValue[ch] * 3.3 * 2.0 / 4095;  // Convert to voltage (12-bit resolution)
  
    // Adjust current calculation using the zero offset
    current[ch] = (rawVoltage[ch] - zeroOffset[ch]) / sensitivity;

    Serial.print("PA");
    Serial.print(ch);
    Serial.print(" | ADC: ");
    Serial.print(readValue[ch]);
    Serial.print(" | Raw Voltage: ");
    Serial.print(rawVoltage[ch], 4);
    Serial.print(" V | Current: ");
    Serial.print(current[ch], 4);
    Serial.println(" A");
  }

  Serial.println("------------------------------------------------");
  delay(100);  // Delay 100ms
}
