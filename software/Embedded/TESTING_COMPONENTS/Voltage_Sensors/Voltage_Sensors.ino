// ADC input pins
const int adcPins[5] = {PA0, PA1, PA2, PA3, PA4};

// Per-channel data
uint16_t readValue[5];
float adcVoltage[5];
float Converter_Voltage[5];

// Constants
const float vRef = 3.3;
const float adcScale = vRef / 4095.0;

// Your calibrated voltage divider ratios (based on 11.93V input)
const float dividerRatios[5] = {
  4.3312,  // PA0
  4.3439,  // PA1
  4.3388,  // PA2
  4.3877,  // PA3
  4.3414   // PA4
};

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);
  Serial.println("Voltage Reader Calibrated & Ready!");
}

void loop() {
  for (int ch = 0; ch < 5; ch++) {
    readValue[ch] = analogRead(adcPins[ch]);
    adcVoltage[ch] = readValue[ch] * adcScale;
    Converter_Voltage[ch] = adcVoltage[ch] * dividerRatios[ch];
    Serial.print("Input Voltage: ");
    Serial.print(Converter_Voltage[ch], 3);
    Serial.println(" V");
  }
  delay(500);  // Half-second refresh
}
