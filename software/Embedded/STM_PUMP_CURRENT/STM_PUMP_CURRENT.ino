#include <SPI.h>
#include <mcp_can.h>

// ----- CONFIGURATION -----
#define SPI_CS_PIN PB8

// ----- ADC INPUT -----
const int adcPins[7] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6};
uint16_t readValue[7];
float rawVoltage[7];
float current[7];
float zeroOffset[7] = {0};

const float sensitivity = 0.1;   // 0.1V/A for 20A sensor
const float vRef = 3.3;          // ADC reference voltage
const float adcScale = vRef * 2.0 / 4095.0; // Scale for 12-bit ADC and voltage divider

// ----- PUMP CONTROL -----
#define PUMP_CW_PIN PA10
#define PUMP_CCW_PIN PB9

// ----- CAN IDs -----
#define Pump_ID      0x300
#define Currents_ID1 0x310
#define Currents_ID2 0x312
#define Currents_ID3 0x315
#define Currents_ID4 0x317
#define Heartbeat_ID 0x320

MCP_CAN CAN(SPI_CS_PIN);

// ----- TIMING -----
unsigned long lastCurrentSendTime = 0;
const unsigned long currentSendInterval = 10000; // 10 seconds (in milliseconds)

void setup() {
  SPI.setMISO(PB4);
  SPI.setMOSI(PB5);
  SPI.setSCLK(PB3);
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(PUMP_CW_PIN, OUTPUT);
  pinMode(PUMP_CCW_PIN, OUTPUT);

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("CAN init failed, retrying...");
    delay(100);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN Ready!");

  // Calibration phase
  Serial.println("Calibrating... Please ensure no current is flowing.");
  for (int ch = 0; ch < 7; ch++) {
    float sumVoltage = 0;
    for (int i = 0; i < 350; i++) {
      readValue[ch] = analogRead(adcPins[ch]);
      sumVoltage += readValue[ch] * adcScale;
      delay(1);
    }
    zeroOffset[ch] = sumVoltage / 350.0;

    Serial.print("Zero Offset CH"); Serial.print(ch); Serial.print(": ");
    Serial.println(zeroOffset[ch], 4);
  }
}

void loop() {
  readCurrentInputs();
  handleCANMessages();

  unsigned long currentMillis = millis();
  if (currentMillis - lastCurrentSendTime >= currentSendInterval) {
    lastCurrentSendTime = currentMillis;
    sendCurrentDataOverCAN();  // Send current data every 10 seconds
    sendHeartbeat();
  }

  delay(100); // Maintain decent loop speed for responsiveness
}

void readCurrentInputs() {
  for (int ch = 0; ch < 7; ch++) {
    readValue[ch] = analogRead(adcPins[ch]);
    rawVoltage[ch] = readValue[ch] * adcScale;
    current[ch] = (rawVoltage[ch] - zeroOffset[ch]) / sensitivity;

    Serial.print("CH"); Serial.print(ch);
    Serial.print(" | V: "); Serial.print(rawVoltage[ch], 3);
    Serial.print(" | Current: "); Serial.print(current[ch], 3);
    Serial.println(" A");
  }
  Serial.println("-----------------------------");
}

void handleCANMessages() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char buf[8];

  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    if (CAN.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      if (rxId == Pump_ID && len >= 1) {
        handlePumpControl(buf[0]);
      } else if (rxId == 0x210 || rxId == 0x212 || rxId == 0x215) {
        // Ignore legacy voltage messages
      } else {
        sendCurrentDataOverCAN();  // Respond with latest current values
      }
    }
  }
}

void handlePumpControl(uint8_t direction) {
  if (direction == 1) {
    digitalWrite(PUMP_CW_PIN, HIGH);
    digitalWrite(PUMP_CCW_PIN, LOW);
    Serial.println("Pump: Clockwise");
  } else if (direction == 2) {
    digitalWrite(PUMP_CW_PIN, LOW);
    digitalWrite(PUMP_CCW_PIN, HIGH);
    Serial.println("Pump: Counter-Clockwise");
  } else {
    digitalWrite(PUMP_CW_PIN, LOW);
    digitalWrite(PUMP_CCW_PIN, LOW);
    Serial.println("Pump: OFF");
  }
}

void sendCurrentDataOverCAN() {
  unsigned char cbuf1[4] = {
    (uint16_t)(current[0] * 100) >> 8, (uint16_t)(current[0] * 100) & 0xFF,
    (uint16_t)(current[1] * 100) >> 8, (uint16_t)(current[1] * 100) & 0xFF
  };
  CAN.sendMsgBuf(Currents_ID1, 0, 4, cbuf1);

  unsigned char cbuf2[4] = {
    (uint16_t)(current[2] * 100) >> 8, (uint16_t)(current[2] * 100) & 0xFF,
    (uint16_t)(current[3] * 100) >> 8, (uint16_t)(current[3] * 100) & 0xFF
  };
  CAN.sendMsgBuf(Currents_ID2, 0, 4, cbuf2);

  unsigned char cbuf3[4] = {
    (uint16_t)(current[4] * 100) >> 8, (uint16_t)(current[4] * 100) & 0xFF,
    (uint16_t)(current[5] * 100) >> 8, (uint16_t)(current[5] * 100) & 0xFF
  };
  CAN.sendMsgBuf(Currents_ID3, 0, 4, cbuf3);

  unsigned char cbuf4[2] = {
    (uint16_t)(current[6] * 100) >> 8, (uint16_t)(current[6] * 100) & 0xFF
  };
  CAN.sendMsgBuf(Currents_ID4, 0, 2, cbuf4);

  Serial.println("Current data sent over CAN.");
}

void sendHeartbeat() {
  unsigned char hb[1] = {0xAA}; // Arbitrary heartbeat value
  CAN.sendMsgBuf(Heartbeat_ID, 0, 1, hb);
}
