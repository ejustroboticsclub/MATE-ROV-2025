
#include <micro_ros_arduino.h>
#include <Servo.h>
#include <string.h>
#include <SPI.h>
#include <mcp_can.h>

// ------------------------------
// ------------------------------

// ----- CONFIGURATION -----
#define TOTAL_THRUSTER_COUNT 7
#define SPI_CS_PIN PB8
#define I2C_ADDR 0x08

// ----- THRUSTER PINS -----
const int thrusterPins[TOTAL_THRUSTER_COUNT] = {
  PB1, PB7, PA7, PA8, PB6, PB0, PA6
};

Servo thrusterServos[TOTAL_THRUSTER_COUNT];

// ----- ADC INPUT -----
const int adcPins[5] = {PA0, PA1, PA2, PA3, PA4};
uint16_t readValue[5];
float adcVoltage[5];
float Converter_Voltage[5];

const float vRef = 3.3;
const float adcScale = vRef / 4095.0;
const float dividerRatios[5] = {4.3312, 4.3439, 4.3388, 4.3877, 4.3414};

// ----- GRIPPERS -----
#define GRIPPER1_OPEN_PIN PB14
#define GRIPPER1_CLOSE_PIN PB13
#define GRIPPER2_OPEN_PIN PA10
#define GRIPPER2_CLOSE_PIN PA9
#define GRIPPER3_PIN PC13  // Center gripper

// ----- CAN IDs -----
#define Thrusters_ID   0x100
#define Grippers_ID_R  0x101
#define Grippers_ID_L  0x102
#define Grippers_ID_C  0x103  // Center gripper CAN ID
#define Voltages_ID1   0x110
#define Voltages_ID2   0x112
#define Voltages_ID3   0x115
#define Heartbeat_ID   0x120

MCP_CAN CAN(SPI_CS_PIN);

// ----- TIMING -----
unsigned long lastVoltageSendTime = 0;
const unsigned long voltageSendInterval = 10000; // 10 seconds

unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatInterval = 1000; // 1 second
bool heartbeatState = false;

void setup() {
  SPI.setMISO(PB4);
  SPI.setMOSI(PB5);
  SPI.setSCLK(PB3);
  Serial.begin(115200);
  analogReadResolution(12);

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("CAN init failed, retrying...");
    delay(100);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN Ready!");

  for (int i = 0; i < TOTAL_THRUSTER_COUNT; i++) {
    thrusterServos[i].attach(thrusterPins[i]);
  }

  pinMode(GRIPPER1_OPEN_PIN, OUTPUT);
  pinMode(GRIPPER1_CLOSE_PIN, OUTPUT);
  pinMode(GRIPPER2_OPEN_PIN, OUTPUT);
  pinMode(GRIPPER2_CLOSE_PIN, OUTPUT);
  pinMode(GRIPPER3_PIN, OUTPUT);
  digitalWrite(GRIPPER3_PIN, LOW);

}

void loop() {
  readVoltageInputs();
  handleCANMessages();

  unsigned long currentMillis = millis();

  if (currentMillis - lastVoltageSendTime >= voltageSendInterval) {
    lastVoltageSendTime = currentMillis;
    sendVoltageDataOverCAN();
  }

  if (currentMillis - lastHeartbeatTime >= heartbeatInterval) {
    lastHeartbeatTime = currentMillis;
    heartbeatState = !heartbeatState;
    sendHeartbeat(heartbeatState);
  }
}

void readVoltageInputs() {
  for (int ch = 0; ch < 5; ch++) {
    readValue[ch] = analogRead(adcPins[ch]);
    adcVoltage[ch] = readValue[ch] * adcScale;
    Converter_Voltage[ch] = adcVoltage[ch] * dividerRatios[ch];
  }
}

void handleCANMessages() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char buf[8];

  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    if (CAN.readMsgBuf(&rxId, &len, buf) != CAN_OK) continue;

    switch (rxId) {
      case Thrusters_ID:
        if (len == TOTAL_THRUSTER_COUNT) {
          for (int i = 0; i < TOTAL_THRUSTER_COUNT; i++) {
            int pwmVal = constrain(buf[i] * 10, 1000, 2000);
            thrusterServos[i].writeMicroseconds(pwmVal);
            Serial.print("Thruster "); Serial.print(i);
            Serial.print(" PWM: "); Serial.println(pwmVal);
          }
        } else {
          Serial.println("Thruster CAN msg length mismatch");
        }
        break;

      case Grippers_ID_R:
        if (len >= 1) handleGripperCAN(buf, len, "Right");
        break;

      case Grippers_ID_L:
        if (len >= 1) handleGripperCAN(buf, len, "Left");
        break;

      case Grippers_ID_C:
        if (len >= 1) handleCenterGripper(buf[0]);
        break;

      default:
        break;
    }
  }
}

void handleGripperCAN(uint8_t *data, uint8_t len, String gripperSide) {
  if (len < 1) return;

  if (data[0]) {
    if (gripperSide == "Right") {
      digitalWrite(GRIPPER1_OPEN_PIN, HIGH);
      delay(50);
      digitalWrite(GRIPPER1_OPEN_PIN, LOW);
      Serial.println("Right Gripper Opened");
    } else {
      digitalWrite(GRIPPER2_OPEN_PIN, HIGH);
      delay(50);
      digitalWrite(GRIPPER2_OPEN_PIN, LOW);
      Serial.println("Left Gripper Opened");
    }
  } else {
    if (gripperSide == "Right") {
      digitalWrite(GRIPPER1_CLOSE_PIN, HIGH);
      delay(50);
      digitalWrite(GRIPPER1_CLOSE_PIN, LOW);
      Serial.println("Right Gripper Closed");
    } else {
      digitalWrite(GRIPPER2_CLOSE_PIN, HIGH);
      delay(50);
      digitalWrite(GRIPPER2_CLOSE_PIN, LOW);
      Serial.println("Left Gripper Closed");
    }
  }
}

void handleCenterGripper(uint8_t command) {
  if (command) {
    digitalWrite(GRIPPER3_PIN, HIGH);
    Serial.println("Center Gripper Opened");
  } else {
    digitalWrite(GRIPPER3_PIN, LOW);
    Serial.println("Center Gripper Closed");
  }
}

void sendVoltageDataOverCAN() {
  unsigned char vbuf1[4] = {
    (uint16_t)(Converter_Voltage[0] * 100) >> 8, (uint16_t)(Converter_Voltage[0] * 100) & 0xFF,
    (uint16_t)(Converter_Voltage[1] * 100) >> 8, (uint16_t)(Converter_Voltage[1] * 100) & 0xFF
  };
  CAN.sendMsgBuf(Voltages_ID1, 0, 4, vbuf1);

  unsigned char vbuf2[4] = {
    (uint16_t)(Converter_Voltage[2] * 100) >> 8, (uint16_t)(Converter_Voltage[2] * 100) & 0xFF,
    (uint16_t)(Converter_Voltage[3] * 100) >> 8, (uint16_t)(Converter_Voltage[3] * 100) & 0xFF
  };
  CAN.sendMsgBuf(Voltages_ID2, 0, 4, vbuf2);

  unsigned char vbuf3[2] = {
    (uint16_t)(Converter_Voltage[4] * 100) >> 8, (uint16_t)(Converter_Voltage[4] * 100) & 0xFF
  };
  CAN.sendMsgBuf(Voltages_ID3, 0, 2, vbuf3);
}

void sendHeartbeat(bool state) {
  unsigned char hb[1] = { state ? 0x01 : 0x00 };
  CAN.sendMsgBuf(Heartbeat_ID, 0, 1, hb);
  Serial.print("Heartbeat: ");
  Serial.println(hb[0]);
}
