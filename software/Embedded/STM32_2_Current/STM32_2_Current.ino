#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>

#define THRUSTER_COUNT 7
#define GRIPPER_COUNT 3
#define SPI_CS_PIN PB8 // Change according to your wiring
#define CAN_ID_BASE 0x100
#define HEARTBEAT_CAN_ID 0x200

#define SCLK_pin    PB3
#define MISO_pin    PB4
#define MOSI_pin    PB5

MCP_CAN CAN(SPI_CS_PIN);

struct ThrusterData {
  int pwm;
  float current;
};

ThrusterData thrusters[THRUSTER_COUNT];
Servo thrustersServos[THRUSTER_COUNT];
int grippers[GRIPPER_COUNT] = {0}; // RGB states for each gripper pin

const int thrusterPwmPins[THRUSTER_COUNT] = {PA6, PA7, PB0, PB1, PB6, PB7, PA8}; // Servo control pins
const int gripperPins[GRIPPER_COUNT] = {PB5, PB3, PB4}; // RGB LED control pins

const int heartbeatLedPin = PC13; // Heartbeat LED pin
int heartbeatState = 0;

void setup() {
  SPI.setMISO(PB4);
  SPI.setMOSI(PB5);
  SPI.setSCLK(PB3);
  Serial.begin(115200);

  for (int i = 0; i < THRUSTER_COUNT; i++) {
    thrustersServos[i].attach(thrusterPwmPins[i]);
    thrustersServos[i].write(90); // Start at neutral position (90 degrees)
  }

  for (int i = 0; i < GRIPPER_COUNT; i++) {
    pinMode(gripperPins[i], OUTPUT);
    digitalWrite(gripperPins[i], LOW); // Start with LEDs off
  }

  pinMode(heartbeatLedPin, OUTPUT);
  digitalWrite(heartbeatLedPin, LOW);

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
      Serial.println("CAN init failed, retrying...");
      delay(100);
  }

  Serial.println("CAN Sender Ready!");
  CAN.setMode(MCP_NORMAL);
}

void loop() {
  for (int i = 0; i < THRUSTER_COUNT; i++) {
    int angle = (millis() / 1000) % 180;
    thrustersServos[i].write(angle);

    thrusters[i].pwm = map(angle, 0, 180, 1000, 2000);
    thrusters[i].current = map(abs(angle - 90), 0, 90, 0, 1.5);

    unsigned char buf[4];
    buf[0] = (thrusters[i].pwm >> 8) & 0xFF;
    buf[1] = thrusters[i].pwm & 0xFF;
    int currentInt = (int)(thrusters[i].current * 100);
    buf[2] = (currentInt >> 8) & 0xFF;
    buf[3] = currentInt & 0xFF;

    if (CAN.sendMsgBuf(CAN_ID_BASE + i, 0, 4, buf) == CAN_OK) {
      Serial.print("Sent Thruster ");
      Serial.print(i);
      Serial.print(" - PWM: ");
      Serial.print(thrusters[i].pwm);
      Serial.print(" Current: ");
      Serial.println(thrusters[i].current);
    } else {
      Serial.println("Failed to send thruster data");
    }
  }

  for (int i = 0; i < GRIPPER_COUNT; i++) {
      grippers[i] = !grippers[i];
      digitalWrite(gripperPins[i], grippers[i]);
  }

  unsigned char gripperBuf[GRIPPER_COUNT];
  for (int i = 0; i < GRIPPER_COUNT; i++) {
      gripperBuf[i] = (unsigned char)grippers[i];
  }

  if (CAN.sendMsgBuf(CAN_ID_BASE + THRUSTER_COUNT, 0, GRIPPER_COUNT, gripperBuf) == CAN_OK) {
    Serial.print("Sent Gripper States: ");
    for (int i = 0; i < GRIPPER_COUNT; i++) {
      Serial.print(grippers[i]);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Failed to send gripper data");
  }

  // Heartbeat LED toggle and CAN message
  heartbeatState = !heartbeatState;
  digitalWrite(heartbeatLedPin, heartbeatState);

  unsigned char heartbeatBuf[1] = { (unsigned char)heartbeatState };
  if (CAN.sendMsgBuf(HEARTBEAT_CAN_ID, 0, 1, heartbeatBuf) == CAN_OK) {
    Serial.println("Sent Heartbeat");
  } else {
    Serial.println("Failed to send heartbeat");
  }

  delay(1000);
}
