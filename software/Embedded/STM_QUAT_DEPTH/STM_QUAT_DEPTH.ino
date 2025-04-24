#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include "MS5837.h"

// CAN constants
#define SPI_CS_PIN PB8
#define QUAT_ID    0x200
#define IMU1_ID    0x201
#define IMU2_ID    0x202
#define IMU3_ID    0x203
#define IMU4_ID    0x204
#define DEPTH_ID   0x205
#define Gx_ID      0x206
#define Gy_ID      0x207
#define Gz_ID      0x208
#define HEARTBEAT_ID 0x209

// Sensor values
float q0_, q1_, q2_, q3_;
float Gx, Gy, Gz;
float depth;

// CAN interface
MCP_CAN CAN(SPI_CS_PIN);

// Heartbeat control
uint32_t lastHeartbeat = 0;
const uint32_t heartbeatInterval = 1000; // 1 second
bool ledState = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting system...");

  // I²C as slave
  Wire.begin(8); // Address 8
  Wire.onReceive(receiveEvent);

  // Initialize CAN
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN initialized.");
  } else {
    Serial.println("CAN init failed.");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);

  // Setup PC13 for heartbeat LED
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH); // Off initially (STM32 logic)
}

void loop() {
  // Send quaternion components
  CAN.sendMsgBuf(IMU1_ID, 0, 4, (byte*)&q0_);
  CAN.sendMsgBuf(IMU2_ID, 0, 4, (byte*)&q1_);
  CAN.sendMsgBuf(IMU3_ID, 0, 4, (byte*)&q2_);
  CAN.sendMsgBuf(IMU4_ID, 0, 4, (byte*)&q3_);

  // Send gyroscope values
  CAN.sendMsgBuf(Gx_ID, 0, 4, (byte*)&Gx);
  CAN.sendMsgBuf(Gy_ID, 0, 4, (byte*)&Gy);
  CAN.sendMsgBuf(Gz_ID, 0, 4, (byte*)&Gz);

  // Send depth value
  CAN.sendMsgBuf(DEPTH_ID, 0, 4, (byte*)&depth);

  // Heartbeat every 1 second
  if (millis() - lastHeartbeat >= heartbeatInterval) {
    lastHeartbeat = millis();

    // Toggle LED
    ledState = !ledState;
    digitalWrite(PC13, ledState ? LOW : HIGH); // LOW = ON for STM32

    // Send heartbeat CAN message: 0 = off, 1 = on
    byte heartbeatData[1] = { ledState ? 1 : 0 };
    CAN.sendMsgBuf(HEARTBEAT_ID, 0, 1, heartbeatData);

    Serial.print("Heartbeat sent: ");
    Serial.println(heartbeatData[0]);
  }

  delay(100); // Stabilize timing
}

// I²C receive handler (32 bytes expected)
void receiveEvent(int howMany) {
  if (howMany == 32) {
    byte buffer[32];
    for (int i = 0; i < 32; i++) {
      buffer[i] = Wire.read();
    }

    // Parse incoming float values
    memcpy(&q0_,   buffer + 0,  4);
    memcpy(&q1_,   buffer + 4,  4);
    memcpy(&q2_,   buffer + 8,  4);
    memcpy(&q3_,   buffer + 12, 4);
    memcpy(&Gx,    buffer + 16, 4);
    memcpy(&Gy,    buffer + 20, 4);
    memcpy(&Gz,    buffer + 24, 4);
    memcpy(&depth, buffer + 28, 4);

    // Debug print
    Serial.print("Quat: ");
    Serial.print(q0_); Serial.print(", ");
    Serial.print(q1_); Serial.print(", ");
    Serial.print(q2_); Serial.print(", ");
    Serial.println(q3_);
    Serial.print("Gyro: ");
    Serial.print(Gx); Serial.print(", ");
    Serial.print(Gy); Serial.print(", ");
    Serial.println(Gz);
    Serial.print("Depth: ");
    Serial.println(depth);

  } else {
    while (Wire.available()) Wire.read(); // Flush buffer
    Serial.print("Invalid I2C length (expected 32): ");
    Serial.println(howMany);
  }
}
