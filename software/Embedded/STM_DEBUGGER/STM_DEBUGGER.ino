#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include "MS5837.h"

// CAN constants
#define SPI_CS_PIN PB8
#define DEPTH_ID   0x205
#define Pump_ID    0x300
#define Currents_ID1 0x310
#define Currents_ID2 0x312
#define Currents_ID3 0x315
#define Currents_ID4 0x317
#define Voltages_ID1 0x110
#define Voltages_ID2 0x112
#define Voltages_ID3 0x115
#define Thrusters_ID 0x100
#define Grippers_ID_R  0x101
#define Grippers_ID_L  0x102

// Heartbeat CAN IDs
#define HEARTBEAT_ID1 0x320
#define HEARTBEAT_ID2 0x209
#define HEARTBEAT_ID3 0x120

MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("CAN Receiver Ready...");

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN initialized.");
  } else {
    Serial.println("CAN init failed.");
    while (1); 
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println("Waiting for all CAN data...");
}

void loop() {
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    byte len;
    byte data[8];

    CAN.readMsgBuf(&rxId, &len, data);

    if (rxId == Currents_ID1 || rxId == Currents_ID2 || rxId == Currents_ID3 || rxId == Currents_ID4) {
      printCurrentData(rxId, data, len);
    }
    else if (rxId == Voltages_ID1 || rxId == Voltages_ID2 || rxId == Voltages_ID3) {
      printVoltageData(rxId, data, len);
    }
    else if ((rxId == HEARTBEAT_ID1 || rxId == HEARTBEAT_ID2 || rxId == HEARTBEAT_ID3) && len >= 1) {
      handleHeartbeat(rxId, data[0]);
    }
  }
}

void printCurrentData(unsigned long rxId, byte* data, byte len) {
  Serial.print("Received Currents (ID 0x");
  Serial.print(rxId, HEX);
  Serial.print("): ");

  for (int i = 0; i < len; i += 2) {
    if (i + 1 < len) {
      uint16_t raw = (data[i] << 8) | data[i + 1];
      float current = raw / 100.0;
      Serial.print(current, 2);
      Serial.print(" A");
      if (i + 2 < len) Serial.print(" | ");
    }
  }
  Serial.println();
}

void printVoltageData(unsigned long rxId, byte* data, byte len) {
  Serial.print("Received Voltages (ID 0x");
  Serial.print(rxId, HEX);
  Serial.print("): ");

  for (int i = 0; i < len; i += 2) {
    if (i + 1 < len) {
      uint16_t raw = (data[i] << 8) | data[i + 1];
      float voltage = raw / 100.0;
      Serial.print(voltage, 2);
      Serial.print(" V");
      if (i + 2 < len) Serial.print(" | ");
    }
  }
  Serial.println();
}

void handleHeartbeat(unsigned long rxId, uint8_t value) {
  Serial.print("Heartbeat from 0x");
  Serial.print(rxId, HEX);
  Serial.print(": ");
  if (value == 1) {
    Serial.println("Alive");
  } else if (value == 0) {
    Serial.println("Not Responding");
  } else {
    Serial.print("Unknown value: ");
    Serial.println(value);
  }
}
