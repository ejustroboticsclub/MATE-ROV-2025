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

float Gx, Gy, Gz;

float depth;

// Quaternion storage
float q0_, q1_, q2_, q3_;


//-----------------------
int Filter = 1;
//-----------------------


// CAN interface
MCP_CAN CAN(SPI_CS_PIN);


void setup() {
  Serial.begin(115200);
  Serial.println("Starting system...");

  // I²C
  Wire.begin(8); // Slave address
  Wire.onReceive(receiveEvent);

  // Wire.onRequest(requestEvent);

  // CAN
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN initialized.");
  } else {
    Serial.println("CAN init failed.");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
}

void loop() {
  // Convert quaternion values to byte array
  byte imuData[16];  
  memcpy(imuData, &q0_, 4);
  memcpy(imuData + 4, &q1_, 4);
  memcpy(imuData + 8, &q2_, 4);
  memcpy(imuData + 12, &q3_, 4);

  byte Gyro[12];  
  memcpy(Gyro, &Gx, 4);
  memcpy(Gyro + 4, &Gy, 4);
  memcpy(Gyro + 8, &Gz, 4);
  
  byte Depth[4];
  memcpy(Depth, &depth, 4);

  delay(100);
  
  // Send quaternion data over CAN
  byte imuData0[4];  // 4 bytes for a single float
  memcpy(imuData0, &q0_, 4);
  CAN.sendMsgBuf(IMU1_ID, 0, 4, imuData0);  

  // Send q1 over CAN (IMU2_ID)
  byte imuData1[4];  // 4 bytes for a single float
  memcpy(imuData1, &q1_, 4);
  CAN.sendMsgBuf(IMU2_ID, 0, 4, imuData1);

  // Send q2 over CAN (IMU3_ID)
  byte imuData2[4];  // 4 bytes for a single float
  memcpy(imuData2, &q2_, 4);
  CAN.sendMsgBuf(IMU3_ID, 0, 4, imuData2);

  // Send q3 over CAN (IMU4_ID)
  byte imuData3[4];  // 4 bytes for a single float
  memcpy(imuData3, &q3_, 4);
  CAN.sendMsgBuf(IMU4_ID, 0, 4, imuData3);

  byte Gyro0[4];
  memcpy(Gyro0, &Gx, 4);
  CAN.sendMsgBuf(Gx_ID,0, 4, Gyro0);

  byte Gyro1[4];
  memcpy(Gyro1, &Gy, 4);
  CAN.sendMsgBuf(Gy_ID,0, 4, Gyro1);

  byte Gyro2[4];
  memcpy(Gyro2, &Gz, 4);
  CAN.sendMsgBuf(Gz_ID,0, 4, Gyro2);
  
  byte Depth0[4];
  memcpy(Depth0, &depth, 4);
  CAN.sendMsgBuf(DEPTH_ID, 0, 4, Depth0);
  
  delay(100); // Add delay between sends for stability



  // Check for incoming CAN message

  //-----------------------------------------------
  
  // if (CAN.checkReceive() == CAN_MSGAVAIL) {
  //   long unsigned int rxId;
  //   byte len;
  //   byte data[8];
  //   CAN.readMsgBuf(&rxId, &len, data);

  //   if (rxId == QUAT_ID && len >= 1) {
  //     byte cmd = data[0];
  //     switch (cmd) {
  //       case 0x01:
  //         Serial.println("CALIBRATE command received via CAN.");
  //         Filter = 2;
  //         break;
  //       case 0x02:
  //         Serial.println("RESET command received via CAN.");
  //         Filter = 3;
  //         break;
  //       default:
  //         Serial.print("Unknown CAN command: 0x");
  //         Serial.println(cmd, HEX);
  //         break;
  //     }
  //   }
  // }

  //-----------------------------------------------

}

// I²C Receive: only receive quaternion data (4 doubles = 32 bytes)
void receiveEvent(int howMany) {
  if (howMany == 32) {
    byte buffer[32];
    for (int i = 0; i < 32; i++) {
      buffer[i] = Wire.read();
    }

    memcpy(&q0_, buffer,      4);
    memcpy(&q1_, buffer + 4,  4);
    memcpy(&q2_, buffer + 8, 4);
    memcpy(&q3_, buffer + 12, 4);
    memcpy(&Gx,  buffer + 16, 4);
    memcpy(&Gy,  buffer + 20, 4);
    memcpy(&Gz,  buffer + 24, 4);
    memcpy(&depth,  buffer + 28, 4);

    Serial.print("Received quaternion: ");
    Serial.print(q0_); Serial.print(", ");
    Serial.print(q1_); Serial.print(", ");
    Serial.print(q2_); Serial.print(", ");
    Serial.println(q3_);
    Serial.println(Gx);
    Serial.println(Gy);
    Serial.println(Gz);
    Serial.println(depth);

  } else {
    // Ignore anything that's not a full quaternion
    while (Wire.available()) Wire.read();
    Serial.print("Invalid I2C length (expected 32): ");
    Serial.println(howMany);
  }
}

// void requestEvent() {
//   Wire.write((uint8_t*)&Filter, 2);
// }
