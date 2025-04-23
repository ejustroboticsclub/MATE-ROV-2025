#define TARGET_STM32F4  // For Micro-Ros Library to Operate Correctly

#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include "MS5837.h"
#include <LoayEthernetUdp.h>
#include <LoayEthernet.h>
#include <SPI.h>
#include <micro_ros_arduino.h>
#include <native_ethernet_transport.cpp>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>  // For publishing Float32 (voltage and current)
#include <std_msgs/msg/int8_multi_array.h>  // For publishing Int8MultiArray (heartbeat)

#define W5500_CS_pin PA15
#define SPI_CS_PIN PB8
#define SCLK_pin PA5
#define MISO_pin PA6
#define MOSI_pin PA7
#define Currents_ID1 0x310
#define Currents_ID2 0x312
#define Currents_ID3 0x315
#define Currents_ID4 0x317
#define Voltages_ID1 0x110
#define Voltages_ID2 0x112
#define Voltages_ID3 0x115

// Heartbeat CAN IDs
#define HEARTBEAT_ID1 0x320
#define HEARTBEAT_ID2 0x209
#define HEARTBEAT_ID3 0x120

MCP_CAN CAN(SPI_CS_PIN);

IPAddress Agent_IP(192, 168, 1, 101); 
IPAddress STM_IP(192, 168, 1, 105); 


// ROS Publishers
rcl_publisher_t publisher_voltage;
rcl_publisher_t publisher_current;
rcl_publisher_t publisher_heartbeat;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;


int8_t heartbeat_values[3];  // Store values for HEARTBEAT_ID1, HEARTBEAT_ID2, and HEARTBEAT_ID3
int heartbeat_index = 0;  // To track which heartbeat we're receiving

void setup() {
  Serial.begin(115200);

  byte STM_MACaddress[] = { 0xAA, 0xEE, 0xCC, 0xEE, 0xDD, 0xEE };
  set_microros_native_ethernet_udp_transports(STM_MACaddress, STM_IP, Agent_IP, 9999, W5500_CS_pin, SCLK_pin, MISO_pin, MOSI_pin);

  Serial.println("CAN Receiver Ready...");

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN initialized.");
  } else {
    Serial.println("CAN init failed.");
    while (1); 
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println("Waiting for all CAN data...");

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "rov_node", "", &support);
  
  rclc_publisher_init_default(&publisher_voltage, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "ROV/voltage");
  rclc_publisher_init_default(&publisher_current, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "ROV/current");
  rclc_publisher_init_default(&publisher_heartbeat, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray), "ROV/indicators");

  // Initialize executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  Serial.println("ROS Initialized.");
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

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
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

      // Publish current to ROS topic
      std_msgs__msg__Float32 current_msg;
      current_msg.data = current;
      rcl_publish(&publisher_current, &current_msg, NULL);

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

      // Publish voltage to ROS topic
      std_msgs__msg__Float32 voltage_msg;
      voltage_msg.data = voltage;
      rcl_publish(&publisher_voltage, &voltage_msg, NULL);

      if (i + 2 < len) Serial.print(" | ");
    }
  }
  Serial.println();
}

void handleHeartbeat(unsigned long rxId, uint8_t value) {
  Serial.print("Heartbeat from 0x");
  Serial.print(rxId, HEX);
  Serial.print(": ");
  
  if (rxId == HEARTBEAT_ID1) {
    heartbeat_values[0] = value;
  } else if (rxId == HEARTBEAT_ID2) {
    heartbeat_values[1] = value;
  } else if (rxId == HEARTBEAT_ID3) {
    heartbeat_values[2] = value;
  }

  if (value == 1) {
    Serial.println("Alive");
  } else if (value == 0) {
    Serial.println("Not Responding");
  } else {
    Serial.print("Unknown value: ");
    Serial.println(value);
  }

  if (heartbeat_index == 2) {
    std_msgs__msg__Int8MultiArray heartbeat_msg;
    heartbeat_msg.data.data = heartbeat_values;  
    heartbeat_msg.data.size = 3; 
    heartbeat_msg.data.capacity = 3;  

    rcl_publish(&publisher_heartbeat, &heartbeat_msg, NULL);

    heartbeat_index = 0;
  } else {
    heartbeat_index++;
  }
}
