#define TARGET_STM32F4  // Required for Micro-Ros Library

#define W5500_CS_pin PA15
#define CAN_CS_pin PB8
#define SCLK_pin PB3
#define MISO_pin PB4
#define MOSI_pin PB5

#include <LoayEthernetUdp.h>
#include <LoayEthernet.h>
#include <SPI.h>
#include <mcp_can.h>
#include <micro_ros_arduino.h>
#include <native_ethernet_transport.cpp>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>  // Heartbeat, Depth
#include <sensor_msgs/msg/imu.h>   // IMU message

#define RESET_PIN PB12
#define Reset_DURATION 7000
unsigned long startTime;
unsigned long Current_Time;

MCP_CAN CAN(CAN_CS_pin);  // CAN Module

// IMU variables
float ax = 0, ay = 0, az = 0;
float vx = 0, vy = 0, vz = 0;
float depth = 0;
bool buttonState = false;
int button_num = 0;

// Heartbeat variables
float debugger_heartbeat = 0.0;
float node1_heartbeat = 0.0;

IPAddress STM_IP(192, 168, 1, 2);
IPAddress Agent_IP(192, 168, 1, 4);

// Publishers
rcl_publisher_t imu_publisher;         // IMU Data
rcl_publisher_t heartbeat_publisher;   // Debugger Heartbeat
rcl_publisher_t node1_heartbeat_pub;   // Node 1 Heartbeat
rcl_publisher_t depth_button_pub;      // Depth & Button State

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32 heartbeat_msg;
std_msgs__msg__Float32 node1_heartbeat_msg;
std_msgs__msg__Float32 depth_msg;
std_msgs__msg__Int32 button_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

#define LED_PIN PC13

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    Current_Time = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    if (Current_Time - startTime >= Reset_DURATION) {
      digitalWrite(RESET_PIN, LOW);
    }
  }
}

void setup() {
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  startTime = millis();

  Serial.begin(115200);
  byte STM_MACaddress[] = { 0xAA, 0xEE, 0xCC, 0xEE, 0xDD, 0xEE };

  set_microros_native_ethernet_udp_transports(STM_MACaddress, STM_IP, Agent_IP, 9999, W5500_CS_pin, SCLK_pin, MISO_pin, MOSI_pin);

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("CAN init failed, retrying...");
    delay(100);
  }
  Serial.println("CAN Receiver Ready!");
  CAN.setMode(MCP_NORMAL);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create ROS Node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_stm_ethernet_node", "ROV", &support));

  // Initialize Publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &imu_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data"
  ));

  RCCHECK(rclc_publisher_init_best_effort(
      &heartbeat_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "debugger/heartbeat"
  ));

  RCCHECK(rclc_publisher_init_best_effort(
      &node1_heartbeat_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "node1/heartbeat"
  ));

  RCCHECK(rclc_publisher_init_best_effort(
      &depth_button_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "depth_button_state/depth"
  ));

  RCCHECK(rclc_publisher_init_best_effort(
      &depth_button_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "depth_button_state/button"
  ));

  // Initialize executor with multiple publishers
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  Serial.println("ROS 2 Micro-ROS Node Initialized!");
}

void loop() {
  handleCAN();  // Read CAN messages and update variables

  // Publish IMU Data
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  imu_msg.angular_velocity.x = vx;
  imu_msg.angular_velocity.y = vy;
  imu_msg.angular_velocity.z = vz;
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

  // Publish Debugger Heartbeat (increments forever)
  debugger_heartbeat += 1.0;
  heartbeat_msg.data = debugger_heartbeat;
  RCSOFTCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));

  // Publish Node 1 Heartbeat (received from CAN)
  node1_heartbeat_msg.data = node1_heartbeat;
  RCSOFTCHECK(rcl_publish(&node1_heartbeat_pub, &node1_heartbeat_msg, NULL));

  // Publish Depth and Button State
  depth_msg.data = depth;
  button_msg.data = buttonState ? 1 : 0;
  RCSOFTCHECK(rcl_publish(&depth_button_pub, &depth_msg, NULL));
  RCSOFTCHECK(rcl_publish(&depth_button_pub, &button_msg, NULL));

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

void handleCAN() {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        long unsigned int rxId;
        unsigned char len;
        byte rxBuf[8];

        CAN.readMsgBuf(&rxId, &len, rxBuf);

        switch (rxId) {
            case 0x036:
                memcpy(&ax, &rxBuf[0], 4);
                memcpy(&ay, &rxBuf[4], 4);
                break;
            case 0x037:
                memcpy(&az, &rxBuf[0], 4);
                memcpy(&vx, &rxBuf[4], 4);
                break;
            case 0x038:
                memcpy(&vy, &rxBuf[0], 4);
                memcpy(&vz, &rxBuf[4], 4);
                break;
            case 0x039:
                memcpy(&depth, &rxBuf[0], 4);
                memcpy(&buttonState, &rxBuf[4], 1);
                break;
            case 0x040:
                memcpy(&node1_heartbeat, &rxBuf[0], 4);
                break;
        }
    }
}
