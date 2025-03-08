#define TARGET_STM32F4  // For Micro-Ros Library to Operate Correctly

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

#define RESET_PIN PB12       // Self Reset Pin
#define Reset_DURATION 7000  // Error Loop Reset Duration in milliseconds
unsigned long startTime;     //Used to self reset the STM32
unsigned long Current_Time;  //Used to self reset the STM32

MCP_CAN CAN(CAN_CS_pin);

#define THRUSTER_COUNT 5
#define GRIPPER_COUNT 3

#define HEARTBEAT_CAN_ID 0x200
#define MAX_JITTER 1000

struct HeartbeatSender {
  unsigned long last_heartbeat;
  unsigned long last_interval;
  uint8_t sequence;
  int status;
};

HeartbeatSender heartbeat_sender = {0, 0, 0, 0};

struct ThrusterData {
  int pwm;
  float current;
};

ThrusterData thrusters[THRUSTER_COUNT];
int grippers[GRIPPER_COUNT] = {0};

float ax = 0, ay = 0, az = 0, vx = 0, vy = 0, vz = 0, depth = 0;
bool buttonState = false;
int button_num = 0;

IPAddress STM_IP(192, 168, 0, 7);  // Assigned static IP
IPAddress Agent_IP(192, 168, 0, 8);    // Agent (Laptop) IP

// Publishers
rcl_publisher_t publisher_thrusters;
rcl_publisher_t publisher_grippers;
rcl_publisher_t publisher_heartbeat;

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

  RCCHECK(rclc_node_init_default(&node, "micro_ros_stm_ethernet_node", "ROV", &support));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_thrusters,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ThrusterData"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_grippers,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "GripperState"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_heartbeat,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "HeartbeatStatus"));
}

void loop() {
  handleCAN();
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

void handleCAN() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    long unsigned int rxId;
    unsigned char len;
    byte rxBuf[8];

    CAN.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId >= 0x100 && rxId < 0x100 + THRUSTER_COUNT) {
      int index = rxId - 0x100;
      thrusters[index].pwm = (rxBuf[0] << 8) | rxBuf[1];
      thrusters[index].current = ((rxBuf[2] << 8) | rxBuf[3]) / 100.0;

      std_msgs__msg__Int32 msg;
      msg.data = thrusters[index].pwm;
      RCSOFTCHECK(rcl_publish(&publisher_thrusters, &msg, NULL));
    } else if (rxId == 0x100 + THRUSTER_COUNT) {
      for (int i = 0; i < GRIPPER_COUNT; i++) {
        grippers[i] = rxBuf[i];

        std_msgs__msg__Int32 msg;
        msg.data = grippers[i];
        RCSOFTCHECK(rcl_publish(&publisher_grippers, &msg, NULL));
      }
    }
      else if (rxId == HEARTBEAT_CAN_ID) {
      handleHeartbeat(rxBuf);
    }
  }
}

void handleHeartbeat(byte *rxBuf) {
  unsigned long current_time = millis();
  uint8_t sequence_num = rxBuf[0];

  int32_t interval = current_time - heartbeat_sender.last_heartbeat;

  if (sequence_num == 0 && heartbeat_sender.sequence != 0) {
    heartbeat_sender.status = 1; // DEVICE_RESTARTED
  } else if (heartbeat_sender.last_interval != 0 && interval - heartbeat_sender.last_interval > MAX_JITTER) {
    heartbeat_sender.status = 2; // DEVICE_WARNING
  } else if (sequence_num == heartbeat_sender.sequence) {
    heartbeat_sender.status = 2; // DEVICE_WARNING: Duplicate heartbeat
  } else if (sequence_num < heartbeat_sender.sequence && sequence_num != 0) {
    heartbeat_sender.status = 2; // DEVICE_WARNING: Out-of-order heartbeat
  } else {
    heartbeat_sender.status = 0; // OK
  }

  heartbeat_sender.sequence = sequence_num;
  heartbeat_sender.last_heartbeat = current_time;
  heartbeat_sender.last_interval = interval;

  std_msgs__msg__Int32 msg;
  msg.data = heartbeat_sender.status;
  RCSOFTCHECK(rcl_publish(&publisher_heartbeat, &msg, NULL));
}

