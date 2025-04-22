#define TARGET_STM32F4  // For Micro-Ros Library to Operate Correctly

//Ethernet Module SPI Pins
#define CS_pin PA15
#define SCLK_pin PB3
#define MISO_pin PB4
#define MOSI_pin PB5

#define CAN_CS PB8  // CAN Bus Module CS Pin

#include <LoayEthernetUdp.h>
#include <LoayEthernet.h>
#include <SPI.h>
#include <mcp2515.h>

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

IPAddress arduino_ip(192, 168, 100, 200);  // Assigned static IP
IPAddress agent_ip(192, 168, 100, 245);    // Agent (Laptop) IP

// Publishers
rcl_publisher_t publisher;
rcl_publisher_t publisher_neg;
rcl_publisher_t publisher_received;  // publisher for received data from can

// Subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 received_msg;

std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 msg_neg;
std_msgs__msg__Int32 msg_received;  // Message to store received data

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

MCP2515 mcp2515(CAN_CS);

struct can_frame canMsg;

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
      digitalWrite(RESET_PIN, LOW);  // Set pin LOW after Reset_DURATION
    }
  }
}

// Subscriber callback
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  Serial.print("Received: ");
  Serial.println(msg->data);

  // Publish received data
  msg_received.data = msg->data;
  RCSOFTCHECK(rcl_publish(&publisher_received, &msg_received, NULL));
}

void setup() {
  Serial.begin(115200);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);  // Start HIGH
  startTime = millis();           // Record start time

  Serial.begin(115200);
  byte arduino_mac[] = { 0xAA, 0xEE, 0xCC, 0xEE, 0xDD, 0xEE };  // MAC Address

  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999, CS_pin, SCLK_pin, MISO_pin, MOSI_pin);  //Custom Transporter

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);  // Full 11-bit mask (Mask 0)
  mcp2515.setFilter(MCP2515::RXF0, false, 0x200);      // Accept messages with ID 0x200
  //mcp2515.setFilter(MCP_RXM1, false, 0x100);         // Accept only messages with ID 0x100
  //mcp2515.setFilter(MCP_RXM2, false, 0x300);         // Accept only messages with ID 0x300
  //mcp2515.setFilter(MCP_RXM3, 0x400, false);         // Accept only messages with ID 0x400

  mcp2515.setNormalMode();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_stm_ethernet_node", "ROV", &support));

  // Create publishers
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "PositiveNumber"));  //Keeps increasing forever

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_neg,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "NegativeNumber"));   //Keeps decreasing forever

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_received,  // New publisher
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ReceivedNumber"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Subscriber"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &received_msg, &subscription_callback, ON_NEW_DATA));

  msg.data = 0;
}

void loop() {
  Handle_CAN();

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  // Publish negative value
  msg_neg.data = -msg.data;
  RCSOFTCHECK(rcl_publish(&publisher_neg, &msg_neg, NULL));

  msg.data++;

  // Spin executor to process incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}


void Handle_CAN() {

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Since we set a filter, only messages with CAN ID 0x200 should be received.
    Serial.print("Received CAN ID: 0x");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(" Data: ");
    for (uint8_t i = 0; i < canMsg.can_dlc; i++) {
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}
