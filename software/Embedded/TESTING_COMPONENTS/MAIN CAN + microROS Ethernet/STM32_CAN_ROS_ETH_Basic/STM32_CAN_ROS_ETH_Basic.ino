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

//SPIClass SPI_2(MOSI_pin, MISO_pin, SCLK_pin);  // MOSI, MISO, SCK
MCP_CAN CAN(CAN_CS_pin);               // Use SPI2

float ax = 0, ay = 0, az = 0, vx = 0, vy = 0, vz = 0, depth = 0;
bool buttonState = false;
int button_num = 0;

IPAddress STM_IP(192, 168, 0, 7);  // Assigned static IP
IPAddress Agent_IP(192, 168, 0, 8);    // Agent (Laptop) IP

// Publishers
rcl_publisher_t publisher;
rcl_publisher_t publisher_Button;
rcl_publisher_t publisher_received;  // New publisher for received data

// Subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 received_msg;

std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 button_state;
std_msgs__msg__Int32 msg_received;  // Message to store received dapth data

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
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);  // Start HIGH
  startTime = millis();           // Record start time

  Serial.begin(115200);
  byte STM_MACaddress[] = { 0xAA, 0xEE, 0xCC, 0xEE, 0xDD, 0xEE };  // MAC Address

  set_microros_native_ethernet_udp_transports(STM_MACaddress, STM_IP, Agent_IP, 9999, W5500_CS_pin, SCLK_pin, MISO_pin, MOSI_pin);  //Custom Transporter

  
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("CAN init failed, retrying...");
    delay(100);
  }
  Serial.println("CAN Receiver Ready!");
  CAN.setMode(MCP_NORMAL);  // Send And Recieve

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
    "PositiveNumber")); // A Positive Number Thatv Keeps Increasing Forever

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_Button,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ButtonState"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_received,  // New publisher
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Depth"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Subscriber")); //Just a subscriber only for test

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &received_msg, &subscription_callback, ON_NEW_DATA));

  msg.data = 0;
}

void loop() {

  handleCAN();  // Call function to process CAN messages

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  msg.data++;

  // Publish Button State 
  button_state.data = button_num; //The state of a button on another board, recieved from CAN
  RCSOFTCHECK(rcl_publish(&publisher_Button, &button_state, NULL));

  msg_received.data = depth;
  RCSOFTCHECK(rcl_publish(&publisher_received, &msg_received, NULL));
  
  // Spin executor to process incoming messages
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

                if (buttonState == false) {
                button_num = button_num + 1;
                }

                // Print once all values are updated
                Serial.print("ax: "); Serial.print(ax, 2);
                Serial.print(" | ay: "); Serial.print(ay, 2);
                Serial.print(" | az: "); Serial.print(az, 2);
                Serial.print(" | vx: "); Serial.print(vx, 2);
                Serial.print(" | vy: "); Serial.print(vy, 2);
                Serial.print(" | vz: "); Serial.print(vz, 2);
                Serial.print(" | Depth: "); Serial.print(depth, 2);
                Serial.print("m | Button: "); Serial.println(buttonState ? "PRESSED" : "RELEASED");
                break;
        }
    }
}
