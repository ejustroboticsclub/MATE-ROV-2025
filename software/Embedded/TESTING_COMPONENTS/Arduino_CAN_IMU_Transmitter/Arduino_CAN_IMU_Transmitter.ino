#include <SPI.h>
#include <mcp_can.h>

#define SPI_CS_PIN 10  // Change according to your wiring
MCP_CAN CAN(SPI_CS_PIN);

void setup() {
    Serial.begin(115200);
    pinMode(2, INPUT_PULLUP);
    while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
        Serial.println("CAN init failed, retrying...");
        delay(100);
    }
    
    Serial.println("CAN Sender Ready!");
    CAN.setMode(MCP_NORMAL);
}

void loop() {

    //Dummy IMU/Depth Values
    float ax = 1.23, ay = -4.56, az = 7.89;
    float vx = -2.34, vy = 5.67, vz = -8.90;
    float depth = 50.50;
    bool buttonState = digitalRead(PA2);  // Simulating "PRESSED" Test


    byte data[8];
    memcpy(&data[0], &ax, 4);
    memcpy(&data[4], &ay, 4);
    CAN.sendMsgBuf(0x036, 0, 8, data);
    
    memcpy(&data[0], &az, 4);
    memcpy(&data[4], &vx, 4);
    CAN.sendMsgBuf(0x037, 0, 8, data);

    memcpy(&data[0], &vy, 4);
    memcpy(&data[4], &vz, 4);
    CAN.sendMsgBuf(0x038, 0, 8, data);

    memcpy(&data[0], &depth, 4);
    memcpy(&data[4], &buttonState, 1);
    CAN.sendMsgBuf(0x039, 0, 8, data);

    Serial.print("Sent Data -> ");
    Serial.print("ax: "); Serial.print(ax, 2);
    Serial.print(" | ay: "); Serial.print(ay, 2);
    Serial.print(" | az: "); Serial.print(az, 2);
    Serial.print(" | vx: "); Serial.print(vx, 2);
    Serial.print(" | vy: "); Serial.print(vy, 2);
    Serial.print(" | vz: "); Serial.print(vz, 2);
    Serial.print(" | Depth: "); Serial.print(depth, 2);
    Serial.print("m | Button: "); Serial.println(buttonState ? "PRESSED" : "RELEASED");


}
