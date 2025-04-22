#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);  // CS Pin

struct can_frame canMsg;

void setup() {
    Serial.begin(115200);
    SPI.begin();

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    Serial.println("CAN Receiver Ready!");
}

void loop() {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        Serial.print("Received CAN ID: 0x");
        Serial.println(canMsg.can_id, HEX);

        // Process data based on CAN ID
        if (canMsg.can_id == 0x036) { // Humidity & Temperature
            Serial.print(" Humidity: ");
            Serial.print(canMsg.data[0]);
            Serial.print("%, Temperature: ");
            Serial.print(canMsg.data[1]);
            Serial.println("°C");
        } 
        else if (canMsg.can_id == 0x045) { // Pressure
            int pressure = (canMsg.data[0] << 8) | canMsg.data[1];
            Serial.print(" Pressure: ");
            Serial.print(pressure);
            Serial.println(" hPa");
        } 
        else {
            Serial.print(" Unknown Data: ");
            for (int i = 0; i < canMsg.can_dlc; i++) {
                Serial.print(canMsg.data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
    }
}
