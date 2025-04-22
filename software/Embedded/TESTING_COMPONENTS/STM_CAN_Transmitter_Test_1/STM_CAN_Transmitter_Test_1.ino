#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(PB8);  // CS Pin

struct can_frame canMsg;

void setup() {
    Serial.begin(115200);

    SPI.setMISO(PB4);
    SPI.setMOSI(PB5);
    SPI.setSCLK(PB3);
    SPI.begin();
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    
    Serial.println("CAN Transmitter Ready!");
}

void loop() {
    // Generate random sensor values
    int humidity = random(50, 100);  // Simulated humidity (50-100%)
    int temperature = random(20, 40); // Simulated temperature (20-40°C)
    int pressure = random(950, 1050); // Simulated pressure (950-1050 hPa)

    // First CAN message (ID 0x036) - Humidity & Temperature
    canMsg.can_id  = 0x036;
    canMsg.can_dlc = 4;
    canMsg.data[0] = humidity;
    canMsg.data[1] = temperature;
    canMsg.data[2] = 0xAA; // Example fixed value
    canMsg.data[3] = 0xBB;

    mcp2515.sendMessage(&canMsg);
    Serial.print("Sent CAN ID: 0x036 - Humidity: ");
    Serial.print(humidity);
    Serial.print("%, Temperature: ");
    Serial.print(temperature);

    delay(500);

    // Second CAN message (ID 0x045) - Pressure
    canMsg.can_id  = 0x045;
    canMsg.can_dlc = 2;
    canMsg.data[0] = pressure >> 8;  // High byte
    canMsg.data[1] = pressure & 0xFF; // Low byte

    mcp2515.sendMessage(&canMsg);
    Serial.print("  -   Sent CAN ID: 0x045 - Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");

    delay(1000);
}
