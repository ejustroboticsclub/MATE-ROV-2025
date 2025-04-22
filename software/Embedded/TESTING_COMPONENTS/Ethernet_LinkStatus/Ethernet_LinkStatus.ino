#include <SPI.h>
#include <LoayEthernet.h>

#define CS_pin PA15
#define SCLK_pin    PA5
#define MISO_pin    PA6
#define MOSI_pin    PA7
/*
W5500 Module   ->   STM32F401 
-------------------------------
CS             ->   PA3
MOSI           ->   PA7
MISO           ->   PA6
SCK            ->   PA5
RESET          ->   RESET
VCC            ->   5V or 3.3V
GND            ->   GND
*/
  
byte mac[] = { 0xAA, 0xEE, 0xCC, 0xEE, 0xDD, 0xEE };  // Your MAC address
//IPAddress ip(192, 168, 1, 200);                       // Static IP address

EthernetServer server(80);  // Start HTTP server on port 80

void setup() {
  Serial.begin(9600);
  // Initialize Ethernet with DHCP
  Ethernet.begin(mac, CS_pin, SCLK_pin, MISO_pin, MOSI_pin);

  Serial.println("Ethernet initialized!");
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());

  // Start the server 
  server.begin();
}

void loop() {
  EthernetClient client = server.available();

  if (client) {
    Serial.println("Client connected");

    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);  // Print the request to Serial Monitor

        // HTTP response
        if (c == '\n' && currentLineIsBlank) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/plain");
          client.println("Connection: close");
          client.println();
          client.println("Hello World, from STM32!");
          break;
        }

        if (c == '\n') {
          currentLineIsBlank = true;
        } else if (c != '\r') {
          currentLineIsBlank = false;
        }
      }
    }
    delay(1);
    client.stop();  // Close the connection
    Serial.println("Client disconnected");
  }
}
