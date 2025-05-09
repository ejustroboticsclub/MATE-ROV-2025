/*
 modified 12 Aug 2013
 by Soohwan Kim (suhwan@wiznet.co.kr)

 - 10 Apr. 2015
 Added support for Arduino Ethernet Shield 2
 by Arduino.org team
 
 */
#ifndef ethernet_h
#define ethernet_h

#include <inttypes.h>
#include "utility/w5500.h"
#include "IPAddress.h"
#include "EthernetClient.h"
#include "EthernetServer.h"
#include "Dhcp.h"



class EthernetClass {
private:
  IPAddress _dnsServerAddress;
  char* _dnsDomainName;
  char* _hostName;
  DhcpClass* _dhcp;
public:
  uint8_t w5500_cspin;

  static uint8_t _state[MAX_SOCK_NUM];
  static uint16_t _server_port[MAX_SOCK_NUM];

  EthernetClass() { _dhcp = NULL; w5500_cspin = PA15; }
  void init(uint8_t _cspin = PA15) { w5500_cspin = _cspin; }

#if defined(WIZ550io_WITH_MACADDRESS)
  // Initialize function when use the ioShield serise (included WIZ550io)
  // WIZ550io has a MAC address which is written after reset.
  // Default IP, Gateway and subnet address are also writen.
  // so, It needs some initial time. please refer WIZ550io Datasheet in details.
  //int begin(uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
  //void begin(IPAddress local_ip, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
  //void begin(IPAddress local_ip, IPAddress dns_server, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
  //void begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
  //void begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
#else
  // Initialize the Ethernet shield to use the provided MAC address and gain the rest of the
  // configuration through DHCP.
  // Returns 0 if the DHCP configuration failed, and 1 if it succeeded
  int begin(uint8_t *mac_address, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
  //void begin(uint8_t *mac_address, IPAddress local_ip, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
  //void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
  //void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);
  //void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet, uint8_t w5500_cspin, uint8_t Sclk, uint8_t miSO, uint8_t moSI);

#endif
  
  int maintain();

  IPAddress localIP();
  IPAddress subnetMask();
  IPAddress gatewayIP();
  IPAddress dnsServerIP();
  char* dnsDomainName();
  char* hostName();

  friend class EthernetClient;
  friend class EthernetServer;
};

extern EthernetClass Ethernet;

#endif
