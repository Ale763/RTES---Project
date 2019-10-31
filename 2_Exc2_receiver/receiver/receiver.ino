#include <LoRa.h>

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    868E6 //915E6
#define PABOOST true

int counter = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("LoRa Sender");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND, PABOOST))
  {
    Serial.println("Starting LoRa failed!");
    while(1);
  }
}

void loop() {
  
}

void receive() 
{
    // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) 
  {
    // Received a packet
    Serial.print("Received packet");

    //Read packet
    while(LoRa.available()) 
    {
      Serial.print((char)LoRa.read());  
    }

    //Print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
