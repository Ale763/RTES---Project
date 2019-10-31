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
String inputPacket = "Ping";
bool starter = true;
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
  if (starter) 
  {
    send(inputPacket);
    starter = false;
  }
  inputPacket = receive();
  delay(2000);
  send(inputPacket);

}

void send(String packet) {
  Serial.println("Sending packet: ");
  LoRa.beginPacket();
  Serial.println("Begin sending packet: ");
  LoRa.print("Message: ");
  LoRa.print("Ping");
  Serial.println("Ending sending packet: ");
  LoRa.endPacket();
  Serial.println("Packet sent");
}

String receive() 
{
  Serial.print("Listening for packet: ");
    // Try to parse packet
  int packetSize = LoRa.parsePacket();
  while (packetSize == 0)
  {
    packetSize = LoRa.parsePacket();
  }
  // Received a packet
  Serial.print("Received packet ");

  //Read packet
  String input = "";
  while(LoRa.available()) 
  {
    input += (char)LoRa.read();
  }
  Serial.print("Packet: ");
  Serial.println(input);  
  return input;
}
