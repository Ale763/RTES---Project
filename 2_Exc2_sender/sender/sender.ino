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
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send packet
  LoRa.beginPacket();
  LoRa.print("Hello");
  LoRa.print(counter);
  LoRa.endPacket();
  counter++;
  delay(5000);

}
