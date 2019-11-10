#include <LoRa.h>
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <semphr.h>
#include <avr/wdt.h>
#include <avr/power.h>
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    868E6 //915E6
#define PABOOST true

int counter = 0;
int sleepTime = 0;
String incomingPacket;
String inputPacket = "Ping";
bool sending = true;
unsigned long receive0 = 0;
unsigned long receive1 = 0;
TaskHandle_t receiveBeaconTaskHandle;
TickType_t receiveDelay;
void setup() 
{
  Serial.begin(9600);                             // Set data rate in bits for serial transmission
  while(!Serial);                                 // Wait for Serial to become ready
  Serial.println("PR booting...");
  Serial.print("Sizeof int");
  Serial.println(sizeof(int));
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND, PABOOST))
  {
    Serial.println("PR booting failed!");
    while(1);
  }

  /* Setup tasks */
  /* Send Beacon Task */
  xTaskCreate(
      receiveBeacon,
      "receiveBeacon",
      128,  // Stack size
      NULL, //paramaters task
      3,    // priority
      &receiveBeaconTaskHandle);
}

void loop() {
  
  //incomingPacket = receive();
}

void receiveBeacon(void *pvParameters)
{
  unsigned long milisBefore;
  unsigned long milisAfter;
  String sleepTimeString = "";
  while(1)
  {
    receive0 = millis();
    Serial.println("Before receive");
    sleepTime = receive();
    Serial.println("After receive");
    // sleepTime = sleepTimeString.toInt();
    Serial.print("sleepTime: ");
    Serial.println(sleepTime);
    receive1 = millis();
    receiveDelay = (sleepTime - (receive1 - receive0) - 200) / portTICK_PERIOD_MS;

    milisBefore = millis();
    vTaskDelay(receiveDelay);
    milisAfter = millis();
    Serial.println(milisAfter-milisBefore);
  }
}

void send(int packet) {
  String packetString = String(packet);
  //Serial.println("Sending packet: ");
  LoRa.beginPacket();
  //Serial.println("Begin sending packet: ");
  LoRa.print(packetString);
  //Serial.println("Ending sending packet: ");
  LoRa.endPacket();
  //Serial.println("Packet sent");
}

int receive()
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
  String input;
  while (LoRa.available())
  {
    // Serial.println(LoRa.read());
    input += (char)LoRa.read();
  }
  Serial.println(input.toInt());

  return input.toInt();
}
