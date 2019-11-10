#include "Arduino.h"
#include <LoRa.h>
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
unsigned long send0 = 0;
unsigned long send1 = 0;

TaskHandle_t sendBeaconTaskHandle;
TaskHandle_t receiveBeaconTaskHandle;
TickType_t sendDelay;

void send(char* packet) {

  Serial.println("Sending packet: ");
  LoRa.beginPacket();
  LoRa.print(packet);
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
  Serial.print("Received packet: ");

  //Read packet
  String input = "";
  while(LoRa.available()) 
  {
    input += (char)LoRa.read();
  }
  Serial.println(input);  
  return input;
}



void sendBeacon(void *pvParameters)
{
  unsigned long milisBefore;
  unsigned long milisAfter;
  char sleepTimeString[6];
  while(1)
  {
    send0 = millis();
    sleepTime = (rand() % 9 + 2);
    String ch = String ("sleepTime");
  
    Serial.print("sleepTime: ");
    Serial.println(ch);
    Serial.print("sleepTimeString: ");
    itoa(sleepTime, sleepTimeString, 10);
    String str = String(sleepTimeString);
    Serial.println(str);

    send(sleepTimeString);
    send1 = millis();

    sendDelay = (sleepTime - (send1 - send0) - 0.04*sleepTime+50) / portTICK_PERIOD_MS;
    if (sendDelay< 0) sendDelay = 0;

//    Serial.print("sendDelay: ");
//    Serial.println(sendDelay);
    milisBefore = millis();
    vTaskDelay(sendDelay);
    milisAfter = millis();
    Serial.println(milisAfter-milisBefore);
  }
}

void receiveBeaconResponse(void *pvParameters) 
{
  unsigned long milisBefore;
  unsigned long milisAfter;
  String sleepTimeString = "";
  while (1)
  {
    /*receive0 = millis();
    Serial.println("Before receive");
    sleepTimeString = receive();
    Serial.println("After receive");
    sleepTime = sleepTimeString.toInt();
    Serial.print("sleepTime: ");
    Serial.println(sleepTime);
    receive1 = millis();
    receiveDelay = (sleepTime - (receive1 - receive0) - 200) / portTICK_PERIOD_MS;

    milisBefore = millis();
    vTaskDelay(receiveDelay);
    milisAfter = millis();
    Serial.println(milisAfter - milisBefore);*/
  }
}




void setup() 
{
  Serial.begin(9600);                             // Set data rate in bits for serial transmission
  while(!Serial);                                 // Wait for Serial to become ready
  Serial.println("BBB Sender booting...");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND, PABOOST))
  {
    Serial.println("BBB Sender booting failed!");
    while(1);
  }

  /* Setup tasks */
  /* Send Beacon Task */
  xTaskCreate(
      sendBeacon,
      "sendBeacon",
      128,  // Stack size
      NULL, //paramaters task
      3,    // priority
      &sendBeaconTaskHandle);

  Serial.print("TIME: ");
  Serial.println(millis());
}

void loop() {
  
  //incomingPacket = receive();
}

