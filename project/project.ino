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

int packetCounter = 0;
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
  Serial.println(sizeof(float));
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
//    receive0 = millis();
    sleepTime = receive();
    send(sleepTime, chipTemp(chipTempRaw()));
    packetCounter++;
    if (packetCounter == 20) break;
//    receive1 = millis();
//    int computationDelay = (receive1 - receive0) + 500;
//    if (computationDelay < 0) computationDelay = 500;
//    receiveDelay = (sleepTime - computationDelay) / portTICK_PERIOD_MS;
//    Serial.print("computationDelay: ");
//    Serial.println(computationDelay);
//    Serial.print("receiveDelay: ");
//    Serial.println(receiveDelay);
//    if (receiveDelay <= 0) receiveDelay = 1 ;
    milisBefore = millis();
    receiveDelay = (sleepTime - (0.045 * sleepTime) - 150) / portTICK_PERIOD_MS;
    vTaskDelay(receiveDelay);
    milisAfter = millis();
    Serial.print("Seconds slept: ");
    Serial.println(milisAfter-milisBefore);
  }
  Serial.println("Deep sleep mode");
}

void send(int sleepTime, float temp) {
  byte* sleepTimeBytes = (byte*)&sleepTime;
  byte* tempBytes = (byte*)&temp;
  
  LoRa.beginPacket();
  LoRa.write(sleepTimeBytes, sizeof(sleepTime));
  LoRa.write(tempBytes, sizeof(temp));
  LoRa.endPacket();
}

int receive()
{
//  Serial.print("Listening for packet: ");
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  while (packetSize == 0)
  {
    packetSize = LoRa.parsePacket();
  }
  // Received a packet
//  Serial.println("Received packet ");

  //Read packet
  byte buffer[sizeof(int)] = {};
  int i = 0;

  Serial.print("Reading packet: ");
  while (LoRa.available() && i< sizeof(int))
  {
    // Serial.println(LoRa.read());
    buffer[i++] = LoRa.read();
//    Serial.print(buffer[i-1]);
  }
  int input = (int)(buffer[0] | buffer[1] << 8);
  Serial.print("Packet: ");
  Serial.println(packetCounter);
  Serial.print("Received & converted sleepTime: ");
  Serial.println(input);

  return input;
}

float chipTemp(float raw) {
  const float chipTempOffset = -142.5;
  const float chipTempCoeff = .558;
  return((raw * chipTempCoeff) + chipTempOffset);
}

int chipTempRaw(void) {
  ADCSRA &= ~(_BV(ADATE) |_BV(ADIE));       // Clear auto trigger and interrupt enable
  ADCSRA |= _BV(ADEN) | _BV(ADSC);          // Enable ADC and start conversion
  
  while((ADCSRA & _BV(ADSC)));              // Wait until conversion is mobdro finished   
  int result = (ADCL | (ADCH << 8));
  return result;
}
