#include <LoRa.h>
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <semphr.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <EDB.h>
#include <EEPROM.h>

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    8695E5 //915E6
#define PABOOST true

// Uncomment the line appropriate for your platform
#define TABLE_SIZE 512 // Arduino 168 or greater

// The number of demo records that should be created.  This should be less 
// than (TABLE_SIZE - sizeof(EDB_Header)) / sizeof(LogEvent).  If it is higher, 
// operations will return EDB_OUT_OF_RANGE for all records outside the usable range.
#define RECORDS_TO_CREATE 10

#define READ_LATEST_TEMP  1
#define READ_ALL_TEMP     2
#define ENABLE_LOW_OP     3



int packetCounter = 0;
int sleepTime = 0;

int currentRecordNo = 0;
float currentTemp;
String incomingPacket;
String inputPacket = "Ping";
bool sending = true;
unsigned long receive0 = 0;
unsigned long receive1 = 0;
TaskHandle_t receiveBeaconTaskHandle;
TickType_t receiveDelay;
TaskHandle_t userCommandTaskHandle;
// TickType_t receiveDelay;


struct DBEntry {
  int sleepduration;
  float temperature;
} 
dBEntry;

// The read and write handlers for using the EEPROM Library
void writer(unsigned long address, byte data)
{
  EEPROM.write(address, data);
}

byte reader(unsigned long address)
{
  return EEPROM.read(address);
}


EDB db(&writer, &reader);

void setup() 
{
  Serial.begin(9600);                             // Set data rate in bits for serial transmission
  while(!Serial);                                 // Wait for Serial to become ready
  Serial.println("PR booting...");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND, PABOOST))
  {
    Serial.println("PR booting failed!");
    while(1);
  }

  db.create(0, TABLE_SIZE, sizeof(dBEntry));

  writeToDB(1,2);
  writeToDB(2,4);
  /* Setup tasks */
  /* Send Beacon Task */
  xTaskCreate(
      receiveBeacon,
      "receiveBeacon",
      128,  // Stack size
      NULL, //paramaters task
      1,    // priority
      &receiveBeaconTaskHandle);

  /* User Command Task */
  xTaskCreate(
      handleUserCommands,
      "handleUserCommands",
      128,  // Stack size
      NULL, //paramaters task
      2,    // priority
      &userCommandTaskHandle);
}

void loop() 
{
  
}


void readLatest()
{
    db.readRec(currentRecordNo, EDB_REC dBEntry);
    Serial.print("Time: "); Serial.println(dBEntry.sleepduration);
    Serial.print("Temp: "); Serial.println(dBEntry.temperature);  
}

void readAll()
{
  int i;
  for( i= 0; i < currentRecordNo; i++)
  {
    db.readRec(i, EDB_REC dBEntry);
    Serial.print("Time: "); Serial.println(dBEntry.sleepduration);
    Serial.print("Temp: "); Serial.println(dBEntry.temperature);  
  }
  
}

void handleUserCommands(void *pvParameters)
{
  while(1)
  {
    int userInput;

    while (Serial.available() == 0){
      vTaskDelay(1);  // One tick delay in between reads so that other processes get the chance to run.
    }
    userInput = Serial.parseInt();

    if (userInput == READ_LATEST_TEMP)
    { 
      Serial.println("Execute - Read latest temperature entry");
      readLatest();
    }
    else if (userInput == READ_ALL_TEMP)
    {
      Serial.println("Execute - Read all temperature entries");
      readAll();
    }
    else if (userInput == ENABLE_LOW_OP) 
    {
      Serial.println("Execute - Enable low operation mode");
    }
    
  }
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
    currentTemp = chipTemp(chipTempRaw());
    send(sleepTime, currentTemp);
    writeToDB(sleepTime, currentTemp);
    packetCounter++;
    milisBefore = millis();
    receiveDelay = (sleepTime - (0.045 * sleepTime) - 150) / portTICK_PERIOD_MS;
    vTaskDelay(receiveDelay);
    milisAfter = millis();
    Serial.print("Seconds slept: ");
    Serial.println(milisAfter-milisBefore);
    if (packetCounter == 20) break;
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

void writeToDB(int sleepTime, float temp){

    if( currentRecordNo == RECORDS_TO_CREATE)
    {
        Serial.println("Table full exception");
    }
    currentRecordNo++;
    dBEntry.sleepduration = sleepTime;
    dBEntry.temperature = temp;
    db.appendRec(EDB_REC dBEntry);
}

int receive()
{
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  // Keep waiting for packet
  while (packetSize == 0)
  {
    packetSize = LoRa.parsePacket();
  }

  //Read packet
  byte buffer[sizeof(int)] = {};
  int i = 0;

  Serial.println("Reading packet: ");
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
