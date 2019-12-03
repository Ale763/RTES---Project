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
#include <queue.h>


#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    8695E5 //915E6
#define PABOOST true
#define configUSE_IDLE_HOOK 1

//source: https://github.com/jwhiddon/EDB
#define TABLE_SIZE 512 // Arduino 168 or greater

// The number of demo records that should be created.  This should be less
// than (TABLE_SIZE - sizeof(EDB_Header)) / sizeof(LogEvent).  If it is higher,
// operations will return EDB_OUT_OF_RANGE for all records outside the usable range.
#define READ_LATEST_TEMP  1
#define READ_ALL_TEMP     2
#define ENABLE_LOW_OP     3
#define DESTROY_DB        4
#define TABLE_SIZE        2048
#define RECORDS_TO_CREATE 150

// source: https://github.com/rocketscream/Low-Power/blob/master/LowPower.cpp
// Timer 4 PRR bit is currently not defined in iom32u4.h
#ifndef PRTIM4
  #define PRTIM4 4
#endif

// Timer 4 power reduction macro is not defined currently in power.h
#ifndef power_timer4_disable
  #define power_timer4_disable() (PRR1 |= (uint8_t)(1 << PRTIM4))
#endif

#ifndef power_timer4_enable
  #define power_timer4_enable() (PRR1 &= (uint8_t) ~(1 << PRTIM4))
#endif

bool DEBUG  = false;
int amtOfPackagesBeforeSleep = 20;
int packetCounter = 0;

const int wakeUpPin = 3;

int currentRecordNo = 0;
TaskHandle_t receiveBeaconTaskHandle;
TickType_t receiveDelay;
TaskHandle_t userCommandTaskHandle;
//TaskHandle_t consumeQueueTaskHandle;
SemaphoreHandle_t dbSemaphoreHandle;

// The read and write handlers for using the EEPROM Library
void
writer(unsigned long address, byte data)
{
  EEPROM.write(address, data);
}
byte reader(unsigned long address) {
  return EEPROM.read(address);
}
// Create an EDB object with the appropriate write and read handlers
EDB db(&writer, &reader);

//QueueHandle_t loraQueue;
QueueHandle_t sleepQueue; 

struct DBEntry {
  char id[5];
  int sleepduration;
  float temperature;
}
dBEntry;

void setup()
{
  Serial.begin(9600);                             // Set data rate in bits for serial transmission
  // Wait for Serial to become ready
  // while (!Serial) {;}

  // Initializing LoRa
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND, PABOOST))
  {
    debugPrintln("PR booting failed - LoRa could not be initialized");
    while (1);
  }

  dbSemaphoreHandle = xSemaphoreCreateBinary();
  if (dbSemaphoreHandle == NULL)
  {
    debugPrintln("PR booting failed - DB Semaphore could not be initialized");
    while (1);
  }
  xSemaphoreGive(dbSemaphoreHandle);

  // Initializing EDB  
  bool filled = db.open(0) == EDB_OK && db.readRec(1, EDB_REC dBEntry) == EDB_OK;
  if (!filled)
  {
    if (db.create(0, TABLE_SIZE, sizeof(dBEntry)) != EDB_OK)
    {
      debugPrintln("PR booting failed - EDB could not be initialized");
      while (1);
    }
  }
  else
  {
    currentRecordNo = db.count();
  }
  
  // Initializing Queue
  sleepQueue = xQueueCreate(20, sizeof(int));
  if (sleepQueue == NULL)
  {
    debugPrintln("PR booting failed - sleepQueue could not be initialized");
    while (1)
      ;
  }

  // Initializing temperature sensor
  ADMUX = _BV(REFS1) | _BV(REFS0) | 7; // Set internal V reference, temperature reading
  ADCSRB = 0x20;                       // ref  24.6
  delay(100);
  chipTempRaw(); // discard first sample

  // Set waku-up pin
  pinMode(wakeUpPin, INPUT);

  /* Setup tasks */
  /* Send Beacon Task */
  xTaskCreate(
    receiveBeacon,
    "receiveBeacon",
    128,  // Stack size
    NULL, //paramaters task
    2,    // priority
    &receiveBeaconTaskHandle);

  /* User Command Task */
  xTaskCreate(
    handleUserCommands,
    "handleUserCommands",
    128,  // Stack size
    NULL, //paramaters task
    1,    // priority
    &userCommandTaskHandle);
}

void loop()
{

}

// --------------------------------------------------------------------------------------
// Sleep utility
int calculateSleepTime(int sleeptime)
{
  return ((int)(sleeptime*0.9572224005-39.20652397)) / portTICK_PERIOD_MS;
}

// --------------------------------------------------------------------------------------
// Tasks

void handleUserCommands(void *pvParameters)
{
  int sleeptime = 0;
  while (1)
  {
    int userInput;
    if (Serial.available() == 0) 
    {
      while (xQueueReceive(sleepQueue, &sleeptime, 0) == pdPASS)
      {
        sleeptime = calculateSleepTime(sleeptime);
        if(xQueuePeek(sleepQueue, &sleeptime, 0) != pdPASS)
        {
          break;
        }
      }
      vTaskDelay(sleeptime);      
    }else{
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
        // Will not get printed because deepSleep is faster than the output to the Serial
        // Serial.println("Execute - Enable low operation mode");
        deepSleep();
      }
      else if (userInput == DESTROY_DB)
      {
        if (xSemaphoreTake(dbSemaphoreHandle, portMAX_DELAY) == pdTRUE)
        {
          Serial.println("Execute - Clear database");
          currentRecordNo = 0;
          if (db.create(0, TABLE_SIZE, sizeof(dBEntry)) != EDB_OK)
            debugPrintln("Error destroying db");
          xSemaphoreGive(dbSemaphoreHandle);
        }
      }
    } 
  }
}

void receiveBeacon(void *pvParameters)
{
  int sleeptime = 0;
  int before;
  int after;
  while (1)
  {
    debugPrint("Packet: "); debugPrintln(packetCounter);
    
    char rawBeacon[7];
    receive(rawBeacon);
    struct DBEntry beacon = processBeacon(rawBeacon);
    sleeptime = calculateSleepTime(beacon.sleepduration);
    
    packetCounter++;
    if (packetCounter == amtOfPackagesBeforeSleep)
      deepSleep();

    before = millis();
    vTaskDelay(sleeptime);
    after = millis();
    debugPrint("Sleep: ");debugPrintln(after-before);
  }
}

void vApplicationIdleHook(void)
{
  digitalWrite(LED_BUILTIN, LOW);

  power_timer4_disable();
  power_timer3_disable();
  power_timer1_disable();

  power_usart1_disable();
  power_twi_disable();
  
  // Disable ADC
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();

  set_sleep_mode(SLEEP_MODE_IDLE);
  cli();
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();
  sei();

  // Enable ADC
  power_adc_enable();
  ADCSRA |= (1 << ADEN);
  
  power_timer4_enable();
  power_timer3_enable();
  power_timer1_enable();

  power_usart1_enable();
  power_twi_enable();

  
}

// --------------------------------------------------------------------------------------
// Communication functionality

void send(float temp)
{
  char tempbuffer[6];
  dtostrf(temp, 4, 2, tempbuffer);

  LoRa.beginPacket();
  LoRa.print(tempbuffer);
  LoRa.endPacket();
}

char* receive(char* buffer)
{
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  // Keep waiting for packet
  while (packetSize == 0)
  {
    packetSize = LoRa.parsePacket();
  }
  //char buffer[7];
  memset(buffer, '\0', 7);

  // Fill buffer from LoRa
  int i = 0;
  while (LoRa.available() && i < 7)
  {
    buffer[i] = (char)LoRa.read();
    i++;
  }

  debugPrintln(buffer);

  return buffer;
  //return sleep;
}

struct DBEntry processBeacon(char* buffer)
{
  struct DBEntry local;
  char id[5];
  parseID(buffer, id);
  int sleep;
  parseSleep(buffer, &sleep);
  
  memcpy(local.id, id, sizeof(id));
  local.sleepduration = sleep;
  local.temperature = chipTemp(chipTempRaw());
  debugPrintln(local.temperature);
  send(local.temperature);
  writeToDB(&local);
  xQueueSend(sleepQueue, &sleep, 0);
}

void parseID(char* buffer, char* id)
{
  for (int j = 0; j < 4; ++j)
  {
    id[j] = buffer[j];
  }
  id[4] = '\0';
}

void parseSleep(char* buffer, int* sleep)
{
  char sleeptime[3];
  // Get sleeptime
  for (int j = 4; j < 6; ++j)
  {
    sleeptime[j - 4] = buffer[j];
  }
  sleeptime[2] = '\0';
  *sleep = atoi(sleeptime) * 1000;
  if (*sleep < 2000 || *sleep == NULL || *sleep > 10000)
    *sleep = 2000; // If sleep is faulty parsed, return minimum sleeptime
}

// --------------------------------------------------------------------------------------
// deep sleep methods
// Put the Arduino to deep sleep. Only an interrupt can wake it up.
// used ideas and some snippets from: http://www.gammon.com.au/forum/?id=11497
void deepSleep()
{

  digitalWrite(LED_BUILTIN, LOW);

  //disable tasks. lower CPU speed?
  vTaskSuspendAll();

  // Disable USB if it exists
#ifdef USBCON
  USBDevice.detach();
#endif


  TXLED0;      //To turn TXLED pin off
  RXLED0;      //To turn RXLED pin off
  
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;  //disable ADC
  
  power_all_disable();  // turn off all modules

  // Ensure no floating pins
  for(int i=0; i<A5 ; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // Disable digital input buffers on all analog input pins
  // by setting bits 0-5 to one.
  DIDR0 = DIDR0 | B00111111;

  // Set sleep to full power down.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  cli(); //disable interrupts because next section is critical.


  wdt_disable(); // disable watchdog timer interrupt.


  //define interrupt
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);

  sei(); //enable interrupts

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!

  // On wakeUp...

  detachInterrupt(digitalPinToInterrupt(wakeUpPin));
  Serial.begin(9600);
  // put everything on again
  power_all_enable();

  ADCSRA = old_ADCSRA;

  // Enable USB if it exists
#if defined(USBCON) && !defined(USE_TINYUSB)
  USBDevice.attach();
#endif

  xTaskResumeAll();


}

void wakeUp()
{


}


// --------------------------------------------------------------------------------------
// Temperature sensor methods
// source: https://forum.pjrc.com/threads/54445-Teensy-(ATmega32U4)-on-chip-temperature-sensor

float chipTemp(float raw)
{
  const float chipTempOffset = -142.4;
  const float chipTempCoeff = .558;
  return ((raw * chipTempCoeff) + chipTempOffset);
}

int chipTempRaw(void)
{
  ADCSRA &= ~(_BV(ADATE) | _BV(ADIE)); // Clear auto trigger and interrupt enable
  ADCSRA |= _BV(ADEN) | _BV(ADSC);     // Enable ADC and start conversion

  while ((ADCSRA & _BV(ADSC))); // Wait until conversion is mobdro finished
  int result = (ADCL | (ADCH << 8));
  return result;
}

// --------------------------------------------------------------------------------------
// EDB functionality

void writeToDB(struct DBEntry *entry)
{
  if (xSemaphoreTake(dbSemaphoreHandle, portMAX_DELAY) == pdTRUE)
  {
    if (currentRecordNo == RECORDS_TO_CREATE)
    {
      Serial.println("Table full exception");
    }
    db.appendRec(EDB_REC *entry);
    currentRecordNo++;
    xSemaphoreGive(dbSemaphoreHandle);
  }
}

void readLatest()
{
  if (xSemaphoreTake(dbSemaphoreHandle, portMAX_DELAY) == pdTRUE)
  {
    db.readRec(currentRecordNo, EDB_REC dBEntry);
    readEntry(dBEntry);
    xSemaphoreGive(dbSemaphoreHandle);
  }
}

void readAll()
{
  if (xSemaphoreTake(dbSemaphoreHandle, portMAX_DELAY) == pdTRUE)
  {
    int i;
    for (i = 1; i <= currentRecordNo; i++)
    {
      db.readRec(i, EDB_REC dBEntry);
      readEntry(dBEntry);
    }
    xSemaphoreGive(dbSemaphoreHandle);
  }
}

void readEntry(struct DBEntry entry)
{
  Serial.print("\tID: ");
  Serial.println(entry.id);
  Serial.print("\tTime: ");
  Serial.println(entry.sleepduration);
  Serial.print("\tTemp: ");
  Serial.println(entry.temperature);
}

// --------------------------------------------------------------------------------------
// Debugprint

void debugPrint(String message)
{
  if (DEBUG)
  {
    Serial.print(message);
  }
}

void debugPrintln(String message)
{
  if (DEBUG)
  {
    Serial.println(message);
  }
}

void debugPrint(char* message)
{
  if (DEBUG)
  {
    Serial.print(message);
  }
}

void debugPrintln(char* message)
{
  if (DEBUG)
  {
    Serial.println(message);
  }
}

void debugPrintln(int message)
{
  if (DEBUG)
  {
    Serial.println(message);
  }
}

void debugPrintln(float message)
{
  if (DEBUG)
  {
    Serial.println(message);
  }
}
