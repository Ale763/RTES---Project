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
#define configUSE_TICKLESS_IDLE 0

// Uncomment the line appropriate for your platform
#define TABLE_SIZE 512 // Arduino 168 or greater

// The number of demo records that should be created.  This should be less
// than (TABLE_SIZE - sizeof(EDB_Header)) / sizeof(LogEvent).  If it is higher,
// operations will return EDB_OUT_OF_RANGE for all records outside the usable range.
#define READ_LATEST_TEMP  1
#define READ_ALL_TEMP     2
#define ENABLE_LOW_OP     3
#define TABLE_SIZE        512
#define RECORDS_TO_CREATE 10



bool DEBUG  = true;
int packetCounter = 0;
int sleepTime = 0;

const int wakeUpPin = 2;

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
TaskHandle_t consumeQueueTaskHandle;
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

QueueHandle_t loraQueue;

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
  debugPrintln("PR booting...");

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
  if (db.create(0, TABLE_SIZE, sizeof(dBEntry)) != EDB_OK)
  {
    debugPrintln("PR booting failed - EDB could not be initialized");
    while (1);
  }

  // Initializing Queue
  loraQueue = xQueueCreate(20, sizeof(dBEntry));
  if (loraQueue == NULL)
  {
    debugPrintln("PR booting failed - Queue could not be initialized");
    while (1);
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
    3,    // priority
    &receiveBeaconTaskHandle);

  /* Send Beacon Task */
  xTaskCreate(
    consumeQueue,
    "consumeQueue",
    128,  // Stack size
    NULL, //paramaters task
    1,    // priority
    &consumeQueueTaskHandle);

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

// --------------------------------------------------------------------------------------
// Tasks

void handleUserCommands(void *pvParameters)
{
  while (1)
  {
    int userInput;

    while (Serial.available() == 0) {
      vTaskDelay(500 / portTICK_PERIOD_MS); // One tick delay in between reads so that other processes get the chance to run.
    }
    userInput = Serial.parseInt();

    if (userInput == READ_LATEST_TEMP)
    {
      debugPrintln("Execute - Read latest temperature entry");
      readLatest();
    }
    else if (userInput == READ_ALL_TEMP)
    {
      debugPrintln("Execute - Read all temperature entries");
      readAll();
    }
    else if (userInput == ENABLE_LOW_OP)
    {
      debugPrintln("Execute - Enable low operation mode");
      deepSleep();
    }

  }
}

void receiveBeacon(void *pvParameters)
{
  unsigned long milisBefore;
  unsigned long milisAfter;
  while (1)
  {
    debugPrint("Packet: "); debugPrintln(packetCounter);
    sleepTime = receive();
    packetCounter++;
    milisBefore = millis();
    receiveDelay = (sleepTime - (0.060 * sleepTime) - 70) / portTICK_PERIOD_MS;
    vTaskDelay(receiveDelay);
    milisAfter = millis();
    debugPrint("Seconds slept: ");
    int sleepduration = (int)(milisAfter - milisBefore);
    debugPrint("Sleepduration "); debugPrintln(sleepduration);
    if (packetCounter == 20) deepSleep();
  }

}

void consumeQueue(void *pvParameters)
{
  struct DBEntry local;
  while (1)
  {
    if (xQueueReceive(loraQueue, &local, portMAX_DELAY) == pdPASS)
    {
      local.temperature = chipTemp(chipTempRaw());
      writeToDB(&local);
      send(local.temperature);
    }
    vTaskDelay(1);
  }


}

// --------------------------------------------------------------------------------------
// Communication functionality

void send(float temp)
{
  // byte* sleepTimeBytes = (byte*)&sleepTime;
  // byte* tempBytes = (byte*)&temp;

  char tempbuffer[6];
  dtostrf(temp, 4, 2, tempbuffer);

  LoRa.beginPacket();
  LoRa.print(tempbuffer);
  LoRa.endPacket();
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
  char buffer[7];
  memset(buffer, '\0', 7);

  // Fill buffer from LoRa
  int i = 0;
  while (LoRa.available() && i < 7)
  {
    buffer[i] = (char)LoRa.read();
    Serial.print(buffer[i++]);
  }
  Serial.println("");

  char id[5];
  parseID(buffer, id);
  int sleep;
  parseSleep(buffer, &sleep);

  struct DBEntry local;
  memcpy(local.id, id, sizeof(id));
  local.sleepduration = sleep;
  xQueueSend(loraQueue, &local, portMAX_DELAY);

  return sleep;
}

void parseID(char* buffer, char* id)
{
  // Get ID
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
  if (*sleep < 2000 || *sleep == NULL)
    *sleep = 2000; // If sleep is faulty parsed, return minimum sleeptime
}

// --------------------------------------------------------------------------------------
// deep sleep methods
// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void deepSleep()
{


  //disable tasks. lower CPU speed?
  vTaskSuspendAll();

  // Disable USB if it exists
#ifdef USBCON
  USBDevice.detach();
#endif


  TXLED0;      //To turn TXLED pin off
  RXLED0;      //To turn RXLED pin off

  ADCSRA &= ~(1 << ADEN); //what is this?
  power_all_disable();  // turn off all modules

  //LoRa.sleep();

  // Disable digital input buffers on all analog input pins
  // by setting bits 0-5 to one.
  DIDR0 = DIDR0 | B00111111;

  // Set sleep to full power down.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  cli(); //disable interrupts because next section is critical.


  wdt_disable(); // disable watchdog timer interrupt.


  //define interrupt
  attachInterrupt(digitalPinToInterrupt(DI0), wakeUp, FALLING);

  sei(); //enable interrupts

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!

  // On wakeUp...

  detachInterrupt(digitalPinToInterrupt(DI0));
  Serial.begin(9600);
  // put everything on again
  power_all_enable();


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

float chipTemp(float raw)
{
  const float chipTempOffset = -142.5;
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
    currentRecordNo++;
    db.appendRec(EDB_REC * entry);
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
    for (i = 0; i < currentRecordNo; i++)
    {
      db.readRec(i, EDB_REC dBEntry);
      readEntry(dBEntry);
    }
    xSemaphoreGive(dbSemaphoreHandle);
  }
}

void readEntry(struct DBEntry entry)
{
  debugPrint("ID: ");
  debugPrintln(entry.id);
  debugPrint("Time: ");
  debugPrintln(entry.sleepduration);
  debugPrint("Temp: ");
  debugPrintln(entry.temperature);
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
