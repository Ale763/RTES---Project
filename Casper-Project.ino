#include <Arduino_FreeRTOS.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <semphr.h>
#include <EDB.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/power.h>
#define TABLE_SIZE 512

/* Task handles */
TaskHandle_t TaskHandle_UserInput;
TaskHandle_t TaskHandle_TemperatureReadStore;
TaskHandle_t TaskHandle_Crash;

/* Delay times */
const TickType_t tempStoreDelay = 500 / portTICK_PERIOD_MS; // 500 ms
const TickType_t crashDelay = 144 / portTICK_PERIOD_MS;     // 144 ms

/* Database */
EDB db(&writer, &reader);
int temperatureTimestamp = 1;

/* Used pins */
int inPinWakeUp = 3;  // Sleep in
int inPinCrash = 2;   // Crash in
int outPinCrash = 13; // Crash out

/* Sleeping flag */
bool isSleeping = false;

/* Database mutex */
SemaphoreHandle_t dbSemaphore = NULL;

/* Record scheme for database */
struct EventRecord {
  int timeStamp;
  double temp;
};


void setup() {
  /* Setup serial baud */
  Serial.begin(9600);
  
  /* Initialize database */
  db.create(0, TABLE_SIZE, sizeof(EventRecord));

  /* Setup mutex for consistent database access */
  dbSemaphore = xSemaphoreCreateMutex();
  
  /* Setup output pins */
  pinMode(outPinCrash, OUTPUT);

  /* Setup interrupt pins */
  pinMode(inPinWakeUp, INPUT_PULLUP);
  pinMode(inPinCrash, INPUT_PULLUP);

  /* Attach the external interrupt for the crash detection */
  attachInterrupt(digitalPinToInterrupt(inPinCrash), ExternalInterruptCrashing, FALLING);

  /* Setup tasks */
  /* User input task */
  xTaskCreate(
    UserInput
    , (const portCHAR *) "UserInput" 
    , 128  // Stack size
    , NULL //paramaters task
    , 3  // priority
    , &TaskHandle_UserInput);

  /* Temperature store task */
  xTaskCreate(
    TemperatureReadStore
    , (const portCHAR *) "TemperatureReadStore"
    , 128 // This stack size can be checked & adjusted by reading Highwater
    , NULL
    , 2  // priority
    , &TaskHandle_TemperatureReadStore);

  /* Crash task */
  xTaskCreate(
    onCrash,
    (const portCHAR *) "OnCrash",
    128,
    NULL,
    1,
    &TaskHandle_Crash);
}

void loop()
{
  if (isSleeping) { // While the device should sleep
    sleepNow();     // Put the device to sleep
  }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void UserInput(void *pvParameters)
{
  for (;;)
  { 
    int input = readInt(); // Ask for input

    if (input == 1) { // Print last temperature
      printLastN(1);
    }
    else if (input == 2) { // Enter sleepmode
      isSleeping = true;
      sleepNow();
    }
    else if (input == 3) { // Print all temperatures
      printLastN(0);
    }

    vTaskDelay(1);  // One tick delay in between reads for stability.
  }
}

/* Task routine of the TemperatureReadStore task.
 * This task reads the temperature every 500 ms and stores it in the database.
 */
void TemperatureReadStore(void *pvParameters)
{
  for (;;)
  {
    double temp = GetTemp();
    storeTemp(temp); // Store temperature in database
    vTaskDelay(tempStoreDelay);  // Unblock task every 500 ms.
  }
}

/*
 * Task routine of the Crash task. Blocks until it receives a notification
 * given by an ISR caused by the interrupt given by the collision detector.
 */
void onCrash(void *pvParameters) {
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))  // Wait for notification given by the collision detector
    {
      vTaskDelay(crashDelay);                     // Wait 144 ms
      digitalWrite(outPinCrash, HIGH);            // Set pin 13 high (is also connected with LED_BUILTIN)
    }
  }
}

/*
 * Returns the calibrated temperature in degrees Celcius.
 */
double GetTemp(void) {
  unsigned int wADC;
  double temp;

  // Set the references and mux according to the spec cheat
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0));
  // ADC
  ADCSRA |= _BV(ADEN);
  ADCSRB |= _BV(MUX5);

  delay(15); // Delay for stability
  
  ADCSRA |= _BV(ADSC);

  while (bit_is_set(ADCSRA, ADSC)); // Wait for conversion to finish

  wADC = ADCL + (ADCH << 8);

  temp = (wADC - 284.55) / 1.22; // 284.55 is the calibration offset

  return temp;
}


/* The read and write handlers for using the EEPROM Library.
 * The database uses EEPROM to store the values, by definition of the read and write handlers.
 */
void writer(unsigned long address, byte data) {
  EEPROM.write(address, data);
}

byte reader(unsigned long address) {
  return EEPROM.read(address);
}

/* Function to read an integer from serial */
int readInt() {
  if (Serial.available() != 0) {
    int out = Serial.parseInt();
    
    return out;
  }
  return NULL;
}

/* Print last n records of the database */
/* if n is equal to 0, it prints all the data in the database*/
void printLastN(int n) {
  int dbCount = db.count();

  if (n >= dbCount || n == 0) {
    n = dbCount;
  }
  
  int startIndex = dbCount - n;

  for (int i = startIndex; i < startIndex + n; i++) {
    EventRecord record;

    if (xSemaphoreTake(dbSemaphore, portMAX_DELAY) == pdTRUE) { // Wait for the availability of the semaphore to ensure consistency
      db.readRec(i + 1, EDB_REC record); // READ ith+1 record from the database: START AT 1
      
      xSemaphoreGive(dbSemaphore); // Unlock semaphore
    }

    Serial.println(record.temp);
  }
}

/* Store temperature in database record */
void storeTemp(double temp) {
  EventRecord record;

  if (xSemaphoreTake(dbSemaphore, portMAX_DELAY) == pdTRUE) { // Wait for the availability of the semaphore to ensure consistency
    record.timeStamp = temperatureTimestamp;
    record.temp = temp;
    db.appendRec(EDB_REC record); // Write temperature to the database
    temperatureTimestamp++;
    xSemaphoreGive(dbSemaphore); // Unlock semaphore
  }
}

/* ------------ SLEEP ---------------*/
void sleepNow() {
  
  USBDevice.detach();

  Serial.flush();             // Flush serial buffer
  suspendTasks();             // Suspend all the tasks

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // Set the sleep mode to "Power Down"
  
  cli();                        // Disable interrupts
  sleep_enable();               // Enables sleep bit in the mcucr register
  wdt_disable();                // Disable the watchdogtimer to avoid constant watchdog interrupts
  attachInterrupt(digitalPinToInterrupt(inPinWakeUp), ExternalInterruptWakeUp, FALLING); // Attach wake-up interrupt
  sei();                      // Enable interrupts

  /* Disable LEDs */
  TXLED0;
  RXLED0;
  digitalWrite(LED_BUILTIN, LOW);

  sleep_cpu();                  // Put device to sleep
  // Continue here on wake up
  sleep_disable();             
  power_all_enable();
                           
  if (!isSleeping) { // Flag is made false in ISR of the wake-up interrupt
    detachInterrupt(digitalPinToInterrupt(inPinWakeUp));
    wdt_enable(WDTO_15MS);      // Enable the watchdog timer
    USBDevice.attach();
    Serial.begin(9600);
    resumeTasks();              // Resume required tasks    
  }
}

void resumeTasks() {
  vTaskResume(TaskHandle_TemperatureReadStore);
  vTaskResume(TaskHandle_UserInput);
}

void suspendTasks() {
  vTaskSuspend(TaskHandle_TemperatureReadStore);
  vTaskSuspend(TaskHandle_UserInput);
}

/* INTERRUPT HANDLERS */
void ExternalInterruptWakeUp() {
  if (isSleeping) {
    isSleeping = false;
  }
}

void ExternalInterruptCrashing() {
  detachInterrupt(digitalPinToInterrupt(inPinCrash));
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskHandle_Crash, &xHigherPriorityTaskWoken); // Send notification to crash task, which will immediately unblock it.
  portYIELD(); // Force context switch
}