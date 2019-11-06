#include "Arduino.h"
#include <EDB.h>
#include <EEPROM.h>

// Uncomment the line appropriate for your platform
#define TABLE_SIZE 512 // Arduino 168 or greater

// The number of demo records that should be created.  This should be less 
// than (TABLE_SIZE - sizeof(EDB_Header)) / sizeof(tableEntry).  If it is higher, 
// operations will return EDB_OUT_OF_RANGE for all records outside the usable range.
#define RECORDS_TO_CREATE 10

// Arbitrary record definition for this table.  
// This should be modified to reflect your record needs.
struct TableEntry {
  int id;
  int temperature;//TODO: data structure for temp + amt of sleep
} 
tableEntry;

// The read and write handlers for using the EEPROM Library
void writer(unsigned long address, byte data)
{
  EEPROM.write(address, data);
}

byte reader(unsigned long address)
{
  return EEPROM.read(address);
}

// Create an EDB object with the appropriate write and read handlers
EDB db(&writer, &reader);

void setup()
{
  
  Serial.begin(9600);
  while(!Serial);
  int recno;
  bool filled = db.open(0) == EDB_OK && db.readRec(1, EDB_REC tableEntry) == EDB_OK && tableEntry.id != 0;
  if (!filled)
  {
    Serial.println("Table was empty...Filling table");
    db.create(0, TABLE_SIZE, sizeof(tableEntry));
    for (recno = 1; recno <= RECORDS_TO_CREATE; recno++)
    {
      tableEntry.id = recno; 
      tableEntry.temperature = recno * 2;
      db.appendRec(EDB_REC tableEntry);
    }
  } else 
  {
      Serial.println("Table was filled!");
  }
  Serial.print("Record Count: "); Serial.println(db.count());
  for (recno = 1; recno <= RECORDS_TO_CREATE; recno++)
  {
    if (db.readRec(recno, EDB_REC tableEntry) == EDB_OK)
    {
      Serial.print("ID: "); Serial.println(tableEntry.id);
      Serial.print("Temp: "); Serial.println(tableEntry.temperature);
    }
    else 
    {
      Serial.println("No record");
    }
  }
  if (filled) 
  {
      db.clear();
  }
}

void loop()
{
}
