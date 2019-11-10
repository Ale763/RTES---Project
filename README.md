# RTES---Project

## Questions

1. Synchronization
   - How handle too long waiting (suspend/error)
   - Guard time dynamisch/static
   - Data from database/linked list
   - What is timestamp/should it be saved
   - Denken aan eeprom vol
   - Printen sequentieel


   - In wat formaat data doorsturen?
   - Hoe afhandelen ontvangen vs user commands



2. Project requirements
   - At boot keep listening for beacon with radio 
     - Keep listening with radio open
     - Beacon specifies exact next transmission time of next beacon
   - When beacon arrives
     - Store next wake-up time in db
     - Read, store & send temp
     - Sleep till next beacon
   - After 20 beacons: enter ultra low-power operation mode
     - Minimize power
     - Wake up through external interrupt
   - Support for 3 Serial commands 
      1. Read latest temp + beacon details from db (print to serial)
      2. Same as before, but for all entries
      3. Enable low power operation mode
          - Same details as before
   - Db sync with semaphores 


3. Data structure
   - LinkedList
     - List
       - Head
       - Tail? (double linked list)
     - Node
       - Beacon details
       - Temperature