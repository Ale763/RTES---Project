#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control

// how many times remain to sleep before wake up
int nbr_remaining; 

// pin on which a led is attached on the board
#define led 13

// interrupt raised by the watchdog firing
// when the watchdog fires during sleep, this function will be executed
// remember that interrupts are disabled in ISR functions
ISR(WDT_vect)
{
        // not hanging, just waiting
        // reset the watchdog
        wdt_reset();
}

// function to configure the watchdog: let it sleep 8 seconds before firing
// when firing, configure it for resuming program execution
void configure_wdt(int maxMS)
{
    int wdto,actualMS; 
    
    if((maxMS >= 8000) || (maxMS == 0)) {
        wdto     = WDTO_8S;
        actualMS = 8000;
    } else if(maxMS >= 4000) {
        wdto     = WDTO_4S;
        actualMS = 4000;
    } else if(maxMS >= 2000) {
        wdto     = WDTO_2S;
        actualMS = 2000;
    } else if(maxMS >= 1000) {
        wdto     = WDTO_1S;
        actualMS = 1000;
    } else if(maxMS >= 500) {
        wdto     = WDTO_500MS;
        actualMS = 500;
    } else if(maxMS >= 250) {
        wdto     = WDTO_250MS;
        actualMS = 250;
    } else if(maxMS >= 120) {
        wdto     = WDTO_120MS;
        actualMS = 120;
    } else if(maxMS >= 60) {
        wdto     = WDTO_60MS;
        actualMS = 60;
    } else if(maxMS >= 30) {
        wdto     = WDTO_30MS;
        actualMS = 30;
    } else {
        wdto     = WDTO_15MS;
        actualMS = 15;
    }

    // Build watchdog prescaler register value before timing critical code.
    uint8_t wdps = ((wdto & 0x08 ? 1 : 0) << WDP3) |
                   ((wdto & 0x04 ? 1 : 0) << WDP2) |
                   ((wdto & 0x02 ? 1 : 0) << WDP1) |
                   ((wdto & 0x01 ? 1 : 0) << WDP0);

 
  cli();                           // disable interrupts for changing the registers

    // First clear any previous watchdog reset.
    MCUSR &= ~(1<<WDRF);
    // Now change the watchdog prescaler and interrupt enable bit so the
    // watchdog reset only triggers the interrupt (and wakes from deep sleep)
    // and not a full device reset.  This is a timing critical section of
    // code that must happen in 4 cycles.
    WDTCSR |= (1<<WDCE) | (1<<WDE);  // Set WDCE and WDE to enable changes.
    WDTCSR = wdps;                   // Set the prescaler bit values.
    WDTCSR |= (1<<WDIE);             // Enable only watchdog interrupts.
    
    // Critical section finished, re-enable interrupts.

  sei();                           // re-enable interrupts

  // reminder of the definitions for the time before firing
  // delay interval patterns:
  //  16 ms:     0b000000
  //  500 ms:    0b000101
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001
 
}

// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void sleep(int ncycles)
{  
  nbr_remaining = ncycles; // defines how many cycles should sleep

  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  // Turn off the ADC while asleep.
  power_adc_disable();

  // Disable USB if it exists
#ifdef USBCON
    //USBCON |= _BV(FRZCLK); // freeze USB clock
    //PLLCSR &= ~_BV(PLLE);  // turn off USB PLL
    //USBCON &= ~_BV(USBE);  // disable USB
#endif
 
  while (nbr_remaining > 0){ // while some cycles left, sleep!

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point if the
  // watchdog is configured for resume rather than restart
 
  // When awake, disable sleep mode
  sleep_disable();
  
  // we have slept one time more
  nbr_remaining = nbr_remaining - 1;
 
  }
 
  // put everything on again
  power_all_enable();
 
}

void setup(){
  
  // use led 13 and put it in low mode
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  
  delay(1000);
  
  // configure the watchdog
  configure_wdt(1000);

  // blink twice
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 

}

void loop(){

  // sleep for a given number of cycles (here, 5 * 8 seconds) in lowest power mode
  sleep(5);

  // usefull stuff should be done here before next long sleep
  // blink three times
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 

}
