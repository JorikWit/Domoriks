#include <avr/wdt.h>

int loop_count = 0;
int wdt_counter = 0;

void setup() {

  Serial.begin(9600);
  Serial.println("Starting up...");
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay (500);
  watchdogStart();
}

void watchdogStart(void)
{
  cli();  // disable all interrupts
  wdt_reset(); // reset the WDT timer
/*
//WDTCSR configuration:
WDIE = 1: //Interrupt Enable
WDE = 1 : //Reset Enable
WDP3 = 0 :
For 2000ms Time-out WDP2 = 1 :
For 2000ms Time-out WDP1 = 1 :
For 2000ms Time-out WDP0 = 1 :
    For 2000ms Time-out
    */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE); // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (0<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  sei();
}

void watchdogArm(void)
{
  cli();  // disable all interrupts
  wdt_reset(); // reset the WDT timer
/*
//WDTCSR configuration:
WDIE = 1: //Interrupt Enable
WDE = 1 : //Reset Enable
WDP3 = 0 :
For 2000ms Time-out WDP2 = 1 :
For 2000ms Time-out WDP1 = 1 :
For 2000ms Time-out WDP0 = 1 :
    For 2000ms Time-out
    */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE); // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  sei();
}

void loop()
{
  for (int i = 0; i <= loop_count;i++){
    digitalWrite(13,HIGH);
    delay(100);
    digitalWrite(13,LOW);
    delay(100);
  }
  loop_count++;
  wdt_reset();
  watchdogStart();
  wdt_counter = 0;
  Serial.print(loop_count);
  Serial.print(". Watchdog fed in approx. ");
  Serial.print(loop_count*200);
  Serial.println(" milliseconds.");
}
ISR(WDT_vect) // Watchdog timer interrupt.
{
  if(wdt_counter==0)
  {
  wdt_counter++;
  watchdogArm();
  }
}
