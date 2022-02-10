/*
Laser Zeroing Tool
*/
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>

void setLaser(bool state);
void sleep();

const int LASER_PIN = 0;  // PB0
const int BUTTON_PIN = 4; // PB4
const int LED_PIN = 3;    // PB3
const int VDIV_PIN = 2;   // PB2
const int VCOMP_PIN = 1;  // PB1

const unsigned long debounceDelay = 50; // 50ms
const unsigned long powerDownDelay = 15000; // 15 seconds
unsigned long buttonLastChangeTime = 0;
unsigned long turnOnTime = 0;

int buttonState = HIGH;

bool laserState = LOW;

bool justAwoke = false;

// cppcheck-suppress unusedFunction
void setup() {
  // Set pin modes.
  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(VDIV_PIN, OUTPUT);
  digitalWrite(VDIV_PIN, LOW);

  setLaser(true);
  digitalWrite(VDIV_PIN, HIGH);
  
  // Configure comparator: AN1 (pin 6 on DIP-8) on negative input,
  // 1.1V bandgap reference on positive input. Disable the digital
  // input buffer on the AN1 pin for slight power savings.
  ADCSRA=0x00;
  ADCSRB=0x00;
  ACSR |= (1<<ACBG);
  DIDR0 |= (1<<AIN1D);

  // Configure pin change interrupt
  PCMSK = _BV(PCINT4);                    // Only PB4 raises interrupt
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts

  // Get starting button data
  buttonLastChangeTime = millis();
  turnOnTime = millis();

  // blink to indicate setup
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  digitalWrite(LED_PIN, LOW);
}

// the loop function runs over and over again forever
// cppcheck-suppress unusedFunction
void loop() {

  // Low battery check: compare the voltage divider input to the
  // 1.1v reference
  // Enable the divider by connecting the low side to logic 0, then
  // read the comparator output, then disable the divider by
  // connecting the low side to logic 1.
  // This prevents the resistive loss on the divider when it is not in use.
  digitalWrite(VDIV_PIN, LOW);
  int comp = ACSR & _BV(ACO);
  digitalWrite(VDIV_PIN, HIGH);
  bool lowBattery = !!comp;

  // Light laser for a 3-second interval. Solid for good battery voltage, or blinky for low battery
  if (lowBattery) {
    setLaser(true);
    delay(2250);
    setLaser(false);
    delay(250);
    setLaser(true);
    delay(250);
    setLaser(false);
    delay(250);
  }
  else {
    setLaser(true);
    delay(3000);
  }

  if ((millis() - turnOnTime) > powerDownDelay) {
    sleep();
  }
}

void setLaser(bool state)
{
  laserState = state;
  digitalWrite(LASER_PIN, laserState);
}

void sleep()
{
  // Prepare for Power Down sleep
  setLaser(false);               // turn off the laser
  digitalWrite(VDIV_PIN, HIGH); // turn off the voltage divider
  ACSR &= ~(1<<ACBG);           // Turn off voltage reference
  
  cli();                                  // Disable interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();                                  // Enable interrupts
  

  // Sleep until wake event (any interrupt) happens.
  justAwoke = true;  // Get ready for waking up later
  sleep_cpu();                            // ZZZzzzz.
  
  // Now we are awake again!
  cli();                                  // Disable interrupts
  sleep_disable();                        // Clear SE bit

  // Turn on voltage reference
  ACSR |= (1<<ACBG);

  sei();                                  // Enable interrupts

  setLaser(true);
  turnOnTime = millis();
}

// Pin change interrupt triggered by both button presses and releases.
ISR(PCINT0_vect) {

  // Crude debounce
  delay(20);

  // Did we just wake up?
  if (justAwoke) {
    justAwoke = false;
    return;
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    // Button press
    if (laserState) {
        setLaser(false);
        // Force quick sleep by backdating turnOnTime
        turnOnTime = millis() - powerDownDelay;
    }
    else
    {
      setLaser(true);
      turnOnTime = millis();
    }
  }
}