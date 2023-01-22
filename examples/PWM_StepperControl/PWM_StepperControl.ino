/****************************************************************************************************************************
  PWM_StepperControl.ino
  For Arduino megaAVR ATMEGA4809-based boards (UNO WiFi Rev2, NANO_EVERY, etc. )
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/megaAVR_PWM
  Licensed under MIT license

  This is pure hardware-based PWM
*****************************************************************************************************************************/
/******************************************************************************************************************************
  Pins can be used for hardware-PWM
  // For ATmega4809 (Nano Every, Uno WiFi Rev2, etc.)
  TCA0 (16-bit) used by PWM generation on pins 5, 9 and 10
  TCB0 (16-bit) used by PWM generation on pin 6
  TCB1 (16-bit) used by PWM generation on pin 3
  TCB2 (16-bit)
  TCB3 (16-bit)
  ////////////////////////////////////////////
  // For ATmega4809 (Nano Every, Uno WiFi Rev2, etc.)
  Pin  3 => TIMERB1,       //  3 PF5,  8-bit PWM, 16-bit counter
  Pin  5 => TIMERA0,       //  5 PB2, 16-bit PWM, 16-bit counter
  Pin  6 => TIMERB0,       //  6 PF4,  8-bit PWM, 16-bit counter
  Pin  9 => TIMERA0,       //  9 PB0, 16-bit PWM, 16-bit counter
  Pin 10 => TIMERA0,       // 10 PB1, 16-bit PWM, 16-bit counter
  ////////////////////////////////////////////
******************************************************************************************************************************/

// Use with Stepper-Motor driver, such as TMC2209

#define _PWM_LOGLEVEL_        4

#include "megaAVR_PWM.h"

#define USING_TIMERB        true

#if USING_TIMERB
  // Pins tested OK in Nano Every ATmega4809
  #define STEP_PIN      3            // TimerB1, for higher frequencies, up to 100KHz
  //#define STEP_PIN      6            // TimerB0, for higher frequencies, up to 100KHz
#elif USING_ARDUINO_MEGA_AVR_CORE
  // Pins tested OK in Nano Every ATmega4809 using Arduino megaAVR core
  // TimerA0 somehow can't be used with MegaCoreX
  #define STEP_PIN      5            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
  //#define STEP_PIN      9            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
  //#define STEP_PIN     10            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
#else
  #error TimerA0 to be used with Arduino megaAVR Core
#endif

#define DIR_PIN       4

////////////////////////////////////////////

megaAVR_PWM* stepper;

void setSpeed(int speed)
{
  if (speed == 0)
  {
    // Use DC = 0 to stop stepper
    stepper->setPWM(STEP_PIN, 500, 0);
  }
  else
  {
    //  Set the frequency of the PWM output and a duty cycle of 50%
    digitalWrite(DIR_PIN, (speed < 0));
    stepper->setPWM(STEP_PIN, abs(speed), 50);
  }
}

void setup() 
{
  pinMode(DIR_PIN, OUTPUT);
  
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(100);

  Serial.print(F("\nStarting PWM_StepperControl on "));
  Serial.println(BOARD_NAME);
  Serial.println(MEGA_AVR_PWM_VERSION);
  
  // Create PWM object and passed just a random frequency of 500
  // The duty cycle is how you turn the motor on and off
  stepper = new megaAVR_PWM(STEP_PIN, 500, 0);
}

void loop() 
{
  setSpeed(1000);
  delay(3000);

  // Stop before reversing
  setSpeed(0);
  delay(3000);

  // Reversing
  setSpeed(-500);
  delay(3000);

  // Stop before reversing
  setSpeed(0);
  delay(3000);
}
