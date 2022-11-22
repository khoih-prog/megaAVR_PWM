/****************************************************************************************************************************
  PWM_MultiChannel.ino
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

#define _PWM_LOGLEVEL_        1

#include "megaAVR_PWM.h"

#define USING_TIMERB        true

#if USING_TIMERB
// Pins tested OK in Nano Every ATmega4809
#define pinToUse      3            // TimerB1, for higher frequencies, up to 100KHz
//#define pinToUse      6            // TimerB0, for higher frequencies, up to 100KHz

uint32_t PWM_Pins[]       = { 3, 6 };

#elif USING_ARDUINO_MEGA_AVR_CORE
// Pins tested OK in Nano Every ATmega4809
#define pinToUse      5            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
//#define pinToUse      9            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
//#define pinToUse     10            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock

// Many pins to be used for TimerA0, provided same frequency, but different DC
uint32_t PWM_Pins[]       = { 5, 9, 10 };

#else
#error TimerA0 to be used with Arduino megaAVR Core
#endif

////////////////////////////////////////////

#define NUM_OF_PINS       ( sizeof(PWM_Pins) / sizeof(uint32_t) )

float dutyCycle[]  = { 20.0f, 50.0f, 80.0f };

//creates pwm instances
megaAVR_PWM* PWM_Instance[NUM_OF_PINS];

// Must be same frequency for same timer
float freq = 1000.0f;

char dashLine[] = "=====================================================================================";

void printPWMInfo(megaAVR_PWM* PWM_Instance)
{
  Serial.println(dashLine);
  Serial.print("Actual data: pin = ");
  Serial.print(PWM_Instance->getPin());
  Serial.print(", PWM DC = ");
  Serial.print(PWM_Instance->getActualDutyCycle());
  Serial.print(", PWMPeriod = ");
  Serial.print(PWM_Instance->getPWMPeriod());
  Serial.print(", PWM Freq (Hz) = ");
  Serial.println(PWM_Instance->getActualFreq(), 4);
  Serial.println(dashLine);
}

void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  //delay(1000);

  Serial.print(F("\nStarting PWM_MultiChannel on "));
  Serial.println(BOARD_NAME);
  Serial.println(MEGA_AVR_PWM_VERSION);

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    PWM_Instance[index] = new megaAVR_PWM(PWM_Pins[index], freq, dutyCycle[index]);

    if (PWM_Instance[index])
    {
      PWM_Instance[index]->setPWM();
    }
  }

  Serial.println(dashLine);
  Serial.println("Index\tPin\tPWM_freq\tDutyCycle\tActual Freq");
  Serial.println(dashLine);

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    if (PWM_Instance[index])
    {
      Serial.print(index);
      Serial.print("\t");
      Serial.print(PWM_Pins[index]);
      Serial.print("\t");
      Serial.print(freq);
      Serial.print("\t\t");
      Serial.print(dutyCycle[index]);
      Serial.print("\t\t");
      Serial.println(PWM_Instance[index]->getActualFreq(), 4);
    }
    else
    {
      Serial.println();
    }
  }

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    printPWMInfo(PWM_Instance[index]);
  }
}

void loop()
{
  //Long delay has no effect on the operation of hardware-based PWM channels
  delay(1000000);
}
