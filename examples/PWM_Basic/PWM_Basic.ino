/****************************************************************************************************************************
  PWM_Basic.ino

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

#define _PWM_LOGLEVEL_        4

#include "megaAVR_PWM.h"

#define USING_TIMERB        true

#if USING_TIMERB
  // Pins tested OK in Nano Every ATmega4809
  #define pinToUse      3            // TimerB1, for higher frequencies, up to 100KHz
  //#define pinToUse      6            // TimerB0, for higher frequencies, up to 100KHz
#elif USING_ARDUINO_MEGA_AVR_CORE
  // Pins tested OK in Nano Every ATmega4809 using Arduino megaAVR core
  // TimerA0 somehow can't be used with MegaCoreX
  #define pinToUse      5            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
  //#define pinToUse      9            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
  //#define pinToUse     10            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
#else
  #error TimerA0 to be used with Arduino megaAVR Core
#endif

////////////////////////////////////////////

//creates pwm instance
megaAVR_PWM* PWM_Instance;

float frequency;
float dutyCycle;

////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  //delay(1000);

  Serial.print(F("\nStarting PWM_Basic on "));
  Serial.println(BOARD_NAME);
  Serial.println(MEGA_AVR_PWM_VERSION);

  //assigns PWM frequency of 1KHz and a duty cycle of 0%
  PWM_Instance = new megaAVR_PWM(pinToUse, 1000, 0);
}

void loop()
{
#if USING_TIMERB
  // Using higher Hz for TimerB, min is 980.34 Hz
  frequency = 40000;
  //frequency = 1000;
#else
  // Using much lower Hz for TimerA
  frequency = 1000;
#endif

  dutyCycle = 30;

  PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);

  delay(5000);
  dutyCycle = 90;

  PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);

  //while (true)
  delay(5000);
}
