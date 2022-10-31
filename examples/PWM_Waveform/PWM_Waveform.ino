/****************************************************************************************************************************
  PWM_Waveform.ino
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

typedef struct
{
  uint16_t level;
} PWD_Data;

// Data for 1000Hz, PWMPeriod = 8000
PWD_Data PWM_data[] =
{
  {    0 },
  {  500 },
  { 1000 },
  { 1500 },
  { 2000 },
  { 2500 },
  { 3000 },
  { 3500 },
  { 4000 },
  { 4500 },
  { 5000 },
  { 5500 },
  { 6000 },
  { 6500 },
  { 7000 },
  { 7500 },
  { 8000 },
  { 7500 },
  { 7000 },
  { 6500 },
  { 6000 },
  { 5500 },
  { 5000 },
  { 4500 },
  { 4000 },
  { 3500 },
  { 3000 },
  { 2500 },
  { 2000 },
  { 1500 },
  { 1000 },
  {  500 },
  {    0 },
};

#define NUM_PWM_POINTS      ( sizeof(PWM_data) / sizeof(PWD_Data) )

float frequency;

// You can select any value
PWD_Data PWM_data_idle = PWM_data[0];

char dashLine[] = "============================================================================================";

void printPWMInfo(megaAVR_PWM* PWM_Instance)
{
  Serial.println(dashLine);
  Serial.print("Actual data: pin = ");
  Serial.print(PWM_Instance->getPin());
  Serial.print(", PWM DutyCycle = ");
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

  Serial.print(F("\nStarting PWM_Waveform on "));
  Serial.println(BOARD_NAME);
  Serial.println(MEGA_AVR_PWM_VERSION);

  frequency = 1000.0f;

  // Create a dummy instance
  PWM_Instance = new megaAVR_PWM(pinToUse, frequency, 0);

  if (PWM_Instance)
  {
    // setPWM_manual(uint8_t pin, uint16_t level)
    PWM_Instance->setPWM(pinToUse, frequency, 0);

    printPWMInfo(PWM_Instance);
  }
}

void updateDC()
{
  static uint16_t index = 0;

  // Mapping data to any other frequency from original data for 1000Hz, PWMPeriod = 8000
  PWM_Instance->setPWM_manual(pinToUse, ( ( (uint32_t) PWM_data[index].level * PWM_Instance->getPWMPeriod() ) / 8000) );

  // Use at low freq to check
  //printPWMInfo(PWM_Instance);

  index = (index + 1) % NUM_PWM_POINTS;
}

void check_status()
{
#define UPDATE_INTERVAL     50L

  static unsigned long update_timeout = UPDATE_INTERVAL;

  // Update DC every UPDATE_INTERVAL (100) milliseconds
  if (millis() > update_timeout)
  {
    updateDC();
    update_timeout = millis() + UPDATE_INTERVAL;
  }
}

void loop()
{
  check_status();
}
