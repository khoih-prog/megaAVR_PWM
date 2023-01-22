/****************************************************************************************************************************
  megaAVR_PWM.h
  For Arduino megaAVR ATMEGA4809-based boards (UNO WiFi Rev2, NANO_EVERY, etc. )
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/megaAVR_PWM
  Licensed under MIT license

  This is pure hardware-based PWM

  Version: 1.0.1

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K.Hoang      31/10/2022 Initial coding for megaAVR ATMEGA4809-based boards (UNO WiFi Rev2, NANO_EVERY, etc. )
  1.0.1   K Hoang      22/01/2023 Add `PWM_StepperControl` example
*****************************************************************************************************************************/

#pragma once

#ifndef MEGA_AVR_PWM_H
#define MEGA_AVR_PWM_H

#if defined(BOARD_NAME)
  #undef BOARD_NAME
#endif

#ifndef _PWM_LOGLEVEL_
  #define _PWM_LOGLEVEL_        1
#endif

#if ( defined(__AVR_ATmega4809__) || defined(ARDUINO_AVR_UNO_WIFI_REV2) || defined(ARDUINO_AVR_NANO_EVERY) || \
      defined(ARDUINO_AVR_ATmega4809) || defined(ARDUINO_AVR_ATmega4808) || defined(ARDUINO_AVR_ATmega3209) || \
      defined(ARDUINO_AVR_ATmega3208) || defined(ARDUINO_AVR_ATmega1609) || defined(ARDUINO_AVR_ATmega1608) || \
      defined(ARDUINO_AVR_ATmega809) || defined(ARDUINO_AVR_ATmega808) )
#if !defined(BOARD_NAME)
  #if defined(ARDUINO_AVR_UNO_WIFI_REV2)
    #define USING_ARDUINO_MEGA_AVR_CORE     true
    #define BOARD_NAME      "megaAVR UNO WiFi Rev2"
  #elif defined(ARDUINO_AVR_NANO_EVERY)
    #define USING_ARDUINO_MEGA_AVR_CORE     true
    #define BOARD_NAME      "megaAVR Nano Every"
  #else
    #define USING_ARDUINO_MEGA_AVR_CORE     false

    #if ( defined(ARDUINO_AVR_ATmega4809) && defined(UNO_WIFI_REV2_PINOUT) )
      #define BOARD_NAME      "MegaCoreX UNO WiFi Rev2"
    #elif ( defined(ARDUINO_AVR_ATmega4809) && defined(NANO_EVERY_PINOUT) )
      #define BOARD_NAME      "MegaCoreX Nano Every"
    #elif defined(ARDUINO_AVR_ATmega4809)
      #define BOARD_NAME      "MegaCoreX ATmega4809"
    #elif defined(ARDUINO_AVR_ATmega4808)
      #define BOARD_NAME      "MegaCoreX ATmega4808"
    #elif defined(ARDUINO_AVR_ATmega3209)
      #define BOARD_NAME      "MegaCoreX ATmega3209"
    #elif defined(ARDUINO_AVR_ATmega3208)
      #define BOARD_NAME      "MegaCoreX ATmega3208"
    #elif defined(ARDUINO_AVR_ATmega1609)
      #define BOARD_NAME      "MegaCoreX ATmega1609"
    #elif defined(ARDUINO_AVR_ATmega1608)
      #define BOARD_NAME      "MegaCoreX ATmega1608"
    #elif defined(ARDUINO_AVR_ATmega809)
      #define BOARD_NAME      "MegaCoreX ATmega809"
    #elif defined(ARDUINO_AVR_ATmega808)
      #define BOARD_NAME      "MegaCoreX ATmega808"
    #else
      #define BOARD_NAME      "megaAVR Unknown"
    #endif
  #endif
#endif
#else
#error This is designed only for Arduino or MegaCoreX megaAVR board! Please check your Tools->Board setting
#endif

////////////////////////////////////////

#ifndef MEGA_AVR_PWM_VERSION
  #define MEGA_AVR_PWM_VERSION           F("megaAVR_PWM v1.0.1")

  #define MEGA_AVR_PWM_VERSION_MAJOR     1
  #define MEGA_AVR_PWM_VERSION_MINOR     0
  #define MEGA_AVR_PWM_VERSION_PATCH     1

  #define MEGA_AVR_PWM_VERSION_INT      1000001
#endif

////////////////////////////////////////

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "pins_arduino.h"

#include "PWM_Generic_Debug.h"

////////////////////////////////////////

#define MAX_COUNT_8BIT            255
#define MAX_COUNT_16BIT           65535

////////////////////////////////////////

/*****************************************************************************************

  // From ~/.arduino15/packages/arduino/7.3.0-atmel3.6.1-arduino5/avr/include/avr/iom4809.h

  //#define TCB0                  (*(TCB_t *) 0x0A80) // 16-bit Timer Type B
  //#define TCB1                  (*(TCB_t *) 0x0A90) // 16-bit Timer Type B
  //#define TCB2                  (*(TCB_t *) 0x0AA0) // 16-bit Timer Type B
  //#define TCB3                  (*(TCB_t *) 0x0AB0) // 16-bit Timer Type B

  //
  typedef enum TCB_CLKSEL_enum
  {
    TCB_CLKSEL_CLKDIV1_gc = (0x00<<1),  // CLK_PER (No Prescaling)
    TCB_CLKSEL_CLKDIV2_gc = (0x01<<1),  // CLK_PER/2 (From Prescaler)
    TCB_CLKSEL_CLKTCA_gc = (0x02<<1),   // Use Clock from TCA
  } TCB_CLKSEL_t;

  //
  typedef enum TCB_CNTMODE_enum
  {
    TCB_CNTMODE_INT_gc = (0x00<<0),       // Periodic Interrupt
    TCB_CNTMODE_TIMEOUT_gc = (0x01<<0),   // Periodic Timeout
    TCB_CNTMODE_CAPT_gc = (0x02<<0),      // Input Capture Event
    TCB_CNTMODE_FRQ_gc = (0x03<<0),       // Input Capture Frequency measurement
    TCB_CNTMODE_PW_gc = (0x04<<0),        // Input Capture Pulse-Width measurement
    TCB_CNTMODE_FRQPW_gc = (0x05<<0),     // Input Capture Frequency and Pulse-Width measurement
    TCB_CNTMODE_SINGLE_gc = (0x06<<0),    // Single Shot
    TCB_CNTMODE_PWM8_gc = (0x07<<0),      // 8-bit PWM
  } TCB_CNTMODE_t;

*****************************************************************************************/

/*
  Pins can be used for hardware-PWM
  // For ATmega4809 (Nano Every, Uno WiFi Rev2, etc.)
  TCA0 (16-bit) used by PWM generation on pins 5, 9 and 10
  TCB0 (16-bit) used by PWM generation on pin 6
  TCB1 (16-bit) used by PWM generation on pin 3
  TCB2 (16-bit)
  TCB3 (16-bit) used by millis()
  ////////////////////////////////////////////
  // For ATmega4809 (Nano Every, Uno WiFi Rev2, etc.)
  Pin  3 => TIMERB1,       //  3 PF5,  8-bit PWM, 16-bit counter
  Pin  5 => TIMERA0,       //  5 PB2, 16-bit PWM, 16-bit counter, 250KHz
  Pin  6 => TIMERB0,       //  6 PF4,  8-bit PWM, 16-bit counter
  Pin  9 => TIMERA0,       //  9 PB0, 16-bit PWM, 16-bit counter, 250KHz
  Pin 10 => TIMERA0,       // 10 PB1, 16-bit PWM, 16-bit counter, 250KHz
  ////////////////////////////////////////////

  const uint8_t digital_pin_to_timer[] =
  {
    NOT_ON_TIMER,  // 0 PC5/USART1_Rx
    NOT_ON_TIMER,  // 1 PC4/USART1_Tx
    NOT_ON_TIMER,  // 2 PA0
    TIMERB1,       // 3 PF5
    NOT_ON_TIMER,  // 4 PC6
    TIMERA0,       // 5 PB2
    TIMERB0,       // 6 PF4
    NOT_ON_TIMER,  // 7 PA1
    NOT_ON_TIMER,  // 8 PE3
    TIMERA0,       // 9 PB0
    TIMERA0,       // 10 PB1
    NOT_ON_TIMER,  // 11 PE0
    NOT_ON_TIMER,  // 12 PE1
    NOT_ON_TIMER,  // 13 PE2
    NOT_ON_TIMER,  // 14 PD3/AI3
    NOT_ON_TIMER,  // 15 PD2/AI2
    NOT_ON_TIMER,  // 16 PD1/AI1
    NOT_ON_TIMER,  // 17 PD0/AI0
    NOT_ON_TIMER,  // 18 PF2/AI12
    NOT_ON_TIMER,  // 19 PF3/AI13
    NOT_ON_TIMER,  // 20 PD4/AI4
    NOT_ON_TIMER,  // 21 PD5/AI5
    NOT_ON_TIMER,  // 22 PA2/TWI_SDA
    NOT_ON_TIMER,  // 23 PA3/TWI_SCL
    NOT_ON_TIMER,  // 24 PB5/USART3_Rx
    NOT_ON_TIMER,  // 25 PB4/USART3_Tx
  };

  // Use this for accessing PINnCTRL register
  const uint8_t digital_pin_to_bit_position[] =
  {
    PIN5_bp,  // 0 PC5/USART1_Rx
    PIN4_bp,  // 1 PC4/USART1_Tx
    PIN0_bp,  // 2 PA0
    PIN5_bp,  // 3 PF5
    PIN6_bp,  // 4 PC6
    PIN2_bp,  // 5 PB2
    PIN4_bp,  // 6 PF4
    PIN1_bp,  // 7 PA1
    PIN3_bp,  // 8 PE3
    PIN0_bp,  // 9 PB0
    PIN1_bp,  // 10 PB1
    PIN0_bp,  // 11 PE0
    PIN1_bp,  // 12 PE1
    PIN2_bp,  // 13 PE2
    PIN3_bp,  // 14 PD3/AI3
    PIN2_bp,  // 15 PD2/AI2
    PIN1_bp,  // 16 PD1/AI1
    PIN0_bp,  // 17 PD0/AI0
    PIN2_bp,  // 18 PF2/AI12
    PIN3_bp,  // 19 PF3/AI13
    PIN4_bp,  // 20 PD4/AI4
    PIN5_bp,  // 21 PD5/AI5
    PIN2_bp,  // 22 PA2/TWI_SDA
    PIN3_bp,  // 23 PA3/TWI_SCL
    PIN5_bp,  // 24 PB5/USART3_Rx
    PIN4_bp,  // 25 PB4/USART3_Tx
  };
*/

// #define digitalPinToTimer(pin)       ( (pin < NUM_TOTAL_PINS) ? digital_pin_to_timer[pin] : NOT_ON_TIMER )
// #define digitalPinToBitPosition(pin) ( (pin < NUM_TOTAL_PINS) ? digital_pin_to_bit_position[pin] : NOT_A_PIN )

#define TIMERA_RESOLUTION 65536UL   // TimerA is 16 bit for PWM
#define TIMERB_RESOLUTION 255UL     // TimerB is  8 bit for PWM

// Count only TCB0-TCB3
enum
{
  HW_TIMER_0 = 0,
  HW_TIMER_1,
  HW_TIMER_2,
  HW_TIMER_3,
  NUM_HW_TIMERS
};

////////////////////////////////////////

#if ( defined(__AVR_ATmega4809__) || defined(__AVR_ATmega3209__) || defined(__AVR_ATmega1609__) || defined(__AVR_ATmega809__) )
#if (_PWM_LOGLEVEL_ > 3)
  #warning Using __AVR_ATmegaXX09__ architecture
#endif

#define PWM_USING_ATMEGA_XX09       true

TCB_t* TimerTCB[ NUM_HW_TIMERS ] = { &TCB0, &TCB1, &TCB2, &TCB3 };

#elif ( defined(__AVR_ATmega4808__) || defined(__AVR_ATmega3208__) || defined(__AVR_ATmega1608__) || defined(__AVR_ATmega808__) )
#if (_PWM_LOGLEVEL_ > 3)
  #warning Using __AVR_ATmegaXX08__ architecture
#endif

#define PWM_USING_ATMEGA_XX08       true

TCB_t* TimerTCB[ NUM_HW_TIMERS ] = { &TCB0, &TCB1, &TCB2 };

#endif

////////////////////////////////////////

#define CLK_TCA_FREQ            (250000UL)

#define TCB_CLKSEL_CLKDIV1      0x00
#define TCB_CLKSEL_CLKDIV2      0x02
#define TCB_CLKSEL_CLKDIV64     0x04      // TCA clock = 250KHz
#define TCB_CLKSEL_CLKTCA       0x04      // TCA clock = 250KHz

////////////////////////////////////////
////////////////////////////////////////

class megaAVR_PWM
{
  public:

    megaAVR_PWM(const uint8_t& pin, const float& frequency, const float& dutycycle)
    {
      _pin        = pin;
      _frequency  = frequency;
      _dutycycle  = map(dutycycle, 0, 100.0f, 0, MAX_COUNT_16BIT);

      _timer      = digitalPinToTimer(pin);

      PWM_LOGDEBUG1("megaAVR_PWM: _dutycycle =", _dutycycle);

      pinMode(pin, OUTPUT);
    }

    ///////////////////////////////////////////

    ~megaAVR_PWM();

    ///////////////////////////////////////////
    ///////////////////////////////////////////

  private:

    ///////////////////////////////////////////

    void setPeriod_TimerA0(unsigned long microseconds) __attribute__((always_inline))
    {
      // TCA Clock is 250KHz
      const uint32_t cycles = microseconds / 4;

      PWM_LOGDEBUG3("setPeriod_TimerA0: F_CPU =", F_CPU, ", cycles =", cycles);

      pwmPeriod = cycles;

      TCA0.SINGLE.PER = pwmPeriod;

      _actualFrequency =  250000UL / pwmPeriod;

      PWM_LOGDEBUG3("setPeriod_TimerA0: pwmPeriod =", pwmPeriod, ", _actualFrequency =", _actualFrequency);
    }

    ///////////////////////////////////////////

    void setPeriod_TimerB(unsigned long microseconds) __attribute__((always_inline))
    {
      const uint32_t cycles = ( ( (F_CPU / 100000) * microseconds ) / 10 );

      uint32_t cur_TCB_CLKSEL_VALUE = TCB_CLKSEL_CLKDIV1;

      PWM_LOGDEBUG3("setPeriod_TimerB: F_CPU =", F_CPU, ", cycles =", cycles);

      if (cycles < TIMERB_RESOLUTION)
      {
        PWM_LOGDEBUG("setPeriod_TimerB: cycles < TIMERB_RESOLUTION, using divider = 1");
        cur_TCB_CLKSEL_VALUE = TCB_CLKSEL_CLKDIV1;
        pwmPeriod = cycles;
      }
      else if (cycles / 2 < TIMERB_RESOLUTION)
      {
        PWM_LOGDEBUG("setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 2, using divider = 2");
        cur_TCB_CLKSEL_VALUE = TCB_CLKSEL_CLKDIV2;
        pwmPeriod = cycles / 2;
      }
      else if (cycles / 64 < TIMERB_RESOLUTION)
      {
        PWM_LOGDEBUG("setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64");
        cur_TCB_CLKSEL_VALUE = TCB_CLKSEL_CLKDIV64;
        pwmPeriod = cycles / 64;
      }
      else
      {
        PWM_LOGDEBUG1("setPeriod_TimerB: Error, min freq (Hz) =", (float) F_CPU / (64 *  TIMERB_RESOLUTION) );

        pwmPeriod = TIMERB_RESOLUTION - 1;
      }

      timer_B = ((TCB_t *)&TCB0 + (_timer - TIMERB0));

      timer_B->CTRLA = cur_TCB_CLKSEL_VALUE | TCB_ENABLE_bm;       // Enable timer

      // Assign 8-bit period TODO, mapping to 8-bit
      timer_B->CCMPL = pwmPeriod;

      _actualFrequency = (float) F_CPU / ( cycles);

      PWM_LOGDEBUG3("setPeriod_TimerB: pwmPeriod =", pwmPeriod, ", _actualFrequency =", _actualFrequency);
    }

    ///////////////////////////////////////////
    ///////////////////////////////////////////

  public:

    // dutycycle from 0-65536 for 0%-100% to make use of 16-bit top register
    bool setPWM_Int(const uint8_t& pin, const float& frequency, uint16_t dutycycle)
    {
      uint16_t MAX_COUNT = MAX_COUNT_16BIT;

      uint8_t bit_pos  = digitalPinToBitPosition(pin);

      if (bit_pos == NOT_A_PIN)
        return false;

      pinMode(pin, OUTPUT);

      if ( (_timer == TIMERB0) || (_timer == TIMERB1) || (_timer == TIMERB2) || (_timer == TIMERB3)  )
      {
        // Convert to 8-bit
        dutycycle >>= 8;

        MAX_COUNT = MAX_COUNT_8BIT;
      }
      else
      {
        // Keep 16 bit dutycycle
        MAX_COUNT = MAX_COUNT_16BIT;
      }

      PWM_LOGDEBUG1("setPWM_Int: input dutycycle =", dutycycle);

      //if (_dutycycle == 0)
      if (dutycycle == 0)
      {
        digitalWrite(pin, LOW);
      }
      //else if (_dutycycle >= MAX_COUNT)
      else if (dutycycle >= MAX_COUNT)
      {
        PWM_LOGERROR1("Error, exceeded _dutycycle, reset to ", MAX_COUNT);

        digitalWrite(pin, HIGH);
      }

      PWM_LOGDEBUG1("setPWM_Int: _timer =", _timer);

      switch (_timer)
      {
        //////////////////////////////////////
        //////////////////////////////////////

        case TIMERA0:

          PWM_LOGDEBUG("setPWM_Int: TIMERA0");

          setPeriod_TimerA0(1000000UL / frequency);

          //_dutycycle = ( ((uint32_t) pwmPeriod * dutycycle) + 1 ) >> 16;
          _dutycycle = ( (uint32_t) pwmPeriod * dutycycle ) >> 16;

          // start from 0, so to add 1 to DC and period
          PWM_LOGDEBUG3("setPWM_Int: TIMERA0, _dutycycle =", _dutycycle, ", DC % =", (_dutycycle + 1) * 100.0f / pwmPeriod);

          // Calculate correct compare buffer register
          timer_cmp_out = ((uint16_t*) (&TCA0.SINGLE.CMP0BUF)) + bit_pos;

          // Configure duty cycle for correct compare channel
          //(*timer_cmp_out) = _dutycycle;
          (*timer_cmp_out) = _dutycycle + 1;

          // Enable output on pin
          TCA0.SINGLE.CTRLB |= (1 << (TCA_SINGLE_CMP0EN_bp + bit_pos));

          break;

        case TIMERB0:
        case TIMERB1:
        case TIMERB2:
        case TIMERB3:

          // Get pointer to timer
          timer_B = ((TCB_t *)&TCB0 + (_timer - TIMERB0));

          // 8 bit PWM mode
          timer_B->CTRLB = (TCB_CNTMODE_PWM8_gc);

          setPeriod_TimerB(1000000UL / frequency);

          //_dutycycle = ( ((uint32_t) pwmPeriod * dutycycle) + 1) >> 8;
          _dutycycle = ( (uint32_t) pwmPeriod * dutycycle ) >> 8;

          // start from 0, so to add 1 to DC and period
          PWM_LOGDEBUG3("setPWM_Int: TIMERB, _dutycycle =", _dutycycle, ", DC % =", (_dutycycle + 1) * 100.0f / pwmPeriod);

          // set duty cycle
          //timer_B->CCMPH = _dutycycle;
          timer_B->CCMPH = _dutycycle + 1;

          // Enable Timer Output
          timer_B->CTRLB |= (TCB_CCMPEN_bm);

          break;

        //////////////////////////////////////
        //////////////////////////////////////

        case NOT_ON_TIMER:
        default:

          PWM_LOGERROR1("Error, not usable for PWM, pin =", pin);

          if (_dutycycle < MAX_COUNT_16BIT / 2)
          {
            digitalWrite(pin, LOW);
          }
          else
          {
            digitalWrite(pin, HIGH);
          }

          return false;
      }

      return true;
    }

    ///////////////////////////////////////////

    bool setPWM()
    {
      return setPWM_Int(_pin, _frequency, _dutycycle);
    }

    ///////////////////////////////////////////

    bool setPWM(const uint8_t& pin, const float& frequency, const float& dutycycle)
    {
      _dutycycle = map(dutycycle, 0, 100.0f, 0, MAX_COUNT_16BIT);

      PWM_LOGDEBUG1("setPWM: _dutycycle =", _dutycycle);

      return setPWM_Int(pin, frequency, _dutycycle);
    }

    ///////////////////////////////////////////

    bool setPWM_Period(const uint8_t& pin, const float& period_us, const float& dutycycle)
    {
      _dutycycle = map(dutycycle, 0, 100.0f, 0, MAX_COUNT_16BIT);

      PWM_LOGDEBUG1("setPWM_Period: _dutycycle =", _dutycycle);

      return setPWM_Int(pin, 1000000.0f / period_us, _dutycycle);
    }

    ///////////////////////////////////////////

    // Must have same frequency
    bool setPWM_manual(const uint8_t& pin, const uint16_t& DCValue)
    {
      uint8_t bit_pos  = digitalPinToBitPosition(pin);

      // Not the same pin or overvalue
      if ( (_pin != pin) || (DCValue > pwmPeriod) )
        return false;

      switch (_timer)
      {
        //////////////////////////////////////
        //////////////////////////////////////

        case TIMERA0:
          // Configure duty cycle for correct compare channel
          (*timer_cmp_out) = DCValue;

          // Enable output on pin
          TCA0.SINGLE.CTRLB |= (1 << (TCA_SINGLE_CMP0EN_bp + bit_pos));

          break;

        case TIMERB0:
        case TIMERB1:
        case TIMERB2:
        case TIMERB3:

          // set duty cycle
          timer_B->CCMPH = DCValue;

          // Enable Timer Output
          timer_B->CTRLB |= (TCB_CCMPEN_bm);

          break;

        //////////////////////////////////////
        //////////////////////////////////////

        case NOT_ON_TIMER:
        default:

          PWM_LOGERROR1("Error, not usable for PWM, pin =", pin);

          if (_dutycycle < MAX_COUNT_16BIT / 2)
          {
            digitalWrite(pin, LOW);
          }
          else
          {
            digitalWrite(pin, HIGH);
          }

          return false;
      }

      PWM_LOGDEBUG7("PWM enabled, DCValue =", DCValue, ", pwmPeriod =", pwmPeriod,
                    ", _frequency =", _frequency, ", _actualFrequency =", _actualFrequency);

      return true;
    }

    ///////////////////////////////////////////

    inline float getActualDutyCycle()
    {
      // start from 0, so to add 1 to DC and period
      return ( (float) (_dutycycle + 1) * 100 / pwmPeriod);
    }

    ///////////////////////////////////////////

    inline float getActualFreq()
    {
      return _actualFrequency;
    }

    ///////////////////////////////////////////

    inline float getPWMPeriod()
    {
      return pwmPeriod;
    }

    ///////////////////////////////////////////

    inline uint32_t get_freq_CPU()
    {
      return freq_CPU;
    }

    ///////////////////////////////////////////

    inline uint32_t getPin()
    {
      return _pin;
    }

    ///////////////////////////////////////////////////////////////////

  private:

    uint32_t        freq_CPU;

    float           _actualFrequency;
    float           _frequency;

    // dutycycle from 0-65535 for 0%-100% to make use of 16-bit top register
    // _dutycycle = map(dutycycle, 0, 100.0f, 0, 65535) for better accuracy
    uint16_t        _dutycycle;
    //////////

    uint8_t         _pin;

#define INVALID_TIMER     255

    uint8_t         _timer = INVALID_TIMER;

    TCB_t*          timer_B;

    uint16_t*       timer_cmp_out;

    uint16_t        pwmPeriod;

    ///////////////////////////////////////////
};

///////////////////////////////////////////


#endif    // MEGA_AVR_PWM_H

