# megaAVR_PWM Library

[![arduino-library-badge](https://www.ardu-badge.com/badge/megaAVR_PWM.svg?)](https://www.ardu-badge.com/megaAVR_PWM)
[![GitHub release](https://img.shields.io/github/release/khoih-prog/megaAVR_PWM.svg)](https://github.com/khoih-prog/megaAVR_PWM/releases)
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/khoih-prog/megaAVR_PWM/blob/main/LICENSE)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](#Contributing)
[![GitHub issues](https://img.shields.io/github/issues/khoih-prog/megaAVR_PWM.svg)](http://github.com/khoih-prog/megaAVR_PWM/issues)

<a href="https://www.buymeacoffee.com/khoihprog6" title="Donate to my libraries using BuyMeACoffee"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Donate to my libraries using BuyMeACoffee" style="height: 50px !important;width: 181px !important;" ></a>
<a href="https://www.buymeacoffee.com/khoihprog6" title="Donate to my libraries using BuyMeACoffee"><img src="https://img.shields.io/badge/buy%20me%20a%20coffee-donate-orange.svg?logo=buy-me-a-coffee&logoColor=FFDD00" style="height: 20px !important;width: 200px !important;" ></a>

---
---

## Table of Contents

* [Why do we need this megaAVR_PWM library](#why-do-we-need-this-megaAVR_PWM-library)
  * [Features](#features)
  * [Why using hardware-based PWM is better](#why-using-hardware-based-pwm-is-better)
  * [Currently supported Boards](#currently-supported-boards)
* [Changelog](changelog.md)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
  * [Use Arduino Library Manager](#use-arduino-library-manager)
  * [Manual Install](#manual-install)
  * [VS Code & PlatformIO](#vs-code--platformio)
* [More useful Information](#more-useful-information)
  * [1. Documents](#1-Documents)
  * [2. Timer TCB0-TCB3](#2-Timer-TCB0-TCB3)
  * [3. Important Notes](#3-important-notes)
* [Usage](#usage)
  * [1. Create PWM Instance with Pin, Frequency and dutycycle](#1-Create-PWM-Instance-with-Pin-Frequency-and-dutycycle)
  * [2. Initialize PWM Instance](#2-Initialize-PWM-Instance)
  * [3. Set or change PWM frequency or dutyCycle](#3-set-or-change-PWM-frequency-or-dutyCycle)
  * [4. Set or change PWM frequency and dutyCycle manually and efficiently in waveform creation](#4-Set-or-change-PWM-frequency-and-dutyCycle-manually-and-efficiently-in-waveform-creation)
* [Examples](#examples)
  * [ 1. PWM_Basic](examples/PWM_Basic)
  * [ 2. PWM_DynamicDutyCycle](examples/PWM_DynamicDutyCycle) 
  * [ 3. PWM_DynamicDutyCycle_Int](examples/PWM_DynamicDutyCycle_Int)
  * [ 4. PWM_DynamicFreq](examples/PWM_DynamicFreq)
  * [ 5. PWM_Multi](examples/PWM_Multi)
  * [ 6. PWM_MultiChannel](examples/PWM_MultiChannel)
  * [ 7. PWM_Waveform](examples/PWM_Waveform)
* [Example PWM_Multi](#example-PWM_Multi)
* [Debug Terminal Output Samples](#debug-terminal-output-samples)
  * [1. PWM_DynamicDutyCycle on MegaCoreX Nano Every](#1-PWM_DynamicDutyCycle-on-MegaCoreX-Nano-Every)
  * [2. PWM_Multi on megaAVR Nano Every](#2-PWM_Multi-on-megaAVR-Nano-Every)
  * [3. PWM_DynamicFreq on megaAVR Nano Every](#3-PWM_DynamicFreq-on-megaAVR-Nano-Every)
  * [4. PWM_Waveform on megaAVR Nano Every](#4-PWM_Waveform-on-megaAVR-Nano-Every)
* [Debug](#debug)
* [Troubleshooting](#troubleshooting)
* [Issues](#issues)
* [TO DO](#to-do)
* [DONE](#done)
* [Contributions and Thanks](#contributions-and-thanks)
* [Contributing](#contributing)
* [License](#license)
* [Copyright](#copyright)

---
---

### Why do we need this [megaAVR_PWM library](https://github.com/khoih-prog/megaAVR_PWM)

### Features

This hardware-based PWM library enables you to use Hardware-PWM on megaAVR-based boards to create and output PWM. These purely hardware-based PWM channels can generate very high PWM frequencies, depending on CPU clock and acceptable accuracy, due to 8 or 16-bit PWM / Timer registers.

This library is using the **same or similar functions** as other FastPWM libraries, as follows, to enable you to **port your PWM code easily between platforms**

- [**RP2040_PWM**](https://github.com/khoih-prog/RP2040_PWM)
- [**AVR_PWM**](https://github.com/khoih-prog/AVR_PWM)
- [**megaAVR_PWM**](https://github.com/khoih-prog/megaAVR_PWM)
- [**ESP32_FastPWM**](https://github.com/khoih-prog/ESP32_FastPWM)
- [**SAMD_PWM**](https://github.com/khoih-prog/SAMD_PWM)
- [**SAMDUE_PWM**](https://github.com/khoih-prog/SAMDUE_PWM)
- [**nRF52_PWM**](https://github.com/khoih-prog/nRF52_PWM)
- [**Teensy_PWM**](https://github.com/khoih-prog/Teensy_PWM)
- [**ATtiny_PWM**](https://github.com/khoih-prog/ATtiny_PWM)
- [**Portenta_H7_PWM**](https://github.com/khoih-prog/Portenta_H7_PWM)
- [**MBED_RP2040_PWM**](https://github.com/khoih-prog/MBED_RP2040_PWM)
- [**nRF52_MBED_PWM**](https://github.com/khoih-prog/nRF52_MBED_PWM)
- [**STM32_PWM**](https://github.com/khoih-prog/STM32_PWM)


---

The most important feature is they're purely hardware-based PWM channels. Therefore, their operations are **not blocked by bad-behaving software functions / tasks**.

This important feature is absolutely necessary for mission-critical tasks. These hardware PWM-channels, still work even if other software functions are blocking. Moreover, they are much more precise (certainly depending on clock frequency accuracy) than other software timers using millis() or micros(). That's necessary if you need to control external systems (Servo, etc.) requiring better accuracy.

New efficient `setPWM_manual()` function enables waveform creation using PWM.

The [**PWM_Multi**](examples/PWM_Multi) example will demonstrate the usage of multichannel PWM using multiple Hardware-PWM blocks (slices). The 2 independent Hardware-PWM channels are used **to control 2 different PWM outputs**, with totally independent frequencies and dutycycles on `Arduino Mega`.

Being hardware-based PWM, their executions are not blocked by bad-behaving functions / tasks, such as connecting to WiFi, Internet or Blynk services.

This non-being-blocked important feature is absolutely necessary for mission-critical tasks.


---

#### Why using hardware-based PWM is better

Imagine you have a system with a **mission-critical** function, controlling a robot or doing something much more important. You normally use a software timer to poll, or even place the function in loop(). But what if another function is **blocking** the loop() or setup().

So your function **might not be executed, and the result would be disastrous.**

You'd prefer to have your function called, no matter what happening with other functions (busy loop, bug, etc.).

The correct choice is to use `hardware-based PWM`.

These hardware-based PWM channels still work even if other software functions are blocking. Moreover, they are much more **precise** (certainly depending on clock frequency accuracy) than other software-based PWMs, using millis() or micros().

Functions using normal software-based PWMs, relying on loop() and calling millis(), won't work if the loop() or setup() is blocked by certain operation. For example, certain function is blocking while it's connecting to WiFi or some services.


---

### Currently supported Boards

1. **megaAVR-based boards** such as **UNO WiFi Rev2, AVR_Nano_Every, etc.**, using [`Arduino megaAVR core`](https://github.com/arduino/ArduinoCore-megaavr)
2. **megaAVR-based boards** such as **UNO WiFi Rev2, AVR_Nano_Every, ATmega4809, ATmega4808, ATmega3209, ATmega3208, ATmega1609, ATmega1608, ATmega809, ATmega808, etc.**, using [`MegaCoreX megaAVR core`](https://github.com/MCUdude/MegaCoreX)


---
---

## Prerequisites

1. [`Arduino IDE 1.8.19+` for Arduino](https://github.com/arduino/Arduino). [![GitHub release](https://img.shields.io/github/release/arduino/Arduino.svg)](https://github.com/arduino/Arduino/releases/latest)
2. [`Arduino megaAVR core 1.8.7+`](https://github.com/arduino/ArduinoCore-megaavr/releases) for Arduino megaAVR boards. Use Arduino Board Manager to install.
3. [`MegaCoreX megaAVR core 1.1.0+`](https://github.com/MCUdude/MegaCoreX/releases) for Arduino megaAVR boards.  [![GitHub release](https://img.shields.io/github/release/MCUdude/MegaCoreX.svg)](https://github.com/MCUdude/MegaCoreX/releases/latest). Follow [**How to install**](https://github.com/MCUdude/MegaCoreX#how-to-install).
 
 
---
---

## Installation

### Use Arduino Library Manager

The best and easiest way is to use `Arduino Library Manager`. Search for [**megaAVR_PWM**](https://github.com/khoih-prog/megaAVR_PWM), then select / install the latest version.
You can also use this link [![arduino-library-badge](https://www.ardu-badge.com/badge/megaAVR_PWM.svg?)](https://www.ardu-badge.com/megaAVR_PWM) for more detailed instructions.

### Manual Install

Another way to install is to:

1. Navigate to [**megaAVR_PWM**](https://github.com/khoih-prog/megaAVR_PWM) page.
2. Download the latest release `megaAVR_PWM-main.zip`.
3. Extract the zip file to `megaAVR_PWM-main` directory 
4. Copy whole `megaAVR_PWM-main` folder to Arduino libraries' directory such as `~/Arduino/libraries/`.

### VS Code & PlatformIO

1. Install [VS Code](https://code.visualstudio.com/)
2. Install [PlatformIO](https://platformio.org/platformio-ide)
3. Install [**megaAVR_PWM** library](https://registry.platformio.org/libraries/khoih-prog/megaAVR_PWM) by using [Library Manager](https://registry.platformio.org/libraries/khoih-prog/megaAVR_PWM/installation). Search for **megaAVR_PWM** in [Platform.io Author's Libraries](https://platformio.org/lib/search?query=author:%22Khoi%20Hoang%22)
4. Use included [platformio.ini](platformio/platformio.ini) file from examples to ensure that all dependent libraries will installed automatically. Please visit documentation for the other options and examples at [Project Configuration File](https://docs.platformio.org/page/projectconf.html)

---
---

## More useful Information

### 1. Documents

1. [Arduino 101: Timers and Interrupts](https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072)
2. [megaAVR0-series-Family-Data-Sheet](http://ww1.microchip.com/downloads/en/DeviceDoc/megaAVR0-series-Family-Data-Sheet-DS40002015B.pdf)

### 2. Timer TCB0-TCB3

TCB0-TCB3 are 16-bit timers.

### 3. Important Notes

Before using any Timer, you have to make sure the **Timer has not been used by any other purpose.**


```
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
```


---
---

## Usage

Before using any PWM `Timer` and `channel`, you have to make sure the `Timer` and `channel` has not been used by any other purpose.

#### 1. Create PWM Instance with Pin, Frequency and dutycycle

```cpp
megaAVR_PWM* PWM_Instance;

PWM_Instance = new megaAVR_PWM(PWM_Pins, freq, dutyCycle);
```

#### 2. Initialize PWM Instance

```cpp
if (PWM_Instance)
{
  PWM_Instance->setPWM();
}
```

#### 3. Set or change PWM frequency or dutyCycle

To use `float new_dutyCycle`

```cpp
PWM_Instance->setPWM(PWM_Pins, new_frequency, new_dutyCycle);
```

such as 

```cpp
dutyCycle = 10.0f;
  
Serial.print(F("Change PWM DutyCycle to ")); Serial.println(dutyCycle);
PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
```

---

To use `uint32_t new_dutyCycle` = `(real_dutyCycle * 65536) / 100`


```cpp
PWM_Instance->setPWM_Int(PWM_Pins, new_frequency, new_dutyCycle);
```

such as for `real_dutyCycle = 50%`

```cpp
// 50% dutyCycle = (real_dutyCycle * 65535) / 100
dutyCycle = 32767;

Serial.print(F("Change PWM DutyCycle to (%) "));
Serial.println((float) dutyCycle * 100 / 65536);
PWM_Instance->setPWM_Int(pinToUse, frequency, dutyCycle);
```

for `real_dutyCycle = 50%`

```cpp
// 20% dutyCycle = (real_dutyCycle * 65535) / 100
dutyCycle = 13107;

Serial.print(F("Change PWM DutyCycle to (%) "));
Serial.println((float) dutyCycle * 100 / 65536);
PWM_Instance->setPWM_Int(pinToUse, frequency, dutyCycle);
```

#### 4. Set or change PWM frequency and dutyCycle manually and efficiently in waveform creation

Function prototype

```cpp
bool setPWM_manual(const uint8_t& pin, const uint16_t& DCValue);
```

Need to call only once for each pin


```cpp
PWM_Instance->setPWM(PWM_Pins, frequency, dutyCycle);
```

after that, if just changing `dutyCycle` / `level`, use 

```cpp
PWM_Instance->setPWM_manual(PWM_Pins, new_level);
```


---
---

### Examples: 

 1. [PWM_Basic](examples/PWM_Basic)
 2. [PWM_DynamicDutyCycle](examples/PWM_DynamicDutyCycle)
 3. [PWM_DynamicDutyCycle_Int](examples/PWM_DynamicDutyCycle_Int)
 4. [PWM_DynamicFreq](examples/PWM_DynamicFreq)
 5. [PWM_Multi](examples/PWM_Multi)
 6. [PWM_MultiChannel](examples/PWM_MultiChannel)
 7. [PWM_Waveform](examples/PWM_Waveform)

 
---
---

### Example [PWM_Multi](examples/PWM_Multi)

https://github.com/khoih-prog/megaAVR_PWM/blob/7c73a23cf395fb3e7f8bdea6015bd412fbaefd8b/examples/PWM_Multi/PWM_Multi.ino#L11-L136


---
---

### Debug Terminal Output Samples

### 1. PWM_DynamicDutyCycle on MegaCoreX Nano Every

The following is the sample terminal output when running example [PWM_DynamicDutyCycle](examples/PWM_DynamicDutyCycle) on **megaAVR Nano Every** using `MegaCoreX`, to demonstrate the ability to provide high PWM frequencies and ability to change DutyCycle `on-the-fly`.


```
Starting PWM_DynamicDutyCycle on MegaCoreX Nano Every
megaAVR_PWM v1.0.0
[PWM] megaAVR_PWM: _dutycycle = 32767
[PWM] setPWM_Int: input dutycycle = 127
[PWM] setPWM_Int: _timer = 3
[PWM] setPeriod_TimerB: F_CPU = 16000000 , cycles = 3200
[PWM] setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64
[PWM] setPeriod_TimerB: pwmPeriod = 50 , _actualFrequency = 5000.00
[PWM] setPWM_Int: TIMERB, _dutycycle = 24 , DC % = 50.00
=====================================================================================
Change PWM DutyCycle to 90.00
[PWM] setPWM: _dutycycle = 58981
[PWM] setPWM_Int: input dutycycle = 230
[PWM] setPWM_Int: _timer = 3
[PWM] setPeriod_TimerB: F_CPU = 16000000 , cycles = 16000
[PWM] setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64
[PWM] setPeriod_TimerB: pwmPeriod = 250 , _actualFrequency = 1000.00
[PWM] setPWM_Int: TIMERB, _dutycycle = 224 , DC % = 90.00
=====================================================================================
Actual data: pin = 3, PWM DC = 90.00, PWMPeriod = 250.00, PWM Freq (Hz) = 1000.0000
=====================================================================================
Change PWM DutyCycle to 10.00
[PWM] setPWM: _dutycycle = 6553
[PWM] setPWM_Int: input dutycycle = 25
[PWM] setPWM_Int: _timer = 3
[PWM] setPeriod_TimerB: F_CPU = 16000000 , cycles = 16000
[PWM] setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64
[PWM] setPeriod_TimerB: pwmPeriod = 250 , _actualFrequency = 1000.00
[PWM] setPWM_Int: TIMERB, _dutycycle = 24 , DC % = 10.00
=====================================================================================
Actual data: pin = 3, PWM DC = 10.00, PWMPeriod = 250.00, PWM Freq (Hz) = 1000.0000
=====================================================================================
Change PWM DutyCycle to 90.00
[PWM] setPWM: _dutycycle = 58981
[PWM] setPWM_Int: input dutycycle = 230
[PWM] setPWM_Int: _timer = 3
[PWM] setPeriod_TimerB: F_CPU = 16000000 , cycles = 16000
[PWM] setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64
[PWM] setPeriod_TimerB: pwmPeriod = 250 , _actualFrequency = 1000.00
[PWM] setPWM_Int: TIMERB, _dutycycle = 224 , DC % = 90.00
=====================================================================================
Actual data: pin = 3, PWM DC = 90.00, PWMPeriod = 250.00, PWM Freq (Hz) = 1000.0000
=====================================================================================
```

---

### 2. PWM_Multi on megaAVR Nano Every

The following is the sample terminal output when running example [**PWM_Multi**](examples/PWM_Multi) on **megaAVR Nano Every**, to demonstrate the ability to provide high PWM frequencies on multiple `PWM-capable` pins.

```
Starting PWM_Multi on megaAVR Nano Every
megaAVR_PWM v1.0.0
=====================================================================================
Index	Pin	PWM_freq	DutyCycle	Actual Freq
=====================================================================================
0	3	1000.00		10.00		1000.0000
1	6	4000.00		50.00		4000.0000
=====================================================================================
Actual data: pin = 3, PWM DC = 10.00, PWMPeriod = 250.00, PWM Freq (Hz) = 1000.0000
=====================================================================================
=====================================================================================
Actual data: pin = 6, PWM DC = 50.00, PWMPeriod = 62.00, PWM Freq (Hz) = 4000.0000
=====================================================================================
```

---

### 3. PWM_DynamicFreq on megaAVR Nano Every

The following is the sample terminal output when running example [**PWM_DynamicFreq**](examples/PWM_DynamicFreq) on **megaAVR Nano Every**, to demonstrate the ability to change dynamically PWM frequencies.

```
Starting PWM_DynamicFreq on megaAVR Nano Every
megaAVR_PWM v1.0.0
[PWM] megaAVR_PWM: _dutycycle = 32767
[PWM] setPWM_Int: input dutycycle = 127
[PWM] setPWM_Int: _timer = 3
[PWM] setPeriod_TimerB: F_CPU = 16000000 , cycles = 16000
[PWM] setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64
[PWM] setPeriod_TimerB: pwmPeriod = 250 , _actualFrequency = 1000.00
[PWM] setPWM_Int: TIMERB, _dutycycle = 124 , DC % = 50.00
=====================================================================================
Change PWM Freq to 4000.00
[PWM] setPWM: _dutycycle = 32767
[PWM] setPWM_Int: input dutycycle = 127
[PWM] setPWM_Int: _timer = 3
[PWM] setPeriod_TimerB: F_CPU = 16000000 , cycles = 4000
[PWM] setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64
[PWM] setPeriod_TimerB: pwmPeriod = 62 , _actualFrequency = 4000.00
[PWM] setPWM_Int: TIMERB, _dutycycle = 30 , DC % = 50.00
=====================================================================================
Actual data: pin = 3, PWM DC = 50.00, PWMPeriod = 62.00, PWM Freq (Hz) = 4000.0000
=====================================================================================
Change PWM Freq to 1000.00
[PWM] setPWM: _dutycycle = 32767
[PWM] setPWM_Int: input dutycycle = 127
[PWM] setPWM_Int: _timer = 3
[PWM] setPeriod_TimerB: F_CPU = 16000000 , cycles = 16000
[PWM] setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64
[PWM] setPeriod_TimerB: pwmPeriod = 250 , _actualFrequency = 1000.00
[PWM] setPWM_Int: TIMERB, _dutycycle = 124 , DC % = 50.00
=====================================================================================
Actual data: pin = 3, PWM DC = 50.00, PWMPeriod = 250.00, PWM Freq (Hz) = 1000.0000
=====================================================================================
```

---


### 4. PWM_Waveform on megaAVR Nano Every

The following is the sample terminal output when running example [**PWM_Waveform**](examples/PWM_Waveform) on **megaAVR Nano Every**, to demonstrate how to use the `setPWM_manual()` function in wafeform creation


```
Starting PWM_Waveform on megaAVR Nano Every
megaAVR_PWM v1.0.0
[PWM] megaAVR_PWM: _dutycycle = 0
[PWM] setPWM: _dutycycle = 0
[PWM] setPWM_Int: input dutycycle = 0
[PWM] setPWM_Int: _timer = 3
[PWM] setPeriod_TimerB: F_CPU = 16000000 , cycles = 16000
[PWM] setPeriod_TimerB: cycles < TIMERB_RESOLUTION * 64, using divider = 64
[PWM] setPeriod_TimerB: pwmPeriod = 250 , _actualFrequency = 1000.00
[PWM] setPWM_Int: TIMERB, _dutycycle = 0 , DC % = 0.40
============================================================================================
Actual data: pin = 3, PWM DutyCycle = 0.40, PWMPeriod = 250.00, PWM Freq (Hz) = 1000.0000
============================================================================================
[PWM] PWM enabled, DCValue = 0 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 15 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 31 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 46 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 62 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 78 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 93 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 109 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 125 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 140 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 156 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 171 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 187 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 203 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 218 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 234 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 250 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 234 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 218 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 203 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 187 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 171 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 156 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 140 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 125 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 109 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 93 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 78 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 62 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 46 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 31 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 15 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
[PWM] PWM enabled, DCValue = 0 , pwmPeriod = 250 , _frequency = 1000.00 , _actualFrequency = 1000.00
```

---
---

### Debug

Debug is enabled by default on Serial.

You can also change the debugging level `_PWM_LOGLEVEL_` from 0 to 4

```cpp
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_     0
```

---

### Troubleshooting

If you get compilation errors, more often than not, you may need to install a newer version of the core for Arduino boards.

Sometimes, the library will only work if you update the board core to the latest version because I am using newly added functions.


---
---

### Issues

Submit issues to: [megaAVR_PWM issues](https://github.com/khoih-prog/megaAVR_PWM/issues)

---

## TO DO

1. Search for bug and improvement.
2. Similar features for remaining Arduino boards

---

## DONE

 1. Basic hardware-based multi-channel PWMs for **megaAVR-based boards** such as `UNO WiFi Rev2`, `AVR_Nano_Every`, etc.**, using either
 - [`Arduino megaAVR core`](https://github.com/arduino/ArduinoCore-megaavr) or
 - [`MegaCoreX megaAVR core`](https://github.com/MCUdude/MegaCoreX)



---
---

### Contributions and Thanks

Many thanks for everyone for bug reporting, new feature suggesting, testing and contributing to the development of this library.

  
---

## Contributing

If you want to contribute to this project:

- Report bugs and errors
- Ask for enhancements
- Create issues and pull requests
- Tell other people about this library

---

### License

- The library is licensed under [MIT](https://github.com/khoih-prog/megaAVR_PWM/blob/main/LICENSE)

---

## Copyright

Copyright 2022- Khoi Hoang
