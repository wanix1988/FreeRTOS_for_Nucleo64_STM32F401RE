# Porting FreeRTOS to ST official Nucleo64_STM32F401RE board

## Description
[Nucleo64-STM32F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)

The STM32 Nucleo-64 board provides an affordable and flexible way for users to try out new concepts and 
build prototypes by choosing from the various combinations of performance and power consumption features provided by the STM32 microcontroller. 
For the compatible boards, the internal or external SMPS significantly reduces power consumption in Run mode.

The ARDUINO® Uno V3 connectivity support and the ST morpho headers allow the easy expansion of the functionality of the STM32 Nucleo open development platform with a wide choice of specialized shields.

The STM32 Nucleo-64 board does not require any separate probe as it **integrates the ST-LINK debugger/programmer**.

The STM32 Nucleo-64 board comes with the STM32 comprehensive free software libraries and examples available with the STM32Cube MCU Package.

![Nucleo64_STM32F401RE](https://www.st.com/bin/ecommerce/api/image.PF260000.en.feature-description-include-personalized-no-cpn-medium.jpg)

## All features
- Common features
  - STM32 microcontroller in an LQFP64 or LQFP48 package
  - 1 user LED shared with ARDUINO®
  - 1 user and 1 reset push-buttons
  - 32.768 kHz crystal oscillator
  - Board connectors:
    - ARDUINO® Uno V3 expansion connector
    - ST morpho extension pin headers for full access to all STM32 I/Os
  - Flexible power-supply options: ST-LINK USB VBUS or external sources
  - Comprehensive free software libraries and examples available with the STM32Cube MCU Package
  - Support of a wide choice of Integrated Development Environments (IDEs) including IAR Embedded Workbench®, MDK-ARM, and STM32CubeIDE
- Features specific to some of the boards
  - External or internal SMPS to generate Vcore logic supply:NUCLEO-L412RB-P, NUCLEO-L433RC-P, NUCLEO-L452RE-P, and NUCLEO-U545RE-Q
  - 24 MHz or 48 MHz HSE:NUCLEO-C031C6, NUCLEO-G431RB, NUCLEO-G474RE, and NUCLEO-G491RE
  - User USB Device full speed, or USB SNK/UFP full speed:NUCLEO-H503RB, NUCLEO-H533RE, and NUCLEO-U545RE-Q
  - Cryptography:NUCLEO-H533RE, NUCLEO-U083RC, and NUCLEO-U545RE-Q
  - Board connectors:
    - External SMPS experimentation dedicated connector:NUCLEO-L412RB-P, NUCLEO-L433RC-P, NUCLEO-L452RE-P, and NUCLEO-U545RE-Q
    - USB Type-C®, Micro-B, or Mini-B connector for the ST-LINK
    - USB Type-C® user connector:NUCLEO-H503RB, NUCLEO-H533RE, and NUCLEO-U545RE-Q
    - MIPI® debug connector:NUCLEO-G431RB, NUCLEO-G474RE, NUCLEO-G491RE, NUCLEO-H503RB, NUCLEO-H533RE, NUCLEO-U031R8, and NUCLEO-U083RC
  - On-board ST-LINK (STLINK/V2-1, STLINK-V3E, STLINK-V2EC, or STLINK-V3EC) debugger/programmer with USB re-enumeration capability: mass storage, Virtual COM port, and debug port

## FreeRTOS

**version: v202107.00**

Download from https://www.freertos.org.

## Build
1. Open Project.uvprojx with Keil μVision 5
2. Build
```
Rebuild started: Project: Project
*** Using Compiler 'V5.06 update 7 (build 960)', folder: 'C:\Keil_v5\ARM\ARMCC\Bin'
Rebuild target 'STM32F4xx-Nucleo'
compiling stm32f4xx_nucleo.c...
compiling stm32f4xx_hal.c...
compiling stm32f4xx_hal_cortex.c...
compiling stm32f4xx_hal_i2c.c...
compiling stm32f4xx_hal_i2c_ex.c...
compiling stm32f4xx_hal_gpio.c...
compiling stm32f4xx_hal_usart.c...
compiling stm32f4xx_hal_dma.c...
compiling stm32f4xx_hal_uart.c...
compiling stm32f4xx_hal_dma2d.c...
compiling stm32f4xx_hal_rcc.c...
compiling stm32f4xx_hal_dma_ex.c...
compiling stm32f4xx_hal_rcc_ex.c...
compiling stm32f4xx_hal_spi.c...
assembling startup_stm32f401xe.s...
compiling stm32f4xx_hal_adc.c...
compiling stm32f4xx_hal_msp.c...
compiling stm32f4xx_it.c...
compiling stm32f4xx_hal_adc_ex.c...
compiling croutine.c...
compiling event_groups.c...
compiling system_stm32f4xx.c...
compiling list.c...
compiling main.c...
compiling stream_buffer.c...
compiling queue.c...
compiling tasks.c...
compiling heap_4.c...
compiling timers.c...
compiling port.c...
linking...
Program Size: Code=13032 RO-data=468 RW-data=180 ZI-data=78196  
FromELF: creating hex file...
"STM32F4xx-Nucleo\STM32F4xx-Nucleo.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:55
```

## Download Programm to flash memory
Click the Load button on toolbar, download programm to flash memory

![image](https://github.com/wanix1988/FreeRTOS_for_Nucleo64_STM32F401RE/assets/2557883/4030785c-fa4e-40bc-b0af-d52c9e93e7cc)

## Run on Nucleo64_STM32F401RE board

connect the board and computer with USB cable， and set the baud rate to **115200**, then you can see the output below:

```
led blink...
led blink...
led blink...
```


