STM: I2C Peripherals and OS
Contributors
Johan Trippitelli (ID: 260917958)

Abstract
This project aims to familiarize users with multiple input sensor readings, I2C configuration/use, UART configuration/use, and OS threading. The tasks involve reading values from four sensors connected to I2C, converting them from analog to digital, and displaying them on the terminal using UART.

Table of Contents
Introduction
Part 1: I2C and UART Configuration
A. MX Setup
B. User Code and Results
Part 2: CMSIS RTOS and FreeRTOS
A. MX Configuration
B. User Code and Results
Introduction
This lab report is divided into two parts. Part 1 covers I2C and UART configuration, while Part 2 introduces CMSIS RTOS and FreeRTOS. Each part consists of two sections: A) MX configuration and necessary code alterations, and B) user code implementation and results.

The primary objective is to read values from four sensors (humidity, acceleration, magnetic fields, and pressure) connected to I2C, convert them to digital format, and display them on the terminal using UART at a frequency of 10Hz.

Part 1: I2C and UART Configuration
A. MX Setup
The microprocessor's pinout configuration includes push button and LED pins (PC13 and PB14, respectively), I2C2 peripherals (PB10 and PB11), and USART1 (PB6 and PB7). Additionally, TIM2 is configured for interrupt-based sensor reading.

USART1 is set up in asynchronous mode, I2C2 settings are left unchanged, and the push button triggers a global interrupt. TIM2 is configured to generate interrupts at a frequency of 10Hz.

B. User Code and Results
BSP files are imported, and the lab's concept involves using the push button to switch between sensors displayed on the terminal through UART. Two functions, HAL_GPIO_EXTI_Callback and HAL_TIM_PeriodElapsedCallback, handle push button actions and sensor reading, respectively.

The combination of these functions enables proper sensor reading and UART display. The results on the terminal show the values of the four sensors after pressing the push button four times.

Part 2: CMSIS RTOS and FreeRTOS
A. MX Configuration
Part 2 aims to achieve the same goal as Part 1—displaying four sensor values at a 10Hz rate through UART. The key difference is the use of an OS (FreeRTOS) to manage three tasks: button detection/output switch, data transmission to UART, and sensor reading.

The pinout configuration remains the same as Part 1, but FreeRTOS in CMSIS-V1 mode is activated. The sysclock is changed to TIM6, and three tasks—StartReadData, StartTransmitData, and StartButtonPressed—are created with appropriate stack size configurations.

B. User Code and Results
The generated code sets up everything needed for the OS. User code involves writing task logic. StartButtonPressed function handles button interrupts, toggles the LED, and updates valChoice. StartReadData initiates sensor reading, and StartTransmitData prints sensor values based on valChoice.

Results on the terminal after pressing the push button four times rapidly demonstrate continuous printing of sensor data, with the button merely switching which sensor transmits data.
