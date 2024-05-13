# Embedded-intelligent-control-platform

Throughout this project, an embedded control loop system was designed in order to control the position of a John Deere tractor front-end loader. The platform also calculated vibration thresholds based on frequency analysis. Finally, everything is reported to a dashboard. The project implements advanced communication protocols such as CAN, USB, I2C, UART, MQTT and HTTP. Everything was designed within a STM32 NUCLEO-H745ZI-Q which was helpful due to its dual core nature, paralelism and RTOS implementation.}

## General requirements
* Computer with the STM32 cube IDE
* Intermediate knowledge of microcontrollers
* Basic understanding of control systems
* Intermediate understanding of C
* Access to the following materials:
  * STM32 NUCLEO-H745ZI-Q 
  * High reduction DC motor
  * AS5600 magnetic sensor
  * Access to pieces for rotary encoder (view https://github.com/scottbez1/AS5600Knob)
  * LEDs
  * 220/330 ohm resistors
 
## Schematic diagram
