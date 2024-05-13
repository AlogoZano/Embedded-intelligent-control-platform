# Embedded-intelligent-control-platform

Throughout this project, an embedded control loop system was designed in order to control the position of a John Deere tractor front-end loader. The platform also calculated vibration thresholds based on frequency analysis. Finally, everything is reported to a dashboard. The project implements advanced communication protocols such as CAN, USB, I2C, UART, MQTT and HTTP. Everything was designed within a STM32 NUCLEO-H745ZI-Q which was helpful due to its dual core nature, paralelism and RTOS implementation.}

## General requirements
* Computer with the STM32 cube IDE
* Intermediate knowledge of microcontrollers
* Basic understanding of control systems
* Intermediate understanding of C
* Access to Arduino Cloud 
* Access to the following materials:
  * STM32 NUCLEO-H745ZI-Q 
  * High reduction DC motor (view https://articulo.mercadolibre.com.mx/MLM-1870381714-motor-dc-12v390-dc-caja-de-cambios-_JM?matt_tool=28238160&utm_source=google_shopping&utm_medium=organic)
  * AS5600 magnetic sensor
  * Pieces for rotary encoder (view https://github.com/scottbez1/AS5600Knob)
  * MPU6050 IMU
  * ESP32 WROOM 32
  * MCP2515 (CAN Bus SPI)
  * SN65HVD230 (CAN transceiver)
  * 2 - push buttons
  * 2 - 330 ohm resistor
  * OLED Display witth SH1106 or SSD1306 driver (https://github.com/afiskon/stm32-ssd1306/tree/master)
  * 5 meters of 16 AWG cable
 
## Schematic diagram
<p align="center">
<img src="https://github.com/AlogoZano/Embedded-intelligent-control-platform/assets/160699916/5e30e50e-0bb3-4c6c-8b05-f9ddd1a9df8b"/>
</p>

## Dashboard
This mobile dashboard was created using Arduino cloud and controls position, queries the actual position and error coming from the vibration analysis via the FFT development.
<p align="center">
<img src="https://github.com/AlogoZano/Embedded-intelligent-control-platform/assets/160699916/b01e0992-a99c-4e3f-a558-ee4340a9a0b3"/>
</p>

## Control response (time domain)
The PID controller response follos the reference accurately with a settling time of 0.6s and a 2.3% overshoot.
<p align="center">
<img src="https://github.com/AlogoZano/Embedded-intelligent-control-platform/assets/160699916/122b8105-12b6-431d-bd51-1bfa85cf9602"/>
</p>

## Full demonstration
View here: https://github.com/AlogoZano/Embedded-intelligent-control-platform/tree/main/Demo

