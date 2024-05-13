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
![image](https://github.com/AlogoZano/Embedded-intelligent-control-platform/assets/160699916/1260de76-67e5-4461-923f-0cb766dae576)
![image](https://github.com/AlogoZano/Embedded-intelligent-control-platform/assets/160699916/767687ed-9999-4c29-8f0d-d015d3d94185)
![image](https://github.com/AlogoZano/Embedded-intelligent-control-platform/assets/160699916/8a342156-2b87-4f18-93ac-b6c458667d0d)
![image](https://github.com/AlogoZano/Embedded-intelligent-control-platform/assets/160699916/082b27fc-8c57-4f49-b993-400a9e494bfd)

