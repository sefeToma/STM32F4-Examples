# STM32F4-Examples

Examples for STM32F411E-DISCOVERY with and without HAL library.

The main code is written in C language.

Every examples also has a .hex file which you can upload to your STM board with STM32 ST-LINK Utility program.

# Example description
## Without HAL

STM32F4_PWM - Turn on LEDs using PMW. Green LED is on for 4 seconds, Orange LED for 3 seconds, Red LED for 2 seconds and Blue LED for 1 second.

STM32F4_TIM - Every second an interupt is triggered and turns on or off a LED.


## CubeIDE Generated

STM32F4_PWM_Cube - Change the brightness of the LEDs using PWM. 

STM32F4_Cap_Sensing - Using capacitive sensing to turn on 4 LEDs with self made buttons from aluminium foil. Buttons are connected to PB0, PB1, PC1 and PC2 pins.  

