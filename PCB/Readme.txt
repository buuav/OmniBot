Instruction for importing mbed source code to the online compiler:

1. You need to import the mbed SDK (called "mbed-src") to your project (for your "mbed.h"). The link of the SDK is:
https://developer.mbed.org/handbook/mbed-SDK

2. You need to comment a line in the mbed SDK. The procedure is:
a. open the online compiler
b. navigate to:
mbed-src/targets/cmsis/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F411RE/system_stm32f4xx.c
c. find the function called "void SystemInit(void)"
d. comment "SetSysClock();" inside the function found in the previous step.

Warning: failing to comment this line will let the MCU go to a infinite dead loop before entering main().
