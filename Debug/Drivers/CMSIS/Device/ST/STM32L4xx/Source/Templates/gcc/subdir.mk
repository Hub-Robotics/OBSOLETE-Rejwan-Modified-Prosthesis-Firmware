################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l476xx.s 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l476xx.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/%.o: ../Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/%.s
	arm-atollic-eabi-gcc -c -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\Third_Party\FatFs\src" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\Third_Party\FatFs\src\drivers" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\IncludesSRC" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\CMSIS\Device\ST\STM32L4xx\Include" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\CMSIS\Include" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\STM32L4xx_HAL_Driver\Inc" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\ST\STM32_USB_Device_Library\Class\MSC\Inc" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\ST\STM32_USB_Device_Library\Core\Inc" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\DMP" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\SystickAppTimer" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\MCP25625" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\MPU9255" -g -Wa,--no-warn -x assembler-with-cpp -specs=nano.specs -o "$@" "$<"

