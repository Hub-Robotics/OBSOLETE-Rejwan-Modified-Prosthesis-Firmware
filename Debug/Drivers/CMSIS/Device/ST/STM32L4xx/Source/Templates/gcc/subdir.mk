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
	arm-atollic-eabi-gcc -c -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:\Projects\UA\knee_control\Firmware\Drivers\MPU9255" -I"C:\Projects\UA\knee_control\Firmware\SysTick_app_timer" -I"C:\Projects\UA\knee_control\Firmware\eMPL" -I"C:\Projects\UA\knee_control\Firmware\SysTick_app_timer" -I"C:\Projects\UA\knee_control\Firmware\Drivers\MPU9255" -I"C:\Projects\UA\knee_control\Firmware\eMPL" -g -Wa,--no-warn -x assembler-with-cpp -specs=nano.specs -o "$@" "$<"

