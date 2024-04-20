################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/drivers/%.o: ../Middlewares/Third_Party/FatFs/src/drivers/%.c
	arm-atollic-eabi-gcc -c -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\Third_Party\FatFs\src" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\Third_Party\FatFs\src\drivers" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\IncludesSRC" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\CMSIS\Device\ST\STM32L4xx\Include" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\CMSIS\Include" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\STM32L4xx_HAL_Driver\Inc" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\ST\STM32_USB_Device_Library\Class\MSC\Inc" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\ST\STM32_USB_Device_Library\Core\Inc" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\DMP" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Middlewares\SystickAppTimer" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\MCP25625" -I"C:\OneDrive\Clients\002 - Prof Shen\12 Code\01 Embedded Firmware\01 Rejwan Modified Prosthesis Control\After DMP\Drivers\MPU9255" -O0 -ffunction-sections -fdata-sections -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@" "$<"

