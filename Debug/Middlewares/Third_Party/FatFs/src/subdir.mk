################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/diskio.c \
../Middlewares/Third_Party/FatFs/src/ff.c \
../Middlewares/Third_Party/FatFs/src/ff_gen_drv.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/diskio.o \
./Middlewares/Third_Party/FatFs/src/ff.o \
./Middlewares/Third_Party/FatFs/src/ff_gen_drv.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/diskio.d \
./Middlewares/Third_Party/FatFs/src/ff.d \
./Middlewares/Third_Party/FatFs/src/ff_gen_drv.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/%.o: ../Middlewares/Third_Party/FatFs/src/%.c
	arm-atollic-eabi-gcc -c -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Middlewares\Third_Party\FatFs\src" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Middlewares\Third_Party\FatFs\src\drivers" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\IncludesSRC" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Drivers\CMSIS\Device\ST\STM32L4xx\Include" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Drivers\CMSIS\Include" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Drivers\STM32L4xx_HAL_Driver\Inc" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Middlewares\ST\STM32_USB_Device_Library\Class\MSC\Inc" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Middlewares\ST\STM32_USB_Device_Library\Core\Inc" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Middlewares\DMP" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Middlewares\SystickAppTimer" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Drivers\MCP25625" -I"C:\Projects\UA\knee_control\Rejwan-Prosthesis-Modified-Firmware\Drivers\MPU9255" -Os -ffunction-sections -fdata-sections -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@" "$<"

