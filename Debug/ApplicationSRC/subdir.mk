################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ApplicationSRC/CAN.c \
../ApplicationSRC/EPOS4.c \
../ApplicationSRC/MadgwickAHRS.c \
../ApplicationSRC/StateFormulas.c \
../ApplicationSRC/bsp_driver_sd.c \
../ApplicationSRC/controller.c \
../ApplicationSRC/fatfs.c \
../ApplicationSRC/gpio_functions.c \
../ApplicationSRC/knee_control.c \
../ApplicationSRC/main.c \
../ApplicationSRC/mcp25625.c \
../ApplicationSRC/peripherals.c \
../ApplicationSRC/sensor.c \
../ApplicationSRC/spi1.c \
../ApplicationSRC/spi2.c \
../ApplicationSRC/stm32l4xx_hal_msp.c \
../ApplicationSRC/stm32l4xx_it.c \
../ApplicationSRC/system_stm32l4xx.c \
../ApplicationSRC/time_functions.c \
../ApplicationSRC/usbd_conf.c \
../ApplicationSRC/usbd_desc.c \
../ApplicationSRC/usbd_storage.c 

OBJS += \
./ApplicationSRC/CAN.o \
./ApplicationSRC/EPOS4.o \
./ApplicationSRC/MadgwickAHRS.o \
./ApplicationSRC/StateFormulas.o \
./ApplicationSRC/bsp_driver_sd.o \
./ApplicationSRC/controller.o \
./ApplicationSRC/fatfs.o \
./ApplicationSRC/gpio_functions.o \
./ApplicationSRC/knee_control.o \
./ApplicationSRC/main.o \
./ApplicationSRC/mcp25625.o \
./ApplicationSRC/peripherals.o \
./ApplicationSRC/sensor.o \
./ApplicationSRC/spi1.o \
./ApplicationSRC/spi2.o \
./ApplicationSRC/stm32l4xx_hal_msp.o \
./ApplicationSRC/stm32l4xx_it.o \
./ApplicationSRC/system_stm32l4xx.o \
./ApplicationSRC/time_functions.o \
./ApplicationSRC/usbd_conf.o \
./ApplicationSRC/usbd_desc.o \
./ApplicationSRC/usbd_storage.o 

C_DEPS += \
./ApplicationSRC/CAN.d \
./ApplicationSRC/EPOS4.d \
./ApplicationSRC/MadgwickAHRS.d \
./ApplicationSRC/StateFormulas.d \
./ApplicationSRC/bsp_driver_sd.d \
./ApplicationSRC/controller.d \
./ApplicationSRC/fatfs.d \
./ApplicationSRC/gpio_functions.d \
./ApplicationSRC/knee_control.d \
./ApplicationSRC/main.d \
./ApplicationSRC/mcp25625.d \
./ApplicationSRC/peripherals.d \
./ApplicationSRC/sensor.d \
./ApplicationSRC/spi1.d \
./ApplicationSRC/spi2.d \
./ApplicationSRC/stm32l4xx_hal_msp.d \
./ApplicationSRC/stm32l4xx_it.d \
./ApplicationSRC/system_stm32l4xx.d \
./ApplicationSRC/time_functions.d \
./ApplicationSRC/usbd_conf.d \
./ApplicationSRC/usbd_desc.d \
./ApplicationSRC/usbd_storage.d 


# Each subdirectory must supply rules for building sources it contributes
ApplicationSRC/%.o: ../ApplicationSRC/%.c
	arm-atollic-eabi-gcc -c -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:\Projects\UA\knee_control\Firmware\Drivers\CMSIS\Device\ST\STM32L4xx\Include" -I"C:\Projects\UA\knee_control\Firmware\Middlewares\Third_Party\FatFs\src" -I"C:\Projects\UA\knee_control\Firmware\Drivers\STM32L4xx_HAL_Driver\Inc" -I"C:\Projects\UA\knee_control\Firmware\Drivers\STM32L4xx_HAL_Driver\Inc\Legacy" -I"C:\Projects\UA\knee_control\Firmware\Middlewares\ST\STM32_USB_Device_Library\Core\Inc" -I"C:\Projects\UA\knee_control\Firmware\Middlewares\ST\STM32_USB_Device_Library\Class\MSC\Inc" -I"C:\Projects\UA\knee_control\Firmware\Middlewares\Third_Party\FatFs\src\drivers" -I"C:\Projects\UA\knee_control\Firmware\Drivers\CMSIS\Include" -I"C:\Projects\UA\knee_control\Firmware\IncludesSRC" -Os -ffunction-sections -fdata-sections -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@" "$<"
