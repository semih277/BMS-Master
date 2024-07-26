################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ADBMS.c \
../Core/Src/EEPROM.c \
../Core/Src/adBms6830CmdList.c \
../Core/Src/adBms6830ParseCreate.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c 

C_DEPS += \
./Core/Src/ADBMS.d \
./Core/Src/EEPROM.d \
./Core/Src/adBms6830CmdList.d \
./Core/Src/adBms6830ParseCreate.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d 

OBJS += \
./Core/Src/ADBMS.o \
./Core/Src/EEPROM.o \
./Core/Src/adBms6830CmdList.o \
./Core/Src/adBms6830ParseCreate.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/SoC" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS/include" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/Config" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/OS" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/SEGGER" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/USB_DEVICE/App" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/USB_DEVICE/Target" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ADBMS.cyclo ./Core/Src/ADBMS.d ./Core/Src/ADBMS.o ./Core/Src/ADBMS.su ./Core/Src/EEPROM.cyclo ./Core/Src/EEPROM.d ./Core/Src/EEPROM.o ./Core/Src/EEPROM.su ./Core/Src/adBms6830CmdList.cyclo ./Core/Src/adBms6830CmdList.d ./Core/Src/adBms6830CmdList.o ./Core/Src/adBms6830CmdList.su ./Core/Src/adBms6830ParseCreate.cyclo ./Core/Src/adBms6830ParseCreate.d ./Core/Src/adBms6830ParseCreate.o ./Core/Src/adBms6830ParseCreate.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su

.PHONY: clean-Core-2f-Src

