################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SoC/BatterySOCEstimationV2.c \
../SoC/BatterySOCEstimationV2_data.c 

C_DEPS += \
./SoC/BatterySOCEstimationV2.d \
./SoC/BatterySOCEstimationV2_data.d 

OBJS += \
./SoC/BatterySOCEstimationV2.o \
./SoC/BatterySOCEstimationV2_data.o 


# Each subdirectory must supply rules for building sources it contributes
SoC/%.o SoC/%.su SoC/%.cyclo: ../SoC/%.c SoC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/SoC" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS/include" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/Config" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/OS" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/SEGGER" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/USB_DEVICE/App" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/USB_DEVICE/Target" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SoC

clean-SoC:
	-$(RM) ./SoC/BatterySOCEstimationV2.cyclo ./SoC/BatterySOCEstimationV2.d ./SoC/BatterySOCEstimationV2.o ./SoC/BatterySOCEstimationV2.su ./SoC/BatterySOCEstimationV2_data.cyclo ./SoC/BatterySOCEstimationV2_data.d ./SoC/BatterySOCEstimationV2_data.o ./SoC/BatterySOCEstimationV2_data.su

.PHONY: clean-SoC

