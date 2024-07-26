################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/FreeRTOS/portable/MemMang/heap_4.c 

C_DEPS += \
./ThirdParty/FreeRTOS/portable/MemMang/heap_4.d 

OBJS += \
./ThirdParty/FreeRTOS/portable/MemMang/heap_4.o 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/FreeRTOS/portable/MemMang/%.o ThirdParty/FreeRTOS/portable/MemMang/%.su ThirdParty/FreeRTOS/portable/MemMang/%.cyclo: ../ThirdParty/FreeRTOS/portable/MemMang/%.c ThirdParty/FreeRTOS/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/SoC" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS/include" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/Config" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/OS" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/SEGGER" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/USB_DEVICE/App" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/USB_DEVICE/Target" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-MemMang

clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-MemMang:
	-$(RM) ./ThirdParty/FreeRTOS/portable/MemMang/heap_4.cyclo ./ThirdParty/FreeRTOS/portable/MemMang/heap_4.d ./ThirdParty/FreeRTOS/portable/MemMang/heap_4.o ./ThirdParty/FreeRTOS/portable/MemMang/heap_4.su

.PHONY: clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-MemMang

