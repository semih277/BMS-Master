################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/FreeRTOS/croutine.c \
../ThirdParty/FreeRTOS/event_groups.c \
../ThirdParty/FreeRTOS/list.c \
../ThirdParty/FreeRTOS/queue.c \
../ThirdParty/FreeRTOS/stream_buffer.c \
../ThirdParty/FreeRTOS/tasks.c \
../ThirdParty/FreeRTOS/timers.c 

C_DEPS += \
./ThirdParty/FreeRTOS/croutine.d \
./ThirdParty/FreeRTOS/event_groups.d \
./ThirdParty/FreeRTOS/list.d \
./ThirdParty/FreeRTOS/queue.d \
./ThirdParty/FreeRTOS/stream_buffer.d \
./ThirdParty/FreeRTOS/tasks.d \
./ThirdParty/FreeRTOS/timers.d 

OBJS += \
./ThirdParty/FreeRTOS/croutine.o \
./ThirdParty/FreeRTOS/event_groups.o \
./ThirdParty/FreeRTOS/list.o \
./ThirdParty/FreeRTOS/queue.o \
./ThirdParty/FreeRTOS/stream_buffer.o \
./ThirdParty/FreeRTOS/tasks.o \
./ThirdParty/FreeRTOS/timers.o 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/FreeRTOS/%.o ThirdParty/FreeRTOS/%.su ThirdParty/FreeRTOS/%.cyclo: ../ThirdParty/FreeRTOS/%.c ThirdParty/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/SoC" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS/include" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/Config" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/OS" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/ThirdParty/SEGGER/SEGGER" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/USB_DEVICE/App" -I"C:/STM/ws1/bms-master-firmware-2024-main/bms-master-firmware-2024-main24.07.2024/bms-master-firmware-2024-main/USB_DEVICE/Target" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-FreeRTOS

clean-ThirdParty-2f-FreeRTOS:
	-$(RM) ./ThirdParty/FreeRTOS/croutine.cyclo ./ThirdParty/FreeRTOS/croutine.d ./ThirdParty/FreeRTOS/croutine.o ./ThirdParty/FreeRTOS/croutine.su ./ThirdParty/FreeRTOS/event_groups.cyclo ./ThirdParty/FreeRTOS/event_groups.d ./ThirdParty/FreeRTOS/event_groups.o ./ThirdParty/FreeRTOS/event_groups.su ./ThirdParty/FreeRTOS/list.cyclo ./ThirdParty/FreeRTOS/list.d ./ThirdParty/FreeRTOS/list.o ./ThirdParty/FreeRTOS/list.su ./ThirdParty/FreeRTOS/queue.cyclo ./ThirdParty/FreeRTOS/queue.d ./ThirdParty/FreeRTOS/queue.o ./ThirdParty/FreeRTOS/queue.su ./ThirdParty/FreeRTOS/stream_buffer.cyclo ./ThirdParty/FreeRTOS/stream_buffer.d ./ThirdParty/FreeRTOS/stream_buffer.o ./ThirdParty/FreeRTOS/stream_buffer.su ./ThirdParty/FreeRTOS/tasks.cyclo ./ThirdParty/FreeRTOS/tasks.d ./ThirdParty/FreeRTOS/tasks.o ./ThirdParty/FreeRTOS/tasks.su ./ThirdParty/FreeRTOS/timers.cyclo ./ThirdParty/FreeRTOS/timers.d ./ThirdParty/FreeRTOS/timers.o ./ThirdParty/FreeRTOS/timers.su

.PHONY: clean-ThirdParty-2f-FreeRTOS

