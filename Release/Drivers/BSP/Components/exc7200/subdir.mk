################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/exc7200/exc7200.c 

C_DEPS += \
./Drivers/BSP/Components/exc7200/exc7200.d 

OBJS += \
./Drivers/BSP/Components/exc7200/exc7200.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/exc7200/exc7200.o: ../Drivers/BSP/Components/exc7200/exc7200.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -I../STM32469I-Discovery -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/exc7200/exc7200.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

