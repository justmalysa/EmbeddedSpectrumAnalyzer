################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/generated/gui_generated/src/screen1_screen/Screen1ViewBase.cpp 

OBJS += \
./TouchGFX/generated/gui_generated/src/screen1_screen/Screen1ViewBase.o 

CPP_DEPS += \
./TouchGFX/generated/gui_generated/src/screen1_screen/Screen1ViewBase.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/generated/gui_generated/src/screen1_screen/Screen1ViewBase.o: ../TouchGFX/generated/gui_generated/src/screen1_screen/Screen1ViewBase.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"TouchGFX/generated/gui_generated/src/screen1_screen/Screen1ViewBase.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

