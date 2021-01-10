################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Container.cpp \
../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ListLayout.cpp \
../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ModalWindow.cpp \
../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ScrollableContainer.cpp \
../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SlideMenu.cpp \
../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Slider.cpp \
../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SwipeContainer.cpp \
../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ZoomAnimationImage.cpp 

OBJS += \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Container.o \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ListLayout.o \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ModalWindow.o \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ScrollableContainer.o \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SlideMenu.o \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Slider.o \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SwipeContainer.o \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ZoomAnimationImage.o 

CPP_DEPS += \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Container.d \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ListLayout.d \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ModalWindow.d \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ScrollableContainer.d \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SlideMenu.d \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Slider.d \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SwipeContainer.d \
./Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ZoomAnimationImage.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Container.o: ../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Container.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Container.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ListLayout.o: ../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ListLayout.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ListLayout.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ModalWindow.o: ../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ModalWindow.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ModalWindow.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ScrollableContainer.o: ../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ScrollableContainer.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ScrollableContainer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SlideMenu.o: ../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SlideMenu.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SlideMenu.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Slider.o: ../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Slider.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/Slider.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SwipeContainer.o: ../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SwipeContainer.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/SwipeContainer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ZoomAnimationImage.o: ../Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ZoomAnimationImage.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../TouchGFX/gui/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IDrivers/BSP/Components/ts3510 -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/images/include -IDrivers/BSP/Components/ampire480272 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/TouchGFX/touchgfx/framework/include -ITouchGFX/gui/include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/BSP/Components/exc7200 -I../Drivers/BSP/Components/stmpe811 -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IMiddlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -ITouchGFX/generated/texts/include -I../Drivers/BSP/Components/otm8009a -I../Drivers/BSP/Components/Common -ITouchGFX/generated/fonts/include -I../Middlewares/Third_Party/FatFs/src -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/otm8009a -IInc -I../TouchGFX/generated/fonts/include -I../Drivers/BSP/Components/ts3510 -ITouchGFX/generated/gui_generated/include -ISrc -IDrivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/BSP/Components/stmpe811 -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IDrivers/BSP/Components/Common -I../TouchGFX/generated/gui_generated/include -IMiddlewares/ST/TouchGFX/touchgfx/framework/include -IDrivers/STM32F4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -ITouchGFX/generated/images/include -IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IDrivers/BSP/Components/exc7200 -I../Inc -I../Drivers/BSP/Components/ampire480272 -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/TouchGFX/touchgfx/framework/source/touchgfx/containers/ZoomAnimationImage.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

