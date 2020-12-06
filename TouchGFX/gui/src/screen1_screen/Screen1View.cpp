#include <gui/screen1_screen/Screen1View.hpp>

#ifndef SIMULATOR

extern "C"{
#include "string.h"
#include <stm32f4xx_hal.h>
}
#endif

Screen1View::Screen1View() {

}

void Screen1View::setupScreen() {
	Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen() {
	Screen1ViewBase::tearDownScreen();
}

void Screen1View::LED1_button_clicked()
{
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
}
