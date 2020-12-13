#include <gui/screen1_screen/Screen1View.hpp>

#ifndef SIMULATOR

extern "C"{
#include "string.h"
#include <stm32f4xx_hal.h>
#include <arm_math.h>
}
#endif

#define CANVAS_BUFFER_SIZE 8192

class Utils
{
public:
    static int randomNumberBetween(int lowest, int highest)
    {
#ifdef SIMULATOR
        return lowest + (highest - lowest) * rand() / RAND_MAX;
#else
        uint32_t random = (touchgfx::HAL::getInstance()->getCPUCycles() * HAL::getInstance()->getCPUCycles());
        return lowest + (random % (highest - lowest));
#endif
    }

};

static void generate_sine(float32_t *data, size_t size, int f, int fs)
{
	float Ts = 1/((float)fs);
	for(size_t i = 0; i<size; i++)
	{
		data[i] = arm_sin_f32(2*3.14*f*Ts*i);
	}
}

Screen1View::Screen1View() {

}

void Screen1View::setupScreen() {
	Screen1ViewBase::setupScreen();
	size_t size = 50;
	float32_t data_in[size] = {0};
	//float32_t data_out[size] = {0};
	//float32_t data_mag_out[size] = {0};
	int f = 1;
	int fs = 10;
	generate_sine(data_in, size, f, fs);

    tickCounter = 0;

    static uint8_t canvasBuffer[CANVAS_BUFFER_SIZE];
    CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);

    // Place the graph on the screen
    graph.setXY(0, 0);

    // Set the outer dimensions and color of the graph
    graph.setup(800, 240, Color::getColorFrom24BitRGB(0xFF, 0xFF, 0xAC));

    // Set the range for the x and y axis of the graph. That is
    // the max and min x/y value that can be displayed inside the
    // dimension of the graph.
    graph.setRange(0, 800, 0, 240);

    // Set the line width in pixels
    graph.setLineWidth(10);

    add(graph);

    for(size_t i = 0; i<size; i++)
    {
    	graph.addValue(i*10, (data_in[i] + 1.0f)*120.0f);
    }
}

void Screen1View::tearDownScreen() {
	Screen1ViewBase::tearDownScreen();
}

void Screen1View::handleTickEvent()
{
    // Number of ticks between inserting a point in the graph
    /*int interval = 5;

    if (tickCounter % interval == 0)
    {
        // Insert a point in the graph.
        // The Y value is a random number in the y range of the graph.
        graph.addValue(tickCounter / interval, Utils::randomNumberBetween(graph.getRangeBottom(), graph.getRangeTop()));
    }

    if (tickCounter == 13 * interval)
    {
        // Reduce the line width
        graph.setLineWidth(graph.getLineWidth<float>() - 0.6f);
        graph.invalidate();
    }

    if (tickCounter == 25 * interval)
    {
        // Delete some points
        graph.deleteValue(2);
        graph.deleteValue(4);
        graph.deleteValue(5);
    }

    if (tickCounter == 37 * interval)
    {
        // Change the range of the Y axis
        graph.setRange(0, 50, 0, 400);
        graph.invalidate();
    }

    tickCounter++;


    if (tickCounter == 50 * interval)
    {
        // Reset the graph and start over
        graph.setRange(0, 50, 0, 200);
        graph.setLineWidth(2);
        graph.clear();
        graph.invalidate();

        tickCounter = 0;
    }*/
}

void Screen1View::LED1_button_clicked()
{
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
}
