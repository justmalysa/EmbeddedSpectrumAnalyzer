#include <gui/screen1_screen/Screen1View.hpp>

#ifndef SIMULATOR

extern "C"{
#include "string.h"
#include <stm32f4xx_hal.h>
#include <arm_math.h>
#include "arm_math.h"
#include "arm_const_structs.h"
}
#endif

#define CANVAS_BUFFER_SIZE 8192
#define FFT_SIZE 50

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

extern arm_rfft_fast_instance_f32 app_arm_rfft_fast_sR_f32_len512;

float32_t data_in[FFT_SIZE] = {0};
float32_t data_out[FFT_SIZE] = {0};
float32_t data_mag_out[FFT_SIZE] = {0};

Screen1View::Screen1View() {

}

void Screen1View::setupScreen() {
	Screen1ViewBase::setupScreen();
	float32_t max_spectrum = 0;
	size_t size = FFT_SIZE;
	int f = 1;
	int fs = 10;

    /* Generation sine function */
	generate_sine(data_in, size, f, fs);

    static uint8_t canvasBuffer[CANVAS_BUFFER_SIZE];
    CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);

    // Place the graph on the screen
    graph.setXY(100, 0);

    // Set the outer dimensions and color of the graph
    graph.setup(700, 240, Color::getColorFrom24BitRGB(0x00, 0xFF, 0xFF));

    // Set the range for the x and y axis of the graph. That is
    // the max and min x/y value that can be displayed inside the
    // dimension of the graph.
    graph.setRange(0, 700, 0, 240);

    // Set the line width in pixels
    graph.setLineWidth(5);

    add(graph);

    for(size_t i = 0; i<size; i++)
    {
    	graph.addValue(i*10, (data_in[i] + 1.0f)*110.0f + 10.0f);
    }

    /*** SPECTRUM ***/
	/* Process the data through the RFFT/RIFFT module */
	arm_rfft_fast_f32(&app_arm_rfft_fast_sR_f32_len512, data_in, data_out, 0);

	/* Process the data through the Complex Magnitude Module for
	  calculating the magnitude at each bin */
	arm_cmplx_mag_f32(data_out+2, data_mag_out+1, size/2 - 1);
	/* Handle special cases */
	data_mag_out[0] = fabs(data_out[0]);
	data_mag_out[size/2] = fabs(data_out[1]);

    // Place the graph on the screen
    spectrum.setXY(100, 120);

        // Set the outer dimensions and color of the graph
    spectrum.setup(700, 240, Color::getColorFrom24BitRGB(0x00, 0xFF, 0xFF));

        // Set the range for the x and y axis of the graph. That is
        // the max and min x/y value that can be displayed inside the
        // dimension of the graph.
    spectrum.setRange(0, 700, 0, 240);

        // Set the line width in pixels
    spectrum.setLineWidth(5);

    add(spectrum);

    for(size_t i = 0; i<size; i++)
    {
        if(data_mag_out[i] > max_spectrum)
        {
        	max_spectrum = data_mag_out[i];
        }
    }

    for(size_t i = 0; i<size; i++)
    {
        spectrum.addValue(i*10, (data_mag_out[i]/max_spectrum)*110.0f);
    }
}

void Screen1View::tearDownScreen() {
	Screen1ViewBase::tearDownScreen();
}
