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
#define ADC_BUFFER_SIZE    512
#define GRAPH_MAX_PTS      64

#define GRAPH_WIDTH_START_PX  100
#define GRAPH_WIDTH_END_PX    700
#define GRAPH_WIDTH_PX        (GRAPH_WIDTH_END_PX - GRAPH_WIDTH_START_PX)

#define GRAPH_HEIGHT_START_PX 0
#define GRAPH_HEIGHT_END_PX   240
#define GRAPH_HEIGHT_PX       (GRAPH_HEIGHT_END_PX - GRAPH_HEIGHT_START_PX)

#define X_AXIS_SPACING_PX     (GRAPH_WIDTH_PX / GRAPH_MAX_PTS)

#define Y_AXIS_OFFSET_PX      10
#define Y_AXIS_SIZE_PX        (GRAPH_HEIGHT_PX - (Y_AXIS_OFFSET_PX * 2))
#define Y_AXIS_SIZE_FFT_PX    200

#define SIGNAL_MAX_V		  (3.3f)
#define SIGNAL_MAX_LSB		  (4095)

#define SIGNAL_LSB_TO_V(x)    ((x) * (SIGNAL_MAX_V / ((float)SIGNAL_MAX_LSB)))
#define SIGNAL_LSB_TO_PX(x)   ((((x) * Y_AXIS_SIZE_PX) / SIGNAL_MAX_LSB) + Y_AXIS_OFFSET_PX)

#define FFT_X_AXIS_SPACING    ((ADC_BUFFER_SIZE / 2) / GRAPH_MAX_PTS)

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


extern arm_rfft_fast_instance_f32 app_arm_rfft_fast_sR_f32_len512;
extern TIM_HandleTypeDef htim2;

float32_t data_in[ADC_BUFFER_SIZE] = {0};
float32_t data_out[ADC_BUFFER_SIZE] = {0};
float32_t data_mag_out[ADC_BUFFER_SIZE] = {0};

Screen1View::Screen1View() {

}

void Screen1View::setupScreen() {
	Screen1ViewBase::setupScreen();

    static uint8_t canvasBuffer[CANVAS_BUFFER_SIZE];
    CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);

    // Place the graph on the screen
    graph.setXY(GRAPH_WIDTH_START_PX, 0);

    // Set the outer dimensions and color of the graph
    graph.setup(GRAPH_WIDTH_PX, GRAPH_HEIGHT_PX, Color::getColorFrom24BitRGB(0x00, 0xFF, 0xFF));

    // Set the range for the x and y axis of the graph. That is
    // the max and min x/y value that can be displayed inside the
    // dimension of the graph.
    graph.setRange(0, GRAPH_WIDTH_PX, 0, GRAPH_HEIGHT_PX);

    // Set the line width in pixels
    graph.setLineWidth(5);

    add(graph);

    // Place the graph on the screen
    spectrum.setXY(GRAPH_WIDTH_START_PX, Y_AXIS_SIZE_FFT_PX);

    // Set the outer dimensions and color of the graph
    spectrum.setup(GRAPH_WIDTH_PX, GRAPH_HEIGHT_PX, Color::getColorFrom24BitRGB(0x00, 0xFF, 0xFF));

    // Set the range for the x and y axis of the graph. That is
    // the max and min x/y value that can be displayed inside the
    // dimension of the graph.
    spectrum.setRange(0, GRAPH_WIDTH_PX, 0, GRAPH_HEIGHT_PX);

    // Set the line width in pixels
    spectrum.setLineWidth(5);

    add(spectrum);

    /* start ADC sampling timer */
    HAL_TIM_Base_Start_IT(&htim2);
}

void Screen1View::tearDownScreen() {
	Screen1ViewBase::tearDownScreen();
}

void Screen1View::updateVal(uint16_t * adc_buffer_ptr)
{
	float32_t max_spectrum = 0;

	/* convert ADC value to voltage in float format */
	for(size_t i=0; i < ADC_BUFFER_SIZE; i++)
	{
		data_in[i] = SIGNAL_LSB_TO_V((float32_t)adc_buffer_ptr[i]);
	}

	/* display signal in time domain */
	for(size_t i = 0; i < GRAPH_MAX_PTS; i++)
    {
        uint32_t x = i * X_AXIS_SPACING_PX;
        graph.deleteValue(x);
        graph.addValue(x, SIGNAL_LSB_TO_PX((uint32_t)adc_buffer_ptr[i]));
    }

	/* Process the data through the RFFT/RIFFT module */
	arm_rfft_fast_f32(&app_arm_rfft_fast_sR_f32_len512, data_in, data_out, 0);

	/* Process the data through the Complex Magnitude Module for
	  calculating the magnitude at each bin */
	arm_cmplx_mag_f32(data_out + 2, data_mag_out + 1, ADC_BUFFER_SIZE/2 - 1);
	/* Handle special cases */
	data_mag_out[0] = (fabs(data_out[0])) / ADC_BUFFER_SIZE;
	data_mag_out[ADC_BUFFER_SIZE / 2] = fabs(data_out[1]);

	/* Display spectrum graph*/
    for(size_t i = 0; i < ADC_BUFFER_SIZE; i++)
    {
        if(data_mag_out[i] > max_spectrum)
        {
        	max_spectrum = data_mag_out[i];
        }
    }

    float32_t scale = ((float)Y_AXIS_SIZE_FFT_PX) / max_spectrum;
    for(size_t i = 0; i < GRAPH_MAX_PTS; i++)
    {
        uint32_t sample_idx = i * FFT_X_AXIS_SPACING;
        uint32_t x = i * X_AXIS_SPACING_PX;
        spectrum.deleteValue(x);
        spectrum.addValue(x, data_mag_out[sample_idx] * scale);
    }
}
