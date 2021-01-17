#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

uint16_t * adc_buffer_ptr;

extern "C"
{
	xQueueHandle messageQ;
}
Model::Model() :
modelListener(0)
{
	messageQ = xQueueGenericCreate(1,4,1);
}

void Model::tick()
{
	if(xQueueReceive(messageQ, &adc_buffer_ptr, 0) == pdTRUE){
		modelListener->setNewValue(adc_buffer_ptr);
	}
}
