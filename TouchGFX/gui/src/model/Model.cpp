#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

uint16_t value = 0;

extern "C"
{
	xQueueHandle messageQ;
}
Model::Model() :
modelListener(0)
{
	messageQ = xQueueGenericCreate(1,3,1);
}

void Model::tick()
{
	if(xQueueReceive(messageQ, &value, 0) == pdTRUE){
		modelListener->setNewValue(value);
	}
}
