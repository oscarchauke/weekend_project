/*
 * common.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Oscar
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "stm32f0xx_hal.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "comms.h"

typedef struct{
	uint32_t timestamp;
	float value;
	char sensorID[5];
}SensorDataPoint;


#endif /* INC_COMMON_H_ */
