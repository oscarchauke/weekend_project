/*
 * DHT11.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Oscar
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "common.h"


void DHT11_init(TIM_HandleTypeDef *htim);


void DHT11_Read(SensorDataPoint *temp, SensorDataPoint *real_hum);


#endif /* INC_DHT11_H_ */
