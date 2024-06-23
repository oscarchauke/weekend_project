/*
 * comms.h
 *
 *  Created on: Apr 27, 2024
 *      Author: Oscar
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_

#include "stm32f0xx_hal.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define PACKET_DATA_MAX_LENGTH				(32)

typedef struct comms_packet_t
{
	uint8_t identifier;
	uint8_t length;
    uint8_t data[PACKET_DATA_MAX_LENGTH];
    uint8_t crc;
}comms_packet_t;

typedef struct {
	comms_packet_t *buffer;
    uint8_t size;
    uint8_t head; // Index to write to
    uint8_t tail; // Index to read from
} comms_packet_buffer;

typedef enum comms_state_t
{
    COMMS_ID_STATE,
	COMMS_LENGTH_STATE,
    COMMS_DATA_STATE,
    COMMS_CRC_STATE
} comms_state_t;

enum PACKET_IDENTIFIERS{
	CONSOLE_LOG,
	COMMS_PACKET,
	HARDWARE_COMMAND_PACKET,
	SENSOR_DATA_PACKET,
};

// TODO: is this a good idea?
enum COMMS_MESSAGES{
	ACK,
	NOT_ACK,
	RETRANSMIT_REQUEST,
	BUFFER_FULL,
	BUFFER_EMPTY
};

typedef struct comms_t{
	UART_HandleTypeDef *huart;
	comms_packet_buffer *buffer;
	comms_state_t state;
}comms_t;

void comms_init(UART_HandleTypeDef *huart);
void comms_state_machine();
void comms_send_packet(comms_packet_t *packet);

comms_packet_t comms_buffer_read();
uint8_t comms_buffer_is_full();
uint8_t comms_buffer_is_empty();

void console_log(char *message);

#endif /* INC_COMMS_H_ */
