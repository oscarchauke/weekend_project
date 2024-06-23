/*
 * comms.c
 *
 *  Created on: Apr 27, 2024
 *      Author: Oscar
 */

#include "comms.h"

#include <string.h>

#define COMMS_BUFFER_CAPACITY		(10)

#define MODULE_NAME					"COMMS"

static comms_t comms;

comms_packet_t temporary_packet;
comms_packet_t previous_packet;

static comms_packet_t retransmit_packet = {
		.identifier = 0,
		.length = 1,
		.data = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
		.crc = 0x12
};

static comms_packet_t acknowledge_packet = {
		.identifier=0,
		.length = 1,
		.data = {0},
		.crc = 0x15
};

uint8_t is_ack_packet(comms_packet_t *packet){
	if(packet->identifier != 0){
		return 0;
	}

	if(packet->length != 1){
		return 0;
	}

	if(packet->data[0] != 0){
		return 0;
	}

	return 1;
}
// TODO: Find a way to test if the buffer is working correctly
static void comms_buffer_write(comms_packet_t packet);

uint8_t crc8(uint8_t *data, size_t len);

void comms_init(UART_HandleTypeDef *huart1){
	comms.huart = huart1;
	comms.state = COMMS_ID_STATE;

	// Create comms buffer
    comms_packet_buffer* rb = (comms_packet_buffer*)malloc(sizeof(comms_packet_buffer));
    rb->buffer = (comms_packet_t*)malloc(COMMS_BUFFER_CAPACITY * sizeof(comms_packet_t));
    rb->size = 0;
    rb->head = 0;
    rb->tail = 0;
    comms.buffer = rb;
}

// TODO: UART receive error handing
void comms_state_machine(){
	switch (comms.state) {
		case COMMS_ID_STATE:
			if(HAL_UART_Receive(comms.huart, &(temporary_packet.identifier), 1, 1000) == HAL_OK){
				comms.state = COMMS_LENGTH_STATE;
			}
			break;

		case COMMS_LENGTH_STATE:
			if(HAL_UART_Receive(comms.huart, &(temporary_packet.length), 1, 1000) == HAL_OK){
				comms.state = COMMS_DATA_STATE;
			}
			break;

		case COMMS_DATA_STATE:
			if(HAL_UART_Receive(comms.huart, (temporary_packet.data), (temporary_packet.length), 1000)== HAL_OK){
				comms.state = COMMS_CRC_STATE;
			}
			break;

		case COMMS_CRC_STATE:
			if(HAL_UART_Receive(comms.huart, &(temporary_packet.crc), 1, 1000) == HAL_OK){
				// Check CRC8
				uint8_t crc = crc8((uint8_t*)&temporary_packet, (temporary_packet.length + 2));	// Add 2 to the data length to accommodate the identifier byte and length byte
				if(crc == temporary_packet.crc){
					// packet valid

					if(!is_ack_packet(&temporary_packet)){
						comms_buffer_write(temporary_packet);
						comms_send_packet(&acknowledge_packet);
					}

				}else{
					comms_send_packet(&retransmit_packet);
				}
				comms.state = COMMS_ID_STATE;
			}
			break;
		default:
			break;
	}
}

void comms_send_packet(comms_packet_t *packet) {
    uint8_t i;
    HAL_UART_Transmit(comms.huart, &(packet->identifier), 1, 100);
    HAL_UART_Transmit(comms.huart, &(packet->length), 1, 100);
    for (i = 0; i < packet->length; ++i) {
        HAL_UART_Transmit(comms.huart, &(packet->data[i]), 1, 100);
    }
    HAL_UART_Transmit(comms.huart, &(packet->crc), 1, 100);
}

// TODO: Find a better way of doing this or delete once the comms module is fully functional
void console_log(char *message){
	int data_length = strlen(message);
	int i;

	for (i = 0; i < data_length; i += PACKET_DATA_MAX_LENGTH) {
		int chunk_length;
	    if (data_length - i > PACKET_DATA_MAX_LENGTH) {
	    	chunk_length = PACKET_DATA_MAX_LENGTH;
	    } else {
	        chunk_length = data_length - i;
	    }

	    comms_packet_t packet;
	    packet.identifier = CONSOLE_LOG; // Example identifier
	    packet.length = chunk_length;
	    memcpy(packet.data, &message[i], chunk_length);

	    packet.crc = 0;
	    comms_send_packet(&packet);
	    }
	}

static void comms_buffer_write(comms_packet_t packet){
	if(comms.buffer->size == COMMS_BUFFER_CAPACITY){
		// Buffer is full, overwrite the oldest data
		comms.buffer->tail = (comms.buffer->tail + 1) % COMMS_BUFFER_CAPACITY;
	}else{
		comms.buffer->size++;
	}
	comms.buffer->buffer[comms.buffer->head] = packet;
	comms.buffer->head = (comms.buffer->head + 1) % COMMS_BUFFER_CAPACITY;

	// Update previous packet
	memcpy(&previous_packet, &temporary_packet, sizeof(comms_packet_t));
}

comms_packet_t comms_buffer_read(){
    comms_packet_t packet = {0}; // Empty packet
    if (comms.buffer->size == 0) {
    	// TODO: Think of a way to handle this
        return packet; // Buffer is empty
    }

    packet = comms.buffer->buffer[comms.buffer->tail];
    comms.buffer->tail = (comms.buffer->tail + 1) % COMMS_BUFFER_CAPACITY;
    comms.buffer->size++;
    return packet;
}

uint8_t comms_buffer_is_full(){
	return comms.buffer->size == COMMS_BUFFER_CAPACITY;
}

uint8_t comms_buffer_is_empty(){
	return comms.buffer->size == 0;
}

uint8_t crc8(uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    size_t i, j;

    for (i = 0; i < len; ++i) {
        crc ^= data[i];
        for (j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
