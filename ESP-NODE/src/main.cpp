#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "comms.h"

uint8_t data[2];
comms_packet_t packet;

void taskComms(void *pvParameters);

void setup()
{
  Serial2.begin(115200);
  Serial.begin(115200);
  comms_init();
  Serial.println("Done init comms");

  xTaskCreate(
      taskComms,   // Function that implements the task
      "TaskComms", // Text name for the task (optional)
      4096,        // Stack size (words, not bytes)
      NULL,        // Task parameters
      1,           // Priority (1 is default priority)
      NULL         // Task handle (optional)
  );

	static comms_packet_t retransmit_packet = {
			.identifier = 2,
			.length = 5,
			.data = {1,3,0,0,4,0,0,0,0,0,0,0,0,0,0,0},
			.crc = 0x12
	};
	comms_send_packet(&retransmit_packet);
}

void loop()
{
  for (int i = 1; i < 4; i++)
  {
    data[0] = i;
    data[1] = i*packet.crc;
    comms_create_packet(&packet, SENSOR_DATA_PACKET, 2, data);
    delay(5000);
  }
}

void taskComms(void *pvParameters)
{
  for (;;)
  {
    // Task code here
    comms_state_machine();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
