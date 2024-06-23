#include <Arduino.h>
#include "freertos/FreeRTOS.h"

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "comms.h"

uint8_t data[2];
comms_packet_t packet;
int counter = 0;
void taskComms(void *pvParameters);

const char *ssid = "@Ruijie-s7B73_iot";
const char *password = "MyWifi121";



void setupWiFi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void getTimeFromNTP()
{
  setupWiFi();
  WiFiUDP udp;
  NTPClient timeClient(udp, "pool.ntp.org");
  timeClient.begin();
  timeClient.update();
  Serial.println(timeClient.getFormattedTime());
  Serial.println(timeClient.getEpochTime());
  timeClient.end();      // End NTP client after getting time
  WiFi.disconnect(true); // Disconnect WiFi
}
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

}

void loop()
{
   for (int i = 1; i < 4; i++)
  {
     data[0] = i;
     data[1] = i * packet.crc;
     comms_create_packet(&packet, SENSOR_DATA_PACKET, 2, data);
     comms_send_packet(&packet);
     vTaskDelay(1000);
   }

  if (!comms_buffer_is_empty())
  {
    counter++;
    comms_packet_t packet = comms_buffer_read();
    // Assume is time request
    getTimeFromNTP();
    Serial.printf("%d ID: %d, Length: %d, Data [%d, %d], CRC: %d\n", counter, packet.identifier, packet.length, packet.data[0], packet.data[1], packet.crc);
  }
}

void taskComms(void *pvParameters)
{
  for (;;)
  {
    comms_state_machine();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
