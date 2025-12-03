#include "global.h"

#include "led_blinky.h"
#include "neo_blinky.h"
#include "temp_humi_monitor.h"
// #include "mainserver.h"
// #include "tinyml.h"
#include "coreiot.h"

// include task
#include "task_check_info.h"
#include "task_toogle_boot.h"
#include "task_wifi.h"
#include "task_webserver.h"
#include "task_core_iot.h"

QueueHandle_t xQueueRelayControl = NULL;
QueueHandle_t xQueueSettings = NULL;

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("ESP32 S3 RTOS Project - Starting");
  Serial.println("========================================");

  initSharedData();
  xQueueRelayControl = xQueueCreate(10, sizeof(DeviceControlCommand));
  if (xQueueRelayControl == NULL)
  {
    Serial.println("[ERROR] Failed to create Relay Queue!");
  }

  xQueueSettings = xQueueCreate(2, sizeof(SettingsCommand));
  if (xQueueSettings == NULL)
  {
    Serial.println("[ERROR] Failed to create Settings Queue!");
  }

  delay(100);

  check_info_File(0);

  Serial.println("[INIT] Creating RTOS tasks...");

  xTaskCreate(Device_Control_Task, "DeviceCtrl", 8192, NULL, 3, NULL);

  // Task 1: LED Blink with Temperature Control
  xTaskCreate(led_blinky,
              "LED_Temp",
              8192,
              NULL,
              2,
              NULL);
  Serial.println("[INIT] - LED Temperature Control Task created");

  // Task 2: NeoPixel Control Based on Humidity
  xTaskCreate(neo_blinky,
              "Neo_Humidity",
              4096,
              NULL,
              2,
              NULL);
  Serial.println("[INIT] - NeoPixel Humidity Control Task created");

  // Task 3a: Temperature and Humidity Sensor Reading
  xTaskCreate(temp_humi_monitor,
              "Sensor_Mon",
              4096,
              NULL,
              4,
              NULL);
  Serial.println("[INIT] - Sensor Monitor Task created");

  // Task 3b: LCD Display with State Management
  xTaskCreate(lcd_display_task,
              "LCD_Display",
              4096,
              NULL,
              2,
              NULL);
  Serial.println("[INIT] - LCD Display Task created");

  // TASK 4: Web Server (WebSocket, OTA)
  xTaskCreate(Webserver_RTOS_Task, "Webserver_Task", 10240, NULL, 3, NULL);

  xTaskCreate(coreiot_task,
              "CoreIOT_Task",
              8192,
              NULL,
              1,
              NULL);

  xTaskCreate(Task_Toogle_BOOT,
              "Task_BOOT",
              2048,
              NULL,
              5,
              NULL);

  Serial.println("[INIT] - CoreIOT Task created");

  Serial.println("========================================");
  Serial.println("All tasks created successfully!");
  Serial.println("System running...");
  Serial.println("========================================\n");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });

  server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/styles.css", "text/css"); });

  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/script.js", "application/javascript"); });

  server.serveStatic("/", LittleFS, "/");
  server.begin();
}

void loop()
{
  if (check_info_File(1))
  {
    if (!Wifi_reconnect())
    {
      Webserver_stop();
    }
    else
    {
      // CORE_IOT_reconnect();
    }
  }
  Webserver_reconnect();

  delay(100);
}
