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

/**
 * ESP32 S3 RTOS Project - Enhanced with Semaphore-based Task Synchronization
 *
 * This project implements three main tasks:
 * 1. LED Blink Task: Temperature-based LED control (3 states)
 * 2. NeoPixel Task: Humidity-based color control (4 colors)
 * 3. Sensor & LCD Task: Temperature/Humidity monitoring with LCD display (3 display states)
 *
 * Key Features:
 * - All tasks synchronized using FreeRTOS semaphores
 * - No global variables for sensor data (all data shared via mutex-protected structures)
 * - Real-time sensor monitoring with DHT20
 * - Visual feedback through LED, NeoPixel, and LCD
 */
QueueHandle_t xQueueRelayControl = NULL;
QueueHandle_t xQueueSettings = NULL;

void setup()
{
  Serial.begin(115200);

  // Wait for serial to initialize
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("ESP32 S3 RTOS Project - Starting");
  Serial.println("========================================");

  // CRITICAL: Initialize shared data structures BEFORE anything else
  initSharedData();
  xQueueRelayControl = xQueueCreate(10, sizeof(DeviceControlCommand));
  if (xQueueRelayControl == NULL)
  {
    Serial.println("[ERROR] Failed to create Relay Queue!");
  }

  // Khởi tạo Queue cho Task 6 (Settings)
  xQueueSettings = xQueueCreate(2, sizeof(SettingsCommand));
  if (xQueueSettings == NULL)
  {
    Serial.println("[ERROR] Failed to create Settings Queue!");
  }

  // Small delay to ensure semaphores are ready
  delay(100);

  // Check configuration file
  check_info_File(0);

  // Create RTOS tasks with priorities
  Serial.println("[INIT] Creating RTOS tasks...");

  xTaskCreate(Device_Control_Task, "DeviceCtrl", 8192, NULL, 3, NULL);
  // Task 1: LED Blink with Temperature Control
  xTaskCreate(led_blinky,
              "LED_Temp",
              8192, // Increased stack for temperature logic
              NULL,
              2, // Priority 2
              NULL);
  Serial.println("[INIT] - LED Temperature Control Task created");

  // Task 2: NeoPixel Control Based on Humidity
  xTaskCreate(neo_blinky,
              "Neo_Humidity",
              4096, // Increased stack size to 4KB (NeoPixel needs more stack)
              NULL,
              2, // Priority 2
              NULL);
  Serial.println("[INIT] - NeoPixel Humidity Control Task created");

  // Task 3a: Temperature and Humidity Sensor Reading
  xTaskCreate(temp_humi_monitor,
              "Sensor_Mon",
              4096, // Increased stack size to 4KB
              NULL,
              4, // Priority 3 (higher priority for sensor reading)
              NULL);
  Serial.println("[INIT] - Sensor Monitor Task created");

  // Task 3b: LCD Display with State Management
  xTaskCreate(lcd_display_task,
              "LCD_Display",
              4096, // Increased stack size to 4KB
              NULL,
              2, // Priority 2
              NULL);
  Serial.println("[INIT] - LCD Display Task created");

  // TASK 4: Web Server (WebSocket, OTA)
  xTaskCreate(Webserver_RTOS_Task, "Webserver_Task", 10240, NULL, 3, NULL);

  // Optional: CoreIOT task for cloud connectivity
  xTaskCreate(coreiot_task,
              "CoreIOT_Task",
              8192,
              NULL,
              1, // Priority 1 (lower priority)
              NULL);
  Serial.println("[INIT] - CoreIOT Task created");

  Serial.println("========================================");
  Serial.println("All tasks created successfully!");
  Serial.println("System running...");
  Serial.println("========================================\n");
}

void loop()
{
  // Main loop handles WiFi and webserver reconnection
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

  // Small delay to prevent watchdog trigger
  delay(100);
}