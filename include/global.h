#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <vector>

// Shared data structure for sensor readings
typedef struct
{
    float temperature;
    float humidity;
    SemaphoreHandle_t mutex; // Mutex for protecting shared data
} SensorData_t;

// Shared data structure for WiFi configuration
typedef struct
{
    String WIFI_SSID;
    String WIFI_PASS;
    String CORE_IOT_TOKEN;
    String CORE_IOT_SERVER;
    String CORE_IOT_PORT;
    boolean isWifiConnected;
    boolean webserver_isrunning; // Webserver state

    boolean led1Override;
    boolean neoOverride;
    boolean relayOverride;

    SemaphoreHandle_t xBinarySemaphoreInternet;
    SemaphoreHandle_t mutex; // Mutex for protecting WiFi data
} WifiConfig_t;

// Global pointer to shared sensor data (initialized in setup)
extern SensorData_t *g_sensorData;
extern WifiConfig_t *g_wifiConfig;
extern std::vector<int> g_userPins;

// Helper functions to safely access shared data
void initSharedData();
void setSensorData(float temp, float humi);
void getSensorData(float *temp, float *humi);

#endif