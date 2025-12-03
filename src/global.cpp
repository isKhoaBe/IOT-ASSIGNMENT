#include "global.h"
#include <vector>

static SensorData_t sensorDataInstance;
static WifiConfig_t wifiConfigInstance;

SensorData_t *g_sensorData = NULL;
WifiConfig_t *g_wifiConfig = NULL;

std::vector<int> g_userPins;

void initSharedData()
{
    g_sensorData = &sensorDataInstance;
    g_sensorData->temperature = 25.0; // Default safe value
    g_sensorData->humidity = 50.0;    // Default safe value
    g_sensorData->mutex = xSemaphoreCreateMutex();

    if (g_sensorData->mutex == NULL)
    {
        Serial.println("[ERROR] Failed to create sensor data mutex!");
    }

    // Initialize WiFi config structure
    g_wifiConfig = &wifiConfigInstance;
    g_wifiConfig->WIFI_SSID = "";
    g_wifiConfig->WIFI_PASS = "";
    g_wifiConfig->CORE_IOT_TOKEN = "";
    g_wifiConfig->CORE_IOT_SERVER = "";
    g_wifiConfig->CORE_IOT_PORT = "";
    g_wifiConfig->isWifiConnected = false;
    g_wifiConfig->webserver_isrunning = false; // Initialize webserver state
    g_wifiConfig->xBinarySemaphoreInternet = xSemaphoreCreateBinary();
    g_wifiConfig->mutex = xSemaphoreCreateMutex();

    g_wifiConfig->led1Override = false;
    g_wifiConfig->neoOverride = false;
    g_wifiConfig->relayOverride = false;

    if (g_wifiConfig->mutex == NULL || g_wifiConfig->xBinarySemaphoreInternet == NULL)
    {
        Serial.println("[ERROR] Failed to create WiFi config semaphores!");
    }

    Serial.println("[INIT] Shared data structures initialized successfully");
}

// Set sensor data with mutex protection
void setSensorData(float temp, float humi)
{
    if (g_sensorData != NULL && g_sensorData->mutex != NULL)
    {
        if (xSemaphoreTake(g_sensorData->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_sensorData->temperature = temp;
            g_sensorData->humidity = humi;
            xSemaphoreGive(g_sensorData->mutex);
        }
        else
        {
            Serial.println("[WARN] Failed to acquire sensor data mutex for writing");
        }
    }
    else
    {
        Serial.println("[ERROR] Sensor data structure not initialized!");
    }
}

// Get sensor data with mutex protection
void getSensorData(float *temp, float *humi)
{
    if (g_sensorData != NULL && g_sensorData->mutex != NULL)
    {
        if (xSemaphoreTake(g_sensorData->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            *temp = g_sensorData->temperature;
            *humi = g_sensorData->humidity;
            xSemaphoreGive(g_sensorData->mutex);
        }
        else
        {
            Serial.println("[WARN] Failed to acquire sensor data mutex for reading");
            // Return default values
            *temp = 25.0;
            *humi = 50.0;
        }
    }
    else
    {
        Serial.println("[ERROR] Sensor data structure not initialized!");
        *temp = 25.0;
        *humi = 50.0;
    }
}