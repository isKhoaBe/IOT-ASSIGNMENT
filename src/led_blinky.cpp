#include "led_blinky.h"
#include "task_webserver.h"
#include "neo_blinky.h"
#include <ArduinoJson.h>
Adafruit_NeoPixel strip(LED_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

/**
 * Task 1: Single LED Blink with Temperature Conditions
 *
 * This task implements three different LED blinking behaviors based on temperature:
 * 1. HOT (> 29°C): Fast blink - 200ms ON, 200ms OFF
 * 2. NORMAL (22-29°C): Normal blink - 1s ON, 1s OFF
 * 3. COLD (< 22°C): Slow ON, fast OFF - 2s ON, 200ms OFF
 *
 * Uses semaphore-protected shared data access to read temperature values
 */

bool isValidOutputPin(int gpio) {
    const int forbidden[] = {0,2,3,19,20,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,46}; 
    for (int f : forbidden) if (gpio == f) return false;

    if (gpio < 0 || gpio > 49) return false;
    return true;
}

void led_blinky(void *pvParameters)
{
    // Wait for shared data structures to be ready
    vTaskDelay(pdMS_TO_TICKS(1000));

    pinMode(LED_GPIO, OUTPUT);

    // Local variables to store sensor readings
    float temperature = 0.0;
    float humidity = 0.0;

    // LED timing variables
    uint16_t onTime = LED_NORMAL_ON_TIME;
    uint16_t offTime = LED_NORMAL_OFF_TIME;

    Serial.println("[LED_BLINK] Task started - Temperature-based LED control");

    while (1)
    {
        bool override = false;
        if (xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            override = g_wifiConfig->led1Override;
            xSemaphoreGive(g_wifiConfig->mutex);
        }

        if (override)
        {
            Serial.println("[LED_BLINK] Override active - Web control taking over");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        // Read sensor data using semaphore-protected function
        getSensorData(&temperature, &humidity);

        // Determine LED blink pattern based on temperature
        if (temperature < TEMP_COLD_THRESHOLD)
        {
            // COLD state: Slow ON, fast OFF
            onTime = LED_COLD_ON_TIME;
            offTime = LED_COLD_OFF_TIME;
            Serial.println("[LED_BLINK] COLD - Slow ON, Fast OFF");
        }
        else if (temperature >= TEMP_COLD_THRESHOLD && temperature <= TEMP_NORMAL_THRESHOLD)
        {
            // NORMAL state: 1s ON, 1s OFF
            onTime = LED_NORMAL_ON_TIME;
            offTime = LED_NORMAL_OFF_TIME;
            Serial.println("[LED_BLINK] NORMAL - 1s ON/OFF");
        }
        else
        {
            // HOT state: Fast blink
            onTime = LED_HOT_ON_TIME;
            offTime = LED_HOT_OFF_TIME;
            Serial.println("[LED_BLINK] HOT - Fast blink");
        }

        // Turn LED ON
        digitalWrite(LED_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(onTime));

        // Turn LED OFF
        digitalWrite(LED_GPIO, LOW);
        vTaskDelay(pdMS_TO_TICKS(offTime));
    }
}

void Device_Control_Task(void *pvParameters)
{
    strip.begin();
    strip.show();
    pinMode(LED_GPIO, OUTPUT);

    DeviceControlCommand cmd;

    while (1)
    {
        if (xQueueReceive(xQueueRelayControl, &cmd, portMAX_DELAY) == pdPASS)
        {
            int pin = cmd.gpioPin;
            bool isWebOn = cmd.newState;

            if (pin == LED_GPIO)
            {
                if (g_wifiConfig != NULL && xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    g_wifiConfig->led1Override = !isWebOn;
                    xSemaphoreGive(g_wifiConfig->mutex);
                }
                if (!isWebOn)
                {
                    digitalWrite(LED_GPIO, LOW); // OFF
                    Serial.println("✅ LED1 (Override) set to OFF");
                }
                else
                {
                    Serial.println("✅ LED1 back to AUTO");
                }
            }

            else if (pin == NEO_PIN)
            {
                if (g_wifiConfig != NULL && xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    g_wifiConfig->neoOverride = !isWebOn;
                    xSemaphoreGive(g_wifiConfig->mutex);
                }

                if (!isWebOn)
                {
                    strip.setPixelColor(0, strip.Color(0, 0, 0)); 
                    strip.show();
                    Serial.println("✅ NeoPixel Force OFF");
                }
                else
                {
                    Serial.println("✅ NeoPixel Back to AUTO");
                }
            }
            else
            {
                if (!isValidOutputPin(pin)) {
                    Serial.printf("⚠️ Invalid GPIO %d received - ignoring\n", pin);
                } else {
                    bool alreadyConfigured = false;
                    for (int p : g_userPins) {
                        if (p == pin) {
                            alreadyConfigured = true; break;
                        }
                    }
                        if (!alreadyConfigured) {
                            pinMode(pin, OUTPUT);
                            digitalWrite(pin, LOW); // default safe state
                            g_userPins.push_back(pin);
                            Serial.printf("ℹ️ Configured GPIO %d as OUTPUT and saved to userPins\n", pin);
                        }
                if (g_wifiConfig != NULL && xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    g_wifiConfig->relayOverride = isWebOn;
                    xSemaphoreGive(g_wifiConfig->mutex);
                }
                digitalWrite(pin, isWebOn ? HIGH : LOW);
                Serial.printf("✅ External Relay (GPIO %d) set to %s\n", pin, isWebOn ? "ON" : "OFF");
            }
        }

            StaticJsonDocument<256> jsonDoc;
            jsonDoc["page"] = "device_update";
            jsonDoc["value"]["gpio"] = pin;
            jsonDoc["value"]["status"] = isWebOn ? "ON" : "OFF";

            String jsonString;
            serializeJson(jsonDoc, jsonString);
            Webserver_sendata(jsonString);
        }
    }
}