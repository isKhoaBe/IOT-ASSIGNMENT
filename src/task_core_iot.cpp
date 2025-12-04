
#include "task_core_iot.h"
#include "task_webserver.h"
#include "led_blinky.h"
#include "neo_blinky.h"

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

constexpr char LED_STATE_ATTR[] = "ledState";

volatile int ledMode = 0;
volatile bool ledState = false;

constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

constexpr int16_t telemetrySendInterval = 10000U;

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
    LED_STATE_ATTR,
};

void processSharedAttributes(const Shared_Attribute_Data &data)
{
    for (auto it = data.begin(); it != data.end(); ++it)
    {
    }
}

// RPC method to SET LED state
RPC_Response setValueLED_GPIO(const RPC_Data &data)
{
    Serial.println("Received RPC: setValueLED_GPIO");
    bool newState = data;
    Serial.print("LED state change to: ");
    Serial.println(newState ? "ON" : "OFF");

    // Send command to Device Control Task via queue
    DeviceControlCommand cmd;
    cmd.gpioPin = LED_GPIO;
    cmd.newState = newState;

    if (xQueueRelayControl != NULL)
    {
        if (xQueueSend(xQueueRelayControl, &cmd, pdMS_TO_TICKS(100)) == pdPASS)
        {
            Serial.println("LED control command sent to queue");
        }
        else
        {
            Serial.println("Failed to send LED control command");
        }
    }

    // Update LED state
    ledState = newState;

    return RPC_Response("setValueLED_GPIO", newState);
}

// RPC method to GET LED state
RPC_Response getValueLED_GPIO(const RPC_Data &data)
{
    Serial.println("Received RPC: getValueLED_GPIO");
    Serial.print("Current LED state: ");
    Serial.println(ledState ? "ON" : "OFF");

    return RPC_Response("getValueLED_GPIO", ledState);
}

// RPC method to SET NEO state
RPC_Response setValueNEO_GPIO(const RPC_Data &data)
{
    Serial.println("Received RPC: setValueNEO_GPIO");
    bool newState = data;
    Serial.print("NEO state change to: ");
    Serial.println(newState ? "ON" : "OFF");

    // Send command to Device Control Task via queue
    DeviceControlCommand cmd;
    cmd.gpioPin = NEO_PIN;
    cmd.newState = newState;

    if (xQueueRelayControl != NULL)
    {
        if (xQueueSend(xQueueRelayControl, &cmd, pdMS_TO_TICKS(100)) == pdPASS)
        {
            Serial.println("NEO control command sent to queue");
        }
        else
        {
            Serial.println("Failed to send NEO control command");
        }
    }

    return RPC_Response("setValueNEO_GPIO", newState);
}

// RPC method to GET NEO state
RPC_Response getValueNEO_GPIO(const RPC_Data &data)
{
    Serial.println("Received RPC: getValueNEO_GPIO");
    // Return current override state (inverted: ON means AUTO, OFF means forced OFF)
    bool neoState = false;
    if (g_wifiConfig != NULL && g_wifiConfig->mutex != NULL)
    {
        if (xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            neoState = !g_wifiConfig->neoOverride; // Invert because ON=AUTO, OFF=forced off
            xSemaphoreGive(g_wifiConfig->mutex);
        }
    }
    Serial.print("Current NEO state: ");
    Serial.println(neoState ? "ON (AUTO)" : "OFF");

    return RPC_Response("getValueNEO_GPIO", neoState);
}

const std::array<RPC_Callback, 4U> callbacks = {
    RPC_Callback{"setValueLED_GPIO", setValueLED_GPIO},
    RPC_Callback{"getValueLED_GPIO", getValueLED_GPIO},
    RPC_Callback{"setValueNEO_GPIO", setValueNEO_GPIO},
    RPC_Callback{"getValueNEO_GPIO", getValueNEO_GPIO}};

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void CORE_IOT_sendata(String mode, String feed, String data)
{
    if (mode == "attribute")
    {
        tb.sendAttributeData(feed.c_str(), data);
    }
    else if (mode == "telemetry")
    {
        float value = data.toFloat();
        tb.sendTelemetryData(feed.c_str(), value);
    }
    else
    {
        // handle unknown mode
    }
}

void CORE_IOT_reconnect()
{
    if (!tb.connected())
    {
        // Get WiFi config data with mutex protection
        String server = "";
        String token = "";
        String port = "";

        if (g_wifiConfig != NULL && g_wifiConfig->mutex != NULL)
        {
            if (xSemaphoreTake(g_wifiConfig->mutex, portMAX_DELAY) == pdTRUE)
            {
                server = g_wifiConfig->CORE_IOT_SERVER;
                token = g_wifiConfig->CORE_IOT_TOKEN;
                port = g_wifiConfig->CORE_IOT_PORT;
                xSemaphoreGive(g_wifiConfig->mutex);
            }
        }

        if (!tb.connect(server.c_str(), token.c_str(), port.toInt()))
        {
            // Serial.println("Failed to connect");
            return;
        }

        tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

        Serial.println("Subscribing for RPC...");
        if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend()))
        {
            // Serial.println("Failed to subscribe for RPC");
            return;
        }

        if (!tb.Shared_Attributes_Subscribe(attributes_callback))
        {
            // Serial.println("Failed to subscribe for shared attribute updates");
            return;
        }

        Serial.println("Subscribe done");

        if (!tb.Shared_Attributes_Request(attribute_shared_request_callback))
        {
            // Serial.println("Failed to request for shared attributes");
            return;
        }
        tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    }
    else if (tb.connected())
    {
        tb.loop();
    }
}