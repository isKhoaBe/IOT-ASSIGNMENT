#include "coreiot.h"
#include "led_blinky.h"
#include "neo_blinky.h"
#include "task_webserver.h"

// ----------- CONFIGURE THESE! -----------
const char *coreIOT_Server = "app.coreiot.io";
const char *coreIOT_Token = "drx8pb6mjnq99pacxaez"; // Device Access Token
const int mqttPort = 1883;
// ----------------------------------------

WiFiClient espClient;
PubSubClient client(espClient);

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    String token = coreIOT_Token;
    if (g_wifiConfig != NULL && g_wifiConfig->mutex != NULL)
    {
      if (xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        if (!g_wifiConfig->CORE_IOT_TOKEN.isEmpty())
        {
          token = g_wifiConfig->CORE_IOT_TOKEN;
        }
        xSemaphoreGive(g_wifiConfig->mutex);
      }
    }

    if (client.connect(clientId.c_str(), token.c_str(), NULL))
    {

      Serial.println("connected to CoreIOT Server!");
      client.subscribe("v1/devices/me/rpc/request/+");
      Serial.println("Subscribed to v1/devices/me/rpc/request/+");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");

  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  Serial.print("Payload: ");
  Serial.println(message);

  // Parse JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  String topicStr = String(topic);
  int lastSlash = topicStr.lastIndexOf('/');
  String requestId = "";
  if (lastSlash > 0)
  {
    requestId = topicStr.substring(lastSlash + 1);
  }

  const char *method = doc["method"];

  // Handle LED GPIO control
  if (strcmp(method, "setValueLED_GPIO") == 0)
  {
    bool newState = doc["params"];
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

    // Send response back to ThingsBoard
    String responseTopic = "v1/devices/me/rpc/response/" + requestId;
    String responsePayload = "{\"result\":" + String(newState) + "}";
    client.publish(responseTopic.c_str(), responsePayload.c_str());
    Serial.println("Response sent: " + responsePayload);
  }

  // Handle getting LED state
  else if (strcmp(method, "getValueLED_GPIO") == 0)
  {
    // Read current LED state from config
    bool currentState = false;
    if (g_wifiConfig != NULL && g_wifiConfig->mutex != NULL)
    {
      if (xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        currentState = !g_wifiConfig->led1Override; // Inverted logic
        xSemaphoreGive(g_wifiConfig->mutex);
      }
    }

    Serial.print("Current LED state: ");
    Serial.println(currentState ? "ON" : "OFF");

    // Send response back to ThingsBoard
    String responseTopic = "v1/devices/me/rpc/response/" + requestId;
    String responsePayload = "{\"result\":" + String(currentState) + "}";
    client.publish(responseTopic.c_str(), responsePayload.c_str());
    Serial.println("Response sent: " + responsePayload);
  }

  // Handle NEO GPIO control
  else if (strcmp(method, "setValueNEO_GPIO") == 0)
  {
    bool newState = doc["params"];
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

    // Send response back to ThingsBoard
    String responseTopic = "v1/devices/me/rpc/response/" + requestId;
    String responsePayload = "{\"result\":" + String(newState) + "}";
    client.publish(responseTopic.c_str(), responsePayload.c_str());
    Serial.println("Response sent: " + responsePayload);
  }

  // Handle getting NEO state
  else if (strcmp(method, "getValueNEO_GPIO") == 0)
  {
    // Read current NEO state from config
    bool neoState = false;
    if (g_wifiConfig != NULL && g_wifiConfig->mutex != NULL)
    {
      if (xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        neoState = !g_wifiConfig->neoOverride; // Inverted logic: ON=AUTO, OFF=forced off
        xSemaphoreGive(g_wifiConfig->mutex);
      }
    }

    Serial.print("Current NEO state: ");
    Serial.println(neoState ? "ON (AUTO)" : "OFF");

    // Send response back to ThingsBoard
    String responseTopic = "v1/devices/me/rpc/response/" + requestId;
    String responsePayload = "{\"result\":" + String(neoState) + "}";
    client.publish(responseTopic.c_str(), responsePayload.c_str());
    Serial.println("Response sent: " + responsePayload);
  }

  else
  {
    Serial.print("Unknown method: ");
    Serial.println(method);

    // Still send a response to avoid timeout
    String responseTopic = "v1/devices/me/rpc/response/" + requestId;
    String responsePayload = "{\"error\":\"Unknown method\"}";
    client.publish(responseTopic.c_str(), responsePayload.c_str());
  }
}

void setup_coreiot()
{
  while (1)
  {
    if (g_wifiConfig != NULL && g_wifiConfig->xBinarySemaphoreInternet != NULL)
    {
      if (xSemaphoreTake(g_wifiConfig->xBinarySemaphoreInternet, portMAX_DELAY))
      {
        break;
      }
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println(" Connected!");

  if (g_wifiConfig != NULL)
  {
    client.setServer(g_wifiConfig->CORE_IOT_SERVER.c_str(), g_wifiConfig->CORE_IOT_PORT.toInt());
  }
  else
  {
    client.setServer(coreIOT_Server, mqttPort);
  }
  client.setCallback(callback);
}

void coreiot_task(void *pvParameters)
{

  setup_coreiot();

  // Local variables for sensor data
  float temperature = 0.0;
  float humidity = 0.0;

  unsigned long lastTelemetryTime = 0;
  const unsigned long telemetryInterval = 10000; // 10 seconds

  while (1)
  {

    if (!client.connected())
    {
      reconnect();
    }
    client.loop();

    // Publish telemetry every 10 seconds
    if (millis() - lastTelemetryTime >= telemetryInterval)
    {
      // Read sensor data using semaphore-protected function
      getSensorData(&temperature, &humidity);

      // Sample payload, publish to 'v1/devices/me/telemetry'
      String payload = "{\"temperature\":" + String(temperature) + ",\"humidity\":" + String(humidity) + "}";

      client.publish("v1/devices/me/telemetry", payload.c_str());

      Serial.println("[CoreIOT] Published payload: " + payload);
      lastTelemetryTime = millis();
    }

    // Small delay to prevent watchdog issues but keep processing RPC
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}