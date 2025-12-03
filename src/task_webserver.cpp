#include "task_webserver.h"
#include "global.h"
#include <ArduinoJson.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void Webserver_sendata(String data)
{
    if (ws.count() > 0)
    {
        ws.textAll(data);
        Serial.println("📤 Đã gửi dữ liệu qua WebSocket: " + data);
    }
    else
    {
        Serial.println("⚠️ Không có client WebSocket nào đang kết nối!");
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    }
    else if (type == WS_EVT_DATA)
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;

        if (info->opcode == WS_TEXT)
        {
            String message;
            message += String((char *)data).substring(0, len);
            // parseJson(message, true);
            handleWebSocketMessage(message);
        }
    }
}

void connnectWSV()
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(LittleFS, "/index.html", "text/html"); });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(LittleFS, "/script.js", "application/javascript"); });
    server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(LittleFS, "/styles.css", "text/css"); });
    server.begin();
    ElegantOTA.begin(&server);

    // Set webserver running state with mutex protection
    if (g_wifiConfig != NULL && g_wifiConfig->mutex != NULL)
    {
        if (xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_wifiConfig->webserver_isrunning = true;
            xSemaphoreGive(g_wifiConfig->mutex);
        }
    }
}

void Webserver_stop()
{
    ws.closeAll();
    server.end();

    if (g_wifiConfig != NULL && g_wifiConfig->mutex != NULL)
    {
        if (xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_wifiConfig->webserver_isrunning = false;
            xSemaphoreGive(g_wifiConfig->mutex);
        }
    }
    Serial.println("Web Server Stopped");
}

void Webserver_reconnect()
{
    bool isRunning = false;
    if (g_wifiConfig != NULL && g_wifiConfig->mutex != NULL)
    {
        if (xSemaphoreTake(g_wifiConfig->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            isRunning = g_wifiConfig->webserver_isrunning;
            xSemaphoreGive(g_wifiConfig->mutex);
        }
    }

    if (!isRunning)
    {
        connnectWSV();
    }
    ElegantOTA.loop();
}

void handleWebSocketMessage(String message)
{
    // Serial.println("📥 WS Received: " + message);

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error)
    {
        Serial.print("⚠️ JSON Error: ");
        Serial.println(error.f_str());
        return;
    }

    String page = doc["page"].as<String>();

    // ========== TASK 4: ĐIỀU KHIỂN THIẾT BỊ (LED/RELAY) ==========
    if (page == "device")
    {
        String status = doc["value"]["status"].as<String>();
        int gpio = doc["value"]["gpio"].as<int>();

        Serial.printf("🔧 Command: GPIO %d -> %s\n", gpio, status.c_str());

        DeviceControlCommand cmd;
        cmd.gpioPin = gpio;
        cmd.newState = (status == "ON");

        if (xQueueRelayControl != NULL)
        {
            if (xQueueSend(xQueueRelayControl, (void *)&cmd, 0) == pdPASS)
            {
                Serial.println("✅ Sent to Relay Queue");
            }
            else
            {
                Serial.println("⚠️ Relay Queue Full!");
            }
        }
    }

    // ========== TASK 6: CẤU HÌNH WIFI/COREIOT ==========
    else if (page == "setting") // Lưu ý: "setting" (số ít) khớp với JS
    {
        SettingsCommand settings;
        // Copy dữ liệu từ JSON sang struct (Dùng String object trong struct)
        settings.ssid = doc["value"]["ssid"].as<String>();
        settings.password = doc["value"]["password"].as<String>();
        settings.token = doc["value"]["token"].as<String>();
        settings.server = doc["value"]["server"].as<String>();
        settings.port = doc["value"]["port"].as<int>();

        Serial.println("⚙️ Received Settings Update");

        if (xQueueSettings != NULL)
        {
            if (xQueueSend(xQueueSettings, (void *)&settings, 0) == pdPASS)
            {
                Serial.println("✅ Sent to Settings Queue");
            }
            else
            {
                Serial.println("⚠️ Settings Queue Full!");
            }
        }
    }
}

// Task RTOS quản lý Web Server
void Webserver_RTOS_Task(void *pvParameters)
{
    // Khởi tạo Server ban đầu
    connnectWSV();

    unsigned long last_update = 0;
    const unsigned long update_interval = 500;

    while (1)
    {
        // Logic kiểm tra và kết nối lại WiFi/Server
        if (check_info_File(1)) // Kiểm tra nút BOOT hoặc điều kiện reset
        {
            if (!Wifi_reconnect())
                Webserver_stop();
        }

        Webserver_reconnect();
        if (millis() - last_update > update_interval)
        {
            last_update = millis();

            float temp = 0.0;
            float humi = 0.0;
            getSensorData(&temp, &humi);

            if (!isnan(temp) && !isnan(humi))
            {
                // Tạo JSON: {"page":"home", "value":{"temp":28.5, "humi":60.2}}
                StaticJsonDocument<200> doc;
                doc["page"] = "home";
                JsonObject value = doc.createNestedObject("value");
                value["temp"] = temp;
                value["humi"] = humi;

                String jsonString;
                serializeJson(doc, jsonString);

                // Gửi xuống Web
                Webserver_sendata(jsonString);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay nhỏ để nhường CPU
    }
}
