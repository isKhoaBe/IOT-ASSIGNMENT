#include "temp_humi_monitor.h"

void temp_humi_monitor(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    DHT20 dht20;

    Wire.begin(11, 12);
    vTaskDelay(pdMS_TO_TICKS(100));

    dht20.begin();
    vTaskDelay(pdMS_TO_TICKS(500));

    float temperature = 0.0;
    float humidity = 0.0;

    while (1)
    {
        dht20.read();
        temperature = dht20.getTemperature();
        humidity = dht20.getHumidity();

        // Check if any reads failed
        if (isnan(temperature) || isnan(humidity))
        {
            Serial.println("[SENSOR] ERROR: Failed to read from DHT sensor!");
            temperature = 25.0; // Default safe values
            humidity = 50.0;
        }

        // Update shared data using semaphore-protected function
        setSensorData(temperature, humidity);

        // Print the results to serial monitor
        Serial.print("[SENSOR] Temp: ");
        Serial.print(temperature);
        Serial.print("C, Humidity: ");
        Serial.print(humidity);
        Serial.println("%");

        // Read sensor every 1sec
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void lcd_display_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Serial.println("[LCD] Task starting - Initializing display...");

    // Initialize LCD
    LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
    lcd.begin();
    lcd.backlight();
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    vTaskDelay(pdMS_TO_TICKS(1000));
    lcd.clear();

    Serial.println("[LCD] Display initialized successfully");

    float temperature = 0.0;
    float humidity = 0.0;
    DisplayState_t currentState = DISPLAY_NORMAL;
    DisplayState_t previousState = DISPLAY_NORMAL;

    while (1)
    {
        getSensorData(&temperature, &humidity);

        bool isCritical = (temperature < TEMP_CRITICAL_LOW ||
                           temperature > TEMP_CRITICAL_HIGH ||
                           humidity < HUMIDITY_CRITICAL_LOW ||
                           humidity > HUMIDITY_CRITICAL_HIGH);

        bool isWarning = (!isCritical) &&
                         (temperature < TEMP_WARNING_LOW ||
                          temperature > TEMP_WARNING_HIGH ||
                          humidity < HUMIDITY_WARNING_LOW ||
                          humidity > HUMIDITY_WARNING_HIGH);

        if (isCritical)
        {
            currentState = DISPLAY_CRITICAL;
        }
        else if (isWarning)
        {
            currentState = DISPLAY_WARNING;
        }
        else
        {
            currentState = DISPLAY_NORMAL;
        }

        if (currentState != previousState)
        {
            Serial.print("[LCD] State: ");
            if (currentState == DISPLAY_NORMAL)
                Serial.println("NORMAL");
            else if (currentState == DISPLAY_WARNING)
                Serial.println("WARNING");
            else
                Serial.println("CRITICAL");
            previousState = currentState;
        }

        lcd.backlight();
        lcd.clear();

        switch (currentState)
        {
        case DISPLAY_NORMAL:
            // Normal display - standard readings
            lcd.setCursor(0, 0);
            lcd.print("Temp: ");
            lcd.print(temperature, 1);
            lcd.print("C");
            lcd.setCursor(0, 1);
            lcd.print("Humi: ");
            lcd.print(humidity, 1);
            lcd.print("%");
            break;

        case DISPLAY_WARNING:
            // Warning state - display with warning indicator
            lcd.setCursor(0, 0);
            lcd.print("!WARNING!");
            lcd.setCursor(0, 1);
            lcd.print("T:");
            lcd.print(temperature, 1);
            lcd.print(" H:");
            lcd.print(humidity, 1);
            break;

        case DISPLAY_CRITICAL:
            // Critical state - display with critical indicator
            lcd.setCursor(0, 0);
            lcd.print("!!!CRITICAL!!!");
            lcd.setCursor(0, 1);
            lcd.print("T: ");
            lcd.print(temperature, 1);
            lcd.print("H: ");
            lcd.print(humidity, 1);
            break;
        }

        // Update display every 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}