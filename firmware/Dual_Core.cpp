#include <Arduino.h>

void Core0Task(void *parameter)     // Will be using this for the main autopilot calculations
{
    while (true)
    {
        Serial.println("Hello from Core 0");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Core1Task(void *parameter)     // Will be using this for the data logging and telemetry
{
    while (true)
    {
        Serial.println("Hello from Core 1");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    Serial.begin(115200);

    xTaskCreatePinnedToCore(
        Core0Task,       // Function
        "Core0Task",     // Task name
        4096,            // Stack size
        NULL,            // Parameter
        1,               // Priority
        NULL,            // Task handle
        0                // Core
    );

    xTaskCreatePinnedToCore(
        Core1Task,
        "Core1Task",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}

void loop()
{
}
