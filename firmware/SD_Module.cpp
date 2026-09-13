#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#define SD_CS 5

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== SD CARD TEST ===");

    SPI.begin(18, 19, 23, SD_CS);

    if (!SD.begin(SD_CS, SPI, 10000000))
    {
        Serial.println("SD card initialization failed!");
        while (1)
        {
            delay(1000);
        }
    }

    Serial.println("SD card initialized successfully!");

    uint8_t cardType = SD.cardType();

    Serial.print("Card type: ");

    if (cardType == CARD_MMC)
        Serial.println("MMC");
    else if (cardType == CARD_SD)
        Serial.println("SDSC");
    else if (cardType == CARD_SDHC)
        Serial.println("SDHC");
    else
        Serial.println("UNKNOWN");

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);

    Serial.print("Card size: ");
    Serial.print(cardSize);
    Serial.println(" MB");

    // Write test
    File file = SD.open("/test.txt", FILE_WRITE);

    if (!file)
    {
        Serial.println("Failed to open test.txt for writing!");
        return;
    }

    file.println("Project Aether SD test");
    file.println("SD card writing works.");
    file.close();

    Serial.println("Write successful!");

    // Read test
    file = SD.open("/test.txt");

    if (!file)
    {
        Serial.println("Failed to open test.txt for reading!");
        return;
    }

    Serial.println("Contents of test.txt:");

    while (file.available())
    {
        Serial.write(file.read());
    }

    file.close();

    Serial.println();
    Serial.println("=== SD TEST COMPLETE ===");
}

void loop()
{
}
