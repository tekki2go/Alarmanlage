#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <MFRC522.h>
#include <FS.h>

#include "rfid_library.h"

RFID_RC522::RFID_RC522() {
    configFileName = "/rfid/rfid_tags.txt";
}

void RFID_RC522::begin() {
    SPI.begin();
    mfrc522.PCD_Init();
}

bool RFID_RC522::loadConfig() {
    if (SPIFFS.begin()) {
        configFile = SPIFFS.open(configFileName, "r");
        if (configFile) {
            while (configFile.available()) {
                String tag = configFile.readStringUntil('\n');
                tag.trim();
            }
            configFile.close();
            return true;
        }
    }
    return false;
}

bool RFID_RC522::saveConfig() {
    if (SPIFFS.begin()) {
        configFile = SPIFFS.open(configFileName, "w");
        if (configFile) {
            // Write the updated RFID tags to the file here
            configFile.close();
            return true;
        }
    }
    return false;
}

void RFID_RC522::addRFIDTag(const char* tag) {
    // Implement code to add a tag to the config file
    if (configFile) {
        configFile.println(tag);
    }
}

void RFID_RC522::removeRFIDTag(const char* tag) {
    // Implement code to remove a tag from the config file
    if (configFile) {
        configFile.seek(0, SeekSet);
        File tempFile = SPIFFS.open("/rfid/temp.txt", "w");
        while (configFile.available()) {
            String savedTag = configFile.readStringUntil('\n');
            savedTag.trim();
            if (!savedTag.equals(tag)) {
                tempFile.println(savedTag);
            }
        }
        configFile.close();
        tempFile.close();
        SPIFFS.remove(configFileName);
        SPIFFS.rename("/rfid/temp.txt", configFileName);
    }
}

bool RFID_RC522::verifyRFIDTag(const char* tag) {
    // Implement code to verify if a tag is in the config file
    if (configFile) {
        configFile.seek(0, SeekSet);
        while (configFile.available()) {
            String savedTag = configFile.readStringUntil('\n');
            savedTag.trim();
            if (savedTag.equals(tag)) {
                return true;
            }
        }
    }
    return false;
}
