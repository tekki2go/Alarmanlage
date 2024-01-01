#ifndef RFID_RC522_LIBRARY_H
#define RFID_RC522_LIBRARY_H

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <FS.h>

class RFID_RC522 {
public:
    RFID_RC522();
    void begin();
    bool loadConfig();
    bool saveConfig();
    void addRFIDTag(const char* tag);
    void removeRFIDTag(const char* tag);
    bool verifyRFIDTag(const char* tag);

private:
    MFRC522 mfrc522;
    String configFileName;
    File configFile;
};

#endif
