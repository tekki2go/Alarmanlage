#include <Arduino.h>
// FS libs
#include <SPIFFS.h>
#include <FS.h>
#include <SD.h>
// networking
#include <WiFi.h>
#include <WiFiUdp.h>
// communication
#include <Wire.h>
#include <SPI.h>
// devices
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
// RTC
#include <RTClib.h>
#include <NTPClient.h>

//#include <esp_log.h>

// network credentials
const char* ssid = "tkNOC_IoT";
const char* password = "Q9ya&RUxDuVw&A$$w4ZNmkQMNTyKE9ZU";

// NTP Server settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;  // GMT offset in seconds
const int daylightOffset_sec = 3600; // Daylight offset in seconds

// I2C interface used for RTC, LCD and sensors (mainWire)
const int mainSDA = 21;
const int mainSCL = 22;
const int mainClockSpeed = 100000; //100kHz Clock speed

// I2C interface used for high-speed, low-latency (later called fastWire)
const int secondarySDA = 16;
const int secondarySCL = 17;
const int secondaryClockSpeed = 400000; //400kHz clock speed

// RFID reader settings
const int vspi_mosi_pin = 23;
const int vspi_miso_pin = 19;
const int vspi_clk_pin = 18;
const int vspi_ss_pin = 5;
const int rfid_rst_pin = 4;

// Pin settings
const int arm_button_pin = 0; // for arming and disarming alarm system (disarming requires rfid chip)
const int reset_alarm_pin = 2; // for resetting an alarm (requires rfid chip)
const int rfid_setting_pin = 15; // for adding and removing rfid chips (requires rfid chip after first chip is added)

// logging settings
// static const char *TAG = "Alarmanlage"; // Application Tag

// sensor pin settings
const int dht_pin = 4;

// initialize I2C interfaces
TwoWire fastWire = TwoWire(1);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);
RTC_DS1307 rtc; // DS1307 RTC
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C LCD
MFRC522 mfrc522(vspi_ss_pin, rfid_rst_pin); // RFID reader
//DHT dht(dht_pin, DHT11); // DHT Temperature sensor

// variables
bool ARMED = false;

// task handles
TaskHandle_t lcdTask;

// functions
void initial_setup(bool noFile);


// --------- RTC FUNCTIONS ---------

String currentTime() {
    DateTime now = rtc.now();
    char buffer[20]; // Buffer for formatted time
    sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    return String(buffer);
}

// --------- LCD REFRESHER ---------

void lcdJob(void * pvParameters) {
    Serial.println(("lcd refresher running on core %d", xPortGetCoreID()));
    for (;;) {
        // line 1: current time
        lcd.setCursor(0, 0);
        lcd.print(currentTime());

        // line 2: IP
        lcd.setCursor(0, 1);
        lcd.print("IP: ");
        lcd.print(WiFi.localIP());

        // line 3: alarm status
        lcd.setCursor(0, 2);

        // line 4
        lcd.setCursor(0, 3);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --------- RFID ---------

void rfid_init() {
    mfrc522.PCD_Init();

    File whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");

    // Check if whitelist file can be opened
    if (!whitelistFile) {
        Serial.println("Whitelist file not found. Setup required.");
        initial_setup(true);
        return;
    }
    
    // Check if there are any tags in the whitelist
    if (whitelistFile.size() == 0) {
        Serial.println("No tags in the whitelist. Setup required.");
        initial_setup(false);
    }
    
    whitelistFile.close();
}

void save_tag() {
    for (;;) {
        // check if card is present
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            File whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "a");
            
            // check if the whitelist exists and try to save the tag to it
            if (whitelistFile) {
                String tagData = "";
                for (byte i = 0; i < mfrc522.uid.size; i++) {
                    tagData += String(mfrc522.uid.uidByte[i], HEX);
                }
                whitelistFile.println(tagData);
                whitelistFile.close();
                Serial.println("Tag saved successfully.");
            } else {
                Serial.println("Failed to open whitelist file for writing.");
            }
            return;
        }
    }
}

bool delete_tag() {
    for (;;) {
        // check if card is present
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            String tagData = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                tagData += String(mfrc522.uid.uidByte[i], HEX);
            }
            
            File whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");
            File tempFile = SPIFFS.open("/rfid/temp.txt", "w");
            
            // check if the whitelist and tempfile exist and rewrite the whitelist by creating a temp file
            if (whitelistFile && tempFile) {
                String line;
                while (whitelistFile.available()) {
                    line = whitelistFile.readStringUntil('\n');
                    line.trim();
                    if (line != tagData) {
                        tempFile.println(line);
                    }
                }
                whitelistFile.close();
                tempFile.close();
                
                SPIFFS.remove("/rfid/rfid_tags.txt");
                SPIFFS.rename("/rfid/temp.txt", "/rfid/rfid_tags.txt");
                
                return true;
            }
            return false;
        }
    }
}

bool search_tag() {
    for (;;) {
        // check if card is present
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            String tagData = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                tagData += String(mfrc522.uid.uidByte[i], HEX);
            }
            
            File whitelistFile = SPIFFS.open("/rfid/tags.txt", "r");
            
            // look up if the tag exists in the file and return true if it is found
            if (whitelistFile) {
                String line;
                while (whitelistFile.available()) {
                    line = whitelistFile.readStringUntil('\n');
                    line.trim();
                    if (line == tagData) {
                        whitelistFile.close();
                        return true;
                    }
                }
                whitelistFile.close();
                return false;
            }
        }
    }
}

// --------- SETUP ---------

void initial_setup(bool noFile) {
    if (noFile) {
        if (!SPIFFS.exists("/rfid")) {
            if (!SPIFFS.mkdir("/rfid")) {
                Serial.println("Failed to create folder rfid!");
                for (;;);
            }
            File file = SPIFFS.open("/rfid/rfid_tags.txt", "w");
            if (!file) {
                Serial.println("Failed to create file rfid_flags.txt!");
                for (;;);
            }
            file.close();
            Serial.println("Created rfid_tags.txt");
        }
    }
    
    Serial.println("Please hold the first rfid tag to the reader!");
    save_tag();
}

void setup() {
    // initialize serial monitor and logging
    Serial.begin(115200);
    //esp_log_level_set(TAG, ESP_LOG_INFO);

    Serial.println(("Setup Running on Core %d", xPortGetCoreID()));

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // initialize main I2C bus
    //mainWire.begin(mainSDA, mainSCL, mainClockSpeed);
    Wire.begin(mainSDA, mainSCL, mainClockSpeed);

    // initialize lcd display to use mainWire
    lcd.init();
    lcd.clear();
    lcd.backlight();

    // initialize secondary I2C bus
    fastWire.begin(secondarySDA, secondarySCL, secondaryClockSpeed);

    // initialize SPI bus
    SPI.begin(vspi_clk_pin, vspi_miso_pin, vspi_mosi_pin, vspi_ss_pin);

    // Initialize RTC and synchronize with NTP
    if (!rtc.begin(&Wire)) {
        Serial.println("Could not find valid rtc, check wiring!");
    }
    
    timeClient.begin();
    timeClient.update();
    rtc.adjust(DateTime(timeClient.getEpochTime()));
    Serial.println("Initialized RTC, current time: ");
    Serial.println(currentTime());
    
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount file system");
        return;
    }

    rfid_init();

    xTaskCreatePinnedToCore(lcdJob, "lcdJob", 10000, NULL, 1, &lcdTask, 0);
    delay(500);

    Serial.println("Finished Setup");
}

void loop() {

}

