#include <Arduino.h>
#include <SPIFFS.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
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

// logging settings
// static const char *TAG = "Alarmanlage"; // Application Tag

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);

RTC_DS1307 rtc;

String currentTime() {
    DateTime now = rtc.now();
    char buffer[20]; // Buffer for formatted time
    sprintf(buffer, "%02d/%02d/%04d %02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute());
    return String(buffer);
}

void setup() {
    // initialize serial monitor and logging
    Serial.begin(115200);
    //esp_log_level_set(TAG, ESP_LOG_INFO);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // initialize I2C interfaces
    TwoWire mainWire = TwoWire(0);
    TwoWire fastWire = TwoWire(1);

    mainWire.begin(mainSDA, mainSCL, mainClockSpeed);

    fastWire.begin(secondarySDA, secondarySCL, secondaryClockSpeed);

    // Initialize RTC and synchronize with NTP
    if (!rtc.begin(&mainWire)) {
        Serial.println("Could not find valid rtc, check wiring!");
    }
    
    timeClient.begin();
    timeClient.update();
    rtc.adjust(DateTime(timeClient.getEpochTime()));
    Serial.println("Initialized RTC, current time: ");
    Serial.println(currentTime());
}

void loop() {
}

