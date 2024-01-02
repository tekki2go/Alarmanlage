#include <Arduino.h>
// FS-Bibliotheken
#include <SPIFFS.h>
#include <FS.h>
#include <SD.h>
// Netzwerk
#include <WiFi.h>
#include <WiFiUdp.h>
// Kommunikation
#include <Wire.h>
#include <SPI.h>
// Geräte
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
// RTC
#include <RTClib.h>
#include <NTPClient.h>

// Netzwerkdaten
const char* ssid = "tkNOC_IoT";
const char* password = "Q9ya&RUxDuVw&A$$w4ZNmkQMNTyKE9ZU";

// NTP Server Einstellungen
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;  // GMT-Versatz in Sekunden
const int daylightOffset_sec = 3600; // Sommerzeit-Versatz in Sekunden

// I2C-Schnittstelle für RTC, LCD und Sensoren (mainWire)
const int mainSDA = 21;
const int mainSCL = 22;
const int mainClockSpeed = 100000; // 100kHz Taktgeschwindigkeit

// I2C-Schnittstelle für Hochgeschwindigkeits-, Niedriglatenzkommunikation (später als fastWire bezeichnet)
const int secondarySDA = 16;
const int secondarySCL = 17;
const int secondaryClockSpeed = 400000; // 400kHz Taktgeschwindigkeit

// RFID-Lesegerät Einstellungen
const int vspi_mosi_pin = 23;
const int vspi_miso_pin = 19;
const int vspi_clk_pin = 18;
const int vspi_ss_pin = 5;
const int rfid_rst_pin = 4;

// Pin-Einstellungen
const int piezo_pin = 0;
const int led_red_pin = 2;
const int led_green_pin = 15;
const int arm_button_pin = 32;
const int reset_alarm_button_pin = 35;
const int status_button_pin = 34;
const int tag_button_pin = 39;
const int language_button_pin = 36;

// Protokolleinstellungen
// static const char *TAG = "Alarmanlage"; // Anwendungstag

// Sensoreinstellungen für Pins
const int dht_pin = 4;

// Initialisierung der I2C-Schnittstellen
TwoWire fastWire = TwoWire(1);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);
RTC_DS1307 rtc; // DS1307 RTC
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C LCD
MFRC522 mfrc522(vspi_ss_pin, rfid_rst_pin); // RFID-Lesegerät
//DHT dht(dht_pin, DHT11); // DHT-Temperatursensor

// Variablen
bool ARMED = false;

// Task-Handles
TaskHandle_t lcdTask;

// Funktionen
void initial_setup(bool noFile);

// --------- RTC-FUNKTIONEN ---------

String currentTime() {
    DateTime now = rtc.now();
    char buffer[20]; // Puffer für formatierte Zeit
    sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    return String(buffer);
}

// --------- LCD-AKTUALISIERUNG ---------

void lcdJob(void * pvParameters) {
    Serial.println(("LCD-Aktualisierer läuft auf Core %d", xPortGetCoreID()));
    for (;;) {
        // Zeile 1: aktuelle Zeit
        lcd.setCursor(0, 0);
        lcd.print(currentTime());

        // Zeile 2: IP
        lcd.setCursor(0, 1);
        lcd.print("IP: ");
        lcd.print(WiFi.localIP());

        // Zeile 3: Alarmstatus
        lcd.setCursor(0, 2);

        // Zeile 4
        lcd.setCursor(0, 3);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --------- RFID ---------

void rfid_init() {
    mfrc522.PCD_Init();

    File whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");

    // Überprüfung, ob die Whitelist-Datei geöffnet werden kann
    if (!whitelistFile) {
        Serial.println("Whitelist-Datei nicht gefunden. Einrichtung erforderlich.");
        initial_setup(true);
        return;
    }
    
    // Überprüfung, ob Tags in der Whitelist vorhanden sind
    if (whitelistFile.size() == 0) {
        Serial.println("Keine Tags in der Whitelist. Einrichtung erforderlich.");
        initial_setup(false);
    }
    
    whitelistFile.close();
}

void save_tag() {
    for (;;) {
        // Überprüfen, ob eine Karte vorhanden ist
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            File whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "a");
            
            // Überprüfen, ob die Whitelist existiert und versuchen, das Tag darin zu speichern
            if (whitelistFile) {
                String tagData = "";
                for (byte i = 0; i < mfrc522.uid.size; i++) {
                    tagData += String(mfrc522.uid.uidByte[i], HEX);
                }
                whitelistFile.println(tagData);
                whitelistFile.close();
                Serial.println("Tag erfolgreich gespeichert.");
            } else {
                Serial.println("Fehler beim Öffnen der Whitelist-Datei zum Schreiben.");
            }
            return;
        }
    }
}

bool delete_tag() {
    for (;;) {
        // Überprüfen, ob eine Karte vorhanden ist
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            String tagData = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                tagData += String(mfrc522.uid.uidByte[i], HEX);
            }
            
            File whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");
            File tempFile = SPIFFS.open("/rfid/temp.txt", "w");
            
            // Überprüfen, ob die Whitelist und Temporärdatei existieren und die Whitelist neu schreiben, indem eine Temporärdatei erstellt wird
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
        // Überprüfen, ob eine Karte vorhanden ist
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            String tagData = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                tagData += String(mfrc522.uid.uidByte[i], HEX);
            }
            
            File whitelistFile = SPIFFS.open("/rfid/tags.txt", "r");
            
            // Überprüfen, ob das Tag in der Datei vorhanden ist und wahr zurückgeben, wenn es gefunden wird
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

// --------- EINRICHTUNG ---------

void initial_setup(bool noFile) {
    if (noFile) {
        if (!SPIFFS.exists("/rfid")) {
            if (!SPIFFS.mkdir("/rfid")) {
                Serial.println("Ordner rfid konnte nicht erstellt werden!");
                for (;;);
            }
            File file = SPIFFS.open("/rfid/rfid_tags.txt", "w");
            if (!file) {
                Serial.println("Datei rfid_flags.txt konnte nicht erstellt werden!");
                for (;;);
            }
            file.close();
            Serial.println("Created rfid_tags.txt");
        }
    }
    
    Serial.println("Bitte den RFID-Tag an den Leser halten!");
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
        Serial.println("Verbindung zu WiFi wird hergestellt...");
    }
    Serial.println("Mit WiFi verbunden");

    // Initialisieren des Haupt-I2C-Busses
    //mainWire.begin(mainSDA, mainSCL, mainClockSpeed);
    Wire.begin(mainSDA, mainSCL, mainClockSpeed);

    // Initialisieren des LCD-Displays für mainWire
    lcd.init();
    lcd.clear();
    lcd.backlight();

    // Initialisieren des sekundären I2C-Busses
    fastWire.begin(secondarySDA, secondarySCL, secondaryClockSpeed);

    // Initialisieren des SPI-Busses
    SPI.begin(vspi_clk_pin, vspi_miso_pin, vspi_mosi_pin, vspi_ss_pin);

    // RTC initialisieren und mit NTP synchronisieren
    if (!rtc.begin(&Wire)) {
        Serial.println("RTC nicht gefunden, bitte Verkabelung überprüfen!");
    }

    timeClient.begin();
    timeClient.update();
    rtc.adjust(DateTime(timeClient.getEpochTime()));
    Serial.println("RTC initialisiert, aktuelle Zeit: ");
    Serial.println(currentTime());

    if (!SPIFFS.begin(true)) {
        Serial.println("Dateisystem konnte nicht gemountet werden");
        return;
    }

    rfid_init();

    xTaskCreatePinnedToCore(lcdJob, "lcdJob", 10000, NULL, 1, &lcdTask, 0);
    delay(500);

    Serial.println("Einrichtung abgeschlossen");
}

void loop() {
    
}

