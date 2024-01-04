#include <Arduino.h>
// FS-Bibliotheken
#include <SPIFFS.h>
#include <FS.h>
#include <SD.h>
// Netzwerk
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
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
// Telegram-Bot
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

// Netzwerkdaten
const char* ssid = "tkNOC_IoT";
const char* password = "Q9ya&RUxDuVw&A$$w4ZNmkQMNTyKE9ZU";

// Token des Telegram-bots
const char* bot_token = "6923232009:AAGqJnmC9oe915CyHLO3OMCKHy9U8muVG1Q";

// Hier die Chat IDs eintragen (kann man per @myidbot herausfinden)
// Außerdem muss man "Start" auf dem Bot klicken bevor er Nachrichten versenden kann
String chat_ids[] = {
    "5639436151"
};

// Hier Chat-IDs für logging eintragen
String logging_chat_ids[] = {
    "5639436151"
};

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
const int piezo_pin = 0;                 // Piezo-Piepser
const int led_red_pin = 2;               // Rote LED
const int led_green_pin = 15;            // Grüne LED

const int arm_button_pin = 33;           // Taster 1
const int reset_alarm_button_pin = 32;   // Taster 2
const int status_button_pin = 35;        // Taster 3
const int tag_button_pin = 34;           // Taster 4
const int language_button_pin = 39;      // Taster 5

const int photoelectric_sensor_pin = 25; // Lichtschranke
const int radar_sensor_pin = 26;         // Radar
const int microphone_pin = 27;           // Mikrofon
const int dht11_pin = 14;                // DHT11
const int movement_sensor_pin = 12;      // Bewegungsmelder

// Log-Einstellungen
// static const char *TAG = "Alarmanlage"; // Anwendungstag

// Initialisierung der I2C-Schnittstellen
TwoWire fastWire = TwoWire(1);

WiFiUDP ntpUDP; // UDP client, wird vom NTPClient verwendet
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec); // NTP Client, um den RTC aktuell zu halten

RTC_DS1307 rtc;                               // DS1307 RTC
LiquidCrystal_I2C lcd(0x27, 20, 4);           // I2C LCD
MFRC522 mfrc522(vspi_ss_pin, rfid_rst_pin);   // RFID-Lesegerät
DHT dht(dht11_pin, DHT11);                    // DHT-Temperatursensor

// Twitter Bot
WiFiClientSecure client;
UniversalTelegramBot bot(bot_token, client);


// Variablen
bool ARMED = false;    // true = scharfgeschaltet
bool LANGUAGE = false; // false = deutsch, true = englisch

float temperature;     // aktuelle Temperatur
float humidity;        // aktuelle Luftfeuchtigkeit

int status = 0;        // was gerade in der Status-Anzeige gezeigt wird
// 0 = Temperatur + Luftfeuchtigkeit, 1 = IP

// Telegram-Bot prüft alle 1000ms ob neue Nachrichten vorhanden sind
int bot_request_delay = 1000;
unsigned long last_bot_refresh;


// Task-Handles
TaskHandle_t lcdTask;

// Funktionen
void initial_setup(bool noFile);

// --------- LCD-Funktionen ---------

void clearLine(int line) {
    lcd.setCursor(0, line);
    lcd.print("                    ");
}

// --------- RTC-FUNKTIONEN ---------

String currentTime() {
    DateTime now = rtc.now();
    char buffer[20]; // Puffer für formatierte Zeit
    sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    return String(buffer);
}

// --------- JOBS UND INTERRUPTS---------

// lcd-aktualisierung
void lcdJob(void * pvParameters) {
    if (LANGUAGE) Serial.println("lcd refresher running on core " + String(xPortGetCoreID()));
    else Serial.println("LCD-Aktualisierer läuft auf Coree " + String(xPortGetCoreID()));

    lcd.clear();

    for (;;) {
        // Zeile 1: aktuelle Zeit
        lcd.setCursor(0, 0);
        lcd.print(currentTime());

        // Zeile 2: Status
        clearLine(1);
        lcd.setCursor(0, 1);
        switch (status) {
            case 0: 
                lcd.print(String(temperature) + "°C, ");
                lcd.print(String(humidity) + "%");
                break;
            case 1: 
                lcd.print(("IP: " + String(WiFi.localIP())));
                break;
        }

        // Zeile 3: Alarmstatus
        //lcd.setCursor(0, 2);
        //lcd.print("                    "); // zeile löschen

        // Zeile 4
        //lcd.setCursor(0, 3);
        //lcd.print("                    "); // zeile löschen

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void IRAM_ATTR handleArmButtonInterrupt() {

}

void IRAM_ATTR handleResetAlarmButtonInterrupt() {
    
}

void IRAM_ATTR handleStatusButtonInterrupt() {
    status += 1;
    Serial.println("Status: " + String(status));
    if (status > 1) status = 0;
}

void IRAM_ATTR handleTagButtonInterrupt() {
    
}

void IRAM_ATTR handleLanguageButtonInterrupt() {
    LANGUAGE = !LANGUAGE;
}

// --------- RFID ---------

void rfid_init() {
    mfrc522.PCD_Init();

    File whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");

    // Überprüfen, ob die Whitelist-Datei geöffnet werden kann
    if (!whitelistFile) {
        if (LANGUAGE) Serial.println("Whitelist file not found. Setup required.");
        else Serial.println("Whitelist-Datei nicht gefunden. Einrichtung erforderlich.");
        initial_setup(true);
        return;
    }
    
    // Überprüfen, ob Tags in der Whitelist vorhanden sind
    if (whitelistFile.size() == 0) {
        if (LANGUAGE) Serial.println("No tags in the whitelist. Setup required.");
        else Serial.println("Keine Tags in der Whitelist. Einrichtung erforderlich.");
        initial_setup(false);
    }
    
    whitelistFile.close();
}

void save_tag() {
    lcd.setCursor(0, 2);
    if (LANGUAGE) lcd.print("Please hold an RFID");
    else lcd.print(" Bitte den RFID-Tag ");
    lcd.setCursor(0, 3);
    if (LANGUAGE) lcd.print("tag to the reader!");
    else lcd.print("an den Leser halten!");

    for (;;) {
        // Überprüfen, ob eine RFID-Karte vorhanden ist
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
                if (LANGUAGE) Serial.println("Tag saved successfully.");
                else Serial.println("Tag erfolgreich gespeichert.");

                clearLine(2);
                clearLine(3);
                lcd.setCursor(0, 2);
                if (LANGUAGE) lcd.print("RFID-Tag saved.");
                else lcd.print("RFID-Tag gespeichert.");

                delay(1000);
                clearLine(2);
                clearLine(3);
            } else {
                if (LANGUAGE) Serial.println("Failed to open whitelist file for writing.");
                else Serial.println("Fehler beim Öffnen der Whitelist-Datei zum Schreiben.");
                clearLine(2);
                clearLine(3);
                lcd.setCursor(0, 2);
                if (LANGUAGE) lcd.print("Error while saving!");
                else lcd.print("Fehler beim Speichern!");
            }
            return;
        }
    }
}

bool delete_tag() {
    for (;;) {
        // Überprüfen, ob eine RFID-Karte vorhanden ist
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

                if (LANGUAGE) Serial.println("Successfully deleted Tag.");
                else Serial.println("Tag erfolgreich gelöscht");                
                return true;
            }

            if (LANGUAGE) Serial.println("Could not find Tag!");
            else Serial.println("Tag konnte nicht gefunden werden!");
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

// --------- TELEGRAM-BOT --------

void handleNewMessages(int numNewMessages) {
    //Serial.println("Neue Nachrichten werden abgefragt...");
    Serial.println(String(numNewMessages));

    bool is_authorized = false;

    for (int i=0; i<numNewMessages; i++) {
        // Chat id of the requester
        String chatID = String(bot.messages[i].chat_id);
        for (int i = 0; i < sizeof(chat_ids); i++) {
            if (chatID == chat_ids[i]) {
                is_authorized = true;
                break;
            }
        }

        if (!is_authorized) bot.sendMessage(chatID, "Unauthorized user", "");
        
        // Print the received message
        String text = bot.messages[i].text;
        Serial.println(text);

        String from_name = bot.messages[i].from_name;

        if (text == "/start") {
        String welcome = "Willkommen, " + from_name + ".\n";
        welcome += "Benutze einen der folgenden Befehle:\n\n";
        welcome += "/state gibt einen status-bericht aus \n";
        bot.sendMessage(chatID, welcome, "");
        }
        
        if (text == "/state") {
            bot.sendMessage(chatID, "TEST", "");
        }
    }
}

void sendLogMessage(String message) {
    for (int i = 0; i < sizeof(logging_chat_ids); i++) {
        String chat_id = logging_chat_ids[i];
        bot.sendMessage(chat_id, message, "");
    }
}


// --------- EINRICHTUNG ---------

void initial_setup(bool noFile) {
    if (noFile) {
        if (!SPIFFS.exists("/rfid")) {
            if (!SPIFFS.mkdir("/rfid")) {
                if (LANGUAGE) Serial.println("Failed to create folder rfid!");
                else Serial.println("Ordner rfid konnte nicht erstellt werden!");
                for (;;);
            }
            File file = SPIFFS.open("/rfid/rfid_tags.txt", "w");
            if (!file) {
                if (LANGUAGE) Serial.println("Failed to create file rfid_flags.txt!");
                else Serial.println("Datei rfid_flags.txt konnte nicht erstellt werden!");
                for (;;);
            }
            file.close();
            if (LANGUAGE) Serial.println("Successfully created rfid_tags.txt");
            else Serial.println("rfid_tags.txt wurde erfolgreich erstellt");
        }
    }
    
    if (LANGUAGE) Serial.println("Please hold the rfid tag to the reader!");
    else Serial.println("Bitte den RFID-Tag an den Leser halten!");

    save_tag();
}

void setup() {
    // Initialisieren des Seriellen Monitors
    Serial.begin(115200);
    //esp_log_level_set(TAG, ESP_LOG_INFO);

    if (LANGUAGE) Serial.println("Setup running on Core " + String(xPortGetCoreID()));
    else Serial.println("Setup läuft auf Core " + String(xPortGetCoreID()));

    // Mit WLAN verbinden
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (LANGUAGE) Serial.println("Connecting to WiFi...");
        else Serial.println("Verbindung zu WiFi wird hergestellt...");
    }
    if (LANGUAGE) Serial.println("Connected to WiFi");
    else Serial.println("Mit WiFi verbunden");

    // Root-Zertifikat für SSL hinzufügen (Telegram-Bot)
    client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
    //sendLogMessage("beginning...");
    bot.sendMessage("539436151", "TEST", "");
    delay(500);
    
    // Buttons als input konfigurieren
    pinMode(arm_button_pin, INPUT_PULLUP);
    pinMode(reset_alarm_button_pin, INPUT_PULLUP);
    pinMode(status_button_pin, INPUT_PULLUP);
    pinMode(tag_button_pin, INPUT_PULLUP);
    pinMode(language_button_pin, INPUT_PULLUP);

    // Interrupts für Buttons hinzufügen
    attachInterrupt(digitalPinToInterrupt(arm_button_pin), &handleArmButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(reset_alarm_button_pin), &handleResetAlarmButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(status_button_pin), &handleStatusButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(tag_button_pin), &handleTagButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(language_button_pin), &handleLanguageButtonInterrupt, FALLING);

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
        if (LANGUAGE) Serial.println("Could not find valid rtc, check wiring!");
        else Serial.println("RTC nicht gefunden, bitte Verkabelung überprüfen!");
    }

    timeClient.begin();
    timeClient.update();
    rtc.adjust(DateTime(timeClient.getEpochTime()));
    if (LANGUAGE) Serial.println("Initialized RTC, current time: " + String(currentTime()));
    else Serial.println("RTC initialisiert, aktuelle Zeit: " + String(currentTime()));


    if (!SPIFFS.begin(true)) {
        if (LANGUAGE) Serial.println("Failed to mount file system");
        else Serial.println("Dateisystem konnte nicht gemountet werden");
        return;
    }


    rfid_init();


    xTaskCreatePinnedToCore(lcdJob, "lcdJob", 10000, NULL, 1, &lcdTask, 0);
    delay(250);
    //xTaskCreatePinnedToCore(inputChecker, "inputChecker", 10000, NULL, 1, &inputTask, 0);
    //delay(250);

    if (LANGUAGE) Serial.println("Finished Setup");
    else Serial.println("Einrichtung abgeschlossen");
}

void loop() {
    if (millis() > last_bot_refresh + bot_request_delay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    last_bot_refresh = millis();
  }
    delay(1000);
}

