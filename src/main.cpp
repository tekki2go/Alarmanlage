#pragma region includes

#include <Arduino.h>
// FS-Bibliotheken
#include <SPIFFS.h>
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
#include <Adafruit_BMP280.h>
// RTC
#include <RTClib.h>
#include <NTPClient.h>
// Telegram-Bot
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
// ESP Timer
#include <esp_timer.h>
// WDT
//#include "soc/rtc_wdt.h"

#pragma endregion

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


#pragma region settings

// Sensor-Sensibilität
const int pressure_tolerance = 10; //Luftdruck-Toleranz in hPa

// Timings
int alarm_activation_time = 30;        // Zeit in Sekunden, bis die Alarmanlage aktiviert wird
int bot_request_delay = 1000;          // Telegram-Bot prüft alle 1000 ms ob neue Nachrichten vorhanden sind
int debounce_time = 500;               // Button-Interrupts dürfen nur alle 500ms getriggert werden
int sensor_debounce_time = 5000;       // Sensor-Interrupts dürfen nur alle 5 Sekunden getriggert werden

// NTP Server Einstellungen
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;      // GMT-Versatz in Sekunden
const int daylightOffset_sec = 3600;  // Sommerzeit-Versatz in Sekunden

// I2C-Schnittstelle für RTC, LCD und Sensoren (mainWire)
const int mainSDA = 21;
const int mainSCL = 22;
const int mainClockSpeed = 100000; // 100kHz Taktgeschwindigkeit

// RFID-Lesegerät Einstellungen
const int vspi_mosi_pin = 19;
const int vspi_miso_pin = 18;
const int vspi_clk_pin = 5;
const int vspi_ss_pin = 17;
const int rfid_rst_pin = 23;

// Pin-Einstellungen
const int piezo_pin = 4;                 // Piezo-Piepser
const int led_red_pin = 2;               // Rote LED
const int led_green_pin = 0;             // Grüne LED

const int arm_button_pin = 32;           // Taster 1
const int reset_alarm_button_pin = 25;   // Taster 2
const int status_button_pin = 27;        // Taster 3
const int tag_button_pin = 14;           // Taster 4
const int language_button_pin = 12;      // Taster 5

const int photoelectric_sensor_pin = 13; // Lichtschranke
const int radar_sensor_pin = 35;         // Radar
const int microphone_pin = 34;           // Mikrofon
const int dht11_pin = 26;                // DHT11 33
const int movement_sensor_pin = 33;      // Bewegungsmelder 26


// Log-Einstellungen
// static const char *TAG = "Alarmanlage"; // Anwendungstag

#pragma endregion

#pragma region inits
// Initialisierung der I2C-Schnittstellen
TwoWire fastWire = TwoWire(1);

WiFiUDP ntpUDP; // UDP client, wird vom NTPClient verwendet
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec); // NTP Client, um den RTC aktuell zu halten

RTC_DS1307 rtc;                               // DS1307 RTC
LiquidCrystal_I2C lcd(0x27, 20, 4);           // I2C LCD
MFRC522 mfrc522(vspi_ss_pin, rfid_rst_pin);   // RFID-Lesegerät
DHT dht(dht11_pin, DHT11);                    // DHT-Temperatursensor
Adafruit_BMP280 bmp;                          // BMP280 Luftdrucksensor

// Twitter Bot
WiFiClientSecure client;
UniversalTelegramBot bot(bot_token, client);

// Task-Handles
TaskHandle_t lcdTask;
TaskHandle_t sensorTask;

TaskHandle_t armTask;
TaskHandle_t resetTask;

#pragma endregion

#pragma region variables
// Variablen
bool ARMED = false;    // true = scharfgeschaltet
bool ALARM = false;    // true = alarm ausgelöst
bool LANGUAGE = false; // false = deutsch, true = englisch

float temperature;     // aktuelle Temperatur
float humidity;        // aktuelle Luftfeuchtigkeit
float pressure;        // aktueller Luftdruck
float height;          // aktuelle Höhe (m)

int status = 0;        // was gerade in der Status-Anzeige gezeigt wird
// 0 = Temperatur + Luftfeuchtigkeit, 1 = IP, 2 = Luftdruck + höhe

int loop_ticks = 0;    // für starten der jobs

// debounce timings
volatile unsigned long last_arm_interrupt_time = 0;
volatile unsigned long last_reset_interrupt_time = 0;
volatile unsigned long last_status_interrupt_time = 0;
volatile unsigned long last_tag_interrupt_time = 0;
volatile unsigned long last_language_interrupt_time = 0;

// sensor timings
volatile unsigned long last_photoelectric_interrupt_time = 0;

// variable für letzten refresh der nachrichten
unsigned long last_bot_refresh;

// variable zur verhinderung von mehreren I2C-Zugriffen gleichzeitig
volatile bool i2c_lock = false;

// enthält den Zeitstempel der letzten Status-Änderung
String last_status_change;

// Variable für Dateien
File whitelistFile;

#pragma endregion

// Funktionen
void initial_setup(bool noFile);
void IRAM_ATTR handlePhotoelectricInterrupt();

// --------- Nützliche Funktionen ---------

void clearLine(int line) {
    lcd.setCursor(0, line);
    lcd.print("                    ");
}

String currentTime() {
    DateTime now = rtc.now();
    char buffer[20]; // Puffer für formatierte Zeit
    sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    return String(buffer);
}

void lockI2CBus() {
    while (i2c_lock) {
        delay(30);
    }
    i2c_lock = true;
}

void unlockI2CBus() {
    i2c_lock = false;
}

// --------- RFID ---------
#pragma region rfid

void rfid_init() {
    mfrc522.PCD_Init();

    whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");

    // Überprüfen, ob die Whitelist-Datei geöffnet werden kann
    if (!whitelistFile) {
        Serial.println(LANGUAGE ? "Whitelist file not found. Setup required." : "Whitelist-Datei nicht gefunden. Einrichtung erforderlich.");
        initial_setup(true);
        return;
    }
    
    // Überprüfen, ob Tags in der Whitelist vorhanden sind
    if (whitelistFile.size() == 0) {
        Serial.println(LANGUAGE ? "No tags in the whitelist. Setup required." : "Keine Tags in der Whitelist. Einrichtung erforderlich.");
        initial_setup(false);
    }
    
    whitelistFile.close();
}

void save_tag() {
    lcd.setCursor(0, 2);
    lcd.print(LANGUAGE ? "Please hold an RFID" : " Bitte den RFID-Tag ");
    lcd.setCursor(0, 3);
    lcd.print(LANGUAGE ? "tag to the reader!" : "an den Leser halten!");


    for (;;) {
        // Überprüfen, ob eine RFID-Karte vorhanden ist
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "a");
            
            // Überprüfen, ob die Whitelist existiert und versuchen, den Tag darin zu speichern
            if (whitelistFile) {
                String tagData = "";
                for (byte i = 0; i < mfrc522.uid.size; i++) {
                    tagData += String(mfrc522.uid.uidByte[i], HEX);
                }

                Serial.println((LANGUAGE ? "Tag found: " : "Tag gefunden: ") + tagData);

                whitelistFile.println(tagData);
                whitelistFile.close();
                Serial.println(LANGUAGE ? "Tag saved successfully." : "Tag erfolgreich gespeichert.");

                clearLine(2);
                clearLine(3);
                lcd.setCursor(0, 2);
                lcd.print(LANGUAGE ? "RFID-Tag saved." : "RFID-Tag gespeichert");

                delay(1000);
                clearLine(2);
                clearLine(3);
            } else {
                Serial.println(LANGUAGE ? "Failed to open whitelist file for writing." : "Fehler beim Öffnen der Whitelist-Datei zum Schreiben.");
                clearLine(2);
                clearLine(3);
                lcd.setCursor(0, 2);
                lcd.print(LANGUAGE ? "Error while saving!" : "Fehler beim Speichern!");
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

            Serial.println((LANGUAGE ? "Tag found: " : "Tag gefunden: ") + tagData);
            
            whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");
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

                Serial.println(LANGUAGE ? "Successfully deleted Tag." : "Tag erfolgreich gelöscht");    
                return true;
            }

            Serial.println(LANGUAGE ? "Could not find Tag!" : "Tag konnte nicht gefunden werden!");
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

            Serial.println((LANGUAGE ? "Tag found: " : "Tag gefunden: ") + tagData);
            
            whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");
            
            // Überprüfen, ob der Tag in der Datei vorhanden ist und wahr zurückgeben, wenn er gefunden wird
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

int tag_amount() {
    whitelistFile = SPIFFS.open("/rfid/rfid_tags.txt", "r");

    // Prüfen ob die whitelist-datei geöffnet werden kann
    if (!whitelistFile) {
        Serial.println(LANGUAGE ? "Failed to open whitelist file." : "Fehler beim Öffnen der Whitelist-Datei.");
        return -1; // wegen fehler -1 zurückgeben
    }

    int count = 0;
    while (whitelistFile.available()) {
        if (whitelistFile.readStringUntil('\n').length() > 0) {
            count++;
        }
    }

    whitelistFile.close();
    return count;
}

#pragma endregion rfid

// --------- TELEGRAM-BOT --------
#pragma region telegram

void handleNewMessages(int numNewMessages) {
    Serial.print(LANGUAGE ? "Received " + String(numNewMessages) + " new messages." : String(numNewMessages) + " Neue Nachrichten erhalten.");

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

        String from_name = bot.messages[i].from_name;

        if (text == "/start") {
        String welcome = "Willkommen, " + from_name + ".\n";
        welcome += "Benutze einen der folgenden Befehle:\n\n";
        welcome += "/status gibt einen Status-Bericht aus\n";
        welcome += "/sensors gibt die aktuellen Sensor-werte aus\n\n";
        welcome += "Funktionen, die einen RFID-Tag benötigen:\n";
        welcome += "/arm aktiviert die Alarmanlage\n";
        welcome += "/disarm deaktiviert die Alarmanlage\n";
        welcome += "/deactivate stoppt einen aktivierten Alarm und deaktiviert die Alarmanlage";
        bot.sendMessage(chatID, welcome, "");
        }

        
        if (text == "/status") {
            String activated = ARMED ? "Ja" : "Nein";

            String message = "Status-Bericht: \n";
            message += "Aktiviert: " + activated + " \n";
            message += "Letzte Status-Änderung: " + last_status_change + " \n";
            message += "Tags eingespeichert: " + String(tag_amount()) + "\n";
            bot.sendMessage(chatID, message, "");
        }
        
        if (text == "/sensors") {
            String message = "Sensor-Werte: \n";
            message += "Temperatur: " + String(temperature, 1) + " °C\n";
            message += "Luftfeuchtigkeit: " + String(humidity, 0) + " %\n";
            message += "Luftdruck: " + String(pressure, 2) + " hPa\n";
            message += "Höhe: " + String(height, 2) + " m\n";
            bot.sendMessage(chatID, message, "");
        }
    }
}

void sendLogMessage(String message) {
    bot.sendMessage((String) *logging_chat_ids, message, "");
}

#pragma endregion

// --------- SENSOREN --------


// --------- JOBS UND INTERRUPTS---------

#pragma region jobs

// lcd-aktualisierung
void updateLCD() {
    IPAddress ip = WiFi.localIP();
    // Zeile 1: aktuelle Zeit
    lcd.setCursor(0, 0);
    lcd.print(currentTime());

    // Zeile 2: Status
    clearLine(1);
    lcd.setCursor(0, 1);
    switch (status) {
        case 0:
            lcd.print(String(temperature));
            lcd.print("C, ");
            lcd.print(String(humidity));
            lcd.print("%");
            break;
        case 1:
            lcd.print("IP: ");
            lcd.print(ip.toString());
            break;
        case 2:
            lcd.print(String(pressure));
            lcd.print("hPa , ");
            lcd.print(String(height));
            lcd.print("m");
            break;
    }

    if (ALARM & ARMED) {
        Serial.println(LANGUAGE ? "Alarm triggered!" : "Alarm ausgelöst!");

        clearLine(2);
        lcd.setCursor(0, 2);
        lcd.print(LANGUAGE ? "ALARM TRIGGERED" : "  ALARM AUSGELOEST  ");
    }

    // Zeile 3: Alarmstatus
    //lcd.setCursor(0, 2);
    //lcd.print("                    "); // zeile löschen

    // Zeile 4
    //lcd.setCursor(0, 3);
    //lcd.print("                    "); // zeile löschen

    //lcdTask = NULL;
    //vTaskDelete(NULL);
}

void updateSensorData() {
    float new_pressure = bmp.readPressure()/100;
    float new_height = bmp.readAltitude(1013.25);

    if (isnan(new_pressure) || isnan(new_height)) {
        Serial.println(LANGUAGE ? "Failed to read from BMP280 sensor!" : "Konnte nicht vom BMP280-Sensor lesen");
    }
    else {
        pressure = new_pressure;
        height = new_height;
    }

    float new_temperature = dht.readTemperature();
    float new_humidity = dht.readHumidity();

    if (isnan(new_temperature) || isnan(new_humidity)) {
        Serial.println(LANGUAGE ? "Failed to read from DHT sensor!" : "Konnte nicht vom DHT-Sensor lesen");
    }
    else {
        temperature = new_temperature;
        humidity = new_humidity;
    }

    //sensorTask = NULL;
    //vTaskDelete(NULL);
}

void armTaskHandle(void * pvParameters) {
    Serial.println(LANGUAGE ? "Started Arming process on core" + String(xPortGetCoreID()) : "Scharfschaltungs-Prozess startet auf Core" + String(xPortGetCoreID()));

    String action_message = ARMED ? (LANGUAGE ? "Disarming ": "Deaktivierungs") : (LANGUAGE ? "Arming " : "Aktivierungs");
    String process_message = LANGUAGE ? " process started" : "-prozess gestartet";
    String rfid_message = LANGUAGE ? "Hold your RFID-tag on the reader to confirm!" : "Bitte RFID-Tag an den Leser halten zur Bestätigung!";

    Serial.println(action_message + process_message);
    Serial.println(rfid_message);

    clearLine(2);
    lcd.setCursor(0, 2);
    lcd.print(LANGUAGE ? "Please hold RFID-tag" : "Bitte RFID-Tag an");
    clearLine(3);
    lcd.setCursor(0, 3);
    lcd.print(LANGUAGE ? "on the reader." : "den Leser halten");

    bool tag_verified = search_tag();

    clearLine(2);
    clearLine(3);
    lcd.setCursor(0,2);

    if (tag_verified) {
        String message = LANGUAGE ? "Tag verified." : "Tag verifiziert.";
        lcd.print(message);
        Serial.println(message);

        vTaskDelay(500);
        clearLine(2);
        clearLine(3);
        lcd.setCursor(0, 2);

        if (ARMED) {
            lcd.print(LANGUAGE ? "Disarmed." : "Deaktiviert.");
            Serial.println(LANGUAGE ? "Alarm system disarmed." : "Alarmanlage erfolgreich deaktiviert.");

            ARMED = false;
            detachInterrupt(digitalPinToInterrupt(photoelectric_sensor_pin));
        }
        else {
            for (int i = alarm_activation_time; i > 0; i--) {
                Serial.println(LANGUAGE ? ("Activating in " + String(i)) : ("Alarmanlage wird aktiviert in " + String(i)));
                lcd.print(LANGUAGE ? ("Activating in " + String(i)) : ("Aktivierung in " + String(i)));
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            ARMED = true;

            attachInterrupt(digitalPinToInterrupt(photoelectric_sensor_pin), &handlePhotoelectricInterrupt, RISING);

        }
        String last_status_change = currentTime();
        
    }
    else {
        String message = LANGUAGE ? "Wrong tag!" : "Falscher Tag!";
        lcd.print(message);
        Serial.println(message);
        vTaskDelay(500);
    }

    armTask = NULL;
    vTaskDelete(NULL);

}

void resetTaskHandle(void * pvParameters) {


    armTask = NULL;
    vTaskDelete(NULL);
}

#pragma endregion

#pragma region Interrupts

void IRAM_ATTR handleArmButtonInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_arm_interrupt_time > debounce_time) {
        if (armTask == NULL) {
            xTaskCreatePinnedToCore(armTaskHandle, "armTask", 5000, NULL, 0, &armTask, 0);
        }
        last_arm_interrupt_time = interruptTime;
    }
}

void IRAM_ATTR handleResetAlarmButtonInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_reset_interrupt_time > debounce_time) {
        if (resetTask == NULL) {
            xTaskCreatePinnedToCore(resetTaskHandle, "resetTask", 5000, NULL, 0, &resetTask, 0);
        }
        last_reset_interrupt_time = interruptTime;
    }
}

void IRAM_ATTR handleStatusButtonInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_status_interrupt_time > debounce_time) {
        status += 1;
        if (status > 2) status = 0;

        last_status_interrupt_time = interruptTime;
    }
}

void IRAM_ATTR handleTagButtonInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_tag_interrupt_time > debounce_time) {

        last_tag_interrupt_time = interruptTime;
    }
    
}

void IRAM_ATTR handleLanguageButtonInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_language_interrupt_time > debounce_time) {
        LANGUAGE = !LANGUAGE;

        last_language_interrupt_time = interruptTime;
    }
}

void IRAM_ATTR handlePhotoelectricInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_photoelectric_interrupt_time > sensor_debounce_time) {
        ALARM = true;

        last_photoelectric_interrupt_time = interruptTime;
    }
}

#pragma endregion

// --------- EINRICHTUNG ---------
#pragma region setup

void initial_setup(bool noFile) {
    if (noFile) {
        if (!SPIFFS.exists("/rfid")) {
            if (!SPIFFS.mkdir("/rfid")) {
                Serial.println(LANGUAGE ? "Failed to create folder rfid!" : "Ordner rfid konnte nicht erstellt werden!");
                for (;;);
            }
            File file = SPIFFS.open("/rfid/rfid_tags.txt", "w");
            if (!file) {
                Serial.println(LANGUAGE ? "Failed to create file rfid_flags.txt!" : "Datei rfid_flags.txt konnte nicht erstellt werden!");
                for (;;);
            }
            file.close();
            Serial.println(LANGUAGE ? "Successfully created rfid_tags.txt" : "rfid_tags.txt wurde erfolgreich erstellt");
        }
    }
    
    
    save_tag();
}

void setup() {
    // Initialisieren des Seriellen Monitors
    Serial.begin(115200);

    Serial.println(LANGUAGE ? "Setup running on Core " + String(xPortGetCoreID()) : "Setup läuft auf Core " + String(xPortGetCoreID()));

    // Mit WLAN verbinden
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println(LANGUAGE ? "Connecting to WiFi..." : "Verbindung zu WiFi wird hergestellt...");
    }
    Serial.println(LANGUAGE ? "Connected to WiFi" : "Mit WiFi verbunden");

    // Root-Zertifikat für SSL hinzufügen (Telegram-Bot)
    client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
    delay(500);
    
    // Buttons als input konfigurieren
    pinMode(arm_button_pin, INPUT_PULLUP);
    pinMode(reset_alarm_button_pin, INPUT_PULLUP);
    pinMode(status_button_pin, INPUT_PULLUP);
    pinMode(tag_button_pin, INPUT_PULLUP);
    pinMode(language_button_pin, INPUT_PULLUP);

    pinMode(photoelectric_sensor_pin, INPUT);

    // Interrupts für Buttons hinzufügen
    attachInterrupt(digitalPinToInterrupt(arm_button_pin), &handleArmButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(reset_alarm_button_pin), &handleResetAlarmButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(status_button_pin), &handleStatusButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(tag_button_pin), &handleTagButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(language_button_pin), &handleLanguageButtonInterrupt, FALLING);

    // Initialisieren des I2C-Busses
    Wire.begin(mainSDA, mainSCL);

    // Initialisieren des LCD-Displays
    lcd.init();
    lcd.clear();
    lcd.backlight();

    // Initialisieren des SPI-Busses
    SPI.begin(vspi_clk_pin, vspi_miso_pin, vspi_mosi_pin, vspi_ss_pin);

    // RTC initialisieren und mit NTP synchronisieren
    if (!rtc.begin(&Wire)) {
        Serial.println(LANGUAGE ? "Could not find valid rtc, check wiring!" : "RTC nicht gefunden, bitte Verkabelung überprüfen!");
    }

    timeClient.begin();
    timeClient.update();
    rtc.adjust(DateTime(timeClient.getEpochTime()));
    Serial.println(LANGUAGE ? "Initialized RTC, current time: " + String(currentTime()) : "RTC initialisiert, aktuelle Zeit: " + String(currentTime()));

    if (!SPIFFS.begin(true)) {
        Serial.println(LANGUAGE ? "Failed to mount file system" : "Dateisystem konnte nicht gemountet werden");
        return;
    }

    // Sensoren
    // BMP280 (luftdruck)
    if (!bmp.begin()) {
        Serial.println(LANGUAGE ? "Could not find a valid BMP280 !" : "Es konnte kein BMP280 gefunden werden!");
    }

    // DHT11 (Temperatur, Luftfeuchtigkeit)
    dht.begin();


    rfid_init();


    //Serial.println("lcdJob");
    //xTaskCreatePinnedToCore(lcdJob, "lcdJob", 16384, NULL, 1, &lcdTask, 1);
    //delay(2000);
    //Serial.println("sensorJob");
    //xTaskCreatePinnedToCore(sensorJob, "sensorJob", 16384, NULL, 1, &sensorTask, 1);
    //delay(2000);

    //delay(10000);

    sendLogMessage("Startup finished.");

    Serial.println(LANGUAGE ? "Finished Setup" : "Einrichtung abgeschlossen");
}

#pragma endregion

// --------- HAUPT-PROGRAMM ---------
#pragma region loop

void loop() {
    Serial.println("telegram");
    if (millis() > last_bot_refresh + bot_request_delay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
    handleNewMessages(numNewMessages);
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    last_bot_refresh = millis();
    }

    Serial.println("lcdJob");
    //xTaskCreatePinnedToCore(lcdJob, "lcdJob", 16384, NULL, 1, &lcdTask, 1);
    updateLCD();

    Serial.println("sensorJob");
    //xTaskCreatePinnedToCore(sensorJob, "sensorJob", 16384, NULL, 1, &sensorTask, 1);
    updateSensorData();

    //rtc_wdt_feed();
    loop_ticks++;
    Serial.println(loop_ticks);
    delay(500);
}

#pragma endregion
