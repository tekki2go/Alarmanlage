// Alarmanlage by Marc Heß

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
// vermeidung von Race Conditions
#include <mutex>

#pragma endregion

#pragma region important_settings

// Netzwerkdaten
// zuhause
//const char* ssid = "tkNOC_IoT";
//const char* password = "Q9ya&RUxDuVw&A$$w4ZNmkQMNTyKE9ZU";

// Laptop
//const char* ssid = "WIN-452L88FNHCT 3783";
//const char* password = "523q8|1B";

// Handy
//const char* ssid = "Eierfon 13";
//const char* password = "xQA_52Le";

// Klassenzimmer
const char* ssid = "IoT_Elektro";
const char* password = "DIoT-UadKASmS";

// Token des Telegram-bots
const char* bot_token = "6923232009:AAGqJnmC9oe915CyHLO3OMCKHy9U8muVG1Q";

// Hier die Chat IDs eintragen (kann man per @myidbot herausfinden)
// Außerdem muss man "Start" auf dem Bot klicken bevor er Nachrichten versenden kann
String chat_ids[] = {
    "5639436151"
};

// Leiser Modus (deaktiviert Buzzer)
bool silent = false;

#pragma endregion

#pragma region settings

// Sensor-Sensibilität
const int pressure_tolerance = 10; //Luftdruck-Toleranz in hPa

// Timings
int alarm_activation_time = 15;        // Zeit in Sekunden, bis die Alarmanlage aktiviert wird
int debounce_time = 500;               // Button-Interrupts dürfen nur alle 500ms getriggert werden

// NTP Server Einstellungen
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;      // GMT-Versatz in Sekunden
const int daylightOffset_sec = 3600;  // Sommerzeit-Versatz in Sekunden

// I2C-Schnittstelle für RTC, LCD und Sensoren (mainWire)
const int mainSDA = 21;
const int mainSCL = 22;
const int mainClockSpeed = 100000; // 100kHz Taktgeschwindigkeit

// RFID-Lesegerät Einstellungen
const int vspi_mosi_pin = 18;
const int vspi_miso_pin = 19;
const int vspi_clk_pin = 5;
const int vspi_ss_pin = 17;
const int rfid_rst_pin = 23;

// Pin-Einstellungen
const int piezo_pin = 4;                 // Piezo-Piepser
const int led_red_pin = 2;               // Rote LED
const int led_green_pin = 0;             // Grüne LED

const int arm_button_pin = 12;           // Taster 1
const int reset_alarm_button_pin = 14;   // Taster 2
const int status_button_pin = 27;        // Taster 3
const int tag_button_pin = 25;           // Taster 4
const int language_button_pin = 32;      // Taster 5

const int photoelectric_sensor_pin = 13; // Lichtschranke
const int radar_sensor_pin = 35;         // Radar
const int microphone_pin = 34;           // Mikrofon
const int dht11_pin = 33;                // DHT11 33
const int movement_sensor_pin = 26;      // Bewegungsmelder 26

#pragma endregion

#pragma region inits

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
TaskHandle_t armTask = NULL;
TaskHandle_t resetTask = NULL;
TaskHandle_t telegramTask = NULL;
TaskHandle_t outputTask = NULL;
TaskHandle_t saveTagTask = NULL;

// Timer handles
esp_timer_handle_t timer_handle_fast;         // alle 100ms

// Semaphoren
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t lcdMutex;
SemaphoreHandle_t telegramMutex;
//std::mutex telegram_mtx;
//std::mutex i2c_mtx;

#pragma endregion

#pragma region variables
// Variablen
bool ARMED = false;    // true = scharfgeschaltet
bool ALARM = false;    // true = alarm ausgelöst
bool LANGUAGE = false; // false = deutsch, true = englisch

bool ALARM_TRIGGERED[4];
bool PREVIOUS_ALARM[4];
// 0 Bewegungsmelder-alarm
// 1 radar-alarm
// 2 Mikrofon-alarm
// 3 Lichtschranken-alarm

float temperature;     // aktuelle Temperatur
float humidity;        // aktuelle Luftfeuchtigkeit
float pressure;        // aktueller Luftdruck
float height;          // aktuelle Höhe (m)

int status = 0;        // was gerade in der Status-Anzeige gezeigt wird
// 0 = Temperatur + Luftfeuchtigkeit, 1 = IP, 2 = Luftdruck + höhe

// debounce timings
volatile unsigned long last_arm_interrupt_time = 0;
volatile unsigned long last_reset_interrupt_time = 0;
volatile unsigned long last_status_interrupt_time = 0;
volatile unsigned long last_tag_interrupt_time = 0;
volatile unsigned long last_language_interrupt_time = 0;

// variable zur verhinderung von mehreren Alarm-Nachrichten per Telegram und Seriellem monitor
bool alarmMessageSent = false;

// variable zur deaktivierung der Buttons
bool buttons_deactivated = false;

// enthält den Zeitstempel der letzten Status-Änderung
String last_status_change;

// Variable für Dateien
File whitelistFile;

// Timer variablen
volatile uint timer_100ms_count = 0;

// Funktionen
void initial_setup(bool noFile);
void sendLogMessage(String message);
void IRAM_ATTR handlePhotoelectricInterrupt();
void IRAM_ATTR handleRadarInterrupt();
void IRAM_ATTR handleMicrophoneInterrupt();
void IRAM_ATTR handleMovementInterrupt();

#pragma endregion

// --------- FUNKTIONEN ---------

void clearLine(int line) {
    lcd.setCursor(0, line);
    lcd.print("                    ");
}

String currentTime() {
    if (xSemaphoreTake(i2cMutex ,portMAX_DELAY)) {
        DateTime now = rtc.now();
        for (int i = 0; i < 6; i++) {
            char buffer[30]; // Puffer für formatierte Zeit
            sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
            if (strlen(buffer) == 19) {
                xSemaphoreGive(i2cMutex);
                return String(buffer);
            }
            delay(25);
        }
    } xSemaphoreGive(i2cMutex);
    return "failed";
}

void dht11Task() {
    float new_temperature = dht.readTemperature();
    float new_humidity = dht.readHumidity();

    if (isnan(new_temperature) || isnan(new_humidity)) {
        Serial.println(LANGUAGE ? "Failed to read from DHT sensor!" : "Konnte nicht vom DHT-Sensor lesen");
    }
    else {
        temperature = new_temperature;
        humidity = new_humidity;
    }
}

void attachInterrupts() {
    attachInterrupt(digitalPinToInterrupt(photoelectric_sensor_pin), &handlePhotoelectricInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(movement_sensor_pin), &handleMovementInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(radar_sensor_pin), &handleRadarInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(microphone_pin), &handleMicrophoneInterrupt, FALLING);
}

void detachInterrupts() {
    detachInterrupt(digitalPinToInterrupt(photoelectric_sensor_pin));
    detachInterrupt(digitalPinToInterrupt(movement_sensor_pin));
    detachInterrupt(digitalPinToInterrupt(radar_sensor_pin));
    detachInterrupt(digitalPinToInterrupt(microphone_pin));
}

void resetAlarms() {
    
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

void saveTagTaskHandle(void * pvParameters) {

    if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
        lcd.setCursor(0, 2);
        lcd.print(LANGUAGE ? "Please hold an RFID" : " Bitte den RFID-Tag ");
        lcd.setCursor(0, 3);
        lcd.print(LANGUAGE ? "tag to the reader!" : "an den Leser halten!");
    }
    xSemaphoreGive(lcdMutex);

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

                if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
                    clearLine(2);
                    clearLine(3);
                    lcd.setCursor(0, 2);
                    lcd.print(LANGUAGE ? "RFID-Tag saved." : "RFID-Tag gespeichert");
                }
                xSemaphoreGive(lcdMutex);
                delay(1000);
                if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
                    clearLine(2);
                    clearLine(3);
                }
                xSemaphoreGive(lcdMutex);
            } else {
                Serial.println(LANGUAGE ? "Failed to open whitelist file for writing." : "Fehler beim Öffnen der Whitelist-Datei zum Schreiben.");
                if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
                    clearLine(2);
                    clearLine(3);
                    lcd.setCursor(0, 2);
                    lcd.print(LANGUAGE ? "Error while saving!" : "Fehler beim Speichern!");
                }
                xSemaphoreGive(lcdMutex);
            }
            buttons_deactivated = false;
            saveTagTask = NULL;
            vTaskDelete(NULL);
            return;
        }
    }
    buttons_deactivated = false;
    saveTagTask = NULL;
    vTaskDelete(NULL);
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

// --------- JOBS UND INTERRUPTS---------

#pragma region jobs

void armTaskHandle(void * pvParameters) {
    Serial.println(LANGUAGE ? "Started Arming process on core " + String(xPortGetCoreID()) : "Scharfschaltungs-Prozess startet auf Core " + String(xPortGetCoreID()));

    String action_message = ARMED ? (LANGUAGE ? "Disarming ": "Deaktivierungs") : (LANGUAGE ? "Arming " : "Aktivierungs");
    String process_message = LANGUAGE ? " process started" : "-prozess gestartet";
    String rfid_message = LANGUAGE ? "Hold your RFID-tag on the reader to confirm!" : "Bitte RFID-Tag an den Leser halten zur Bestätigung!";

    sendLogMessage(String(action_message + process_message));
    Serial.println(action_message + process_message);
    Serial.println(rfid_message);

    if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
        clearLine(2);
        lcd.setCursor(0, 2);
        lcd.print(LANGUAGE ? "Please hold RFID-tag" : "Bitte RFID-Tag an");
        clearLine(3);
        lcd.setCursor(0, 3);
        lcd.print(LANGUAGE ? "on the reader." : "den Leser halten");
    } xSemaphoreGive(lcdMutex);

    bool tag_verified = search_tag();

    if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
        clearLine(2);
        clearLine(3);
        lcd.setCursor(0,2);
    }
    xSemaphoreGive(lcdMutex);

    if (tag_verified) {
        String message = LANGUAGE ? "Tag verified." : "Tag verifiziert.";
        sendLogMessage(message);
        if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
            lcd.print(message);
            Serial.println(message);
        } xSemaphoreGive(lcdMutex);

        vTaskDelay(pdMS_TO_TICKS(500));

        if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
            clearLine(2);
            clearLine(3);
            lcd.setCursor(0, 2);
        } xSemaphoreGive(lcdMutex);

        if (ARMED) {
            if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
                lcd.print(LANGUAGE ? "Disarmed." : "Deaktiviert.");
            } xSemaphoreGive(lcdMutex);
            Serial.println(LANGUAGE ? "Alarm system disarmed." : "Alarmanlage erfolgreich deaktiviert.");
            sendLogMessage("Alarmanlage erfolgreich deaktiviert.");

            ARMED = false;
            // ALARM_TRIGGERED[4] = { false };
            // PREVIOUS_ALARM[4] = { false };
            
            for (auto& alarm : ALARM_TRIGGERED) {
                alarm = false;
            }
            for (auto& alarm : PREVIOUS_ALARM) {
                alarm = false;
            }
            
            last_status_change = currentTime();
            detachInterrupts();
            digitalWrite(led_green_pin, HIGH);
            digitalWrite(led_red_pin, LOW);
            digitalWrite(piezo_pin, LOW);
        }
        else {
            if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
                sendLogMessage("Alarmanlage wird aktiviert in " + String(alarm_activation_time) + " sekunden.");
                for (int i = alarm_activation_time; i > 0; i--) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    String j = String(i);
                    if (sizeof(j) < 16) {j += " ";}
                    Serial.println(LANGUAGE ? ("Activating in " + j) : ("Alarmanlage wird aktiviert in " + j));
                    clearLine(2);
                    lcd.setCursor(0, 2);
                    lcd.print(LANGUAGE ? ("Activating in " + j) : ("Aktivierung in " + j));
                    digitalWrite(led_green_pin, !digitalRead(led_green_pin));
                }
                clearLine(2);
                clearLine(3);
            } xSemaphoreGive(lcdMutex);
            last_status_change = currentTime();
        
            digitalWrite(led_green_pin, LOW);
            digitalWrite(led_red_pin, HIGH);
            ARMED = true;

            attachInterrupts();
        }
        String last_status_change = currentTime();
        
    }
    else {
        String message = LANGUAGE ? "Wrong tag!" : "Falscher Tag!";
        if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
            lcd.print(message);
        } xSemaphoreGive(lcdMutex);
        sendLogMessage(message);
        Serial.println(message);
        vTaskDelay(500);
    }

    buttons_deactivated = false;
    armTask = NULL;
    vTaskDelete(NULL);
}

void resetTaskHandle(void * pvParameters) {
    Serial.println(LANGUAGE ? "Started Resetting process on core" + String(xPortGetCoreID()) : "Alarmstoppungs-Prozess startet auf Core" + String(xPortGetCoreID()));

    if (!ALARM) {
        Serial.println(LANGUAGE ? "No Alarm active!" : "Kein Alarm aktiv!");
        sendLogMessage("Kein Alarm aktiv!");
        armTask = NULL;
        vTaskDelete(NULL);
    }

    String action_message = ARMED ? (LANGUAGE ? "Disarming ": "Deaktivierungs") : (LANGUAGE ? "Arming " : "Aktivierungs");
    String process_message = LANGUAGE ? " process started" : "-prozess gestartet";
    String rfid_message = LANGUAGE ? "Hold your RFID-tag on the reader to confirm!" : "Bitte RFID-Tag an den Leser halten zur Bestätigung!";

    Serial.println(action_message + process_message);
    Serial.println(rfid_message);

    if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
        clearLine(2);
        lcd.setCursor(0, 2);
        lcd.print(LANGUAGE ? "Please hold RFID-tag" : "Bitte RFID-Tag an");
        clearLine(3);
        lcd.setCursor(0, 3);
        lcd.print(LANGUAGE ? "on the reader." : "den Leser halten");
    } xSemaphoreGive(lcdMutex);

    bool tag_verified = search_tag();

    if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
        clearLine(2);
        clearLine(3);
        lcd.setCursor(0,2);
    } xSemaphoreGive(lcdMutex);

    if (tag_verified) {
        detachInterrupts();

        ARMED = false;
        ALARM = false;
        alarmMessageSent = false;

        for (auto& alarm : ALARM_TRIGGERED) {
            alarm = false;
        }
        for (auto& alarm : PREVIOUS_ALARM) {
            alarm = false;
        }

        ALARM_TRIGGERED[4] = { false };
        PREVIOUS_ALARM[4] = { false };

        digitalWrite(piezo_pin, LOW);
        digitalWrite(led_green_pin, HIGH);
        digitalWrite(led_red_pin, LOW);

        if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
            lcd.clear();

            clearLine(2);
            clearLine(3);

            sendLogMessage("Alarmanlage wird reaktiviert in " + String(alarm_activation_time) + " sekunden.");

            for (int i = alarm_activation_time; i > 0; i--) {
                String j = String(i);
                    if (sizeof(j) < 16) {j += " ";}
                Serial.println(LANGUAGE ? ("Reactivating in " + j) : ("Reaktivierung in " + j));
                clearLine(2);
                lcd.setCursor(0, 2);
                lcd.print(LANGUAGE ? ("Reactivating in " + j) : ("Reaktivierung in " + j));
                digitalWrite(led_green_pin, !digitalRead(led_green_pin));
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            clearLine(2);
            clearLine(3);

        } xSemaphoreGive(lcdMutex);

        last_status_change = currentTime();

        digitalWrite(led_green_pin, LOW);
        digitalWrite(led_red_pin, HIGH);
        ARMED = true;

        attachInterrupts();

    }
    else {
        String message = LANGUAGE ? "Wrong tag!" : "Falscher Tag!";
        if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
            lcd.print(message);
        } xSemaphoreGive(lcdMutex);
        sendLogMessage(message);
        Serial.println(message);
        vTaskDelay(500);
    }

    buttons_deactivated = false;
    resetTask = NULL;
    vTaskDelete(NULL);
}

// laufen immer
// Outputs (LEDs, Piezo)
void outputJob(void * pvParameters) {
    uint counter = 0;
    for (;;) {
        if (ALARM & ARMED) {
            digitalWrite(led_green_pin, LOW);
            if (!silent && counter % 2 == 0) {
                digitalWrite(piezo_pin, !digitalRead(piezo_pin));
            }

            digitalWrite(led_red_pin, !digitalRead(led_red_pin));

            counter++;
        }
        delay(500);
    }
}

// lcd-aktualisierung und alarm
void updateLCD() {
    IPAddress ip = WiFi.localIP();
    if (xSemaphoreTake(lcdMutex ,portMAX_DELAY)) {
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
                lcd.print("hPa, ");
                lcd.print(String(height));
                lcd.print("m");
                break;
        }

        if (ALARM & ARMED) {
            String message;
            if (ALARM_TRIGGERED[0]) message += "MOV "; // Movement = Bewegung
            if (ALARM_TRIGGERED[1]) message += "RAD "; // Radar
            if (ALARM_TRIGGERED[2]) message += "MIC "; // Microphone = Mikrofon
            if (ALARM_TRIGGERED[3]) message += "BEAM"; // Beam = Lichtschranke

            if (!alarmMessageSent) {
                Serial.println(LANGUAGE ? "Alarm triggered!" : "Alarm ausgelöst!");
                sendLogMessage("ALARM AUSGELÖST!!!");
                alarmMessageSent = true;

                clearLine(2);
                lcd.setCursor(0, 2);
                lcd.print(LANGUAGE ? "ALARM TRIGGERED" : "  ALARM AUSGELOEST  ");
            }

            clearLine(3);
            lcd.setCursor(0, 3);
            lcd.print(message);

            bool newAlarmTriggered = false;
            message = "";
            for (int i = 0; i < sizeof(ALARM_TRIGGERED); i++) {
                if (ALARM_TRIGGERED[i] && !PREVIOUS_ALARM[i]) {
                    newAlarmTriggered = true;
                    PREVIOUS_ALARM[i] = true;

                    switch (i) {
                        case 0:
                        message += "Bewegungsmelder ";
                        break;
                    case 1:
                        message += "Radar ";
                        break;
                    case 2:
                        message += "Mikrofon ";
                        break;
                    case 3:
                        message += "Lichtschranke";
                        break;
                    }
                }
            }

            if (newAlarmTriggered) {
                sendLogMessage("Neuer Alarm ausgelöst: " + message);
                Serial.println("Neuer Alarm ausgelöst: " + message);
            }
        }
    } xSemaphoreGive(lcdMutex);
}

void updateBMP280Data() {
    if (xSemaphoreTake(i2cMutex ,portMAX_DELAY)) {
        float new_pressure = bmp.readPressure()/100;
        float new_height = bmp.readAltitude(1013.25);

        if (isnan(new_pressure) || isnan(new_height)) {
        Serial.println(LANGUAGE ? "Failed to read from BMP280 sensor!" : "Konnte nicht vom BMP280-Sensor lesen");
        }
        else {
            pressure = new_pressure;
            height = new_height;
        }
    } xSemaphoreGive(i2cMutex);
    delay(50);
}

#pragma endregion

#pragma region Interrupts

// --- BUTTONS ---

void IRAM_ATTR handleArmButtonInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_arm_interrupt_time > debounce_time) {
        if (armTask == NULL && !buttons_deactivated) {
            buttons_deactivated = true;
            xTaskCreatePinnedToCore(armTaskHandle, "armTask", 8192, NULL, 0, &armTask, 0);
        }
        last_arm_interrupt_time = interruptTime;
    }
}

void IRAM_ATTR handleResetAlarmButtonInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_reset_interrupt_time > debounce_time) {
        if (resetTask == NULL && !buttons_deactivated) {
            buttons_deactivated = true;
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

        if (saveTagTask == NULL && !buttons_deactivated) {
            buttons_deactivated = true;
            xTaskCreatePinnedToCore(saveTagTaskHandle, "saveTagTask", 5000, NULL, 0, &saveTagTask, 0);
        }
    }
    
}

void IRAM_ATTR handleLanguageButtonInterrupt() {
    unsigned long interruptTime = millis();

    if (interruptTime - last_language_interrupt_time > debounce_time) {
        LANGUAGE = !LANGUAGE;
        last_language_interrupt_time = interruptTime;
    }
}

// --- SENSOREN ---

void IRAM_ATTR handlePhotoelectricInterrupt() {
    ALARM_TRIGGERED[3] = true;
    ALARM = true;
}

void IRAM_ATTR handleRadarInterrupt() {
    ALARM_TRIGGERED[1] = true;
    ALARM = true;
}

void IRAM_ATTR handleMicrophoneInterrupt() {
    ALARM_TRIGGERED[2] = true;
    ALARM = true;
}

void IRAM_ATTR handleMovementInterrupt() {
    ALARM_TRIGGERED[0] = true;
    ALARM = true;
}

#pragma endregion

// --------- TELEGRAM-BOT --------
#pragma region telegram

void handleNewMessages(int numNewMessages) {
    Serial.println(LANGUAGE ? "Received " + String(numNewMessages) + " new messages." : String(numNewMessages) + " Neue Nachrichten erhalten.");

    bool is_authorized = false;

    for (int i=0; i<numNewMessages; i++) {
        // Chat id of the requester
        String chatID = String(bot.messages[i].chat_id);
        for (int j = 0; j < sizeof(chat_ids); j++) {
            if (chatID == chat_ids[i]) {
                is_authorized = true;
                break;
            }
        }

        if (!is_authorized) {
            bot.sendMessage(chatID, "Unauthorized user", "");
            continue;
        }
        
        // Print the received message
        String text = bot.messages[i].text;

        String from_name = bot.messages[i].from_name;

        // Startnachricht
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
        // Statusnachricht
        else if (text == "/status") {
            String activated = ARMED ? "Ja" : "Nein";

            String message = "Status-Bericht: \n";
            message += "Aktiviert: " + activated + " \n";
            if (last_status_change != "") {message += "Letzte Status-Änderung: " + last_status_change + " \n";}
            else {message += "Letzte Status-Änderung: unbekannt\n";}
            message += "Tags eingespeichert: " + String(tag_amount()) + "";
            bot.sendMessage(chatID, message, "");
        }

        // Sensorabfrage
        else if (text == "/sensors") {
            String message = "Sensor-Werte: \n";
            message += "Temperatur: " + String(temperature, 1) + " °C\n";
            message += "Luftfeuchtigkeit: " + String(humidity, 0) + " %\n";
            message += "Luftdruck: " + String(pressure, 2) + " hPa\n";
            message += "Höhe: " + String(height, 2) + " m";
            bot.sendMessage(chatID, message, "");
        }

        // Aktivierung der Alarmanlage
        else if (text == "/arm") {
            if (ARMED == false) {
                if (armTask == NULL) {
                    bot.sendMessage(chatID, "Bitte RFID-Tag an leser halten! \n weitere Instruktionen werden auf dem LCD-Display erscheinen.", "");
                    xTaskCreatePinnedToCore(armTaskHandle, "armTask", 8192, NULL, 0, &armTask, 0);
                }
                else {
                    bot.sendMessage(chatID, "Alarmanlage wird gerade bereits aktiviert!", "");
                }
            }
            else {
                bot.sendMessage(chatID, "Alarmanlage ist schon aktiviert!", "");
            }
        }

        // Deaktivierung der Alarmanlage
        else if (text == "/disarm") {
            if (ARMED == true) {
                if (armTask == NULL) {
                    bot.sendMessage(chatID, "Bitte RFID-Tag an leser halten! \n weitere Instruktionen werden auf dem LCD-Display erscheinen.", "");
                    xTaskCreatePinnedToCore(armTaskHandle, "armTask", 8192, NULL, 0, &armTask, 0);
                }
                else {
                    bot.sendMessage(chatID, "Alarmanlage wird gerade bereits deaktiviert!", "");
                }
            }
            else {
                bot.sendMessage(chatID, "Alarmanlage ist schon deaktiviert!", "");
            }
        }

        // Abbrechen eines Alarms
        else if (text == "/deactivate") {
            if (ALARM == true) {
                if (armTask == NULL) {
                    bot.sendMessage(chatID, "Bitte RFID-Tag an leser halten! \n weitere Instruktionen werden auf dem LCD-Display erscheinen.", "");
                    xTaskCreatePinnedToCore(resetTaskHandle, "resetTask", 5000, NULL, 0, &resetTask, 0);
                }
                else {
                    bot.sendMessage(chatID, "Alarm wird bereits beendet!", "");
                }
            }
            else {
                bot.sendMessage(chatID, "Kein Alarm ausgelöst!", "");
            }
        }

        else {
            bot.sendMessage(chatID, "Unbekannter befehl!", "");
        }
    }
}

void telegramTaskHandle(void * pvParameters) {
    if (xSemaphoreTake(telegramMutex, portMAX_DELAY)) {
        int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

        while(numNewMessages) {
            handleNewMessages(numNewMessages);
            numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        }
    }
    xSemaphoreGive(telegramMutex);
    
    telegramTask = NULL;
    vTaskDelete(NULL);
}

void sendLogMessage(String message) {
    if (xSemaphoreTake(telegramMutex, portMAX_DELAY)) {
        bot.sendMessage((String) *chat_ids, message, "");
    }
    xSemaphoreGive(telegramMutex);
}

#pragma endregion

// --------- TIMER ---------
#pragma region timers
void timer_100ms_callback(void * pvParameters) {
    timer_100ms_count++;
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
    
    
    xTaskCreatePinnedToCore(saveTagTaskHandle, "saveTagTask", 5000, NULL, 0, &saveTagTask, 0);
}

void setup() {
    // Initialisieren des Seriellen Monitors
    Serial.begin(115200);

    Serial.println(LANGUAGE ? "Setup running on Core " + String(xPortGetCoreID()) : "Setup läuft auf Core " + String(xPortGetCoreID()));

    // Mit WLAN verbinden
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println(LANGUAGE ? ("Connecting to " + String(ssid) + "...") : ("Verbindung zu " + String(ssid) + " wird hergestellt..."));
    }
    Serial.println(LANGUAGE ? "Connected to WiFi" : "Mit WiFi verbunden");

    // Root-Zertifikat für SSL hinzufügen (Telegram-Bot)
    client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
    delay(500);

    // Semaphoren
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {Serial.println(LANGUAGE ? "Error while creating I2C-Semaphore" : "Fehler beim Erstellen der I2C-Semaphore");}
    lcdMutex = xSemaphoreCreateMutex();
    if (lcdMutex == NULL) {Serial.println(LANGUAGE ? "Error while creating LCD-Semaphore" : "Fehler beim Erstellen der LCD-Semaphore");}
    telegramMutex = xSemaphoreCreateMutex();
    if (telegramMutex == NULL) {Serial.println(LANGUAGE ? "Error while creating Telegram-Semaphore" : "Fehler beim Erstellen der Telegram-Semaphore");}
    
    // Buttons als input konfigurieren
    pinMode(arm_button_pin, INPUT_PULLUP);
    pinMode(reset_alarm_button_pin, INPUT_PULLUP);
    pinMode(status_button_pin, INPUT_PULLUP);
    pinMode(tag_button_pin, INPUT_PULLUP);
    pinMode(language_button_pin, INPUT_PULLUP);

    pinMode(photoelectric_sensor_pin, INPUT);
    pinMode(radar_sensor_pin, INPUT);
    pinMode(microphone_pin, INPUT);
    pinMode(movement_sensor_pin, INPUT);

    // Interrupts für Buttons hinzufügen
    attachInterrupt(digitalPinToInterrupt(arm_button_pin), &handleArmButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(reset_alarm_button_pin), &handleResetAlarmButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(status_button_pin), &handleStatusButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(tag_button_pin), &handleTagButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(language_button_pin), &handleLanguageButtonInterrupt, FALLING);

    // outputs konfigurieren
    pinMode(piezo_pin, OUTPUT);
    pinMode(led_red_pin, OUTPUT);
    pinMode(led_green_pin, OUTPUT);

    digitalWrite(piezo_pin, LOW);
    digitalWrite(led_red_pin, LOW);
    digitalWrite(led_green_pin, HIGH);

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

    xTaskCreatePinnedToCore(outputJob, "outputJob", 4096, NULL, 0, &outputTask, 0);

    const esp_timer_create_args_t timer_100ms_args = {
        .callback = &timer_100ms_callback,
        .arg = NULL,
        .name = "Timer100ms"
    };
    esp_timer_create(&timer_100ms_args, &timer_handle_fast);
    esp_timer_start_periodic(timer_handle_fast, 100000); // 100ms

    sendLogMessage("Einrichtung abgeschlossen.");

    Serial.println(LANGUAGE ? "Finished Setup" : "Einrichtung abgeschlossen");
}

#pragma endregion

// --------- HAUPT-PROGRAMM ---------
#pragma region loop

void loop() {
    // alle 3 sekunden
    if (timer_100ms_count % 30 == 0) {
        dht11Task();
        if (telegramTask == NULL) {
            xTaskCreatePinnedToCore(telegramTaskHandle, "telegramTask", 8192, NULL, 0, &telegramTask, 0);
        }
    }

    // alle 2 Sekunden
    if (timer_100ms_count % 20 == 0) {
        if (armTask == NULL && resetTask == NULL && saveTagTask == NULL) updateLCD();

    }

    // jede sekunde
    if (timer_100ms_count % 10 == 0) {
        updateBMP280Data();
    }
}

#pragma endregion
