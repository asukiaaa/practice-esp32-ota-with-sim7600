// https://github.com/vshymanskyy/TinyGSM
// https://qiita.com/m_take/items/35d12f4ea6d5975fa632
// https://github.com/thingsboard/thingsboard-arduino-sdk/blob/master/examples/0004-arduino-sim900_send_telemetry/0004-arduino-sim900_send_telemetry.ino

#include <Arduino.h>

// #define USE_WIFI

/**************************************************************
 *
 * This sketch connects to a website and downloads a page.
 * It can be used to perform HTTP/RESTful API calls.
 *
 * For this example, you need to install ArduinoHttpClient library:
 *   https://github.com/arduino-libraries/ArduinoHttpClient
 *   or from http://librarymanager/all#ArduinoHttpClient
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 * For more HTTP API examples, see ArduinoHttpClient library
 *
 * NOTE: This example may NOT work with the XBee because the
 * HttpClient library does not empty to serial buffer fast enough
 * and the buffer overflow causes the HttpClient library to stall.
 * Boards with faster processors may work, 8MHz boards will not.
 **************************************************************/

// Select your modem:
// #define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM808
// #define TINY_GSM_MODEM_SIM868
// #define TINY_GSM_MODEM_SIM900
// #define TINY_GSM_MODEM_SIM7000
// #define TINY_GSM_MODEM_SIM7000SSL
// #define TINY_GSM_MODEM_SIM7080
// #define TINY_GSM_MODEM_SIM5360
#define TINY_GSM_MODEM_SIM7600
// #define TINY_GSM_MODEM_UBLOX
// #define TINY_GSM_MODEM_SARAR4
// #define TINY_GSM_MODEM_M95
// #define TINY_GSM_MODEM_BG96
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590
// #define TINY_GSM_MODEM_MC60
// #define TINY_GSM_MODEM_MC60E
// #define TINY_GSM_MODEM_ESP8266
// #define TINY_GSM_MODEM_XBEE
// #define TINY_GSM_MODEM_SEQUANS_MONARCH

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#ifndef __AVR_ATmega328P__
#define SerialAT Serial1

// or Software Serial on Uno, Nano
#else
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(2, 3);  // RX, TX
#endif

// Increase RX buffer to capture the entire response
// Chips without internal buffering (A6/A7, ESP8266, M590)
// need enough space in the buffer for the entire response
// else data will be lost (and the http library will fail).
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// #define LOGGING  // <- Logging is for the HTTP library

// Range to attempt to autobaud
// NOTE:  DO NOT AUTOBAUD in production code.  Once you've established
// communication, set a fixed baud rate using modem.setBaud(#).
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
// These defines are only for this example; they are not needed in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
#define GPRS_APN "soracom.io"
#define GPRS_USER "sora"
#define GPRS_PASS "sora"

#include <TinyGsmClient.h>
// #include <ArduinoHttpClient.h>
#include <ThingsBoard.h>

#include "secret.hpp"

// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);

void initNetwork() {
  SerialMon.println("Wait...");

  // Set GSM module baud rate
  TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  // SerialAT.begin(9600);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }
#endif
}

#define ENCRYPTED false

// Firmware title and version used to compare with remote version, to check if
// an update is needed. Title needs to be the same and version needs to be
// different --> downgrading is possible
#if THINGSBOARD_ENABLE_PROGMEM
constexpr char CURRENT_FIRMWARE_TITLE[] PROGMEM = "TEST";
constexpr char CURRENT_FIRMWARE_VERSION[] PROGMEM = "1.0.1";
#else
constexpr char CURRENT_FIRMWARE_TITLE[] = "TEST";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0.0";
#endif

// Firmware state send at the start of the firmware, to inform the cloud about
// the current firmware and that it was installed correctly, especially
// important when using OTA update, because the OTA update sends the last
// firmware state as UPDATING, meaning the device is restarting if the device
// restarted correctly and has the new given firmware title and version it
// should then send thoose to the cloud with the state UPDATED, to inform any
// end user that the device has successfully restarted and does actually contain
// the version it was flashed too
#if THINGSBOARD_ENABLE_PROGMEM
constexpr char FW_STATE_UPDATED[] PROGMEM = "UPDATED";
#else
constexpr char FW_STATE_UPDATED[] = "UPDATED";
#endif

// Maximum amount of retries we attempt to download each firmware chunck over
// MQTT
#if THINGSBOARD_ENABLE_PROGMEM
constexpr uint8_t FIRMWARE_FAILURE_RETRIES PROGMEM = 5U;
#else
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 5U;
#endif
// Size of each firmware chunck downloaded over MQTT,
// increased packet size, might increase download speed
#if THINGSBOARD_ENABLE_PROGMEM
constexpr uint16_t FIRMWARE_PACKET_SIZE PROGMEM = 4096U;
#else
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;
#endif

// // PROGMEM can only be added when using the ESP32 WiFiClient,
// // will cause a crash if using the ESP8266WiFiSTAClass instead.
// #if THINGSBOARD_ENABLE_PROGMEM
// constexpr char WIFI_SSID[] PROGMEM = "YOUR_WIFI_SSID";
// constexpr char WIFI_PASSWORD[] PROGMEM = "YOUR_WIFI_PASSWORD";
// #else
// constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
// constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
// #endif

// MQTT port used to communicate with the server, 1883 is the default
// unencrypted MQTT port, whereas 8883 would be the default encrypted SSL MQTT
// port
#if ENCRYPTED
#if THINGSBOARD_ENABLE_PROGMEM
constexpr uint16_t THINGSBOARD_PORT PROGMEM = 8883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 8883U;
#endif
#else
#if THINGSBOARD_ENABLE_PROGMEM
constexpr uint16_t THINGSBOARD_PORT PROGMEM = 1883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif
#endif

// Maximum size packets will ever be sent or received by the underlying MQTT
// client, if the size is to small messages might not be sent or received
// messages will be discarded
#if THINGSBOARD_ENABLE_PROGMEM
constexpr uint32_t MAX_MESSAGE_SIZE PROGMEM = 512U;
#else
constexpr uint32_t MAX_MESSAGE_SIZE = 512U;
#endif

// Baud rate for the debugging serial connection
// If the Serial output is mangled, ensure to change the monitor speed
// accordingly to this variable
#if THINGSBOARD_ENABLE_PROGMEM
constexpr uint32_t SERIAL_DEBUG_BAUD PROGMEM = 115200U;
#else
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
#endif

// Initialize underlying client, used to establish a connection
#ifdef USE_WIFI
#include <WiFi.h>
#include <WiFiClient.h>
WiFiClient espClient;
ThingsBoard tb(espClient, MAX_MESSAGE_SIZE);
#else
ThingsBoard tb(client, MAX_MESSAGE_SIZE);
#endif
// Initialize ThingsBoard instance with the maximum needed buffer size

// Statuses for updating
bool currentFWSent = false;
bool updateRequestSent = false;

#ifdef USE_WIFI

// /// @brief Initalizes WiFi connection,
// // will endlessly delay until a connection has been successfully established
void InitWiFi() {
#if THINGSBOARD_ENABLE_PROGMEM
  Serial.println(F("Connecting to AP ..."));
#else
  Serial.println("Connecting to AP ...");
#endif
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(wifiSSID, wifiPass);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been succesfully established
    delay(500);
#if THINGSBOARD_ENABLE_PROGMEM
    Serial.print(F("."));
#else
    Serial.print(".");
#endif
  }
#if THINGSBOARD_ENABLE_PROGMEM
  Serial.println(F("Connected to AP"));
#else
  Serial.println("Connected to AP");
#endif
#if ENCRYPTED
  espClient.setCACert(ROOT_CERT);
#endif
}
#endif

/// @brief Updated callback that will be called as soon as the firmware update
/// finishes
/// @param success Either true (update succesfull) or false (update failed)
void updatedCallback(const bool& success) {
  if (success) {
#if THINGSBOARD_ENABLE_PROGMEM
    Serial.println(F("Done, Reboot now"));
#else
    Serial.println("Done, Reboot now");
#endif
#if defined(ESP8266)
    ESP.restart();
#elif defined(ESP32)
    esp_restart();
#endif
    return;
  }
#if THINGSBOARD_ENABLE_PROGMEM
  Serial.println(F("Downloading firmware failed"));
#else
  Serial.println("Downloading firmware failed");
#endif
}

/// @brief Progress callback that will be called every time we downloaded a new
/// chunk successfully
/// @param currentChunk
/// @param totalChuncks
void progressCallback(const uint32_t& currentChunk,
                      const uint32_t& totalChuncks) {
  Serial.printf("Progress %.2f%%\n",
                static_cast<float>(currentChunk * 100U) / totalChuncks);
}

const OTA_Update_Callback callback(&progressCallback, &updatedCallback,
                                   CURRENT_FIRMWARE_TITLE,
                                   CURRENT_FIRMWARE_VERSION,
                                   FIRMWARE_FAILURE_RETRIES,
                                   FIRMWARE_PACKET_SIZE);

void setup() {
  // Initalize serial connection for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
#ifdef USE_WIFI
  InitWiFi();
#else
  initNetwork();
#endif
}

bool modemConnected = false;

void loop() {
  delay(1000);

  if (!modemConnected) {
#if THINGSBOARD_ENABLE_PROGMEM
    Serial.print(F("Waiting for network..."));
#else
    Serial.print("Waiting for network...");
#endif
    if (!modem.waitForNetwork()) {
#if THINGSBOARD_ENABLE_PROGMEM
      Serial.println(F(" fail"));
#else
      Serial.println(" fail");
#endif
      delay(10000);
      return;
    }
#if THINGSBOARD_ENABLE_PROGMEM
    Serial.println(F(" OK"));
#else
    Serial.println(" OK");
#endif

#if THINGSBOARD_ENABLE_PROGMEM
    Serial.print(F("Connecting to "));
#else
    Serial.print("Connecting to ");
#endif
    Serial.print(GPRS_APN);
    if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
#if THINGSBOARD_ENABLE_PROGMEM
      Serial.println(F(" fail"));
#else
      Serial.println(" fail");
#endif
      delay(10000);
      return;
    }

    modemConnected = true;
#if THINGSBOARD_ENABLE_PROGMEM
    Serial.println(F(" OK"));
#else
    Serial.println(" OK");
#endif
  }

  if (!tb.connected()) {
    // Reconnect to the ThingsBoard server,
    // if a connection was disrupted or has not yet been established
    Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER,
                  TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
#if THINGSBOARD_ENABLE_PROGMEM
      Serial.println(F("Failed to connect"));
#else
      Serial.println("Failed to connect");
#endif
      return;
    }
  }

  if (!currentFWSent) {
    currentFWSent = tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE,
                                          CURRENT_FIRMWARE_VERSION) &&
                    tb.Firmware_Send_State(FW_STATE_UPDATED);
  }

  if (!updateRequestSent) {
#if THINGSBOARD_ENABLE_PROGMEM
    Serial.println(F("Firwmare Update Subscription..."));
#else
    Serial.println("Firwmare Update Subscription...");
#endif
    // See https://thingsboard.io/docs/user-guide/ota-updates/
    // to understand how to create a new OTA pacakge and assign it to a device
    // so it can download it.
    updateRequestSent = tb.Subscribe_Firmware_Update(callback);
  }

  tb.loop();
}
