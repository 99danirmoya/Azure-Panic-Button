/* *************************************************************************************************************
****************************    Author  : Ehab Magdy Abdullah                      *****************************
****************************    Linkedin: https://www.linkedin.com/in/ehabmagdyy/  *****************************
****************************    Youtube : https://www.youtube.com/@EhabMagdyy      *****************************
****************************    Forked by MIoT students for CC, UPM                *****************************
************************************************************************************************************* */

// Libraries ---------------------------------------------------------------------------------------------------
#include <Arduino.h>
// C99 libraries
#include <cstdlib>
#include <string.h>
#include <time.h>
// Libraries for MQTT client and WiFi connection
#include <WiFi.h>
#include <mqtt_client.h>
// Azure IoT SDK for C includes
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>
// Additional sample headers
#include "AzIoTSasToken.h"
#include "SerialLogger.h"
#include "iot_configs_1.h"
// OTA libraries
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>                                                                                         // DUE TO THE LIMITATIONS OF FREE TIER AZURE IoT HUB, OFFICIAL ADU IS NOT SUPPORTED. ARDUINO OTA ALTERNATIVE USED INSTEAD FOR CI/CD
// I2C libs
#include <Wire.h>
#include <axp20x.h>                                                                                             // Library for the PMU AXP192

// GPS lib
#include <TinyGPS++.h>                                                                                          // Library for the GPS NEO-6M

// Utility macros, defines and globals -------------------------------------------------------------------------
// Cloud macros
#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp32)"
#define sizeofarray(a) (sizeof(a) / sizeof(a[0]))
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define MQTT_QOS1 1
#define DO_NOT_RETAIN_MSG 0
#define SAS_TOKEN_DURATION_IN_MINUTES 60
#define UNIX_TIME_NOV_13_2017 1510592825

#define PST_TIME_ZONE -8
#define PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF 1

#define GMT_OFFSET_SECS (PST_TIME_ZONE * 3600)
#define GMT_OFFSET_SECS_DST ((PST_TIME_ZONE + PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * 3600)

#define INCOMING_DATA_BUFFER_SIZE 128

// T-Beam macros
#define LED_PIN 4                                                                                               // Define builtin LED pin
#define BUTTON_PIN 38                                                                                           // Define builtin button pin

#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 4
#define PMU_IRQ_PIN 35                                                                                          // PEK (PWR) button interrupt pin on T-Beam

#define SerialGPS Serial1                                                                                       // Se crea un nuevo puerto serie para el GPS (el predeterminado, 'Serial', se usa para el monitor serie)
#define GPS_RX_PIN 34                                                                                           // Pin RX del ESP32 al pin TX del GPS
#define GPS_TX_PIN 12                                                                                           // Pin TX del ESP32 al pin RX del GPS
#define GPS_BAUD_RATE 9600      

// Translate iot_configs.h defines into variables used by the sample
static const char* ssid = IOT_CONFIG_WIFI_SSID;
static const char* password = IOT_CONFIG_WIFI_PASSWORD;
static const char* host = IOT_CONFIG_IOTHUB_FQDN;
static const char* mqtt_broker_uri = "mqtts://" IOT_CONFIG_IOTHUB_FQDN;
static const char* device_id = IOT_CONFIG_DEVICE_ID;
static const int mqtt_port = AZ_IOT_DEFAULT_MQTT_CONNECT_PORT;

// Memory allocated for the sample's variables and structures.
static esp_mqtt_client_handle_t mqtt_client;
static az_iot_hub_client client;

static char mqtt_client_id[128];
static char mqtt_username[128];
static char mqtt_password[200];
static uint8_t sas_signature_buffer[256];
static char telemetry_topic[128];
static uint32_t telemetry_send_count = 0;
static String telemetry_payload = "";

static char incoming_data[INCOMING_DATA_BUFFER_SIZE];

// Auxiliary functions -----------------------------------------------------------------------------------------
#ifndef IOT_CONFIG_USE_X509_CERT
static AzIoTSasToken sasToken(&client, AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY), AZ_SPAN_FROM_BUFFER(sas_signature_buffer), AZ_SPAN_FROM_BUFFER(mqtt_password));
#endif                                                                                                          // IOT_CONFIG_USE_X509_CERT

// Sensor params -----------------------------------------------------------------------------------------------
static AXP20X_Class axp;
static TinyGPSPlus gps;

// Struct para los datos de sensores ---------------------------------------------------------------------------
struct GPSData{
  float lat = 0.0f, lon = 0.0f, alt = 0.0f;                                                                     // Definicion de las variables 'float'
  uint8_t sat = 0;                                                                                              // Variable en la que se guarda el numero de satelites en cobertura del GPS
};

static volatile bool panicFlag = false;
static const bool isSimulated = false;
static bool panicStatus = false, ledBlink = false, ledState = LOW;
static uint8_t heartRate = 0, respiratoryRate = 0, bloodPressureSystolic = 0, bloodPressureDiastolic = 0, oxygenSaturation = 0, satPayload = 0;
static float batVolt = 0.0f, temperature = 0.0f, latPayload = 0.0f, lonPayload = 0.0f, altPayload = 0.0f;

// Task handles ------------------------------------------------------------------------------------------------
static TaskHandle_t azureTaskHandle = NULL, GPSTaskHandle = NULL;

// Queue -------------------------------------------------------------------------------------------------------
static QueueHandle_t GPSQueue = NULL;

// Tasks -------------------------------------------------------------------------------------------------------
static void azureTask(void*);
static void GPSTask(void*);

// Function prototypes -----------------------------------------------------------------------------------------
static void connectToWiFi();
static void setupOTA();
static void initializeTime();
static void receivedCallback(char*, byte*, unsigned int);
static void mqtt_event_handler(void *, esp_event_base_t, int32_t, void *);
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t);
static void initializeIoTHubClient();
static int initializeMqttClient();
static uint32_t getEpochTimeInSecs();
static void establishConnection();
static void directMethodCallback(const char* , const char* , size_t);
static void generateTelemetryPayload();
static void sendTelemetry();
static void panicISR();

static void GPSTask(void* pvParameters){
  while(true){
    GPSData gData;

    // Preparacion del GPS para tener cobertura satelite y poder enviar coordenadas
    while(SerialGPS.available() > 0){                                                                           // Mientras haya bytes disponibles en el puerto serie del GPS
      if(gps.encode(SerialGPS.read())){                                                                         // Se confirma que haya lecturas del GPS
        if (gps.location.isValid()) {                                                                           // Si las coordenadas recogidas son validas
          gData.lat = gps.location.lat();                                                                       // Se guardan en las variables 'lat' y 'lon' dichas coordenadas
          gData.lon = gps.location.lng();
          gData.alt = gps.altitude.meters();
          gData.sat = gps.satellites.value();
        }else{                                                                                                  // En caso de que sean invalidas, se muestra un mensaje en el monitor serie
          gData.lat = 0.0f;                                                                                     // Por marcar un numero definido, la ubicacion se fija a 0 en ambos 'lat' y 'lon'
          gData.lon = 0.0f;
          gData.alt = 0.0f;
          gData.sat = 0;
        }
      }
    }

    if((xTaskGetTickCount() > pdMS_TO_TICKS(15000)) && (gps.charsProcessed() < 10)){                            // Si durante 15 segundos se cumple que los caracteres procesados del GPS son menos de 10, implica que hay un error leyendo el GPS, se reporta el fallo y se bloquea el programa hasta que se solvente el error
      Serial.println(F("\t\t\tGPS no detectado: comprueba el cableado"));
      vTaskDelay(pdMS_TO_TICKS(15000));
    }

    // Queue overwrite as the important thing is to consume the latest data as quickly as possible and not in order, it only returns pdTRUE, so it does not return errors
    xQueueOverwrite(GPSQueue, &gData);

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// AZURE TASK: former loop function ----------------------------------------------------------------------------
static void azureTask(void *pvParameters) {
  TickType_t nextTelemetrySendTime = xTaskGetTickCount();                                                       // Initial time

  while(true) {
    GPSData gData;
    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi();
    }
    else if ((xTaskGetTickCount() > nextTelemetrySendTime) || panicFlag) {
      // Reset interrupt flag and set panic status
      if (panicFlag) {
        Serial.println("\t\t\tButton interrupt triggered telemetry send");
        panicFlag = false;
        ledBlink = true;
        panicStatus = true;
      }

      // Blink LED if enabled
      if (ledBlink) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
      }

      // Simulated sensor values
      batVolt = axp.getBattVoltage();
      temperature = random(359, 422) / 10.0f;
      heartRate = random(50, 150);
      respiratoryRate = random(12, 18);
      bloodPressureSystolic = random(110, 145);
      bloodPressureDiastolic = random(70, 95);
      oxygenSaturation = random(90, 100);

      if(xQueuePeek(GPSQueue, &gData, 0)){
        latPayload = gData.lat;
        lonPayload = gData.lon;
        altPayload = gData.alt;
        satPayload = gData.sat;
      }else{                                                                                                    // Error from xQueuePeek comes when the queue is empty, only at the very beginning
        Serial.println(F("\t\t\tEmpty GPS queue."));
      }

      sendTelemetry();                                                                                          // Send telemetry

      nextTelemetrySendTime = xTaskGetTickCount() + pdMS_TO_TICKS(TELEMETRY_FREQUENCY_MILLISECS);               // Schedule next telemetry send
    }

    // Check for PEK button press
    if(digitalRead(PMU_IRQ_PIN) == LOW){
      axp.readIRQ();

      if(axp.isPEKLongtPressIRQ()){
        Serial.println("\t\t\tLong press detected: Shutting down...");
        axp.shutdown();
      }

      axp.clearIRQ();
    }

    ArduinoOTA.handle();

    vTaskDelay(pdMS_TO_TICKS(100));                                                                             // Yield delay
  }
}
// -------------------------------------------------------------------------------------------------------------

// CONNECT TO WIFI FUNCTION ------------------------------------------------------------------------------------
static void connectToWiFi(){
  Logger.Info("Connecting to WIFI SSID " + String(ssid));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);

    // Check for PEK button press
    if(digitalRead(PMU_IRQ_PIN) == LOW){
      axp.readIRQ();

      if(axp.isPEKLongtPressIRQ()){
        Serial.println("\t\t\tLong press detected: Shutting down...");
        axp.shutdown();
      }

      axp.clearIRQ();
    }
  }

  Serial.println("");

  Logger.Info("WiFi connected, IP address: " + WiFi.localIP().toString());

  if(ledState){
    digitalWrite(LED_PIN, LOW);
  }
}
// -------------------------------------------------------------------------------------------------------------

// SETUP OTA FUNCTION ------------------------------------------------------------------------------------------
static void setupOTA(){
  // Set custom OTA hostname
  ArduinoOTA.setHostname("esp32-ota-test");

  // No authentication by default
  ArduinoOTA.setPassword("pw0123");
  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("OTA service started!");
}
// -------------------------------------------------------------------------------------------------------------

// INITIALIZE FUNCTION -----------------------------------------------------------------------------------------
static void initializeTime(){
  Logger.Info("Setting time using SNTP");

  configTime(GMT_OFFSET_SECS, GMT_OFFSET_SECS_DST, NTP_SERVERS);
  time_t now = time(NULL);
  
  while (now < UNIX_TIME_NOV_13_2017){
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  
  Serial.println("");
  Logger.Info("Time initialized!");
}
// -------------------------------------------------------------------------------------------------------------

// RECEIVE CALLBACK FUNCTION -----------------------------------------------------------------------------------
static void receivedCallback(char* topic, byte* payload, unsigned int length){
  Logger.Info("Received [");
  Logger.Info(topic);
  Logger.Info("]: ");

  for(int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println("");
}
// -------------------------------------------------------------------------------------------------------------

// MQTT EVENT HANDLER FUNCTION ---------------------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3                                        // ESP_ARDUINO_VERSION_MAJOR
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){  // For this condition, use this function signature
  (void)handler_args;
  (void)base;
  (void)event_id;

  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
#else                                                                                                           // ESP_ARDUINO_VERSION_MAJOR
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event){                                             // For a different ESP_ARDUINO_VERSION_MAJOR, use this signature
#endif                                                                                                          // ESP_ARDUINO_VERSION_MAJOR
  
  switch (event->event_id){
    int r;

    case MQTT_EVENT_ERROR:
      Logger.Info("MQTT event MQTT_EVENT_ERROR");
      break;
    case MQTT_EVENT_CONNECTED:
      Logger.Info("MQTT event MQTT_EVENT_CONNECTED");

      // Subscribe to cloud-to-device messages
      r = esp_mqtt_client_subscribe(mqtt_client, AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC, 1);
      if(r == -1){
        Logger.Error("Could not subscribe for cloud-to-device messages.");
      }else{
        Logger.Info("Subscribed for cloud-to-device messages; message id:" + String(r));
      }

      // Subscribe to direct methods
      r = esp_mqtt_client_subscribe(mqtt_client, "$iothub/methods/POST/#", 1);
      if (r == -1) {
        Logger.Error("Could not subscribe for direct methods.");
      }else{
        Logger.Info("Subscribed for direct methods; message id:" + String(r));
      }
      break;

    case MQTT_EVENT_DISCONNECTED:
      Logger.Info("MQTT event MQTT_EVENT_DISCONNECTED");
      break;

    case MQTT_EVENT_SUBSCRIBED:
      Logger.Info("MQTT event MQTT_EVENT_SUBSCRIBED");
      break;

    case MQTT_EVENT_UNSUBSCRIBED:
      Logger.Info("MQTT event MQTT_EVENT_UNSUBSCRIBED");
      break;

    case MQTT_EVENT_PUBLISHED:
      Logger.Info("MQTT event MQTT_EVENT_PUBLISHED");
      break;

    case MQTT_EVENT_DATA:
      {
        Logger.Info("MQTT event MQTT_EVENT_DATA");

        // Extract topic and payload
        char topic[INCOMING_DATA_BUFFER_SIZE];
        char payload[INCOMING_DATA_BUFFER_SIZE];

        int topic_len = min(event->topic_len, INCOMING_DATA_BUFFER_SIZE - 1);
        strncpy(topic, event->topic, topic_len);
        topic[topic_len] = '\0';

        int payload_len = min(event->data_len, INCOMING_DATA_BUFFER_SIZE - 1);
        strncpy(payload, event->data, payload_len);
        payload[payload_len] = '\0';

        Logger.Info("Topic: " + String(topic));
        Logger.Info("Payload: " + String(payload));

        // Copy the topic to incoming_data for direct method processing
        strncpy(incoming_data, topic, INCOMING_DATA_BUFFER_SIZE);

        // Check if the message is a direct method
        if(strstr(topic, "$iothub/methods/POST/") != NULL){
          // Extract method name from the topic
          char method_name[64];
          sscanf(topic, "$iothub/methods/POST/%[^/]", method_name);

          // Handle the direct method
          directMethodCallback(method_name, payload, payload_len);
        }
      }

      break;

    case MQTT_EVENT_BEFORE_CONNECT:
      Logger.Info("MQTT event MQTT_EVENT_BEFORE_CONNECT");
      break;
    default:
      Logger.Error("MQTT event UNKNOWN");
      break;
  }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3                                        // For this ESP_ARDUINO_VERSION_MAJOR, do NOTHING
#else                                                                                                           // ESP_ARDUINO_VERSION_MAJOR
  return ESP_OK;                                                                                                // For others, return "ESP_OK"
#endif                                                                                                          // ESP_ARDUINO_VERSION_MAJOR
}
// -------------------------------------------------------------------------------------------------------------

// INITIALIZE IoT HUB CLIENT FUNCTION --------------------------------------------------------------------------
static void initializeIoTHubClient(){
  az_iot_hub_client_options options = az_iot_hub_client_options_default();
  options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);

  if(az_result_failed(az_iot_hub_client_init(&client, az_span_create((uint8_t*)host, strlen(host)), az_span_create((uint8_t*)device_id, strlen(device_id)), &options))){
    Logger.Error("Failed initializing Azure IoT Hub client");
    return;
  }

  size_t client_id_length;
  if (az_result_failed(az_iot_hub_client_get_client_id(&client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length))){
    Logger.Error("Failed getting client id");
    return;
  }

  if(az_result_failed(az_iot_hub_client_get_user_name(&client, mqtt_username, sizeofarray(mqtt_username), NULL))){
    Logger.Error("Failed to get MQTT clientId, return code");
    return;
  }

  Logger.Info("Client ID: " + String(mqtt_client_id));
  Logger.Info("Username: " + String(mqtt_username));
}
// -------------------------------------------------------------------------------------------------------------

// INITIALIZE MQTT CLIENT FUNCTION -----------------------------------------------------------------------------
static int initializeMqttClient(){
  #ifndef IOT_CONFIG_USE_X509_CERT
    if (sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES) != 0){
      Logger.Error("Failed generating SAS token");
      return 1;
    }
  #endif

  esp_mqtt_client_config_t mqtt_config;
  memset(&mqtt_config, 0, sizeof(mqtt_config));

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    mqtt_config.broker.address.uri = mqtt_broker_uri;
    mqtt_config.broker.address.port = mqtt_port;
    mqtt_config.credentials.client_id = mqtt_client_id;
    mqtt_config.credentials.username = mqtt_username;

    #ifdef IOT_CONFIG_USE_X509_CERT
      LogInfo("MQTT client using X509 Certificate authentication");
      mqtt_config.credentials.authentication.certificate = IOT_CONFIG_DEVICE_CERT;
      mqtt_config.credentials.authentication.certificate_len = (size_t)sizeof(IOT_CONFIG_DEVICE_CERT);
      mqtt_config.credentials.authentication.key = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
      mqtt_config.credentials.authentication.key_len = (size_t)sizeof(IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY);
    #else // Using SAS key
      mqtt_config.credentials.authentication.password = (const char*)az_span_ptr(sasToken.Get());
    #endif

    mqtt_config.session.keepalive = 30;
    mqtt_config.session.disable_clean_session = 0;
    mqtt_config.network.disable_auto_reconnect = false;
    mqtt_config.broker.verification.certificate = (const char*)ca_pem;
    mqtt_config.broker.verification.certificate_len = (size_t)ca_pem_len;
  
  #else // ESP_ARDUINO_VERSION_MAJOR  
    mqtt_config.uri = mqtt_broker_uri;
    mqtt_config.port = mqtt_port;
    mqtt_config.client_id = mqtt_client_id;
    mqtt_config.username = mqtt_username;

    #ifdef IOT_CONFIG_USE_X509_CERT
      Logger.Info("MQTT client using X509 Certificate authentication");
      mqtt_config.client_cert_pem = IOT_CONFIG_DEVICE_CERT;
      mqtt_config.client_key_pem = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
    #else // Using SAS key
      mqtt_config.password = (const char*)az_span_ptr(sasToken.Get());
    #endif

    mqtt_config.keepalive = 30;
    mqtt_config.disable_clean_session = 0;
    mqtt_config.disable_auto_reconnect = false;
    mqtt_config.event_handle = mqtt_event_handler;
    mqtt_config.user_context = NULL;
    mqtt_config.cert_pem = (const char*)ca_pem;
  #endif // ESP_ARDUINO_VERSION_MAJOR

  mqtt_client = esp_mqtt_client_init(&mqtt_config);

  if (mqtt_client == NULL){
    Logger.Error("Failed creating mqtt client");
    return 1;
  }

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
  #endif // ESP_ARDUINO_VERSION_MAJOR

  esp_err_t start_result = esp_mqtt_client_start(mqtt_client);

  if(start_result != ESP_OK){
    Logger.Error("Could not start mqtt client; error code:" + start_result);
    return 1;
  }else{
    Logger.Info("MQTT client started");
    return 0;
  }
}
// -------------------------------------------------------------------------------------------------------------

// GET EPOCH TIME IN SECONDS -----------------------------------------------------------------------------------
static uint32_t getEpochTimeInSecs(){
  return (uint32_t)time(NULL);
}
// -------------------------------------------------------------------------------------------------------------

// ESTABLISH CONNECTION FUNCTION -------------------------------------------------------------------------------
static void establishConnection(){
  connectToWiFi();
  setupOTA();
  initializeTime();
  initializeIoTHubClient();
  (void)initializeMqttClient();
}
// -------------------------------------------------------------------------------------------------------------

// DIRECT METHOD CALLBACK HANDLER ------------------------------------------------------------------------------
static void directMethodCallback(const char* method_name, const char* payload, size_t payload_size) {
  Logger.Info("Direct method received: " + String(method_name));
  Logger.Info("Payload: " + String(payload));

  // Registered direct methods
  bool led_state = false;
  if(strcmp(method_name, "led_on") == 0){
    digitalWrite(LED_PIN, !HIGH);                                                                               // Turn LED on
    led_state = true;
    Logger.Info("LED turned ON");
  }else if(strcmp(method_name, "led_off") == 0){
    digitalWrite(LED_PIN, !LOW);                                                                                // Turn LED off
    led_state = false;
    Logger.Info("LED turned OFF");
  }else if(strcmp(method_name, "panic_off") == 0){
    panicStatus = false;
    ledBlink = false;
    digitalWrite(LED_PIN, LOW);                                                                                 // After patient has been assisted, switch LED back ON (no blinking)
    Logger.Info("Panic situation solved");
  }else{
    Logger.Error("Unknown method: " + String(method_name));
  }

  // Extract the request ID ($rid) from the topic
  char request_id[32];
  if(sscanf(incoming_data, "$iothub/methods/POST/%*[^/]/?$rid=%31[^&]", request_id) != 1){
    Logger.Error("Failed to extract request ID from topic");
    Logger.Info("Topic for debugging: " + String(incoming_data));
    return;
  }

  // Generate the response topic
  char response_topic[128];
  snprintf(response_topic, sizeof(response_topic), "$iothub/methods/res/200/?$rid=%s", request_id);

  // Create a response payload with LED state
  String response_payload = "{}";

  // Publish the response
  if(esp_mqtt_client_publish(mqtt_client, response_topic, response_payload.c_str(), response_payload.length(), MQTT_QOS1, DO_NOT_RETAIN_MSG) == 0){
    Logger.Error("Failed to publish direct method response");
  }else{
    Logger.Info("Direct method response sent successfully");
  }
}
// -------------------------------------------------------------------------------------------------------------

// GENERATE TELEMETRY PAYLOAD JSON -----------------------------------------------------------------------------
static void generateTelemetryPayload(){
  telemetry_payload = "{ \"MessageCounter\": " + String(telemetry_send_count++) +
                      ", \"PanicAlarm\": " + String(panicStatus) +
                      ", \"IsSimulated\": " + String(isSimulated) +
                      ", \"BatteryVoltage\": " + String(batVolt) +
                      ", \"Temperature\": " + String(temperature) +
                      ", \"HeartRate\": " + String(heartRate) +
                      ", \"RespiratoryRate\": " + String(respiratoryRate) +
                      ", \"BloodPressureSystolic\": " + String(bloodPressureSystolic) +
                      ", \"BloodPressureDiastolitic\": " + String(bloodPressureDiastolic) +
                      ", \"OxygenSaturation\": " + String(oxygenSaturation) +
                      ", \"Satellites\": " + String(satPayload) +
                      ", \"Latitude\": " + String(latPayload, 6) +
                      ", \"Longitude\": " + String(lonPayload, 6) +
                      ", \"Altitude\": " + String(altPayload) + " }";
}
// -------------------------------------------------------------------------------------------------------------

// SEND TELEMTETRY ---------------------------------------------------------------------------------------------
static void sendTelemetry(){
  Logger.Info("Sending telemetry ...");

  if (az_result_failed(az_iot_hub_client_telemetry_get_publish_topic(&client, NULL, telemetry_topic, sizeof(telemetry_topic), NULL))){
    Logger.Error("Failed az_iot_hub_client_telemetry_get_publish_topic");
    return;
  }

  generateTelemetryPayload();

  if(esp_mqtt_client_publish(mqtt_client, telemetry_topic, (const char*)telemetry_payload.c_str(), telemetry_payload.length(), MQTT_QOS1, DO_NOT_RETAIN_MSG) == 0){
    Logger.Error("Failed publishing");
  }else{
    Logger.Info("Message published successfully");
  }
}
// -------------------------------------------------------------------------------------------------------------

// PANIC INTERRUPTION SERVICE ROUTINE --------------------------------------------------------------------------
static void panicISR(){
  panicFlag = true;
}
// -------------------------------------------------------------------------------------------------------------

// SETUP -------------------------------------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  Serial.println(F("Panic button v1.0"));

  SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);                                           // Inicializacion de la comunicacion por serial para el GPS

  Wire.begin(SDA_PIN, SCL_PIN);                                                                                 // Initialize I2C bus
  
  if(axp.begin(Wire, AXP192_SLAVE_ADDRESS) != 0){                                                               // "AXP192_SLAVE_ADDRESS" should be "0x34"
    Serial.println("AXP192 not detected!");
    while(1);
  }else{
    Serial.println("AXP192 detected.");
  }

  axp.adc1Enable(AXP202_BATT_VOL_ADC1, true);                                                                   // Enable ADC for battery voltage

  pinMode(PMU_IRQ_PIN, INPUT);                                                                                  // Set up PEK button IRQ pin

  axp.clearIRQ();                                                                                               // Clear any existing IRQs
  axp.enableIRQ(AXP202_PEK_SHORTPRESS_IRQ | AXP202_PEK_LONGPRESS_IRQ, true);                                    // Enable PEK IRQs

  pinMode(BUTTON_PIN, INPUT_PULLUP);                                                                            // Button with internal pull-up
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);

  establishConnection();                                                                                        // Establish Azure IoT Connection

  // Create the queues
  GPSQueue = xQueueCreate(1, sizeof(GPSData));                                                                  // Queue can hold up 1 sensor readings as over write and peek methods will be used and no critical synchronization is required

  // Create tasks and assign them to different cores
  xTaskCreatePinnedToCore(
    azureTask,                                                                                                  /* Function to implement the task */
    "azureTask",                                                                                                /* Name of the task */
    10000,                                                                                                      /* Stack size in bytes */
    NULL,                                                                                                       /* Task input parameter */
    1,                                                                                                          /* Priority of the task */
    &azureTaskHandle,                                                                                           /* Task handle. */
    1                                                                                                           /* Core where the task should run */
  );

  xTaskCreatePinnedToCore(
    GPSTask,                                                                                                    /* Function to implement the task */
    "GPSTask",                                                                                                  /* Name of the task */
    5000,                                                                                                       /* Stack size in bytes */
    NULL,                                                                                                       /* Task input parameter */
    1,                                                                                                          /* Priority of the task */
    &GPSTaskHandle,                                                                                             /* Task handle. */
    0                                                                                                           /* Core where the task should run */
  );

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), panicISR, FALLING);                                        // Establish the pin interrupt to the builtin button, falling as it has inverse logic
}
// -------------------------------------------------------------------------------------------------------------

// LOOP --------------------------------------------------------------------------------------------------------
void loop() {
  delay(1000);                                                                                                  // Empty loop as we are using freeRTOS
}
// -------------------------------------------------------------------------------------------------------------
