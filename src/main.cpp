/*------------------------------------------------COREIOT------------------------------------------------*/
// library
#include <Arduino.h>
#include <Arduino_MQTT_Client.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DHT.h"
#include "DHT20.h"
#include <ThingsBoard.h>
#include <ArduinoOTA.h>

// i2c pin
#define SDA_PIN 11
#define SCL_PIN 12

// relay pin
#define RELAY_1 1
#define RELAY_2 2
#define RELAY_3 3
#define RELAY_4 4

DHT20 dht20;

// connect to access point
const char WIFI_SSID[] = "Wifi11";
const char WIFI_PASSWORD[] = "123456789";

// connect to core thingsboard - core iot
const char THINGSBOARD_SERVER[] = "app.coreiot.io";
const uint16_t THINGSBOARD_PORT = 1883;
// const char ACCESS_TOKEN[] = "7l23s5snln10k419oc0n"; // IOT_DEVICE_01
const char ACCESS_TOKEN[] = "leIAIncH7QfbexKLgP78"; // SENSOR_A1
// const char ACCESS_TOKEN[] = "64YEcnzMlv0qs6DEkZLG"; // aaaa test
// const char ACCESS_TOKEN[] = "AwY21S4gAmvAlCmYgI2b"; // C1

// define max message size
const uint16_t MAX_MESSAGE_SIZE = 2048;

// shared atttribute
void processSharedAttributeUpdate(const JsonObjectConst &data);

// RPC Callback
// RPC_Response processRPCCallback(const JsonObjectConst &data)
// {

// }
void setupRelays()
{
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);

  digitalWrite(RELAY_1, LOW);
  digitalWrite(RELAY_2, LOW);
  digitalWrite(RELAY_3, LOW);
  digitalWrite(RELAY_4, LOW);
}

void setRelayState(int relayNumber, bool state)
{
  digitalWrite(relayNumber, state);
}

RPC_Response RPCControlRelays(const RPC_Data &data)
{
  String method = data["method"].as<String>();
  bool par = data["params"].as<bool>();

  Serial.println("Received RPC method: " + method + ", params: " + String(par));
  if (method == "setValueRelay1")
  {
    Serial.println(String("Relay 1: ") + (par ? "ON" : "OFF"));
    setRelayState(RELAY_1, par);
  }
  else if (method == "setValueRelay2")
  {
    Serial.println(String("Relay 2: ") + (par ? "ON" : "OFF"));
    setRelayState(RELAY_2, par);
  }
  else if (method == "setValueRelay3")
  {
    Serial.println(String("Relay 3: ") + (par ? "ON" : "OFF"));
    setRelayState(RELAY_3, par);
  }
  else if (method == "setValueRelay4")
  {
    Serial.println(String("Relay 4: ") + (par ? "ON" : "OFF"));
    setRelayState(RELAY_4, par);
  }
  // Serial.println("Received Switch state");
  bool newState = data;

  return RPC_Response(NULL, newState);
}

const std::array<RPC_Callback, 4U>
    callbacks = {
        RPC_Callback{"setValueRelay1", RPCControlRelays},
        RPC_Callback{"setValueRelay2", RPCControlRelays},
        RPC_Callback{"setValueRelay3", RPCControlRelays},
        RPC_Callback{"setValueRelay4", RPCControlRelays},
};

//
WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard thingsBoard(mqttClient, MAX_MESSAGE_SIZE);

bool sharedAttributeSubscribed = false;
bool RPCSubscribed = false;

void initWiFi()
{
  Serial.println("Connecting to WiFi ...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    // delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
}

void reconnectWiFi(void *pvParameters)
{
  (void)pvParameters;

  while (true)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      // establish a connection
      initWiFi();
    }
    else
      Serial.println("Wifi already connected");
    // check every 60s
    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

void subscribeRPCCallback(void *pvParameters);

void connectToCoreIoT(void *pvParameters)
{
  (void)pvParameters;

  while (true)
  {
    if (!thingsBoard.connected())
    {
      if (!thingsBoard.connect(THINGSBOARD_SERVER, ACCESS_TOKEN, THINGSBOARD_PORT))
      {
        Serial.println("Failed to connect to Core IoT");
      }
      else
      {
        Serial.println("Connected to Core IoT");
        xTaskCreate(subscribeRPCCallback, "SubscribeRPCCallbackTask", 4096, NULL, 1, NULL);
      }
    }
    else
    {
      Serial.println("Core IoT already connected");
    }
    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

// void subscribeSharedAttribute(void *pvParameters)
// {
//   (void)pvParameters;
//   while (true)
//   {
//     if (!sharedAttributeSubscribed)
//     {
//       if (!sharedAttributeUpdate.Shared_Attributes_Subscribe(callback))
//       {
//         Serial.println("Failed to subscribe for shared attribute update");
//         return;
//       }
//       Serial.println("Subscribe shared attribute done");
//       sharedAttributeSubscribed = true;
//     }
//     vTaskDelay(9000 / portTICK_PERIOD_MS);
//   }
// }

void subscribeRPCCallback(void *pvParameters)
{
  (void)pvParameters;
  while (true)
  {
    if (!RPCSubscribed)
    {
      Serial.println("Subscribing for RPC...");
      if (!thingsBoard.RPC_Subscribe(callbacks.cbegin(), callbacks.cend()))
      {
        Serial.println("Failed to subscribe for RPC");
        return;
      }

      Serial.println("Subscribe RPC done");
      RPCSubscribed = true;
    }
    vTaskDelay(7000 / portTICK_PERIOD_MS);
  }
}

void readDHT20(void *pvParameters)
{
  while (1)
  {
    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();

    // float temperature = random(20, 30); // Simulated temperature
    // float humidity = random(40, 60);    // Simulated humidity

    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");

    bool success = thingsBoard.sendTelemetryData("temperature", temperature) && thingsBoard.sendTelemetryData("humidity", humidity);
    if (success)
      Serial.println("Send data to CoreIoT successfully");
    Serial.println();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

int count = 1;
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial monitor to open
  delay(10000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Initializing DHT20...");

  // Initialize DHT20
  if (!dht20.begin())
  {
    Serial.println("Could not find a valid DHT20 sensor!");
    while (1)
      ;
  }
  initWiFi();
  setupRelays();
  xTaskCreate(reconnectWiFi, "ReconnectWiFiTask", 4096, NULL, 1, NULL);
  xTaskCreate(connectToCoreIoT, "ConnectToCoreIoTTask", 4096, NULL, 1, NULL);
  // xTaskCreate(subscribeSharedAttribute, "SubscribeSharedAtrributeTask", 2048, NULL, 1, NULL);

  xTaskCreate(readDHT20, "ReadDHT20Task", 4096, NULL, 1, NULL);
}

void loop()
{
  // Nothing to do here, FreeRTOS tasks handle the work
  // setRelayState(count, HIGH);
  // Serial.println(count);
  // delay(1000);
  // setRelayState(count, LOW);
  // count = count % 4 + 1;
  // if (!RPCSubscribed)
  // {
  //   Serial.println("Subscribing for RPC...");
  //   if (!thingsBoard.RPC_Subscribe(callbacks.cbegin(), callbacks.cend()))
  //   {
  //     Serial.println("Failed to subscribe for RPC");
  //     return;
  //   }

  //   Serial.println("Subscribe RPC done");
  //   RPCSubscribed = true;
  // }
  thingsBoard.loop();
  delay(1000);
}

// shared attribute
bool turnOn = false;
void processSharedAttributeUpdate(const JsonObjectConst &data)
{
  for (auto it = data.begin(); it != data.end(); ++it)
  {
    Serial.println(it->key().c_str());
    if (strcmp(it->key().c_str(), "turnOn") == 0)
    {
      turnOn = it->value().as<bool>();
      digitalWrite(RELAY_1, turnOn);

      const size_t jsonSize = Helper::Measure_Json(data);
      char buffer[jsonSize];
      serializeJson(data, buffer, jsonSize);
      Serial.println(buffer);

      if (turnOn)
        Serial.println("Turn on RELAY_1");
      else
        Serial.println("Turn off RELAY_1");
    }
  }
}

void processGetValidationValue(const JsonVariantConst &data, JsonDocument &response)
{
  // Process data
  const bool isValid = data["isValid"];
  if (!isValid)
    Serial.println("DHT20 data is not valid");
  else
    Serial.println("DHT20 data is valid");

  // const size_t jsonSize = Helper::Measure_Json(data);
  // char buffer[jsonSize];
  // serializeJson(data, buffer, jsonSize);
  // Serial.println(buffer);
  // Ensure to only pass values do not store by copy, or if they do increase the MaxRPC
  // template parameter accordingly to ensure that the value can be deserialized.RPC_Callback.
  response["isValid"] = isValid;
}
