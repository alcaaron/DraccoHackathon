#include <WiFi.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <HTTPClient.h>


const char* ssid     = "Redmi Note 8T";
const char* password = "tdraaron";


#define MAC_SIZE 15
char sMac[MAC_SIZE];

Adafruit_NeoPixel leds(2, 23, NEO_GRB + NEO_KHZ800);
StaticJsonDocument<256> doc;
WiFiClient client;


// Server, file, and port
const char hostname[] = "192.168.118.182";
const String uri = "/";
const int port = 1337;


// Settings
const float collection_period = 2;      // How long to collect (sec)
const int sample_rate = 100;            // How fast to collect (Hz)
const int num_dec = 7;                  // Number of decimal places
const int num_samples = (int)(collection_period * sample_rate);
const unsigned long timeout = 500;      // Time to wait for server response

const size_t json_capacity = (3 * JSON_ARRAY_SIZE(num_samples)) + JSON_OBJECT_SIZE(3);

DynamicJsonDocument json(json_capacity);

JsonArray json_x = json.createNestedArray("x");
JsonArray json_y = json.createNestedArray("y");
JsonArray json_z = json.createNestedArray("z");


typedef enum _ledState{
    OFF,
    BLINK_RED,
    RED,
    GREEN,
    BLINK_BLUE,
    BLUE
};

_ledState ledState;

static bool accel_found = false;


/* freeRTOS components ***********/

TaskHandle_t    ReadTVOCsensorHandler, sampleAccelerometerHandler;



// Send GET request to server to get "ready" flag
int getServerReadyFlag(unsigned long timeout) {

  int ret_status = -1;
  unsigned long timestamp;

  // Make sure we're connected to WiFi
  if (WiFi.status() == WL_CONNECTED) {

    // Connect to server
#if DEBUG
    Serial.print("Connecting to: ");
    Serial.println(hostname);
#endif
    if (client.connect(hostname, port)) {
    
      // Sent GET request
  #if DEBUG
      Serial.println("Sending GET request");
  #endif
      client.print("GET " + uri + " HTTP/1.1\r\n" + 
                "Host: " + hostname + "\r\n" +
                "Connection: close\r\n\r\n");
      
      // Wait for up to specified time for response from server
      timestamp = millis();
      while (!client.available()) {
        if (millis() > timestamp + timeout) {
          ledState = RED;
#if DEBUG
          Serial.println("GET response timeout");
#endif
          return -1;
        }
      }

      // Header should take up 4 lines, so throw them away
      for (int i = 0; i < 4; i++) {
        if (client.available()) {
          String resp = client.readStringUntil('\r');
        } else {
          return -1;
        }
      }

      // Response from server should be only a 0 or 1
      if (client.available()) {
        String resp = client.readStringUntil('\r');
        resp.trim();
#if DEBUG
        Serial.print("Server response: ");
        Serial.println(resp);
#endif
        if (resp == "0") {
          ret_status = 0;
        } else if (resp == "1") {
          ret_status = 1;
        }
      } else {
        return -1;
      }
    }

    // Close TCP connection
    client.stop();
#if DEBUG
    Serial.println();
    Serial.println("Connection closed");
#endif
  }

  return ret_status;
}

int sendPostRequest(DynamicJsonDocument json, unsigned long timeout) {

  unsigned long timestamp;
  
  // Connect to server
#if DEBUG
  Serial.print("Connecting to ");
  Serial.println(hostname);
#endif
  if (!client.connect(hostname, port)) {
#if DEBUG
    Serial.println("Connection failed");
#endif
    return 0;
  }

  // Send HTTP POST request
#if DEBUG
  Serial.println("Sending POST request");
#endif
  client.print("POST " + uri + " HTTP/1.1\r\n" +
               "Host: " + hostname + "\r\n" +
               "Connection: close\r\n" +
               "Content-Type: application/json\r\n" +
               "Content-Length: " + measureJson(json) + "\r\n" +
               "\r\n");

  // Send JSON data
  serializeJson(json, client);

  // Wait for up to specified time for response from server
  timestamp = millis();
  while (!client.available()) {
    if (millis() > timestamp + timeout) {
      ledState = RED;
#if DEBUG
      Serial.println("GET response timeout");
#endif
      return -1;
    }
  }

  // Print response
#if DEBUG
  while (client.available() ) {
    String ln = client.readStringUntil('\r');
    Serial.print(ln);
  }
  Serial.println();
#endif

  // Close TCP connection
  client.stop();
#if DEBUG
  Serial.println();
  Serial.println("Connection closed");
#endif

  return 1;
}



/* sample analog readings from accelerometer ************/

void vSampleAccTask(void *pvParameters) {

  int x,y,z,i;
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  
  i = 0;
  
  for(;;) {
    x = analogRead(36);
    y = analogRead(39);
    z = analogRead(34);
    
    json_x.add(x);
    json_y.add(y);
    json_z.add(z);

    i++;

    if (i >= num_samples) {
      
      int resp_status = getServerReadyFlag(timeout);
        if (resp_status != 1) {
        ledState = RED;
      } else {
        ledState = BLINK_BLUE;
        sendPostRequest(json, timeout);
        ledState = OFF;
      }
      
      //DynamicJsonDocument json(json_capacity);
      json.clear();
      JsonArray json_x = json.createNestedArray("x");
      JsonArray json_y = json.createNestedArray("y");
      JsonArray json_z = json.createNestedArray("z");
      i = 0; 
    }
  
    vTaskDelay((1000/sample_rate) / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL); 
}


/* read tvoc sensor task ***********/

void vReadTVOCsensorTask(void *pvParameters) {

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  static bool blink; 

  for (;;) {

    switch (ledState) {
      case OFF:       leds.clear(); break;   
      case BLINK_RED: if (blink) {
                          leds.setPixelColor(0, leds.Color(150, 0, 0));
                          leds.setPixelColor(1, leds.Color(0, 0, 0));
                      } else {
                          leds.setPixelColor(0, leds.Color(0, 0, 0));
                          leds.setPixelColor(1, leds.Color(150, 0, 0));
                      }
                      blink = !blink;
                      break;
      case RED:       leds.setPixelColor(0, leds.Color(150, 0, 0));
                      leds.setPixelColor(1, leds.Color(150, 0, 0));
                      break;
      
      case GREEN:     leds.setPixelColor(0, leds.Color(0, 150, 0));
                      leds.setPixelColor(1, leds.Color(0, 150, 0));
                      break;
        
      case BLINK_BLUE:if (blink) {
                          leds.setPixelColor(0, leds.Color(0, 0, 150));
                          leds.setPixelColor(1, leds.Color(0, 0, 0));
                      } else {
                          leds.setPixelColor(0, leds.Color(0, 0, 0));
                          leds.setPixelColor(1, leds.Color(0, 0, 150));
                      }
                      blink = !blink;
                      break;
      case BLUE:      leds.setPixelColor(0, leds.Color(0, 0, 150));
                      leds.setPixelColor(1, leds.Color(0, 0, 150));
                      break;
                      
    }
    leds.show();

    
    //xf = map(x, 0, 4095, -1500, 1500);
    //yf = map(y, 0, 4095, -1500, 1500);
    //zf = map(z, 0, 4095, -1500, 1500);
      
    
    (ledState == BLINK_RED) ? vTaskDelay(300/ portTICK_PERIOD_MS) : vTaskDelay(1000/ portTICK_PERIOD_MS);
    
  }
  vTaskDelete(NULL);
}


String szGetMac()
{
  byte mac[6];
  String szMAC = "";
  char szMac[3];

  WiFi.macAddress(mac);
  //for (int i = 5; i >= 0; i--)
  for (int i = 0; i < 6; i++)
  {
    if (mac[i] > 0x0F)
      sprintf(szMac, "%2X", mac[i]);
    else
      sprintf(szMac, "0%X", mac[i]);
    szMAC += szMac;
  }

  return szMAC;
}





void setup() {
  leds.begin();
  leds.clear();

  xTaskCreatePinnedToCore(
      vReadTVOCsensorTask,
      "read tvoc sensor",
      8192, 
      NULL,
      tskIDLE_PRIORITY + 1,                  /* priority of the task */
      &ReadTVOCsensorHandler,
      0);                 /* Task attached to core 1 */

  ledState = BLINK_RED;
  xTaskNotifyGive(ReadTVOCsensorHandler);

  xTaskCreatePinnedToCore(
      vSampleAccTask,
      "accelerometer sample",
      8192, 
      NULL,
      tskIDLE_PRIORITY + 2,                  /* priority of the task */
      &sampleAccelerometerHandler,
      1);                 /* Task attached to core 1 */

 
  
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

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

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  szGetMac().toCharArray(sMac, MAC_SIZE); 

  ledState = OFF;
  accel_found = true;

  xTaskNotifyGive(sampleAccelerometerHandler);
}


void loop() {
  ArduinoOTA.handle();
}
