/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <acera_detect_inferencing.h>
//#include <acera_detect_prueba_inferencing.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
# include <SoftwareSerial.h>
# include <TinyGPS++.h>
//#include <ArduinoMqttClient.h>
//#include <WiFiNINA.h>

//const char* ssid     = "Livebox6-795D";
//const char* password = "KV7RG7hPRFhz";

// Broker MQTT
//char mqtt_server[] = "192.168.162.182:1883";
//const int mqtt_port = 1883; 
//char ssid[] = "hola";
//char pass[] = "123";

//WiFiClient client;
//MqttClient mqttClient(client);

//WiFiClient espClient;
//PubSubClient client(espClient);
StaticJsonDocument<256> doc;

typedef enum _ledState{
    OFF,
    BLINK_RED,
    RED,
    GREEN,
    BLINK_BLUE,
    BLUE
};

_ledState ledState;

Adafruit_NeoPixel leds(2, 23, NEO_GRB + NEO_KHZ800);

static int fusion_ix = 3;
static const bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal



/**
* @brief      Arduino setup function
*/
void setup()
{
  leds.begin();
  leds.clear();
  
  /* Init serial */
  Serial.begin(115200);
//  gpsSerial.begin(115200);

  Serial.println("Booting");
  /*
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
  */


//  client.setServer(mqtt_server, mqtt_port);
  //client.setCallback(callback);
  //String clientId = "VENT_ANOMALY";
//  client.connect(clientId.c_str(), mqtt_user, mqtt_password);
  
  Serial.println("inference start...");
  leds.setPixelColor(0, leds.Color(0, 0, 255));
  leds.setPixelColor(1, leds.Color(0, 0, 255));
  leds.show();

}


/**
* @brief      Get data and run inferencing
*/
void loop()
{
  ArduinoOTA.handle();
  ei_printf("\nStarting inferencing in 2 seconds...\r\n");

  delay(2000);

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != fusion_ix) {
    ei_printf("ERR: Sensors don't match the sensors required in the model\r\n"
        "Following sensors are required: %s\r\n", EI_CLASSIFIER_FUSION_AXES_STRING);
    return;
  }

  ei_printf("Sampling...\r\n");

  // Allocate a buffer here for the values we'll read from the sensor
  float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

  for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
    // Determine the next tick (and then sleep later)
    int64_t next_tick = (int64_t)micros() + ((int64_t)EI_CLASSIFIER_INTERVAL_MS * 1000);

    buffer[ix]   = float(analogRead(36));
    buffer[ix+1] = float(analogRead(39));
    buffer[ix+2] = float(analogRead(34));

    int64_t wait_time = next_tick - (int64_t)micros();

    if(wait_time > 0) {
      delayMicroseconds(wait_time);
    }
  }

  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    
  if (err != 0) {
    ei_printf("ERR:(%d)\r\n", err);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = { 0 };

  err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR:(%d)\r\n", err);
    return;
  }

  // print the predictions
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\r\n",
    result.timing.dsp, result.timing.classification, result.timing.anomaly);
    
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("%s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);
  }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  //ei_printf("EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME = %d\n", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
  //ei_printf("EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE = %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  if (result.anomaly < 0.85){
    ei_printf("ACERA, con un anomaly score: %.3f\r\n", result.anomaly);
  }
  else{
    ei_printf("OTROS, con un anomaly score: %.3f\r\n", result.anomaly);
  }
  //ei_printf("    anomaly score: %.3f\r\n", result.anomaly);

  /**************************************************************************************************************/
  /* TODO AQUÍ FALTA COMPARAR LA VARIABLE result.anomaly con el valor límite para discernir entre acera o resto */
  /**************************************************************************************************************/
  
  //mqttClient.beginMessage()

  //client.publish(String("/ANOMALY/").c_str(), String(result.anomaly).c_str());
  /*if (result.anomaly > 5) {
    leds.setPixelColor(0, leds.Color(255, 0, 0));
    leds.setPixelColor(1, leds.Color(255, 0, 0));
  } else {
    leds.setPixelColor(0, leds.Color(0, 255, 0));
    leds.setPixelColor(1, leds.Color(0, 255, 0));
  }
  leds.show();
  */
#endif


}
