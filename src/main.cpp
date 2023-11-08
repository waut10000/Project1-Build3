#include <Arduino.h>
#include <OneWire.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_camera.h>
#include <DFRobot_AXP313A.h>
#include <security.h>

// Camera setup
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 45
#define SIOD_GPIO_NUM 1
#define SIOC_GPIO_NUM 2

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 46
#define Y7_GPIO_NUM 8
#define Y6_GPIO_NUM 7
#define Y5_GPIO_NUM 4
#define Y4_GPIO_NUM 41
#define Y3_GPIO_NUM 40
#define Y2_GPIO_NUM 39
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 42
#define PCLK_GPIO_NUM 5

// Temperature setup
#define DS18S20_Pin D2
// Temperature chip i/o
OneWire ds(DS18S20_Pin);
// variable for temperature
float temperature = 0;

// variable for millis
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 5000; // 5 seconds in milliseconds
unsigned long currentMillisPhoto = 0;
unsigned long previousMillisPhoto = 0;
const unsigned long intervalPhoto = 60000; // 60 seconds - 1min in milliseconds

// time for deepsleep
#define timeToSleep 60 // 15min - 900 sec
#define timeToSecond 1000000

// camera loop
bool state = true;

// MQTT topics
const char *mqttPubTopicTemp = "esp32/temp";
const char *mqttPubTopicVisual = "esp32/visual";
const char *mqttSubTopicState = "esp32/state";
// MQTT setup
WiFiClient espClient;
PubSubClient client(espClient);

DFRobot_AXP313A axp;

// add function to this file
void startCameraServer();

void setup_wifi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void cameraSetup()
{
  while (axp.begin() != 0)
  {
    Serial.println("init error");
    delay(1000);
  }
  axp.enableCameraPower(axp.eOV2640); // 设置摄像头供电
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    if (psramFound())
    {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }
  else
  {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif
}

void reconnect() // connect and reconnect to mqtt broker
{
  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {
      Serial.println("Connected to MQTT broker");
      client.subscribe(mqttSubTopicState);
      client.subscribe(mqttPubTopicTemp);
      client.subscribe(mqttPubTopicVisual);
    }
    else
    {
      Serial.println("Failed to connect to MQTT broker. Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("Received message on topic: " + String(topic));
  // Handle incoming MQTT messages if needed

  String msg = "";
  for (int i = 0; i < length; i++)
  {
    msg += (char)payload[i];
  }

  if (msg == "false")
  {
    state = false;
    Serial.println("State has changed to false");
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Waking up...");
  setup_wifi();
  cameraSetup();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  esp_sleep_enable_timer_wakeup(timeToSleep * timeToSecond); // wake up trigger by timer
  delay(1000);
}

void deepsleep() // start deepsleep
{
  Serial.println("Going to sleep now  Zz..");
  delay(1000);
  Serial.flush();
  esp_deep_sleep_start();
}

// source: https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_#target_2
float getTemp()
{
  // returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];
  // error checks
  if (!ds.search(addr))
  {
    // no more sensors on chain, reset search
    ds.reset_search();
    return -500;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    Serial.println("CRC is not valid!");
    return -750;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28)
  {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++)
  { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); // using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

void sendTempToNodeRed(float temp)
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // Publish the temperature reading
  char payload[10];
  dtostrf(temperature, 4, 2, payload); // Convert float to string
  client.publish(mqttPubTopicTemp, payload);
  Serial.println("Temperature send!");
}

/*sources for camera:
  https://wiki.dfrobot.com/SKU_DFR0975_FireBeetle_2_Board_ESP32_S3#target_5
  https://wiki.dfrobot.com/SKU_DFR0975_FireBeetle_2_Board_ESP32_S3_Advanced_Tutorial#target_9
  https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/
  https://arduino.stackexchange.com/questions/72429/esp32-cam-publish-image-to-mqtt
  https://randomnerdtutorials.com/esp32-cam-take-photo-display-web-server/
*/
void sendLinkToNodeRed(String payload) // send immage to node red with mqtt
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  
  client.publish(mqttPubTopicVisual, payload.c_str());
  Serial.println("Sending link to node red with mqtt");
}

void activateStream()
{
  startCameraServer();
  
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream' to connect");
  IPAddress localIP = WiFi.localIP();
  String payload = "http://" + localIP.toString() + ":81/stream";
  sendLinkToNodeRed(payload);
}

void endStream()
{
  String empty = "";
  Serial.println("ending stream");
  sendLinkToNodeRed(empty);
}

void loop()
{
  temperature = getTemp();
  // Get the current time
  currentMillisPhoto = millis();
  // wait 60 seconds
  if (currentMillisPhoto - previousMillisPhoto >= (20 * timeToSecond))
  {
    previousMillisPhoto = currentMillisPhoto;
    sendTempToNodeRed(temperature);
    Serial.println(String(temperature) + " °C");
    Serial.println("Waited for 20 seconds!");
  }

  sendTempToNodeRed(temperature);
  Serial.println(String(temperature) + " °C");

  // if the temp is > 25 °C, take and send picture
  if (temperature > 25)
  {
    activateStream();

    while (state)
    {
      if (!client.connected())
      {
        reconnect();
      }
      client.loop();
      client.subscribe(mqttSubTopicState);

      // Get the current time
      currentMillisPhoto = millis();
      // wait 60 seconds
      if (currentMillisPhoto - previousMillisPhoto >= intervalPhoto)
      {
        previousMillisPhoto = currentMillisPhoto;
        Serial.println("Waited for 60 seconds!");
        temperature = getTemp();
        sendTempToNodeRed(temperature);
      }
    }
    endStream();
    // Get the current time
    currentMillis = millis();
    // wait 5 seconds
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      Serial.println("Waited for 5 seconds!");
      deepsleep();
    }
  }
  else
  {
    endStream();
    // Get the current time
    currentMillis = millis();
    // wait 5 seconds
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      Serial.println("Waited for 5 seconds!");
      deepsleep();
    }
  }
}