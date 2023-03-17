// import required libraries, install via library manager if not in your IDE already
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_camera.h>
#include "Base64.h"
const char* ssid = "REPLACE";
const char* password = "REPLACE";

// "allow anonymous" is set to true on the MQTT broker so not using username/pass for testing
// const char* mqtt_username = "your_MQTT_username"; 
// const char* mqtt_password = "your_MQTT_password";

// set the maximum payload size for images, images fail to publish with out this setting
const int MAX_PAYLOAD = 250000; 
const int bufferSize = 1024 * 43; // 23552 bytes

// Initialize the MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Define the GPIO pin for the button
const int buttonPin = 12; // for camara capture esignal
const int LED_GPIO_PIN = 4; // default PIN for the ESP32cam built-in LED

void setup() {
  // Initialize the serial communication
  Serial.begin(115200);

  // Initialize the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config); // initialize the camera with the given configuration
  if (err != ESP_OK) {// if there's an error initializing the camera, print the error message and return
    Serial.printf("Camera init failed with error 0x%x", err); 
    return;
  }

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { // wait until the connection is established
    delay(1000);
    Serial.println(" ");
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("ESP32cam connected to WiFi, IP received: ");
  Serial.println(WiFi.localIP()); // print the local IP address obtained from the Wi-Fi connection
  

  // Connect to MQTT broker
  String mqtt_server = WiFi.gatewayIP().toString(); // get IP of the MQTT broker from the gateway
  client.setServer(mqtt_server.c_str(), 1883); // connect to the MQTT broker using the IP address and port number
  client.setBufferSize (MAX_PAYLOAD); // set the maximum payload size for MQTT messages
  while (!client.connected()) { // wait until the connection is established
    Serial.print("Connecting to MQTT broker: ");
    Serial.println(mqtt_server.c_str());
    if (client.connect("ESP32camClient")) {
      Serial.println("Connected to MQTT broker");
      Serial.println(" ");
    } else {
      Serial.print("Failed with state ");
      Serial.print(client.state()); // print the current state of the client
      delay(2000); // wait for 2 seconds before attempting to reconnect
    }
  }

  // Initialize the button
  pinMode(buttonPin, INPUT_PULLUP); // set the button pin as input with internal pull-up resistor enabled
  pinMode(LED_GPIO_PIN, OUTPUT);  // set the LED pin as output
}

void imageCapture() {
  camera_fb_t * fb = NULL;
  Serial.println("Image: Capture Sequence Start");
  digitalWrite(LED_GPIO_PIN, HIGH); // turn ON the FLASH LED
  fb = esp_camera_fb_get(); // used to get a single picture.
  if (!fb) {  // if the capture fails, print an error message and return
    Serial.println("Camera capture failed");
    return;
  }
  delay(1000); // wait 1 second
  digitalWrite(LED_GPIO_PIN, LOW); // turn OFF the FLASH LED
  char *input = (char *)fb->buf;
  char output[base64_enc_len(3)];
  String imageFile = "data:image/jpeg;base64,";
  for (int i=0;i<fb->len;i++) {
    base64_encode(output, (input++), 3);
    if (i%3==0) imageFile += String(output);
  }
  int fbLen = imageFile.length();
  Serial.println("Image: Captured"); // print a message when the image is successfully captured
  Serial.println("Image Size: ");
  Serial.println(String(fbLen));
  if (client.connected()) {
    Serial.println("Image Publishing: Started");
    Serial.print("Image Topic: ");
    Serial.println("ESP32CAM/image");
    client.beginPublish("ESP32CAM/image", fbLen, true);

    String str = "";
    for (size_t n=0;n<fbLen;n=n+2048) {
      if (n+2048<fbLen) {
        str = imageFile.substring(n, n+2048);
        client.write((uint8_t*)str.c_str(), 2048);
      }
      else if (fbLen%2048>0) {
        size_t remainder = fbLen%2048;
        str = imageFile.substring(n, n+remainder);
        client.write((uint8_t*)str.c_str(), remainder);
      }
    }  
    client.endPublish();
    delay(500); // wait 1 second
    Serial.println("Image Publish: Finished");
    Serial.println("");
    esp_camera_fb_return(fb);
    
    return;
  }
  
}

void loop() {
  // Check if the button is pressed
  if (digitalRead(buttonPin) == LOW) {
    Serial.println(" ");
    Serial.println("GPIO: PIN 12 - LOW signal detected");
    // Capture Image and publish
    imageCapture();
    delay(1000);
  }

//   Reconnect to MQTT broker if disconnected
  if (!client.connected()) {
    if (client.connect("ESP32camClient")) {
    Serial.println("Reconnected to MQTT broker");
    } else {
    Serial.print("Failed to reconnect to MQTT broker with state ");
    Serial.print(client.state());
    }
  }

// Maintain the MQTT connection
client.loop();
}