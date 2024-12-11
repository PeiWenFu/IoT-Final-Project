#include <Arduino.h>
#include <Ultrasonic.h>
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include <HttpClient.h> 
#include <WiFi.h> 
#include <inttypes.h> 
#include <stdio.h> 
#include "esp_system.h" 
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "nvs.h" 
#include "nvs_flash.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// Pin numbers
#define LIGHT_SENSOR_PIN 33
#define LED_PIN 27
#define ULSO_TRIG_PIN 12
#define ULSO_ECHO_PIN 13
#define BUZZER_PIN 32
#define RX_PIN 15
#define TX_PIN 2
// Thresholds
#define NIGHT_MODE_THRESHOLD 1000
#define OBJECT_DETECTION_THRESHOLD 350
#define MOVING_THRESHOLD 0.5


int lightVal;

Ultrasonic ultrasonic(ULSO_TRIG_PIN, ULSO_ECHO_PIN);
unsigned long beepDuration = 100;  // Duration of each beep (100 ms)
unsigned long pauseDuration = 200; // Initial pause duration (200 ms)
bool isBeeping = false;
TaskHandle_t distanceTaskHandle;
TaskHandle_t buzzerTaskHandle;

LSM6DSO myIMU; 
float offsetX;
float offsetY;
float offsetZ;
unsigned long lastDetectionTime;
const unsigned long detectionCooldown = 1000; // 1 second cooldown
bool movementUpdated;

char ssid[50]; // your network SSID (name) 
char pass[50]; // your network password (use for WPA, or use as key for WEP) 
// Number of milliseconds to wait without receiving any data before we give up 
const int kNetworkTimeout = 30 * 1000; 
// Number of milliseconds to wait if no data is available before trying again 
const int kNetworkDelay = 1000; 

TinyGPSPlus gps;
String latitude;
String longitude;
bool gpsUpdated;

void nvs_access() { 
  // Initialize NVS 
  esp_err_t err = nvs_flash_init(); 
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || 
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) { 
    // NVS partition was truncated and needs to be erased 
    // Retry nvs_flash_init 
    ESP_ERROR_CHECK(nvs_flash_erase()); 
    err = nvs_flash_init(); 
  } 
  ESP_ERROR_CHECK(err); 
  
  // Open 
  Serial.printf("\n"); 
  Serial.printf("Opening Non-Volatile Storage (NVS) handle... "); 
  nvs_handle_t my_handle; 
  err = nvs_open("storage", NVS_READWRITE, &my_handle); 
  if (err != ESP_OK) { 
    Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err)); 
  } else { 
    Serial.printf("Done\n"); 
    Serial.printf("Retrieving SSID/PASSWD\n"); 
    size_t ssid_len; size_t pass_len; 
    err = nvs_get_str(my_handle, "ssid", ssid, &ssid_len); 
    err |= nvs_get_str(my_handle, "pass", pass, &pass_len); 
    switch (err) { 
      case ESP_OK:
        Serial.printf("Done\n"); 
        // Serial.printf("SSID = %s\n", ssid); 
        // Serial.printf("PASSWD = %s\n", pass); 
        break; 
      case ESP_ERR_NVS_NOT_FOUND: 
        Serial.printf("The value is not initialized yet!\n"); 
        break; 
      default: 
        Serial.printf("Error (%s) reading!\n", esp_err_to_name(err)); 
    } 
  } 
  
  // Close 
  nvs_close(my_handle); 
} 

void sendToAWS(String url) {
  int err = 0; 

  WiFiClient c; 
  HttpClient http(c); 

  err = http.get("54.177.79.48", 5000, url.c_str(), NULL);
  if (err == 0) { 
    Serial.println("startedRequest ok"); 
    
    err = http.responseStatusCode(); 
    if (err >= 0) { 
      Serial.print("Got status code: "); 
      Serial.println(err); 
      
      // Usually you'd check that the response code is 200 or a 
      // similar "success" code (200-299) before carrying on, 
      // but we'll print out whatever response we get 
      
      err = http.skipResponseHeaders(); 
      if (err >= 0) { 
        int bodyLen = http.contentLength(); 
        Serial.print("Content length is: "); 
        Serial.println(bodyLen); 
        Serial.println("Body returned follows:"); 
        
        // Now we've got to the body, so we can print it out 
        unsigned long timeoutStart = millis(); 
        char c; 
        // Whilst we haven't timed out & haven't reached the end of the body 
        while ((http.connected() || http.available()) && 
               ((millis() - timeoutStart) < kNetworkTimeout)) { 
          if (http.available()) { 
            c = http.read(); 
            // Print out this character 
            Serial.print(c); 
            
            bodyLen--; 
            // We read something, reset the timeout counter 
            timeoutStart = millis(); 
          } else { 
            // We haven't got any data, so let's pause to allow some to 
            // arrive 
            delay(kNetworkDelay); 
          }
        }
        Serial.println();
        Serial.println();
      } else {
        Serial.print("Failed to skip response headers: "); 
        Serial.println(err); 
      } 
    } else { 
      Serial.print("Getting response failed: "); 
      Serial.println(err); 
    } 
  } else { 
    Serial.print("Connect failed: "); 
    Serial.println(err); 
  } 
  http.stop(); 
}

// Distance measurement task
void distanceTask(void *pvParameters) {
  for(;;) {
    unsigned int distance = ultrasonic.read();

    if (distance < OBJECT_DETECTION_THRESHOLD) {
      pauseDuration = map(distance, 10, OBJECT_DETECTION_THRESHOLD, 10, 200);

      // Notify the buzzer task to run
      vTaskResume(buzzerTaskHandle);
    } else {
      // Turn off buzzer and pause the buzzer task
      noTone(BUZZER_PIN);
      isBeeping = false;
      vTaskSuspend(buzzerTaskHandle); // Suspend the buzzer task when not needed
    }

    vTaskDelay(40 / portTICK_PERIOD_MS); // Run every 10ms
  }
}

// Buzzer control task
void buzzerTask(void *pvParameters) {
  unsigned long lastToggleTime = millis();

  for(;;) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastToggleTime >= (isBeeping ? beepDuration : pauseDuration)) {
      lastToggleTime = currentMillis;

      if (isBeeping) {
        noTone(BUZZER_PIN);
      } else {
        tone(BUZZER_PIN, 1000);
      }

      isBeeping = !isBeeping;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Check every 10ms
  }
}


void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED initialized. ");
  Serial.println(xPortGetCoreID());

  // Create distance and buzzer tasks
  xTaskCreatePinnedToCore(buzzerTask, "Buzzer Task", 2048, NULL, 1, &buzzerTaskHandle, 0);
  delay(500); 
  xTaskCreatePinnedToCore(distanceTask, "Distance Task", 2048, NULL, 1, &distanceTaskHandle, 0);
  delay(500); 
  // Start FreeRTOS scheduler
  vTaskStartScheduler();
  Serial.println("Distance and buzzer tasks created. ");

  // Initialize accelerometer
  Wire.begin();
  delay(10);
  if(myIMU.begin())
      Serial.println("Ready.");
  else { 
      Serial.println("Could not connect to IMU.");
      Serial.println("Freezing");
  }

  if(myIMU.initialize(BASIC_SETTINGS))
      Serial.println("Loaded Settings.");
  
  offsetX = myIMU.readFloatAccelX();
  offsetY = myIMU.readFloatAccelY();
  offsetZ = myIMU.readFloatAccelZ();

  // Connect to WiFi
  nvs_access(); // Retrieve SSID/PASSWD from flash

  delay(1000); 
  Serial.println(); 
  Serial.println(); 
  Serial.print("Connecting to "); 
  Serial.println(ssid); 

  WiFi.begin(ssid, pass); 

  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  } 
  
  Serial.println(""); 
  Serial.println("WiFi connected"); 
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP()); 
  Serial.println("MAC address: "); 
  Serial.println(WiFi.macAddress()); 
  Serial.println();

  // Start gps serial
  Serial2.begin(9600, SERIAL_8N1,TX_PIN, RX_PIN);
}

void loop() {
  // Automatic Night Lighting
  lightVal = analogRead(LIGHT_SENSOR_PIN);
  // Serial.print("lightVal: ");
  // Serial.println(lightVal);
  if (lightVal < NIGHT_MODE_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }


  // Theft Prevention System
  float correctedX = myIMU.readFloatAccelX() - offsetX;
  float correctedY = myIMU.readFloatAccelY() - offsetY;
  float correctedZ = myIMU.readFloatAccelZ() - offsetZ;
  float accelMagnitude = sqrt(pow(correctedX, 2) + 
                              pow(correctedY, 2) + 
                              pow(correctedZ, 2));

  if (accelMagnitude > MOVING_THRESHOLD) {
        movementUpdated = true;
  }


  // Real-time Scooter Location Tracking
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
  if (gps.location.isUpdated()) {
    latitude = String(gps.location.lat(), 6);
    longitude = String(gps.location.lng(), 6);
    gpsUpdated = true;

    // Serial.print("latitude: ");
    // Serial.print(latitude);
    // Serial.print(", longitude: ");
    // Serial.println(longitude);
  }
  if (gps.charsProcessed() < 10)
    Serial.println(F("WARNING: No GPS data.  Check wiring."));

  // Send data to server
  if (millis() - lastDetectionTime > detectionCooldown) {
    if (movementUpdated && gpsUpdated) {
      sendToAWS("/both?message=Movement%20is%20detected%20at%20&latitude=" + latitude + "&longitude=" + longitude);
    } else if (movementUpdated) {
      sendToAWS("/theft?message=Movement%20is%20detected%20at%20");
    } else if (gpsUpdated) {
      sendToAWS("/gps?latitude=" + latitude + "&longitude=" + longitude);
    }
    movementUpdated = false;
    gpsUpdated = false;
    lastDetectionTime = millis();
  }
}
