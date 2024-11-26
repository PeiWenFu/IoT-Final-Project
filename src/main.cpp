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
#define ULSO_TRIG_PIN -1
#define ULSO_ECHO_PIN -1
#define BUZZER_PIN 32
#define RX_PIN -1
#define TX_PIN -1
// Thresholds
#define NIGHT_MODE_THRESHOLD 1000
#define OBJECT_DETECTION_THRESHOLD 200
#define MOVING_THRESHOLD 0.5

int lightVal;

Ultrasonic ultrasonic(ULSO_TRIG_PIN, ULSO_ECHO_PIN);
unsigned long previousMillis = 0;  
unsigned long beepDuration = 100;  // Duration of each beep (100 ms)
unsigned long pauseDuration = 200; // Initial pause duration (200 ms)
bool isBeeping = false;

LSM6DSO myIMU; 
float offsetX;
float offsetY;
float offsetZ;
unsigned long lastDetectionTime;
const unsigned long detectionCooldown = 1000; // 1 second cooldown

char ssid[50]; // your network SSID (name) 
char pass[50]; // your network password (use for WPA, or use 
               // as key for WEP) 

// Number of milliseconds to wait without receiving any data before we give up 
const int kNetworkTimeout = 30 * 1000; 
// Number of milliseconds to wait if no data is available before trying again 
const int kNetworkDelay = 1000; 

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
        //Serial.printf("SSID = %s\n", ssid); 
        //Serial.printf("PASSWD = %s\n", pass); 
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

  err = http.get("54.183.166.218", 5000, url.c_str(), NULL);
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

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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
  nvs_access(); // Retrieve SSID/PASSWD from flash before anything else 

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
  gpsSerial.begin(4800, SERIAL_8N1,RX_PIN, TX_PIN);
}

void loop() {
  // Automatic Night Lighting
  lightVal = analogRead(LIGHT_SENSOR_PIN);
  Serial.print("lightVal: ");
  Serial.println(lightVal);
  if (lightVal < NIGHT_MODE_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }


  // Pedestrian Detection and Warning
  unsigned int distance = ultrasonic.read();

  if (distance < OBJECT_DETECTION_THRESHOLD) {
    // Map the distance (10cm to 200cm) to a pause time range (200ms to 50ms)
    pauseDuration = map(distance, 10, OBJECT_DETECTION_THRESHOLD, 200, 50);
    
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= (isBeeping ? beepDuration : pauseDuration)) {
      previousMillis = currentMillis;

      if (isBeeping) {
        noTone(BUZZER_PIN);
      } else {
        tone(BUZZER_PIN, 1000);
      }

      isBeeping = !isBeeping;
    }
  } else {
    noTone(BUZZER_PIN);
  }


  // Theft Prevention System
  // TODO: determine if the scooter is with the owner
  // if (the owner is not around) {
    float correctedX = myIMU.readFloatAccelX() - offsetX;
    float correctedY = myIMU.readFloatAccelY() - offsetY;
    float correctedZ = myIMU.readFloatAccelZ() - offsetZ;
    float accelMagnitude = sqrt(pow(correctedX, 2) + 
                                pow(correctedY, 2) + 
                                pow(correctedZ, 2));

    if (millis() - lastDetectionTime > detectionCooldown) {
        if (accelMagnitude > MOVING_THRESHOLD) {
          // Get current time
          // FIXME: use Wifi or gps to get time
          time_t now = time(nullptr);
          tm* local_tm = localtime(&now);
          char formatted_tm[80];
          strftime(formatted_tm, sizeof(formatted_tm), "%Y-%m-%d-%H:%M:%S", local_tm);

          // Send warning message to website
          sendToAWS("/theft?message=Movement%20is%20detected%20at%20" + String(formatted_tm));
          // Serial.println("Movement is detected at " + String(formatted_tm));
          lastDetectionTime = millis();
      }
    }
  // }


  // Real-time Scooter Location Tracking
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    String latitute = String(gps.location.lat(), 6);
    String longtitute = String(gps.location.lng(), 6);

    // Send current location to the website
    sendToAWS("/gps?latitute=" + latitute + "&longtitute=" + longtitute);
  }
}
