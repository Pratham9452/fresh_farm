#include <Arduino.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
//device verification:



// Sensor and Credentials Configuration
#define LM35_PIN 34 // ADC pin connected to LM35
const float REF_VOLTAGE = 5.116;    // Reference voltage of the ESP32 ADC (in volts)
const int ADC_RESOLUTION = 15.455705; // 12-bit ADC resolution
const float LM35_SCALE = 100.0;  // LM35 outputs 10mV/°C; scale factor is 100 for °C
#define GPS_RX_PIN          16      // RX pin for GPS module
#define GPS_TX_PIN          17      // TX pin for GPS module
#define GPS_BAUD_RATE       9600    // GPS module baud rate

// WiFi Credentials
#define WIFI_SSID       "Edge20"
#define WIFI_PASSWORD   "123456780"

// AWS IoT Configuration
#define AWS_IOT_ENDPOINT        "a2tcuarbws21nt-ats.iot.eu-north-1.amazonaws.com"
#define AWS_THING_NAME          "FARM_1"
#define AWS_IOT_PUBLISH_TOPIC   "ESP32_DATA"
#define Device_ID                "ESP_1"

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

// WiFi and MQTT Clients
WiFiClientSecure net;
PubSubClient mqttClient(net);

// Sensor Data Variables
float temperature = 0.0;
float humidity = 0.0;
double latitude = 0.0;
double longitude = 0.0;
int satelliteCount = 0;

// AWS IoT Certificates (Replace with your actual certificates)
const char AWS_ROOT_CA[] = R"(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)";

const char AWS_DEVICE_CERT[] = R"(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)";

const char AWS_DEVICE_PRIVATE_KEY[] = R"(
-----BEGIN RSA PRIVATE KEY-----

-----END RSA PRIVATE KEY-----
)";

void setupWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nWiFi Connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
}

void setupAWSIoT() {
    // Configure SSL/TLS certificates
    net.setCACert(AWS_ROOT_CA);
    net.setCertificate(AWS_DEVICE_CERT);
    net.setPrivateKey(AWS_DEVICE_PRIVATE_KEY);

    // Configure MQTT Broker
    mqttClient.setServer(AWS_IOT_ENDPOINT, 8883);
    mqttClient.setBufferSize(512);

    while (!mqttClient.connected()) {
        Serial.print("Connecting to AWS IoT...");
        if (mqttClient.connect(AWS_THING_NAME)) {
            Serial.println("Connected!");
            break;
        } else {
            Serial.print("Failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" Retrying in 5 seconds");
            delay(5000);
        }
    }
}

void readLM35Sensor() {
  // Read ADC value from LM35 sensor
  int adcValue = analogRead(LM35_PIN);

  // Convert ADC value to voltage
  float voltage = (adcValue * REF_VOLTAGE) / ADC_RESOLUTION;

  // Convert voltage to temperature in °C (LM35: 10mV per °C)
  temperature = ((voltage * LM35_SCALE)/1000)+0.25;
}


void readGPSData() {
    while (GPSSerial.available() > 0) {
        if (gps.encode(GPSSerial.read())) {
            if (gps.location.isValid()) {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                satelliteCount = gps.satellites.value();
            }
        }
    }
}

void publishSensorData() {
    if (!mqttClient.connected()) {
        setupAWSIoT();
    }

    StaticJsonDocument<512> jsonDoc;
    
    jsonDoc["device"] = AWS_THING_NAME;
    jsonDoc["device_id"] = Device_ID;
    jsonDoc["temperature"] = temperature;
    jsonDoc["humidity"] = humidity;
    jsonDoc["latitude"] = latitude;
    jsonDoc["longitude"] = longitude;
    jsonDoc["satellites"] = satelliteCount;
    jsonDoc["timestamp"] = millis();

    // Serialize JSON
    char jsonBuffer[512];
    serializeJson(jsonDoc, jsonBuffer);

    // Publish to AWS IoT Topic
    bool publishResult = mqttClient.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
    
    if (publishResult) {
        Serial.println("Successfully published sensor data");
    } else {
        Serial.println("Failed to publish sensor data");
    }
}

void printSensorData() {
    // Print DHT22 Data
    Serial.println("--- LM35 Sensor Data ---");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    // Print GPS Data
    Serial.println("\n--- GPS Sensor Data ---");
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
    
    Serial.print("Satellites: ");
    Serial.println(satelliteCount);
    
    Serial.println("------------------------\n");
}

void setup() {
    // Initialize Serial Communication
    Serial.begin(115200);
    
    // Initialize Sensors
   
    GPSSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // Connect to WiFi
    setupWiFi();
    
    // Connect to AWS IoT
    setupAWSIoT();
}

void loop() {
    // Read Sensor Data
    readLM35Sensor();
    readGPSData();
    
    // Print Sensor Data to Serial Monitor
    printSensorData();
    
    // Publish to AWS IoT
    publishSensorData();
    
    // Maintain MQTT Connection
    mqttClient.loop();
    
    // Wait before next read
    delay(5000);
}
