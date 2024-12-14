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
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)";

const char AWS_DEVICE_CERT[] = R"(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVALab2ON8mebGY8mVK0tjhxP1J9goMA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yNDEyMTMyMDIw
NTZaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDe4Dxl0sUJP0Ghkoep
r9EgB6EQagOb4O7Dqd6AexJoZcypg73VLiQf5RI35vMRF/JEo7QWIEcpvx+UrhAi
Vcwqtulaycl9WnsiA+uCLjPv59dQz/5WwyyTn9tghj4OLbKWYM1lWoomH3zocwKM
qCJjs49KgbFjBOPMFrHbUCRCpArBwUmAHwEPEyegpjwHbfdGKEo5m/kErPFH6r8H
/NxALj8PfdnuzqA4gRyy4nCK1uV43OqxIDrMIPdGQKcVEuKzAQG6c7QySvaCv8K5
X6CnETLSwfv68HoJ08xAzfi6QsOMP3rOLtcPY8VTcPlvSyLnQurAGT09wicvGgqW
fByhAgMBAAGjYDBeMB8GA1UdIwQYMBaAFMIyHT02V0GI+j6OvyQW/dPngSHPMB0G
A1UdDgQWBBTrZUWP/LwQhgToAzPWWcEKVimsqDAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEACNqZB6pvPPmnq/AKvKhLlBFB
xU+QHrkTCWbO+pNQIDFtkTDDiqadxvUlqnyvKXaDOdJpVqGhjMTyZy5xQ3KVb3rJ
GI2qsXzFR/tv81BunBM+gJY+1BqLOzlpYNU0FUV7/E6jxKLUhRsM2TMt7PFE97uB
bpZaOqeCylUW7ru/q/QghlrCTVYiaEfBHK8XnTh4o/wVhW6lm6+slBbe58roOi3P
HtT5DgofwLu9jOjtH1B/tJ+G80McejTeDUiLXb1PGWocYnoyv53+wedUt9bjbKC/
xcSKZ/nh12Yq+zUL2ps/wk+Bu9W+wOmYri6+Uynp3uSClT1LHqt1G8RonOcwtQ==
-----END CERTIFICATE-----
)";

const char AWS_DEVICE_PRIVATE_KEY[] = R"(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEA3uA8ZdLFCT9BoZKHqa/RIAehEGoDm+Duw6negHsSaGXMqYO9
1S4kH+USN+bzERfyRKO0FiBHKb8flK4QIlXMKrbpWsnJfVp7IgPrgi4z7+fXUM/+
VsMsk5/bYIY+Di2ylmDNZVqKJh986HMCjKgiY7OPSoGxYwTjzBax21AkQqQKwcFJ
gB8BDxMnoKY8B233RihKOZv5BKzxR+q/B/zcQC4/D33Z7s6gOIEcsuJwitbleNzq
sSA6zCD3RkCnFRLiswEBunO0Mkr2gr/CuV+gpxEy0sH7+vB6CdPMQM34ukLDjD96
zi7XD2PFU3D5b0si50LqwBk9PcInLxoKlnwcoQIDAQABAoIBAQCvcug44/X2NJSu
45HmM6C1zSuupzYWFLkA0I12qOuxwUmevDXJRtxrO0HfPfBkqWwvOot+y4QZyfnL
akcLMG1bxsfmj2kNRBggi+NuhH6gQesPE91NYbGKuNHSp3rcYH7wwF2Qhviv0CPd
PQRpw8XmpKiNZ5FDuMq2k7sJQYp55f7o/VlkD3KUAqLMTRSFXLXKlJVTh9M/5vUi
tIZI3hZrMdSM+eJFqc36aoPE/vzRdbJBQOaYT2KclgXPDpVw2Nc1g/aTKaM3lp1O
v8FS3jxOuvm+LJm7GLDRpYCwaMuO0GER6nwsT1tpmTD4eLZxkDGCUlL0g63MR6qc
ArX8C+YNAoGBAPNx+JPoizlrmc1qADhUY/g9P2Bj8Gwgk8dtCOS/UhC6Uvf+F+Jh
eMtem0MD4yrTf8hFY6qMniXIyYo6wUpFloTNqHDPeHrpBvnyWKpJr2SFm1smOXFv
cTFyCI71JnoMGnrhDjBIS+I5NDkLtTcEWY5VkxmI9UdWOcx2XMT3qqFbAoGBAOpe
tCtXQRkaqaoWu5r/KqGYzxxl34UDNazyEmRcyB1bRUErQja7W8ILLxrfiaWDKoky
U50d1Y4VZ8dy+a5SJEEfsxCEj/sZlyq4Q1QHMEvJTBQ8V+m8B4sc/r1Nq+Q0+R6f
ktDeOWrhYx3GmeYBLZ3TaSP+eZwj398gikBpT/6zAoGBAL8ofj1+O1ySPkoK8zNI
KNuzFEHXF+F/zyqoVnRwOruB40ZyEkUjrptapWYA9m8ZdlxHCFTSaBhZStAtBLwS
WnfbmWlDsGIq+txnTPIiPAQd2XjAUhjdni9qUIgYvebV/DEC6YZGLrfrnTMyG1Ch
LMVUMta5MYibj3UaeAv+QdDDAoGAMradW8d8oZcAYXqby7UtH1x2bFmq3RhNtAeJ
bUgaUSffu3NF15wBBeLvpWv7V+IjAR7vss6y2BTPqPu7BTpm9+SJFajpktb9u9O1
VehcTfnXyYUpLL1wEG0Mvh+hczb3NOYomaOMXxE4JkhEIt3cZgtJ8+zrmZq1pY1f
/QIMujsCgYAsB7ATxRu1nW3P8AZORCVhGAKIgNieA5+td6xfCMobW9WakGMjkDcP
DV2gdyCclv/L5jvGrgvYg6JJ5VLHl0u9cjGfc5Xai0QoDrf0Qdw2cqG89YdhYFLO
T8PPCg3dghOT7qM/30BZA46nBRSX7n0M0wkUCfVVpTEzm/3aE5+ArA==
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