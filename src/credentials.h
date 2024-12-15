// include/credentials.h
#ifndef CREDENTIALS_H
#define CREDENTIALS_H
 #include <MQTTClient.h>
// WiFi Credentials
#define WIFI_SSID       "WIFI_name"
#define WIFI_PASSWORD   "WIFI_passcode"

// AWS IoT Core Credentials
#define AWS_IOT_ENDPOINT        "********************"
#define AWS_THING_NAME          "fresh_track"

// AWS IoT Core Topic
#define AWS_IOT_PUBLISH_TOPIC   "fresh_track/ESP_data"

// AWS Root CA Certificate (PEM format)
const char AWS_ROOT_CA[] = R"(
-----BEGIN CERTIFICATE-----
//put your root certificate here 
-----END CERTIFICATE-----
)";

// AWS Device Certificate (PEM format)
const char AWS_DEVICE_CERT[] = R"(
-----BEGIN CERTIFICATE-----
// put your device certificate here 
-----END CERTIFICATE-----
)";

// AWS Device Private Key (PEM format)
const char AWS_DEVICE_PRIVATE_KEY[] = R"(
-----BEGIN RSA PRIVATE KEY-----
// put your private key of device here 
-----END RSA PRIVATE KEY-----
)";

#endif // CREDENTIALS_H
