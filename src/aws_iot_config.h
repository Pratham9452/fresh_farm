// include/aws_iot_config.h
#ifndef AWS_IOT_CONFIG_H
#define AWS_IOT_CONFIG_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>

// Maximum size for MQTT message
#define MAX_MQTT_PACKET_SIZE 512

// Function prototypes for AWS IoT handling
void connectToAWS();
bool publishSensorData(float temperature, float humidity, 
                       double latitude, double longitude);
void setupAWSIoTConnection();

#endif // AWS_IOT_CONFIG_H