#ifndef CONFIG_H
#define CONFIG_H

// DHT22 Sensor Configuration
#define DHT_PIN             4       // GPIO pin connected to DHT22
#define DHT_TYPE            DHT22   // DHT sensor type

// NEO-6M GPS Module Configuration
#define GPS_RX_PIN          16      // RX pin for GPS module
#define GPS_TX_PIN          17      // TX pin for GPS module
#define GPS_BAUD_RATE       9600    // GPS module baud rate

// Interval Configuration
#define SENSOR_READ_INTERVAL    5000    // 5 seconds between sensor reads
#define DATA_PUBLISH_INTERVAL   30000   // 30 seconds between AWS IoT publishes

#endif // CONFIG_H