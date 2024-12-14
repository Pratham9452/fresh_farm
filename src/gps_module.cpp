// src/gps_module.cpp
#include <Arduino.h>
#include <TinyGPS++.h>

#include "sensor_config.h"

// External GPS object (already defined in main.cpp)
extern TinyGPSPlus gps;
extern HardwareSerial GPSSerial;

// Function to check GPS data validity
bool isGPSDataValid() {
    return gps.location.isValid() && 
           gps.date.isValid() && 
           gps.time.isValid();
}

// Get detailed GPS information
String getGPSDetailsString() {
    if (!isGPSDataValid()) {
        return "Invalid GPS Data";
    }

    char gpsDetails[200];
    snprintf(gpsDetails, sizeof(gpsDetails), 
        "Lat: %.6f, Lon: %.6f, Alt: %.2f m, "
        "Date: %02d/%02d/%04d, Time: %02d:%02d:%02d, "
        "Satellites: %d, HDOP: %.2f",
        gps.location.lat(), 
        gps.location.lng(), 
        gps.altitude.meters(),
        gps.date.month(), gps.date.day(), gps.date.year(),
        gps.time.hour(), gps.time.minute(), gps.time.second(),
        gps.satellites.value(),
        gps.hdop.hdop()
    );

    return String(gpsDetails);
}

// Calculate distance between two GPS coordinates (Haversine formula)
double calculateDistance(double lat1, double lon1, 
                         double lat2, double lon2) {
    const double R = 6371000; // Earth radius in meters
    
    // Convert latitude and longitude to radians
    double lat1Rad = lat1 * PI / 180.0;
    double lon1Rad = lon1 * PI / 180.0;
    double lat2Rad = lat2 * PI / 180.0;
    double lon2Rad = lon2 * PI / 180.0;
    
    // Differences
    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;
    
    // Haversine formula
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1Rad) * cos(lat2Rad) *
               sin(dLon/2) * sin(dLon/2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return R * c; // Distance in meters
}

// Optional: GPS module diagnostic function
String getGPSDiagnostics() {
    return String("GPS Diagnostics: ") + 
           "Sentences: " + String(gps.passedChecksum()) + 
           ", Failed: " + String(gps.failedChecksum()) +
           ", Satellites: " + String(gps.satellites.value());
}