#include <WiFi.h>
#include "NodeDecisionLibrary.h"

// Wi-Fi Credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// Create an instance of NodeDecisionLibrary
NodeDecisionLibrary logicProcessor;

// Callback function to handle device changes
void callbackFunction(int deviceId, bool value) {
    Serial.printf("Callback triggered! Device ID: %d, Value: %s\n", deviceId, value ? "true" : "false");
}

void setup() {
    // Start Serial communication
    Serial.begin(115200);
    Serial.println("Node Decision Library - Basic Example");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");

    // Set the callback function for device changes
    logicProcessor.setCallback(callbackFunction);

    // Enable debugging for the library
    logicProcessor.isDebug(true);

    // Example JSON payload for decoding
    String exampleJson = R"({
        "data": {
            "n": [
                {
                    "id": 1,
                    "aId": 28,
                    "k": "relay",
                    "i": [
                        {"id": 11, "dt": "boolean"}
                    ],
                    "o": [
                        {"id": 12, "dt": "boolean", "dId": 101, "cId": 201}
                    ]
                }
            ],
            "r": [
                {"id": 1, "i": 11, "o": 12, "c": 201}
            ]
        }
    })";

    // Decode logic data for a specific device
    int deviceId = 101;
    if (logicProcessor.decodeLogicData(exampleJson, deviceId)) {
        Serial.println("Logic data decoded successfully.");
    } else {
        Serial.println("Failed to decode logic data.");
    }

    // Example sensor value JSON
    String sensorValues = R"({
        "currentNodeId": 1,
        "sensorArray": [
            {"deviceId": 101, "value": true}
        ]
    })";

    // Update device values with the JSON data
    logicProcessor.updateDeviceValues(sensorValues);
}

void loop() {
    // Example: Simulate updating sensor values periodically
    String newSensorValues = R"({
        "currentNodeId": 1,
        "sensorArray": [
            {"deviceId": 101, "value": false}
        ]
    })";

    logicProcessor.updateDeviceValues(newSensorValues);

    // Wait for a while before updating again
    delay(10000);
}
