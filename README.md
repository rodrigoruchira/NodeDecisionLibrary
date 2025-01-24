# NodeDecisionLibrary

The `NodeDecisionLibrary` is an Arduino-compatible library designed to process node-based logic for devices like sensors, relays, and actuators. It supports JSON-based configuration, real-time device value updates, and callback mechanisms for handling device state changes.

---

## Features

- Decode logic data from a JSON payload.
- Update device states with sensor inputs in JSON format.
- Trigger a callback function for device state changes.
- Prevent oscillation with a debounce mechanism.
- Debugging capabilities to track library processing.

---

## Installation

1. Open the Arduino IDE.
2. Go to **Tools > Manage Libraries**.
3. Search for **NodeDecisionLibrary**.
4. Click **Install**.

---

## Usage

### 1. Include the Library

```cpp
#include "NodeDecisionLibrary.h"
```

### 2. Create an Instance

```cpp
NodeDecisionLibrary logicProcessor;
```

### 3. Set a Callback Function

Define a callback function to handle device state changes:
```cpp
void callbackFunction(int deviceId, bool value) {
    Serial.printf("Device ID: %d, State: %s\n", deviceId, value ? "ON" : "OFF");
}
```

Set the callback function:
```cpp
logicProcessor.setCallback(callbackFunction);
```

### 4. Decode Logic Data

Decode JSON logic configuration for a device:
```cpp
String exampleJson = R"({
    "data": {
        "n": [
            {
                "id": 1,
                "aId": 28,
                "k": "relay",
                "i": [{"id": 11, "dt": "boolean"}],
                "o": [{"id": 12, "dt": "boolean", "dId": 101, "cId": 201}]
            }
        ],
        "r": [{"id": 1, "i": 11, "o": 12, "c": 201}]
    }
})";

if (logicProcessor.decodeLogicData(exampleJson, 101)) {
    Serial.println("Logic data decoded successfully.");
}
```

### 5. Update Device Values

Pass sensor input data as a JSON payload to update device states:
```cpp
String sensorValues = R"({
    "currentNodeId": 1,
    "sensorArray": [
        {"deviceId": 101, "value": true}
    ]
})";

logicProcessor.updateDeviceValues(sensorValues);
```

### 6. Debugging

Enable or disable debugging output:
```cpp
logicProcessor.isDebug(true); // Enable debugging
logicProcessor.isDebug(false); // Disable debugging
```

---

## Sample Example

### Code

```cpp
#include <WiFi.h>
#include "NodeDecisionLibrary.h"

// Wi-Fi Credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

NodeDecisionLibrary logicProcessor;

void callbackFunction(int deviceId, bool value) {
    Serial.printf("Device ID: %d, Value: %s\n", deviceId, value ? "true" : "false");
}

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");

    logicProcessor.setCallback(callbackFunction);
    logicProcessor.isDebug(true);

    String exampleJson = R"({
        "data": {
            "n": [
                {
                    "id": 1,
                    "aId": 28,
                    "k": "relay",
                    "i": [{"id": 11, "dt": "boolean"}],
                    "o": [{"id": 12, "dt": "boolean", "dId": 101, "cId": 201}]
                }
            ],
            "r": [{"id": 1, "i": 11, "o": 12, "c": 201}]
        }
    })";

    logicProcessor.decodeLogicData(exampleJson, 101);

    String sensorValues = R"({
        "currentNodeId": 1,
        "sensorArray": [
            {"deviceId": 101, "value": true}
        ]
    })";

    logicProcessor.updateDeviceValues(sensorValues);
}

void loop() {
    delay(1000);
}
```

---

## JSON Structure

### Logic Configuration

- **`n`**: Nodes
  - **`id`**: Node ID
  - **`aId`**: Available ID (node type)
  - **`k`**: Kind (e.g., relay)
  - **`i`**: Inputs
    - **`id`**: Input ID
    - **`dt`**: Data type (e.g., "boolean")
  - **`o`**: Outputs
    - **`id`**: Output ID
    - **`dt`**: Data type (e.g., "boolean")
    - **`dId`**: Device ID
    - **`cId`**: Config ID

- **`r`**: Relationships
  - **`i`**: Input ID
  - **`o`**: Output ID
  - **`c`**: Config ID

### Sensor Input Data

- **`sensorArray`**: List of sensors
  - **`deviceId`**: Device ID
  - **`value`**: Value (boolean, int, or float)

---

## Contributing

Contributions are welcome! Please submit pull requests or open issues for any bugs or feature requests.

---

## License

This library is licensed under the MIT License. See the LICENSE file for more details.

---

## Support

If you encounter any issues or have questions, feel free to contact the author or open an issue on GitHub.
