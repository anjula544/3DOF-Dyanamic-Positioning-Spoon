

#include <WiFi.h>
#include <ESP32Firebase.h>
#include <FirebaseJson.h>

// Replace with your network credentials
#define _SSID "HUAWEI Y6 Pro 2019"          // Your WiFi SSID
#define _PASSWORD "absd12345"               // Your WiFi Password
#define REFERENCE_URL "https://dyanamicspoon-default-rtdb.firebaseio.com/"  // Your Firebase project reference URL

Firebase firebase(REFERENCE_URL);

void setup() {
  Serial.begin(9600);         // Initialize serial communication at 9600 baud
  Serial2.begin(9600);        // Initialize serial communication for ESP32

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  // Connect to WiFi
  Serial.println();
  Serial.print("Connecting to: ");
  Serial.println(_SSID);
  WiFi.begin(_SSID, _PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("-");
  }

  Serial.println();
  Serial.println("WiFi Connected");
}

void loop() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    Serial.println("Received: " + data);

    // Convert the received data to JSON format
    FirebaseJson jsonData;
    convertToJson(data, jsonData);

    // Send data to Firebase
    bool success = true;
    size_t count = jsonData.iteratorBegin();
    FirebaseJson::IteratorValue value;
    for (size_t i = 0; i < count; i++) {
      value = jsonData.valueAt(i);
      if (value.type == FirebaseJson::JSON_OBJECT) {
        if (value.value[0] == '\"') {  // If the value is a string
          String temp = value.value;
          temp.remove(0, 1); // Remove the leading quote
          temp.remove(temp.length() - 1, 1); // Remove the trailing quote
          success &= firebase.pushString("/sensorData/" + value.key, temp);
        } else if (value.value.indexOf('.') != -1) {  // If the value is a float
          success &= firebase.pushFloat("/sensorData/" + value.key, value.value.toFloat());
        } else {  // Otherwise, assume the value is an integer
          success &= firebase.pushInt("/sensorData/" + value.key, value.value.toInt());
        }
      }
    }
    jsonData.iteratorEnd();

    if (success) {
      Serial.println("Data sent to Firebase successfully");
    } else {
      Serial.println("Failed to send data to Firebase");
    }
  }
}

// Helper function to convert CSV data to FirebaseJson format
void convertToJson(String data, FirebaseJson &json) {
  // Ensure there's a space after commas for easier parsing
  data.replace(",", ", ");

  // Find the index of the ':' character
  int separatorIndex = data.indexOf(':');

  // If ':' is found, split the data into key and value
  if (separatorIndex != -1) {
    String key = data.substring(0, separatorIndex);
    String value = data.substring(separatorIndex + 1);

    // Trim leading and trailing whitespaces from the key and value
    key.trim();
    value.trim();

    // Set the key-value pair in the JSON object
    json.set(key, value);
  }
}
