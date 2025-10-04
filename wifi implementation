#include <WiFi.h> // Replace with your specific library if needed
#include <HTTPClient.h> // For sending data to a server

// Your network credentials (for testing)
const char* ssid = "YourSSID";
const char* password = "YourPassword";

// Server endpoint to send data (replace with a real server for demo)
const char* serverName = "http://your-cnl-server.com/api/data"; 

// Last successful transmission time
unsigned long lastTransmitTime = 0;
const long nominalInterval = 300000; // 5 minutes in ms

// Wi-Fi Setup
  WiFi.begin(ssid, password);
  lcd.setRGB(0, 0, 255); // Blue for connecting
  lcd.clear();
  lcd.print("Connecting to WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    lcd.print(".");
    Serial.print(".");
  }
  
  lcd.clear();
  lcd.print("WiFi CONNECTED!");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(2000);
  lcd.setRGB(0, 255, 0); // Return to Green

// Function to format and send data over HTTP POST
void transmitData(int score, String anomaly, float gForce, float temp) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    // Construct the data payload (JSON is best practice)
    String jsonPayload = "{\"score\":" + String(score) + 
                         ",\"anomaly\":\"" + anomaly + 
                         "\",\"gForce\":" + String(gForce) + 
                         ",\"temp\":" + String(temp) + "}";
    
    http.begin(serverName); 
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(jsonPayload);
    
    if (httpCode > 0) {
      // Success
      Serial.print("HTTP Success: ");
      Serial.println(httpCode);
      lastTransmitTime = millis(); // Update last success time
    } else {
      // Error
      Serial.print("HTTP Error: ");
      Serial.println(httpCode);
    }
    
    http.end();
  }
}

// ----------------------------------------------------------------
// --- 5. REMOTE COMMUNICATION IN LOOP ---
// ----------------------------------------------------------------
void checkAndTransmit() {
    float gForce = readIMU_GForce(); // Re-read or use global vars
    float internalTemp = readAHT20_Temp();

    // High Priority: CRITICAL/WARNING state
    if (currentThreatScore >= THREAT_LEVEL_WARNING) {
        // In a CRITICAL state, push data frequently
        if (millis() - lastTransmitTime > 2000) { // Send every 2 seconds
             transmitData(currentThreatScore, primaryAnomaly, gForce, internalTemp);
        }
    }
    // Low Priority: NOMINAL state (only push after a long interval)
    else if (millis() - lastTransmitTime > nominalInterval) {
        transmitData(currentThreatScore, primaryAnomaly, gForce, internalTemp);
    }
}

void loop() {
  // 1. READ ALL SENSORS
  // ... (existing code)

  // 2. RUN AUTONOMOUS THREAT LEVEL ASSESSMENT (ATLA)
  // ... (existing code)

  // 3. UPDATE DRIVER INTERFACE (LCD and Buzzer)
  updateDriverInterface();

  // 4. REMOTE COMMUNICATION (NEW)
  checkAndTransmit();

  // Sample Rate: Check sensors every 500 milliseconds
  delay(500); 
}
