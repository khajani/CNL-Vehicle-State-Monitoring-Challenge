#include <WiFiS3.h> // REQUIRED for Arduino Uno R4 WiFi networking
#include <ThingSpeak.h> // REQUIRED for ThingSpeak functions

// -------------------------------------------------------------------
// 1. YOUR PRIVATE CREDENTIALS (REPLACE THE PLACEHOLDER VALUES BELOW)
// -------------------------------------------------------------------
char ssid[] = "Kj's iPhone"; // <-- CHANGE THIS
char pass[] = "1236393639";     // <-- CHANGE THIS

// ThingSpeak Channel and API Key
unsigned long myChannelNumber = 3100192;          // Your ThingSpeak Channel ID (e.g., 123456) <-- CHANGE THIS
const char * myWriteAPIKey = "BAOHBAPAYCW24D6P"; // <-- CHANGE THIS

// -------------------------------------------------------------------
// GLOBAL VARIABLES
// -------------------------------------------------------------------
WiFiClient client;
int status = WL_IDLE_STATUS;

// -------------------------------------------------------------------
// FUNCTION: connectToWiFi
// -------------------------------------------------------------------
void connectToWiFi() {
  // Check for the WiFi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed! Halting.");
    while (true); 
  }

  // Attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    
    // Wait 10 seconds for connection
    delay(10000);
  }

  // Connection successful
  Serial.println("\n✅ Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// -------------------------------------------------------------------
// SETUP FUNCTION
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  connectToWiFi();

  // Initialize ThingSpeak client
  ThingSpeak.begin(client);
}

// -------------------------------------------------------------------
// MAIN LOOP FUNCTION
// -------------------------------------------------------------------
void loop() {
  // Check Wi-Fi connection and attempt to reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Lost WiFi connection. Reconnecting...");
    connectToWiFi();
  }
  
  // --- SENSOR DATA VARIABLES (Simulated for this test) ---
  float score;        // Field 1
  float gForce;       // Field 2
  float temperature;  // Field 3
  float latitude;     // Field 4
  float longitude;    // Field 5
  
  // *** SIMULATED DATA ***
  score       = random(50, 101);             
  gForce      = random(90, 110) / 100.0;     
  temperature = random(1500, 3500) / 100.0;  
  latitude    = random(434000, 434500) / 10000.0;
  longitude   = random(-805500, -805000) / 10000.0; 
  
  
  Serial.println("\n--- Sensor Readings (Simulated) ---");
  Serial.print("F1 (Score): "); Serial.println(score);
  Serial.print("F2 (G-Force): "); Serial.println(gForce);
  Serial.print("F3 (Temp): "); Serial.print(temperature); Serial.println(" C");
  Serial.print("F4 (Lat): "); Serial.println(latitude, 4);
  Serial.print("F5 (Long): "); Serial.println(longitude, 4);
  
  // --- SET UP FIELDS ---
  ThingSpeak.setField(1, score);
  ThingSpeak.setField(2, gForce);
  ThingSpeak.setField(3, temperature);
  ThingSpeak.setField(4, latitude);
  ThingSpeak.setField(5, longitude);
  
  // --- WRITE DATA TO THINGSPEAK ---
  int http_response_code = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  if (http_response_code == 200) {
    Serial.println("✅ Data sent successfully! (HTTP 200)");
  } else {
    // THIS LINE IS CRUCIAL FOR DEBUGGING.
    Serial.print("❌ Problem updating channel. HTTP error code: ");
    Serial.println(http_response_code);
    
    // Most common ThingSpeak error codes:
    // 400: Bad request (usually wrong API key or Channel ID).
    // -301: ThingSpeak.begin(client) failed (Wi-Fi or DNS issue).
  }

  delay(20000); 
}
