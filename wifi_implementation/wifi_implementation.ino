#include <WiFiS3.h> // The correct library for the Uno R4 WiFi

void setup() {
  Serial.begin(115200);
  delay(100);

  // Check for the WiFi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed! Halting.");
    while (true); 
  }

  Serial.println("\nStarting Wi-Fi scan...");
}

void loop() {
  // Clear list of previous networks
  Serial.println("Scanning available networks...");

  // WiFi.scanNetworks returns the number of networks found
  int numNetworks = WiFi.scanNetworks();
  
  if (numNetworks == 0) {
    Serial.println("No networks found.");
  } else {
    Serial.print("Found ");
    Serial.print(numNetworks);
    Serial.println(" networks:");
    
    // Print the list of networks
    for (int i = 0; i < numNetworks; i++) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm)");
      
      // Print the encryption type
      Serial.print(" Enc:");
      Serial.println(WiFi.encryptionType(i));
    }
  }

  // Wait 10 seconds before the next scan
  delay(10000); 
}
