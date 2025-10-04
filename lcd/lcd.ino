// --- 1. LIBRARY INCLUSIONS ---
#include <Wire.h>
#include <rgb_lcd.h>

// --- 2. PIN DEFINITIONS ---
rgb_lcd lcd;

// --- 3. LONG TEXT DEFINITIONS ---
// LONGER text that requires two frames (max 32 chars total)
const char* LONG_ANOMALY = "!!!RADIATION LEAK DETECTED!!!"; // 29 chars
const char* LONG_INSTRUCTION = "SEEK IMMEDIATE SHELTER/CONTACT CNL"; // 34 chars (will be truncated)

// Split into 16-character frames (Line 1 and Line 2)
// NOTE: Text is padded/adjusted for alignment
const char* FRAME_A_LINE1 = "!!!RAD LEAK DTECT!!!"; // Line 1, Chars 1-16
const char* FRAME_B_LINE1 = "ED!!!           ";    // Line 1, Chars 17-32 (Padded to 16)
const char* FRAME_A_LINE2 = "SEEK SHELTER/CNL"; // Line 2, Chars 1-16
const char* FRAME_B_LINE2 = "EMERGENCY CONTACT"; // Line 2, Chars 17-32 (Padded to 16)

const int FLICKER_RATE = 500; // Time each frame is displayed (milliseconds)

// ----------------------------------------------------------------
// --- SETUP FUNCTION ---
// ----------------------------------------------------------------
void setup() {
  Serial.begin(9600); 
  lcd.begin(16, 2);
  lcd.setRGB(100, 100, 255);
  lcd.print("Flicker-Scroll Test");
  delay(2000);
}

// ----------------------------------------------------------------
// --- LOOP FUNCTION ---
// ----------------------------------------------------------------
void loop() {
  unsigned long currentTime = millis();
  
  // Set Flashing Red Color
  if (currentTime % 100 < 50) { 
      lcd.setRGB(255, 0, 0); 
  } else {
      lcd.setRGB(0, 0, 0); 
  }

  // Determine which frame to show
  if (currentTime % (FLICKER_RATE * 2) < FLICKER_RATE) {
    // --- FRAME A ---
    lcd.home();
    lcd.print(FRAME_A_LINE1);
    lcd.setCursor(0, 1);
    lcd.print(FRAME_A_LINE2);
  } else {
    // --- FRAME B ---
    lcd.home();
    lcd.print(FRAME_B_LINE1);
    lcd.setCursor(0, 1);
    lcd.print(FRAME_B_LINE2);
  }

  // Simulate continuous buzzer tone (if testing sound later)
  // digitalWrite(BUZZER_PIN, HIGH);
}
