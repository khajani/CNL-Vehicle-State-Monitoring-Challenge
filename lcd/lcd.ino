// LCD and Buzzer Test Code for Guardian Node Interface
// --- 1. LIBRARY INCLUSIONS ---
#include <Wire.h>
#include <rgb_lcd.h>

// --- 2. PIN DEFINITIONS AND HARDWARE SETUP ---
#define BUZZER_PIN 8      // Digital Pin for the Grove Buzzer

// --- Buzzer Timing Constants ---
#define DOT_MS 100        // Duration for a short buzz (dot)
#define DASH_MS 300       // Duration for a long buzz (dash)
#define GAP_MS 100        // Short gap between tones
#define WORD_GAP_MS 1000  // Long gap between full alarm cycles

// Initialize LCD Object
rgb_lcd lcd;

// --- 3. TEST STATE DEFINITIONS ---
struct TestState {
  int r, g, b;            // RGB Color
  const char* anomaly;    // The anomaly type (used to select alarm pattern)
  const char* line1;      // Text for line 1
  const char* line2;      // Text for line 2 (Actionable Instruction)
  int duration;           // Duration of the state in ms (used for general display time)
};

TestState states[] = {
  // State 0: Initializing (White/Blue)
  {100, 100, 255, "NONE", "Guardian Node Test", "Loading Coded Alarms...", 3000},

  // State 1: Radiation Leak (Flashing Red, SOS Pattern)
  {255, 0, 0, "RADIATION LEAK", "!!! RADIATION LEAK !!!", "SEEK SHELTER & WAIT", 5000},

  // State 2: Fire/Overheat (Flashing Red, Chirp Pattern)
  {255, 0, 0, "FIRE/OVERHEAT", "!!! FIRE/OVERHEAT !!!", "EXTINGUISH/PULL OVER", 5000},
  
  // State 3: Crash/Impact (Flashing Red, Continuous Tone Default)
  {255, 0, 0, "CRASH/IMPACT", "!!! CRASH/IMPACT !!!", "STAY SAFE! CALL CNL!", 5000},
  
  // State 4: NOMINAL (Green) - Rest State
  {0, 255, 0, "NONE", "System NOMINAL", "Test Complete. Check D8.", 3000},
};

const int numStates = sizeof(states) / sizeof(states[0]);
int currentState = 0;
unsigned long stateStartTime;

// ----------------------------------------------------------------
// --- HELPER FUNCTIONS FOR ALARM ---
// ----------------------------------------------------------------

// Helper function to create a single tone for a duration
void buzz(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
  delay(GAP_MS);
}

// Core function to play the specific coded alarm
void playCodeAlarm(const char* anomaly) {
  // Use a string comparison to select the pattern
  if (strcmp(anomaly, "RADIATION LEAK") == 0) {
    // SOS Pattern: dot-dot-dot, dash-dash-dash, dot-dot-dot
    buzz(DOT_MS); buzz(DOT_MS); buzz(DOT_MS);
    buzz(DASH_MS); buzz(DASH_MS); buzz(DASH_MS);
    buzz(DOT_MS); buzz(DOT_MS); buzz(DOT_MS);
    delay(WORD_GAP_MS); // Pause before next cycle check
  }
  else if (strcmp(anomaly, "FIRE/OVERHEAT") == 0) {
    // Quick Repetitive Chirp Pattern (4 fast chirps)
    for(int i = 0; i < 4; i++) {
        buzz(50); // Very short buzz
        delay(50);
    }
    delay(WORD_GAP_MS);
  }
  else {
    // Default or CRASH/IMPACT: Simple, sustained buzz
    digitalWrite(BUZZER_PIN, HIGH); 
  }
}

// ----------------------------------------------------------------
// --- SETUP FUNCTION ---
// ----------------------------------------------------------------
void setup() {
  Serial.begin(9600); 
  
  // LCD Setup
  lcd.begin(16, 2);
  
  // Buzzer Setup
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Start silent
  
  stateStartTime = millis();
}

// ----------------------------------------------------------------
// --- LOOP FUNCTION ---
// ----------------------------------------------------------------
void loop() {
  TestState current = states[currentState];
  unsigned long currentTime = millis();
  
  // --- A. State Transition Logic ---
  if (currentTime - stateStartTime >= current.duration) {
    // If the previous state was a sustained buzz (like CRASH), turn it off before switching
    if (strcmp(current.anomaly, "CRASH/IMPACT") == 0) {
        digitalWrite(BUZZER_PIN, LOW);
    }
    currentState = (currentState + 1) % numStates; 
    stateStartTime = currentTime;
    
    // Force text update on new state
    lcd.clear();
    lcd.home();
    lcd.print(states[currentState].line1);
    lcd.setCursor(0, 1);
    lcd.print(states[currentState].line2);
  }
  
  // Update the current state after transition
  current = states[currentState];

  // --- B. Visual (LCD) Logic ---
  // Flash Red for alert states (1, 2, 3)
  if (currentState >= 1 && currentState <= 3) {
    if (currentTime % 200 < 100) { 
        lcd.setRGB(current.r, current.g, current.b); // Red
    } else {
        lcd.setRGB(0, 0, 0); // Off 
    }
  } else {
    // Set static color for initial/nominal states
    lcd.setRGB(current.r, current.g, current.b);
  }

  // --- C. Auditory (Buzzer) Logic ---
  if (currentState >= 1 && currentState <= 3) {
    // Play the code alarm. This function uses delay() and will block the loop
    // for the duration of the pattern (a few seconds).
    playCodeAlarm(current.anomaly);
  } else {
    // Ensure buzzer is OFF for Nominal states
    digitalWrite(BUZZER_PIN, LOW);
  }
  
  // Add a small delay for stability, especially when not playing a coded alarm
  if (currentState == 0 || currentState == 4) {
      delay(50);
  }
  // When a coded alarm plays, the internal delays in playCodeAlarm() manage timing.
}
