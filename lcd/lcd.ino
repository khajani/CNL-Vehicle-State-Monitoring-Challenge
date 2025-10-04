// --- 1. LIBRARY INCLUSIONS ---
#include <Wire.h>
#include <rgb_lcd.h>

// --- 2. PIN DEFINITIONS AND HARDWARE SETUP ---
// #define BUZZER_PIN 8  // BUZZER PIN COMMENTED OUT FOR SILENT TEST

// Initialize LCD Object
rgb_lcd lcd;

// --- 3. TEST STATE DEFINITIONS ---
struct TestState {
  const char* anomaly;    // Anomaly type (MAX 16 CHARS)
  const char* instruction; // Actionable instruction (MAX 16 CHARS)
  int duration;           // Duration of the state in ms
};

// **TEXT HAS BEEN FINALIZED TO EXACTLY 16 CHARACTERS OR LESS**
TestState states[] = {
  // State 0: Initializing (Blue)
  {"--NODE INIT--", "LOADING DECISIONS", 2000}, // 13 & 16 chars

  // State 1: Radiation Leak Instruction (Critical)
  {"!!!RAD LEAK!!!", "SEEK SHELTER/WAIT", 3000}, // 14 & 16 chars

  // State 2: Fire/Overheat Instruction (Critical)
  {"!!!FIRE/HEAT!!!", "PULL OVER/EXTING", 3000}, // 15 & 16 chars
  
  // State 3: Crash/Impact Instruction (Critical)
  {"!!!CRASH!!!", "SAFE & CALL CNL", 3000}, // 11 & 15 chars
  
  // State 4: Load Shift/Other Generic Warning
  {"LOAD SHIFT LOOSE", "PULL OVER/INSPCT", 3000}, // 16 & 16 chars
  
  // State 5: NOMINAL (Rest State)
  {"SYSTEM NOMINAL", "ALL SYSTEMS OK", 3000}, // 14 & 14 chars
};

const int numStates = sizeof(states) / sizeof(states[0]);
int currentState = 0;
unsigned long stateStartTime;
const int flashRate = 200; // milliseconds for the LCD flash cycle

// ----------------------------------------------------------------
// --- SETUP FUNCTION ---
// ----------------------------------------------------------------
void setup() {
  Serial.begin(9600); 
  
  // LCD Setup
  lcd.begin(16, 2);
  
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
    currentState = (currentState + 1) % numStates; 
    stateStartTime = currentTime;
    
    // Force text update on new state
    lcd.clear();
  }
  
  // Update the current state after transition
  current = states[currentState];

  // --- B. Visual (LCD) Logic ---
  // If not NOMINAL or Initializing (State 1 through 4)
  if (currentState >= 1 && currentState <= 4) {
    // Flashing Red Backlight
    if (currentTime % flashRate < (flashRate / 2)) { 
        lcd.setRGB(255, 0, 0); // Red
    } else {
        lcd.setRGB(0, 0, 0); // Off 
    }
    
    // Line 1: The Anomaly Type (Optimized Text)
    lcd.home();
    lcd.print(current.anomaly);
    
    // Line 2: ACTIONABLE INSTRUCTION (Optimized Text)
    lcd.setCursor(0, 1);
    lcd.print(current.instruction);
    
  } else {
    // NOMINAL/Initial States
    if (currentState == 0) lcd.setRGB(100, 100, 255); // Blue
    if (currentState == 5) lcd.setRGB(0, 255, 0);     // Green

    lcd.home();
    lcd.print(current.anomaly);
    lcd.setCursor(0, 1);
    lcd.print(current.instruction);

    delay(50); // Small delay for stability
  }
}
