// --- 1. LIBRARY INCLUSIONS ---
#include <Wire.h>
#include <rgb_lcd.h>

// --- 2. PIN DEFINITIONS ---
rgb_lcd lcd;

// --- 3. TEST STATE DEFINITIONS ---
struct TestState {
  const char* anomaly;    // Anomaly type (MAX 16 CHARS)
  const char* instruction; // Actionable instruction (MAX 16 CHARS)
  int duration;           // Duration of the state in ms
};

// **TEXT IS GUARANTEED TO BE 16 CHARACTERS OR LESS**
TestState states[] = {
  // State 0: Initializing (Blue)
  {"--NODE INIT DONE--", "LOADING DECISIONS", 2000}, 

  // State 1: Radiation Leak Instruction (Critical)
  {"RAD. LEAK! STOP!", "SHELTER ASAP", 3000}, 

  // State 2: Fire/Overheat Instruction (Critical)
  {"FIRE! CONTAIN!", "PULL OVER", 3000},
  
  // State 3: Crash/Impact Instruction (Critical)
  {"CRASH! DANGER!", "STOP, CALL CNL", 3000},
  
  // State 4: Load Shift/Other Generic Warning
  {"LOAD SHIFTED", "STOP INSPECT", 3000},
  
  // State 5: NOMINAL (Rest State)
  {"SYSTEM NEUTRAL", "ALL SYSTEMS OK", 3000},
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
    
    // Line 1: The Anomaly Type
    lcd.home();
    lcd.print(current.anomaly);
    
    // Line 2: ACTIONABLE INSTRUCTION
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
