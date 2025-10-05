#include <WiFiS3.h>      // REQUIRED for Arduino Uno R4 WiFi networking
#include <ThingSpeak.h>  // REQUIRED for ThingSpeak functions
#include <Adafruit_LIS3MDL.h> // Magnetometer Library
#include <Adafruit_Sensor.h>  // Unified Sensor Library
#include <Wire.h>        // REQUIRED for I2C
#include <rgb_lcd.h>     // REQUIRED for LCD

// Your Team's Libraries for Sensor Processing
#include <DFRobot_BMX160.h>
#include <arduinoFFT.h>

// -------------------- 2. YOUR PRIVATE CREDENTIALS --------------------
char ssid[] = "KJ"; 
char pass[] = "1236393639";

// ThingSpeak Channel and API Key
unsigned long myChannelNumber = 3100192; // REPLACE with your final channel number
const char * myWriteAPIKey = "BAOHBAPAYCW24D6P"; // REPLACE with your final Write API Key

// **MAG/GPS Configuration**
const float DECLINATION_ANGLE = 12.5; // IMPORTANT: Set this for your location's magnetic variation!
const float home_magn = 640.0;    // Magnetometer baseline Norm (Used for deviation comparison)

// -------------------- 3. GLOBAL SYSTEM & SENSOR DEFS & THRESHOLDS --------------------
WiFiClient client;
int status = WL_IDLE_STATUS;

// SENSOR OBJECTS
DFRobot_BMX160 bmx160;
rgb_lcd lcd;               
Adafruit_LIS3MDL lis3mdl; 

// FFT CONFIGURATION
#define SAMPLES 256
#define SAMPLE_FREQ 400
#define IR_PIN A0
#define BANDS 8

// FFT BUFFERS
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLE_FREQ);

// BASELINE DATA
float baselineSpectrum[BANDS];
bool baselineSet = false;

// GYRO/TILT CONFIG
const float gyro_scale_factor = 16384.0; // Scaling factor for +/-2G range
const float criticalAngle = 80.0;

// *** HAZARD LEVEL CHECK (HLC) THRESHOLDS (Used for determining state and additive score) ***
// HARD CRITICAL ALERTS
const float THRESH_RADIATION_CRITICAL = 935.0;  // High IR reading -> Instant Score 10
const float THRESH_G_FORCE_CRASH      = 3.0;    // 3.0G+ is a definite crash -> Instant Score 10
// WARNING ALERTS (used for additive scoring)
const float THRESH_OVERHEAT_WARNING   = 80.0;   // In-box Temp/IR
const float THRESH_TILT_WARNING       = criticalAngle; 
const double THRESH_FFT_WARNING       = 100000.0; 
const float THRESH_MAG_DEVIATION      = 50.0;   
const int THRESH_TOUCH_BREACH         = HIGH;   

// *** ADDITIVE SCORING THRESHOLD ***
const int THRESH_SCORE_CUMULATIVE_CRITICAL = 5; // Cumulative score of 5 or more triggers SOS alert

// --- STATE CONSTANTS (Used INTERNALLY for LCD message lookup and Alarm type) ---
// STATE 1, 3, 10 all result in hazardScore=10 and SOS Alarm.
const int STATE_INIT = 0;
const int STATE_CRITICAL_CUMULATIVE = 1; // Cumulative score >= 5
const int STATE_FIRE = 2;                // No longer used to set primary state, but kept for historical context.
const int STATE_CRASH = 3;               // Hard G-force detection
const int STATE_WARNING = 4;             // Any cumulative score > 0 and < 5
const int STATE_NOMINAL = 5;             // All clear (Score 0)
const int STATE_RAD_LEAK = 10;           // Hard radiation detection


// --- BUZZER AND TOUCH SENSORS ---
#define BUZZER_PIN 8  // Digital pin for the buzzer (D8)
#define TOUCH_PIN 7   // Digital pin for the touch sensor (D7)
#define BUTTON_PIN 6 // Digital pin for baseline re-record button

// Morse Code Timing Constants (SOS Pattern for Critical Alerts)
const int BUZZ_FREQ = 1800; // High-pitched, urgent frequency

// SOS Timing (S.O.S: 3 Short, 3 Long, 3 Short)
const unsigned long MORSE_DOT_MS = 100;
const unsigned long MORSE_DASH_MS = 300;
const unsigned long MORSE_SOS_CYCLE_MS = 3000; // Total cycle length (~3 seconds)

// Single Short Buzz Timing (for Minor Warning States)
const unsigned long BUZZ_PULSE_MS = 100;
const unsigned long BUZZ_CYCLE_MS = 500; // 100ms tone, 400ms silence


// LCD STATE MAPPING (HLC States)
struct TestState {
  const char* anomaly;     // Anomaly type (MAX 16 CHARS)
  const char* instruction; // Actionable instruction (MAX 16 CHARS)
};

// Array index must be continuous (0-6) for LCD lookup.
TestState states[] = {
  // Index 0: STATE_INIT
  {"--INIT DONE--", "LOADING DECISIONS"}, 
  // Index 1: STATE_CRITICAL_CUMULATIVE 
  {"CRIT. WARNING", "PULL OVER, CALL"}, 
  // Index 2: STATE_FIRE (Still shows specific text if needed for state 2)
  {"FIRE! CONTAIN!", "PULL OVER"},
  // Index 3: STATE_CRASH 
  {"CRASH! DANGER!", "STOP, CALL CNL"},
  // Index 4: STATE_WARNING (Generic minor warning)
  {"MINOR WARNING", "CHECK TELEMETRY"},
  // Index 5: STATE_NOMINAL 
  {"SYSTEM NEUTRAL", "ALL SYSTEMS OK"},
  // Index 6: STATE_RAD_LEAK 
  {"RAD. LEAK! STOP!", "SHELTER ASAP"} 
};

int currentState = STATE_INIT;      // Starts in State 0 (Initializing)
int hazardScore = 0;                // Value logged to Field 8 (0-6, or 10)
const int flashRate = 200;          // milliseconds for the LCD flash cycle

// -------------------- 4. HLC (Hazard Level Check) LOGIC --------------------

// Maps the HLC state constant (e.g., 10 or 1) back to the array index 
int getDisplayIndex(int hlcState) {
    switch (hlcState) {
        case STATE_INIT: return 0;
        case STATE_CRITICAL_CUMULATIVE: return 1; 
        case STATE_FIRE: return 2;
        case STATE_CRASH: return 3;
        case STATE_WARNING: return 4;
        case STATE_NOMINAL: return 5;
        case STATE_RAD_LEAK: return 6; 
        default: return 5; // Default to Nominal if state is unknown
    }
}

// -------------------- 5. FFT HELPER FUNCTIONS (Team Code) --------------------
void collectSamples() {
  unsigned long microsPerSample = 1000000UL / SAMPLE_FREQ;
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;
  
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long tStart = micros();
    bmx160.getAllData(&Omagn, &Ogyro, &Oaccel); 

    // Use Z and Y axis for vibration samples
    vReal[i] = sqrt(
      (double)Oaccel.y*Oaccel.y +
      (double)Oaccel.z*Oaccel.z);

    vImag[i] = 0.0;
    while (micros() - tStart < microsPerSample);
  }

  // Remove mean
  double mean = 0.0;
  for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
  mean /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;
}

void runFFT() {
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
}

void recordBaseline() {
  collectSamples();
  runFFT();

  int binsPerBand = (SAMPLES/2) / BANDS;

  for (int b = 0; b < BANDS; b++) {
    float sum = 0;
    for (int i = b*binsPerBand; i < (b+1)*binsPerBand; i++) {
      sum += vReal[i];
    }
    baselineSpectrum[b] = sum / binsPerBand;
  }
  baselineSet = true;
}

double compareToBaseline() {
  if (!baselineSet) return 0.0;
  int binsPerBand = (SAMPLES/2) / BANDS;
  double diff = 0.0;
  
  for (int b = 0; b < BANDS; b++) {
    float sum = 0;
    for (int i = b*binsPerBand; i < (b+1)*binsPerBand; i++) {
      sum += vReal[i];
    }
    float liveAvg = sum / binsPerBand;
    float delta = liveAvg - baselineSpectrum[b];
    // NOTE: Reverting this back to delta * delta (Sum of Squares)
    // Sum of squares is the standard way to calculate spectral energy difference
    // to prevent positive/negative deltas from cancelling out.
    diff += delta * delta; 
  }
  return diff;
}
// -------------------- 6. NETWORK CONNECTION FUNCTION --------------------
void connectToWiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WIFI MODULE FAIL!");
    lcd.setRGB(255, 0, 0); lcd.home(); lcd.print("WIFI MODULE FAIL");
    while (true); 
  }

  while (status != WL_CONNECTED) {
    lcd.setRGB(100, 100, 255); lcd.home(); lcd.print("CONNECTING WIFI");
    lcd.setCursor(0, 1); lcd.print(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("\n✅ Connected to WiFi!");
}

// -------------------- 7. SETUP FUNCTION --------------------
void setup() {
  Serial.begin(115200); 
  lcd.begin(16, 2);
  lcd.setRGB(100, 100, 255); 
  lcd.home(); lcd.print(states[STATE_INIT].anomaly);
  lcd.setCursor(0, 1); lcd.print(states[STATE_INIT].instruction);
  
  // Set up Buzzer, Touch, and Button Pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TOUCH_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT); // Teammate's addition
  
  // IMU INIT (BMX160)
  if (bmx160.begin() != true){
    Serial.println(F("BMX160 INIT FAILED!"));
    lcd.setRGB(255, 0, 0); lcd.home(); lcd.print("BMX160 FAIL!");
    while(1);
  }
  Serial.println(F("BMX160 Initialized"));
  
  // LIS3MDL Init (as requested, though BMX160 is the main source)
  if (!lis3mdl.begin_I2C()) {
    Serial.println("LIS3MDL Init FAILED! (Secondary Mag)");
  }
  
  // RECORD BASELINE
  Serial.println(F("Recording FFT Baseline..."));
  lcd.setRGB(255, 100, 0); lcd.home(); lcd.print("RECORD BASELINE");
  recordBaseline(); 
  Serial.println(F("Baseline Recorded."));

  // NETWORK INIT
  connectToWiFi();
  ThingSpeak.begin(client);

  // --- IR SENSOR STABILIZATION FIX ---
  // FIX: Perform 10 dummy analog reads to stabilize the sensor and the ADC
  // and prevent an immediate false "RAD. LEAK!" alarm on boot.
  Serial.println(F("Stabilizing IR Sensor..."));
  for(int i = 0; i < 10; i++) {
    analogRead(IR_PIN); 
    delay(10); // Small delay between reads
  }
  Serial.print(F("Initial stabilized IR Reading: ")); 
  Serial.println(analogRead(IR_PIN));
  // ------------------------------------
  
  currentState = STATE_NOMINAL; // Start in State 5 (Nominal) after successful setup
}

// -------------------- 8. MAIN LOOP FUNCTION --------------------
void loop() {
  unsigned long currentTime = millis();
  
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  // --- Check for Manual Baseline Re-record ---
  // Pressing the button will re-run the FFT baseline sequence
  if (digitalRead(BUTTON_PIN) == HIGH) { 
      Serial.println(F("Button pressed! Re-recording baseline..."));
      lcd.setRGB(255, 100, 0); lcd.home(); lcd.print("RE-RECORDING");
      lcd.setCursor(0, 1); lcd.print("FFT BASELINE");
      recordBaseline();
      currentState = STATE_NOMINAL; // Reset state after re-record
      delay(2000); // Debounce and show message
  }
  
  // -------------------- A. SENSOR READS & PROCESSING --------------------
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
  
  // 1. TILT / LOAD SHIFT (GYRO)
  float gyrox = Ogyro.x / gyro_scale_factor; 
  float gyroy = Ogyro.y / gyro_scale_factor;
  float gyroz = Ogyro.z / gyro_scale_factor;
  float angleTilt = max(abs(atan2(gyroy, gyroz) * 180 / PI), abs(atan2(-gyrox, sqrt(gyroy * gyroy + gyroz * gyroz)) * 180 / PI));

  // 2. IMPACT / CRASH (ACCEL)
  float accel_raw_norm = sqrt(sq(Oaccel.x) + sq(Oaccel.y) + sq(Oaccel.z));
  float gForce_in_G = accel_raw_norm / 16384.0; // Convert raw LSB to G's
  float accel_x_in_G = Oaccel.x / 16384.0; // X-Axis component in G's (for F5)

  // 3. VIBRATION / INTEGRITY (FFT)
  collectSamples();
  runFFT(); 
  double freq_diff = compareToBaseline();
  const float alpha = 0.2;
  static double smooth_diff = 0;
  smooth_diff = alpha * freq_diff + (1 - alpha) * smooth_diff; // FFT_SCORE

  // 4. MAGNETOMETER (SECURITY)
  float magx = Omagn.x;
  float magy = Omagn.y; 
  float magn = sqrt(sq(magx) + sq(magy)); // Using 2D norm for deviation in transit is often sufficient
  float d_magn = abs(magn - home_magn); // Deviation score
  
  // 5. IR / EXTERNAL SENSOR (FIRE/RADIATION/TEMP)
  float ir_read = analogRead(IR_PIN); 
  
  // 6. TOUCH SENSOR (SECURITY)
  int touch_value = digitalRead(TOUCH_PIN);
  
  // --- HEADING CALCULATION (Sent to F6) ---
  float raw_heading = atan2(magy, magx) * 180.0 / PI;
  float true_heading = raw_heading + DECLINATION_ANGLE;
  if (true_heading < 0) {
    true_heading += 360;
  }
  
  // -------------------- B & C. HAZARD SCORE & STATE DETERMINATION --------------------
  
  // 1. Check for HARD CRITICAL EVENTS (Highest Priority)
  if (ir_read <= THRESH_RADIATION_CRITICAL) {
    currentState = STATE_RAD_LEAK; 
    hazardScore = 10;
  } 
  else if (gForce_in_G >= THRESH_G_FORCE_CRASH) {
    currentState = STATE_CRASH;
    hazardScore = 10;
  } 
  else {
    // 2. Calculate Additive Score for all remaining warnings (Minor/Major)
    int calculatedAdditiveScore = 0;

    // Fire/Overheat (Score +2)
    if (ir_read >= THRESH_OVERHEAT_WARNING) { 
        calculatedAdditiveScore += 2; 
    }

    // Tilt (Load Shift) (Score +1)
    if (angleTilt >= THRESH_TILT_WARNING) { 
        calculatedAdditiveScore += 1; 
    }

    // Vibration (Score +1)
    if (smooth_diff >= THRESH_FFT_WARNING) { 
        calculatedAdditiveScore += 1; 
    }

    // Mag Deviation (Security) (Score +1)
    if (d_magn >= THRESH_MAG_DEVIATION) { 
        calculatedAdditiveScore += 1; 
    }

    // Touch Breach (Security) (Score +1)
    if (touch_value == THRESH_TOUCH_BREACH) { 
        calculatedAdditiveScore += 1; 
    }

    // 3. Determine State based on Cumulative Score
    if (calculatedAdditiveScore >= THRESH_SCORE_CUMULATIVE_CRITICAL) {
      // Multiple warnings reached critical mass 
      currentState = STATE_CRITICAL_CUMULATIVE;
      hazardScore = 10;
    } 
    else if (calculatedAdditiveScore > 0) {
      // Minor warning(s) active (Score 1-4)
      currentState = STATE_WARNING; 
      hazardScore = calculatedAdditiveScore;
    } 
    else {
      // All clear
      currentState = STATE_NOMINAL;
      hazardScore = 0;
    }
  }
  
  // -------------------- D. LCD & BUZZER DISPLAY LOGIC --------------------
  TestState current = states[getDisplayIndex(currentState)]; // Get display text
  static unsigned long last_buzzer_ms = 0; 

  // Check for any Alert State (1, 2, 3, 4, 10)
  if (currentState != STATE_INIT && currentState != STATE_NOMINAL) { 
    
    // --- LCD Flashing Logic (200ms cycle) ---
    // FIX: Backlight stays ON. Flashing between Red and Cyan (instead of Red and OFF).
    if (currentTime % flashRate < (flashRate / 2)) {  
        lcd.setRGB(255, 0, 0); // Red (Critical Alert)
    } else {
        lcd.setRGB(0, 150, 255); // Cyan/Blue (Still ON, but less alarming color)
    }

    // --- Buzzer Logic ---
    // Critical SOS Alarm: STATES 1 (CUMULATIVE), 3 (CRASH), and 10 (RAD LEAK)
    if (currentState == STATE_CRITICAL_CUMULATIVE || currentState == STATE_CRASH || currentState == STATE_RAD_LEAK) {
        // SOS Pattern
        unsigned long t = currentTime - last_buzzer_ms;
        if (t >= MORSE_SOS_CYCLE_MS) {
            last_buzzer_ms = currentTime - (t % MORSE_SOS_CYCLE_MS);
            t = currentTime - last_buzzer_ms;
        }

        // SOS timing logic (S.O.S)
        if ( (t >= 0 && t < MORSE_DOT_MS) || (t >= 200 && t < 300) || (t >= 400 && t < 500) || 
             (t >= 800 && t < 1100) || (t >= 1200 && t < 1500) || (t >= 1600 && t < 1900) || 
             (t >= 2200 && t < 2300) || (t >= 2400 && t < 2500) || (t >= 2600 && t < 2700) ) {
            tone(BUZZER_PIN, BUZZ_FREQ); 
        }
        else {
            noTone(BUZZER_PIN);
        }
    } 
    // Single Short Buzz: STATE 4 (MINOR WARNING) or STATE 2 (FIRE)
    else if (currentState == STATE_WARNING || currentState == STATE_FIRE) {
        // Single Short Buzz pattern (100ms tone, 400ms silence)
        unsigned long time_in_cycle = currentTime - last_buzzer_ms;

        if (time_in_cycle >= BUZZ_CYCLE_MS) {
          last_buzzer_ms = currentTime - (time_in_cycle % BUZZ_CYCLE_MS);
          time_in_cycle = currentTime - last_buzzer_ms; 
        }

        if (time_in_cycle < BUZZ_PULSE_MS) { 
            tone(BUZZER_PIN, BUZZ_FREQ); 
        }
        else {
            noTone(BUZZER_PIN);
        }
    } 
  } 
  // Fixed GREEN for NOMINAL (State 5) - BUZZER OFF
  else if (currentState == STATE_NOMINAL || currentState == STATE_INIT) {
    if (currentState == STATE_NOMINAL) lcd.setRGB(0, 255, 0); 
    else lcd.setRGB(100, 100, 255); 
    
    noTone(BUZZER_PIN);
    last_buzzer_ms = currentTime; 
  }
  
  // Always update the display content
  lcd.home();
  lcd.print(current.anomaly);
  lcd.setCursor(0, 1);
  lcd.print(current.instruction);
  
  // -------------------- E. THINGSPEAK SEND --------------------
  
  // --- SET UP FIELDS ---
  ThingSpeak.setField(1, ir_read);        // Field 1: IR/Radiation/Temp Analog Read (In-Box Temp)
  ThingSpeak.setField(2, gForce_in_G);    // Field 2: Total G-force (Impact, Norm of all axes)
  ThingSpeak.setField(3, (float)smooth_diff);    // Field 3: FFT Vibration Score (Casting double to float fix)
  ThingSpeak.setField(4, angleTilt);      // Field 4: Tilt Angle (Load Shift)
  ThingSpeak.setField(5, accel_x_in_G);   // Field 5: X-Axis Acceleration (G's) - Front/Rear Impact Component
  ThingSpeak.setField(6, true_heading);   // Field 6: True Compass Heading (Degrees)
  ThingSpeak.setField(7, d_magn);         // Field 7: Mag Deviation (Security)
  ThingSpeak.setField(8, hazardScore);    // Field 8: The final Hazard Score (0-6, or 10 for critical)
  
  // Print to Serial for debugging
  Serial.println("\n--- TELLIGENCE Status ---");
  Serial.print("Internal State: "); Serial.print(currentState); Serial.print(" -> "); Serial.println(current.anomaly);
  Serial.print("Logged Hazard Score (F8): "); Serial.println(hazardScore); 
  Serial.print("F1 (IR/Temp): "); Serial.println(ir_read);
  Serial.print("F2 (gForce Norm): "); Serial.println(gForce_in_G);
  Serial.print("F3 (FFT Score): "); Serial.println(smooth_diff);
  Serial.print("Touch State: "); Serial.println(touch_value); // Added for debugging
  
  // WRITE DATA TO THINGSPEAK
  int http_response_code = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  if (http_response_code == 200) {
    Serial.println("✅ Data sent successfully! (HTTP 200)");
  } else {
    Serial.print("❌ Problem updating channel. HTTP error code: ");
    Serial.println(http_response_code);
  }

  delay(20000); // 20-second update cycle
}
