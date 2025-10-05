#include <WiFiS3.h>      // REQUIRED for Arduino Uno R4 WiFi networking
#include <ThingSpeak.h.h>  // REQUIRED for ThingSpeak functions
#include <Adafruit_LIS3MDL.h> // Magnetometer Library
#include <Adafruit_Sensor.h>  // Unified Sensor Library
#include <Wire.h>        // REQUIRED for I2C
#include <rgb_lcd.h>     // REQUIRED for LCD

// NEW: DHT Sensor Library
#include <DHT.h>

// Your Team's Libraries for Sensor Processing
#include <DFRobot_BMX160.h>
#include <arduinoFFT.h>

// -------------------- 2. YOUR PRIVATE CREDENTIALS --------------------
char ssid[] = "KJ"; 
char pass[] = "1236393639";

// ThingSpeak Channel and API Key
unsigned long myChannelNumber = 3100192; // REPLACE with your final channel number
const char * myWriteAPIKey = "NWU0RZBT5OC4183A"; // *** ThingSpeak Error: -301 often means a bad API key. Please double-check this key on your channel. ***

// **MAG/GPS Configuration**
const float DECLINATION_ANGLE = 12.5; // IMPORTANT: Set this for your location's magnetic variation!
// Magnetic baseline Norm (Used for deviation comparison) - Initialized to 0.0, will be recorded by button press if needed.
float home_magn = 0.0;    

// -------------------- 3. GLOBAL SYSTEM & SENSOR DEFS & THRESHOLDS --------------------
WiFiClient client;
int status = WL_IDLE_STATUS;

// SENSOR OBJECTS
DFRobot_BMX160 bmx160;
rgb_lcd lcd;               
Adafruit_LIS3MDL lis3mdl; 

// NEW: DHT SENSOR DEFINITION
#define DHT_PIN 4
#define DHT_TYPE DHT22 // Assuming DHT22 for better precision (change to DHT11 if required)
DHT dht(DHT_PIN, DHT_TYPE);
float temperature_c = 0.0; // F2
float humidity = 0.0;      // Read locally, but not logged to ThingSpeak

// FFT CONFIGURATION
#define SAMPLES 256
#define SAMPLE_FREQ 400
#define IR_PIN A0
#define BANDS 8

// FFT BUFFERS
float vReal[SAMPLES];
float vImag[SABLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLE_FREQ);

// BASELINE DATA
float baselineSpectrum[BANDS];
bool baselineSet = false;

// GYRO/TILT CONFIG
const float gyro_scale_factor = 16384.0; // Scaling factor for +/-2G range
const float criticalAngle = 80.0;

// *** HAZARD LEVEL CHECK (HLC) THRESHOLDS (Used for determining state and additive score) ***
// HARD CRITICAL ALERTS (These cause an immediate State 10 response)
const float THRESH_RADIATION_CRITICAL = 900.0;  // High IR reading -> Instant Score 10 (RAD_LEAK)
const float THRESH_FIRE_CRITICAL      = 850.0;    // Confirmed Fire/Overheat -> Instant Score 10 (FIRE)
const float THRESH_G_FORCE_CRASH      = 3.0;    // 3.0G+ is a definite crash -> Instant Score 10 (CRASH)

// WARNING ALERTS (used for additive scoring - Tier 1 point value)
const float THRESH_IR_WARNING         = 800.0;   // Score +2 initially
const float THRESH_TILT_WARNING       = criticalAngle; // Score +1 initially
const double THRESH_FFT_WARNING       = 50.0; // Score +1 initially
const float THRESH_MAG_DEVIATION      = 50.0;   // Score +1 initially
const int THRESH_TOUCH_BREACH         = HIGH;   // Score +1 (Binary - only one tier)

// NEW THRESHOLDS FOR TIERED SCORING (Tier 2: additive point value)
const float THRESH_IR_SEVERE          = 825.0;  // ADDED: Additive +2 (Total +4)
const float THRESH_TILT_SEVERE        = 90.0;   // Additive +2 (Total +3)
const double THRESH_FFT_SEVERE        = 450.0;  // Additive +2 (Total +3).
const float THRESH_MAG_DEVIATION_SEVERE = 150.0;  // Additive +2 (Total +3)

// *** ADDITIVE SCORING THRESHOLD ***
const int THRESH_SCORE_CUMULATIVE_CRITICAL = 5; // Cumulative score of 5 or more triggers SOS alert
// Minimum score required to trigger a visual/audio warning state (State 4)
const int THRESH_SCORE_MINOR_WARNING = 3; 

// --- STATE CONSTANTS (Used INTERNALLY for LCD message lookup and Alarm type) ---
const int STATE_INIT = 0;
const int STATE_CRITICAL_CUMULATIVE = 1; // Cumulative score >= 5 (Critical)
const int STATE_FIRE = 2;                // Hard Critical Fire Confirmed 
const int STATE_CRASH = 3;               // Hard G-force detection (Critical)
const int STATE_WARNING = 4;             // Any cumulative score >= 3 and < 5 (Minor)
const int STATE_NOMINAL = 5;             // All clear (Score 0, 1, or 2)
const int STATE_RAD_LEAK = 10;           // Hard radiation detection (Critical)


// --- BUZZER AND TOUCH SENSORS ---
#define BUZZER_PIN 8  // Digital pin for the buzzer (D8)
#define TOUCH_PIN 3   // Digital pin for the touch sensor (D3)
#define BUTTON_PIN 6 // Digital pin for baseline re-record button

// Morse Code Timing Constants (SOS Pattern for Critical Alerts)
const int BUZZ_FREQ = 1800; // High-pitched, urgent frequency

// SOS Timing (S.O.S: 3 Short, 3 Long, 3 Short)
const unsigned long MORSE_DOT_MS = 100;
const unsigned long MORSE_DASH_MS = 300;
const unsigned long MORSE_SOS_CYCLE_MS = 3000; 

// Single Short Buzz Timing (for Minor Warning States)
const unsigned long BUZZ_PULSE_MS = 100;
const unsigned long BUZZ_CYCLE_MS = 500; 


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
  // Index 2: STATE_FIRE (Confirmed Fire)
  {"FIRE! CONFIRMED", "PULL OVER, CALL"},
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
const int flashRate = 200;          // milliseconds for the LCD flash cycle (fast red flash)

// --- NON-BLOCKING TIMER VARIABLES ---
const unsigned long THINGSPEAK_INTERVAL_MS = 5000; // 5-second update interval
unsigned long lastThingSpeakTime = 0;
// ------------------------------------

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
    // NOTE: DFRobot library usage.
    bmx160.getAllData(&Omagn, Ogyro, Oaccel); 

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
    // Using Sum of Squares for spectral energy difference
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
  // Set Serial Baud Rate
  Serial.begin(115200); 
  
  // LCD Setup
  lcd.begin(16, 2);
  lcd.setRGB(100, 100, 255); 
  lcd.home(); lcd.print(states[STATE_INIT].anomaly);
  lcd.setCursor(0, 1); lcd.print(states[STATE_INIT].instruction);
  
  // Set up Pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TOUCH_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT); 
  
  // DHT Sensor Init
  dht.begin();
  Serial.println(F("DHT Sensor Initialized."));

  // IMU INIT (BMX160)
  if (bmx160.begin() != true){
    Serial.println(F("BMX160 INIT FAILED!"));
    lcd.setRGB(255, 0, 0); lcd.home(); lcd.print("BMX160 FAIL!");
    while(1);
  }
  
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;
  // Initialize home_magn to a single reading to make initial state valid
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
  Serial.println(F("BMX160 Initialized"));
  
  // LIS3MDL Init
  if (!lis3mdl.begin_I2C()) {
    Serial.println("LIS3MDL Init FAILED! (Secondary Mag)");
  }
  
  // RECORD BASELINE
  Serial.println(F("Recording FFT Baseline..."));
  lcd.setRGB(255, 100, 0); lcd.home(); lcd.print("RECORD BASELINE");
  recordBaseline(); 
  Serial.println(F("Baseline Recorded."));
  
  // Calibration Note
  Serial.print(F("Current Magnetic Baseline (home_magn) set to: "));
  Serial.println(home_magn);

  // NETWORK INIT
  connectToWiFi();
  ThingSpeak.begin(client);

  // --- IR SENSOR STABILIZATION ---
  Serial.println(F("Stabilizing IR Sensor..."));
  for(int i = 0; i < 10; i++) {
    analogRead(IR_PIN); 
    delay(10); 
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
  if (digitalRead(BUTTON_PIN) == HIGH) { 
      Serial.println(F("Button pressed! Re-recording baseline..."));
      lcd.setRGB(255, 100, 0); lcd.home(); lcd.print("RE-RECORDING");
      lcd.setCursor(0, 1); lcd.print("FFT BASELINE");
      recordBaseline();
      currentState = STATE_NOMINAL; 
      delay(2000); 
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
  // Convert raw LSB value to G's
  float gForce_in_G = accel_raw_norm / 16384.0; 

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
  float magn = sqrt(sq(magx) + sq(magy)); 
  float d_magn = abs(magn - home_magn); // Deviation score
  
  // 5. IR / EXTERNAL SENSOR (FIRE/RADIATION/TEMP)
  float ir_read = analogRead(IR_PIN); 
  
  // 6. TOUCH SENSOR (SECURITY)
  int touch_value = digitalRead(TOUCH_PIN);
  
  // 7. TEMP/HUMIDITY SENSOR (New DHT)
  float new_humidity = dht.readHumidity();
  float new_temp_c = dht.readTemperature();

  // Check if any reads failed and use the previous value if they did.
  if (!isnan(new_humidity) && !isnan(new_temp_c)) {
    temperature_c = new_temp_c;
    humidity = new_humidity;
  }
  
  // --- HEADING CALCULATION (Sent to F6) ---
  float raw_heading = atan2(magy, magx) * 180.0 / PI;
  float true_heading = raw_heading + DECLINATION_ANGLE;
  if (true_heading < 0) {
    true_heading += 360;
  }
  
  // -------------------- B & C. HAZARD SCORE & STATE DETERMINATION --------------------
  
  // 1. Check for HARD CRITICAL EVENTS (Highest Priority)
  if (ir_read >= THRESH_RADIATION_CRITICAL) {
    currentState = STATE_RAD_LEAK; 
    hazardScore = 10;
  } 
  else if (gForce_in_G >= THRESH_G_FORCE_CRASH) {
    currentState = STATE_CRASH;
    hazardScore = 10;
  } 
  // Confirmed Fire Check (IR is above 850.0) -> Flashing Red
  else if (ir_read >= THRESH_FIRE_CRITICAL) { 
    currentState = STATE_FIRE;
    hazardScore = 10;
  }
  else {
    // 2. Calculate Additive Score for all remaining warnings (Minor/Major)
    int calculatedAdditiveScore = 0;

    Serial.println("\n--- Additive Score Breakdown ---");

    // === TIER 1: IR Overheat/Fire (Score +2 OR +4 - Highly Additive) ===
    int ir_contribution = 0;
    if (ir_read >= THRESH_IR_WARNING) { 
        ir_contribution += 2; 
        Serial.println("  [+2] IR Overheat/Fire Warning (IR >= 800.0) - Base Score");
    }
    if (ir_read >= THRESH_IR_SEVERE) { 
        ir_contribution += 2; // Adds two more points for reaching severe level (825.0)
        Serial.println("  [+2] IR Overheat/Fire (SEVERE: IR >= 825.0) - Additional Score (Total +4)");
    }
    calculatedAdditiveScore += ir_contribution;
    
    // === TIER 2: Tilt (Load Shift) (Score +1 OR +3 - Additive) ===
    int tilt_contribution = 0;
    if (angleTilt >= THRESH_TILT_WARNING) {
        tilt_contribution += 1;
        Serial.println("  [+1] Tilt/Load Shift Warning (Angle >= 80.0 deg)");
    }
    if (angleTilt >= THRESH_TILT_SEVERE) {
        tilt_contribution += 2; // Adds two more points for reaching severe level
        Serial.println("  [+2] Tilt/Load Shift (SEVERE: Angle >= 90.0 deg) - Additional Score (Total +3)");
    }
    calculatedAdditiveScore += tilt_contribution;


    // === TIER 3: Vibration (Score +1 OR +3 - Additive) ===
    int fft_contribution = 0;
    if (smooth_diff >= THRESH_FFT_WARNING) { 
        fft_contribution += 1; 
        Serial.println("  [+1] Vibration/Integrity Warning (FFT >= 50.0)");
    }
    if (smooth_diff >= THRESH_FFT_SEVERE) { 
        fft_contribution += 2; // Adds two more points for reaching severe level
        Serial.println("  [+2] Vibration/Integrity (SEVERE: FFT >= 450.0) - Additional Score (Total +3)");
    }
    calculatedAdditiveScore += fft_contribution;

    
    // === TIER 4: Mag Deviation (Security) (Score +1 OR +3 - Additive) ===
    int mag_contribution = 0;
    if (d_magn >= THRESH_MAG_DEVIATION) { 
        mag_contribution += 1; 
        Serial.println("  [+1] Magnetic Deviation Warning (Mag Dev >= 50.0)");
    }
    if (d_magn >= THRESH_MAG_DEVIATION_SEVERE) { 
        mag_contribution += 2; // Adds two more points for reaching severe level
        Serial.println("  [+2] Magnetic Deviation (SEVERE: Mag Dev >= 150.0) - Additional Score (Total +3)");
    }
    calculatedAdditiveScore += mag_contribution;


    // === TIER 5: Touch Breach (Fixed +1) ===
    // Touch is a binary sensor (On/Off)
    if (touch_value == THRESH_TOUCH_BREACH) { 
        calculatedAdditiveScore += 1; 
        Serial.println("  [+1] Touch Breach Warning (Touch HIGH)");
    }

    Serial.print("  Current Total Score: "); Serial.println(calculatedAdditiveScore);
    Serial.println("------------------------------");

    // 3. Determine State based on Cumulative Score
    if (calculatedAdditiveScore >= THRESH_SCORE_CUMULATIVE_CRITICAL) {
      // Multiple warnings reached critical mass (Score 5+)
      currentState = STATE_CRITICAL_CUMULATIVE;
      hazardScore = 10; // Logged as 10 for a critical event
    } 
    // Check if score is high enough for a MINOR WARNING (3 or 4)
    else if (calculatedAdditiveScore >= THRESH_SCORE_MINOR_WARNING) {
      // Minor warning(s) active (Score 3 or 4)
      currentState = STATE_WARNING; // Triggers SOLID YELLOW
      hazardScore = calculatedAdditiveScore; // Log the actual score (3 or 4)
    } 
    else {
      // All clear, or insufficient score for a visual warning (scores 0, 1, or 2 are considered Nominal state)
      currentState = STATE_NOMINAL; // Triggers SOLID GREEN
      hazardScore = calculatedAdditiveScore; // Log the raw score (0, 1, or 2)
    }
  }
  
  // -------------------- D. LCD & BUZZER DISPLAY LOGIC (Runs continuously) --------------------
  TestState current = states[getDisplayIndex(currentState)]; // Get display text
  static unsigned long last_buzzer_ms = 0; 

  // Check for any Alert State (1, 2, 3, 4, 10)
  if (currentState != STATE_INIT && currentState != STATE_NOMINAL) { 
    
    // --- LCD Flashing/Color Logic ---
    // CRITICAL Alerts (1, 2, 3, 10) flash RED/BLACK (Fast Red)
    if (currentState == STATE_CRITICAL_CUMULATIVE || currentState == STATE_CRASH || currentState == STATE_RAD_LEAK || currentState == STATE_FIRE) {
        if (currentTime % flashRate < (flashRate / 2)) {  
            lcd.setRGB(255, 0, 0); // Red (Alert)
        } else {
            lcd.setRGB(0, 0, 0); // Black (Off) for high-urgency flash (Fast flash effect)
        }
    } 
    // MINOR Alert (4) is now SOLID YELLOW
    else if (currentState == STATE_WARNING) {
        lcd.setRGB(255, 255, 0); // Solid Yellow (Warning)
    }


    // --- Buzzer Logic (SOS or short pulse) ---
    // Critical states (1, 2, 3, 10) use SOS
    if (currentState == STATE_CRITICAL_CUMULATIVE || currentState == STATE_CRASH || currentState == STATE_RAD_LEAK || currentState == STATE_FIRE) {
        // SOS Pattern Logic... 
        unsigned long t = currentTime - last_buzzer_ms;
        if (t >= MORSE_SOS_CYCLE_MS) {
            last_buzzer_ms = currentTime - (t % MORSE_SOS_CYCLE_MS);
            t = currentTime - last_buzzer_ms;
        }
        if ( (t >= 0 && t < MORSE_DOT_MS) || (t >= 200 && t < 300) || (t >= 400 && t < 500) || 
             (t >= 800 && t < 1100) || (t >= 1200 && t < 1500) || (t >= 1600 && t < 1900) || 
             (t >= 2200 && t < 2300) || (t >= 2400 && t < 2500) || (t >= 2600 && t < 2700) ) {
            tone(BUZZER_PIN, BUZZ_FREQ); 
        } else { noTone(BUZZER_PIN); }
    } 
    // Minor state (4) uses short pulse
    else if (currentState == STATE_WARNING) {
        // Single Short Buzz Pattern Logic... 
        unsigned long time_in_cycle = currentTime - last_buzzer_ms;
        if (time_in_cycle >= BUZZ_CYCLE_MS) {
          last_buzzer_ms = currentTime - (time_in_cycle % BUZZ_CYCLE_MS);
          time_in_cycle = currentTime - last_buzzer_ms; 
        }
        if (time_in_cycle < BUZZ_PULSE_MS) { tone(BUZZER_PIN, BUZZ_FREQ); }
        else { noTone(BUZZER_PIN); }
    } 
  } 
  // Fixed GREEN for NOMINAL (State 5) - BUZZER OFF
  else if (currentState == STATE_NOMINAL || currentState == STATE_INIT) {
    if (currentState == STATE_NOMINAL) lcd.setRGB(0, 255, 0); // Solid Green
    else lcd.setRGB(100, 100, 255); // Blue for Init
    
    noTone(BUZZER_PIN);
    last_buzzer_ms = currentTime; 
  }
  
  // Always update the display content
  lcd.home();
  lcd.print(current.anomaly);
  lcd.setCursor(0, 1);
  lcd.print(current.instruction);
  
  // -------------------- E. SERIAL DEBUGGING (Runs continuously) --------------------
  int touch_value_debug = digitalRead(TOUCH_PIN); 
  Serial.println("\n--- Sensor Readings & State ---");
  Serial.print("Internal State: "); Serial.print(currentState); Serial.print(" -> "); Serial.println(current.anomaly);
  Serial.print("Logged Hazard Score (F8): "); Serial.print(hazardScore);
  
  // More specific messaging based on the new logic
  if (hazardScore == 10) Serial.println(" (CRITICAL RISK: SOS TRIGGERED!)");
  else if (hazardScore >= THRESH_SCORE_MINOR_WARNING) Serial.println(" (MINOR WARNING: Yellow Alert Active)");
  else if (hazardScore > 0) Serial.println(" (Minor Fluctuation: Check readings in ThingSpeak)");
  else Serial.println(" (All Clear)");

  Serial.print("F1 (IR/Rad): "); Serial.println(ir_read);
  Serial.print("F2 (Temp C): "); Serial.println(temperature_c);
  Serial.print("F3 (FFT Score): "); Serial.println(smooth_diff);
  Serial.print("F4 (Tilt): "); Serial.println(angleTilt);
  Serial.print("F5 (G-Force): "); Serial.println(gForce_in_G); // Re-added G-Force logging
  Serial.print("F7 (Mag Dev): "); Serial.println(d_magn);
  Serial.print("Touch State (D3): "); Serial.println(touch_value_debug); 


  // -------------------- F. THINGSPEAK SEND (Runs every 5 seconds) --------------------
  if (currentTime - lastThingSpeakTime >= THINGSPEAK_INTERVAL_MS) {

    // --- SET UP FIELDS ---
    ThingSpeak.setField(1, ir_read);        
    ThingSpeak.setField(2, temperature_c);    // F2: Temperature ONLY
    ThingSpeak.setField(3, (float)smooth_diff); 
    ThingSpeak.setField(4, angleTilt);      
    ThingSpeak.setField(5, gForce_in_G);      // F5: G-Force Norm (re-added for monitoring)
    ThingSpeak.setField(6, true_heading);   
    ThingSpeak.setField(7, d_magn);         
    ThingSpeak.setField(8, hazardScore);    
    
    // WRITE DATA TO THINGSPEAK
    int http_response_code = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    if (http_response_code == 200) {
      Serial.println("✅ Data sent successfully! (HTTP 200)");
    } else {
      Serial.print("❌ Problem updating channel. HTTP error code: ");
      Serial.println(http_response_code);
    }
    
    // Reset the timer for the next 5-second interval
    lastThingSpeakTime = currentTime; 
  }
}
