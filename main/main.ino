// ----------------------------------------------------------------
// --- 1. LIBRARY INCLUSIONS ---
// ----------------------------------------------------------------
#include <Wire.h>
#include <DFRobot_BMX160.h>
#include <Adafruit_AHTx0.h>
#include <rgb_lcd.h>
#include <math.h> // Needed for sqrt() and pow()

// --- Wi-Fi Setup (Conceptual: Replace with your actual library/pins) ---
// Note: Actual Wi-Fi library/code depends heavily on your specific R4 Minima shield/module.
// The functions transmitData() and checkAndTransmit() are provided below.
// const char* ssid = "YourSSID";
// const char* password = "YourPassword";
// const char* serverName = "http://cnl.monitoring.com/api/transport"; 

// ----------------------------------------------------------------
// --- 2. PIN DEFINITIONS AND HARDWARE SETUP ---
// ----------------------------------------------------------------
#define BUZZER_PIN 8      // Digital Pin for the Grove Buzzer
#define IR_SENSOR_PIN A0  // Analog Pin for the IR Radiation Sensor
#define ROTARY_PIN A1     // Analog Pin for the Rotary Angle Sensor (Tie-Down Proxy)

// Magnetometer Home/Calibration Values (From your code)
// These should be set during a calibration routine when the container is secured.
#define HOME_MAGX 160.0
#define HOME_MAGY -30.0
#define HOME_MAGZ -620.0
#define HOME_MAGN 640.0
#define MAG_THRESHOLD 20.0 // Threshold for d_MagN to trigger an alert

// Threat Level and Sensor Thresholds (ATLA CONFIGURATION)
#define THREAT_LEVEL_WARNING 3
#define THREAT_LEVEL_CRITICAL 6
#define G_FORCE_WARNING 1.5   // g
#define G_FORCE_CRITICAL 4.0  // g
#define TILT_CRITICAL 25.0    // degrees
#define TEMP_CRITICAL 50.0    // Celsius (Container Internal Temp)
#define IR_CRITICAL_LOW 500   // Low analog reading indicates strong 'leak'
#define IR_WARNING_LOW 700    // Mid-range reading

// Buzzer Timing Constants (Idea 1)
#define DOT_MS 100
#define DASH_MS 300
#define GAP_MS 100
#define WORD_GAP_MS 1000 

// Wi-Fi Timing Constants (Idea 4)
unsigned long lastTransmitTime = 0;
const long nominalInterval = 300000; // 5 min
const long criticalInterval = 2000;  // 2 sec
const long allClearHoldTime = 300000; // 5 min
unsigned long criticalStateEndTime = 0;

// Initialize Sensor Objects
DFRobot_BMX160 bmx160;
Adafruit_AHTx0 aht;
rgb_lcd lcd;

// STATE AND DATA VARIABLES
int currentThreatScore = 0;
String primaryAnomaly = "NONE";

// ----------------------------------------------------------------
// --- HELPER FUNCTIONS ---
// ----------------------------------------------------------------

// --- Advanced Auditory Alarm (Idea 1) ---
void buzz(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
  delay(GAP_MS);
}

void playCodeAlarm(String anomaly) {
  if (anomaly == "RAD. LEAK! PULL!") {
    // SOS Pattern
    buzz(DOT_MS); buzz(DOT_MS); buzz(DOT_MS);
    buzz(DASH_MS); buzz(DASH_MS); buzz(DASH_MS);
    buzz(DOT_MS); buzz(DOT_MS); buzz(DOT_MS);
    delay(WORD_GAP_MS);
  } 
  else if (anomaly == "FIRE! CONTAIN!") {
    // Quick Repetitive Chirp
    for(int i = 0; i < 4; i++) {
        buzz(50); delay(50);
    }
    delay(WORD_GAP_MS);
  }
  // All other critical alarms (CRASH, TILT, MAGNETOMETER) get the continuous tone
  else {
    digitalWrite(BUZZER_PIN, HIGH); // Continuous tone
  }
}

// --- IMU Reading Functions ---

// 1. Container Security Check (Magnetometer)
// Returns the absolute difference in Magnetometer Magnitude (d_MagN)
float readMagnetometerDiff() {
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
  
  float magx = Omagn.x;
  float magy = Omagn.y;
  float magz = Omagn.z;
  float magn = sqrt(pow(magx, 2) + pow(magy, 2) + pow(magz, 2));

  // d_magn is the key metric for container position/shift (your logic)
  return abs(magn - HOME_MAGN);
}

// 2. Crash Detection (Accelerometer - G-Force Magnitude)
// Returns the G-Force magnitude. Requires calibration/scaling specific to your library setup.
// PLACEHOLDER: Implementation assumes the BMX160 library is set to return G-forces.
float readIMU_GForce() {
  BMX160::Accl_Value accel;
  bmx160.getAllData(); 
  accel = bmx160.getAcceleration();
  
  // Placeholder scaling - ADJUST BASED ON YOUR PARTNER'S CODE!
  float acc_x = accel.x / 16384.0; 
  float acc_y = accel.y / 16384.0; 
  float acc_z = accel.z / 16384.0; 
  
  return sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));
}

// 3. Rollover Detection (Gyroscope/Accelerometer - Tilt Angle)
// Returns the absolute angle of tilt.
float readIMU_Tilt() {
  BMX160::Accl_Value accel;
  bmx160.getAllData(); 
  accel = bmx160.getAcceleration();
  
  // Basic Roll calculation using Accelerometer data
  float acc_y = accel.y / 16384.0; 
  float acc_z = accel.z / 16384.0; 
  float roll = atan2(acc_y, acc_z) * 180.0 / M_PI; 
  
  return abs(roll); 
}

// 4. Internal Package Temperature
float readAHT20_Temp() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); 
  return temp.temperature; 
}

// ----------------------------------------------------------------
// --- CORE LOGIC: Autonomous Threat Level Assessment (ATLA) ---
// ----------------------------------------------------------------
void runATLA(float gForce, float tiltAngle, float internalTemp, int irReading, int rotaryValue, float magDiff) {
  // Reset score and anomaly for the new cycle
  currentThreatScore = 0;
  if (currentThreatScore < THREAT_LEVEL_WARNING) {
      primaryAnomaly = "NONE"; 
  }

  // 1. CRASH / IMPACT CHECK (Highest Priority - Accelerometer)
  if (gForce > G_FORCE_CRITICAL) {
    currentThreatScore = 10;
    primaryAnomaly = "CRASH! DANGER!";
    return; 
  }

  // 2. CONTAINER SECURITY CHECK (Magnetometer/Tilt Combined)
  if (magDiff > MAG_THRESHOLD || tiltAngle > TILT_CRITICAL) {
    // If container moved (magDiff) OR truck rolled (tilt)
    currentThreatScore += 5;
    if (primaryAnomaly == "NONE") {
      if (magDiff > MAG_THRESHOLD) {
          primaryAnomaly = "CONTAINER MOVED";
      } else {
          primaryAnomaly = "TILT/ROLL";
      }
    }
  } else if (gForce > G_FORCE_WARNING) {
    // Rough road warning
    currentThreatScore += 1;
    if (primaryAnomaly == "NONE") primaryAnomaly = "ROUGH ROAD";
  }

  // 3. RADIATION LEAK CHECK
  if (irReading < IR_CRITICAL_LOW) {
    currentThreatScore += 4;
    if (primaryAnomaly == "NONE") primaryAnomaly = "RAD. LEAK! PULL!";
  } else if (irReading < IR_WARNING_LOW) {
    currentThreatScore += 2;
    if (primaryAnomaly == "NONE") primaryAnomaly = "Rad Warning";
  }

  // 4. FIRE / OVERHEAT CHECK
  if (internalTemp > TEMP_CRITICAL) {
    currentThreatScore += 3;
    if (primaryAnomaly == "NONE") primaryAnomaly = "FIRE! CONTAIN!";
  }

  // 5. LOAD SECURITY CHECK (Rotary Sensor Proxy)
  if (rotaryValue < 50) { 
    currentThreatScore += 1;
    if (primaryAnomaly == "NONE") primaryAnomaly = "LOAD SHIFTED";
  }
}

// ----------------------------------------------------------------
// --- DRIVER INTERFACE UPDATE (Idea 2) ---
// ----------------------------------------------------------------
void updateDriverInterface() {
  lcd.clear();
  lcd.home();
  String instruction = "ALL SYSTEMS OK"; // Default instruction

  if (currentThreatScore >= THREAT_LEVEL_CRITICAL) {
    // --- CRITICAL STATE (Flashing Red + Actionable Instructions) ---
    if (millis() % 200 < 100) { 
        lcd.setRGB(255, 0, 0); // Flashing Red
    } else {
        lcd.setRGB(0, 0, 0); 
    }
    
    // Idea 2: Actionable Instruction Logic (Concise Text)
    if (primaryAnomaly == "RAD. LEAK! PULL!") instruction = "SEEK SHELTER ASAP";
    else if (primaryAnomaly == "CRASH! DANGER!") instruction = "STAY PUT CALL CNL";
    else if (primaryAnomaly == "FIRE! CONTAIN!") instruction = "PULL OVER EXTING";
    else if (primaryAnomaly == "TILT/ROLL") instruction = "CRITICAL TILT!";
    else if (primaryAnomaly == "CONTAINER MOVED") instruction = "PULL OVER CHECK";
    else instruction = "EMERGENCY! CALL CNL"; // Default Critical

    // Line 1: Anomaly
    lcd.print(primaryAnomaly);
    // Line 2: Action
    lcd.setCursor(0, 1);
    lcd.print(instruction);

    // Idea 1: Coded Buzzer Alarm
    playCodeAlarm(primaryAnomaly);
    
  } else if (currentThreatScore >= THREAT_LEVEL_WARNING) {
    // --- WARNING STATE (Yellow + Intermittent Buzz) ---
    lcd.setRGB(255, 255, 0); 
    lcd.print("WARNING! Score: ");
    lcd.print(currentThreatScore);
    lcd.setCursor(0, 1);
    lcd.print(primaryAnomaly);

    // Intermittent Buzzer
    if (millis() % 1000 < 500) { 
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
    
  } else {
    // --- NOMINAL STATE (Green + Silent) ---
    lcd.setRGB(0, 255, 0);
    lcd.print("SYSTEM NOMINAL");
    lcd.setCursor(0, 1);
    lcd.print("Temp:");
    lcd.print(readAHT20_Temp(), 1);
    lcd.print("C G:");
    lcd.print(readIMU_GForce(), 1);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ----------------------------------------------------------------
// --- REMOTE COMMUNICATION (Idea 4) ---
// ----------------------------------------------------------------

// Placeholder: transmitData function (Replace with your Wi-Fi implementation)
// This function must be completed using your specific Wi-Fi library
void transmitData(int score, String anomaly, float gForce, float temp, String status) {
  /* if (WiFi.status() == WL_CONNECTED) {
    // Your HTTP POST or other transmission code goes here
    // Example: WiFiClient client; client.connect(server, port); client.print(...);
    lastTransmitTime = millis();
    Serial.println("Data transmitted: " + status);
  } else {
    Serial.println("WiFi not connected. Data buffered.");
  }
  */
}

// Logic to decide WHEN and WHAT to transmit (Idea 4)
void checkAndTransmit() {
    float gForce = readIMU_GForce(); 
    float internalTemp = readAHT20_Temp();
    String status = "NOMINAL_CHECK";
    bool transmitted = false;

    // A. CRITICAL/WARNING STATE (Priority Packet Pushing)
    if (currentThreatScore >= THREAT_LEVEL_WARNING) {
        criticalStateEndTime = millis(); // Reset the 'All Clear' timer
        status = (currentThreatScore >= THREAT_LEVEL_CRITICAL) ? "CRITICAL_ALERT" : "WARNING_ALERT";

        if (millis() - lastTransmitTime > criticalInterval) {
             transmitData(currentThreatScore, primaryAnomaly, gForce, internalTemp, status);
             transmitted = true;
        }
    }
    
    // B. AUTOMATED "ALL-CLEAR" BURST
    else if (criticalStateEndTime != 0) { 
        if (millis() - criticalStateEndTime >= allClearHoldTime) {
            status = "ALL_CLEAR_CONFIRMED";
            transmitData(currentThreatScore, primaryAnomaly, gForce, internalTemp, status);
            criticalStateEndTime = 0; 
            transmitted = true;
        }
    }
    
    // C. NOMINAL STATE (Low-Frequency Check-in)
    if (!transmitted && currentThreatScore == 0) {
        if (millis() - lastTransmitTime > nominalInterval) {
            transmitData(currentThreatScore, primaryAnomaly, gForce, internalTemp, "NOMINAL_CHECK");
        }
    }
}

// ----------------------------------------------------------------
// --- SETUP FUNCTION ---
// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200); 
  Wire.begin(); 
  
  // Hardware Initialization
  if (bmx160.begin() != true) { Serial.println("BMX160 init false"); while(1); }
  if (!aht.begin()) { Serial.println("AHT20 init false"); while(1); }
  // Note: Add WiFi.begin(ssid, password) here once you have your Wi-Fi library ready.

  // Pin Setup
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // LCD Setup
  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0); 
  lcd.print("Guardian Node v2.0");
  lcd.setCursor(0, 1);
  lcd.print("System Initialized");
  delay(2000);
}

// ----------------------------------------------------------------
// --- LOOP FUNCTION (The continuous monitoring cycle) ---
// ----------------------------------------------------------------
void loop() {
  // 1. READ ALL SENSORS
  float gForce = readIMU_GForce();
  float tiltAngle = readIMU_Tilt();
  float internalTemp = readAHT20_Temp();
  int irReading = analogRead(IR_SENSOR_PIN);
  int rotaryValue = analogRead(ROTARY_PIN);
  float magDiff = readMagnetometerDiff(); // NEW: Container Position

  // 2. RUN AUTONOMOUS THREAT LEVEL ASSESSMENT (ATLA)
  runATLA(gForce, tiltAngle, internalTemp, irReading, rotaryValue, magDiff);

  // 3. UPDATE DRIVER INTERFACE (LCD and Buzzer)
  updateDriverInterface();

  // 4. REMOTE COMMUNICATION (Idea 4)
  checkAndTransmit();

  // Sample Rate: Check sensors every 500 milliseconds (half-second)
  delay(500); 
}
