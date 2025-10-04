//Libraries 
#include <Wire.h>
#include <DFRobot_BMX160.h>
#include <Adafruit_AHTx0.h>
#include <rgb_lcd.h>
#include <math.h> 

// WiFi R4 Libraries
#include <WiFiS3.h>
#include <HTTPClient.h>

// WiFi Network Credentials (***YOU MUST UPDATE THESE***) ---
const char* ssid = "YOUR_WIFI_NETWORK_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverName = "http://cnl.monitoring.com/api/transport"; // CNL Server Endpoint

// ----------------------------------------------------------------
// --- 2. PIN DEFINITIONS AND HARDWARE SETUP ---
// ----------------------------------------------------------------
#define BUZZER_PIN 8      
#define IR_SENSOR_PIN A0  
#define ROTARY_PIN A1     

// Magnetometer Home/Calibration Values
#define HOME_MAGN 640.0
#define MAG_THRESHOLD 20.0 

// GPS Placeholder Coordinates (Fixed values to demonstrate location concept)
float latitude = 34.0522;    // Example: Los Angeles Latitude
float longitude = -118.2437; // Example: Los Angeles Longitude

// Hazard Level and Sensor Thresholds (ATLA CONFIGURATION)
// RENAMED from THREAT_LEVEL
#define HAZARD_LEVEL_WARNING 3
#define HAZARD_LEVEL_CRITICAL 6
#define G_FORCE_WARNING 1.5
#define G_FORCE_CRITICAL 4.0
#define TILT_CRITICAL 25.0
#define TEMP_CRITICAL 50.0
#define IR_CRITICAL_LOW 500
#define IR_WARNING_LOW 700

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

// STATE AND DATA VARIABLES (RENAMED)
int currentHazardScore = 0;
String primaryAnomaly = "NONE";

// ----------------------------------------------------------------
// --- HELPER FUNCTIONS (No changes needed) ---
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
    buzz(DOT_MS); buzz(DOT_MS); buzz(DOT_MS);
    buzz(DASH_MS); buzz(DASH_MS); buzz(DASH_MS);
    buzz(DOT_MS); buzz(DOT_MS); buzz(DOT_MS);
    delay(WORD_GAP_MS);
  } 
  else if (anomaly == "FIRE! CONTAIN!") {
    for(int i = 0; i < 4; i++) {
        buzz(50); delay(50);
    }
    delay(WORD_GAP_MS);
  }
  else {
    digitalWrite(BUZZER_PIN, HIGH); 
  }
}

// --- IMU Reading Functions (Magnetometer, Accel, Tilt) ---
float readMagnetometerDiff() {
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
  float magn = sqrt(pow(Omagn.x, 2) + pow(Omagn.y, 2) + pow(Omagn.z, 2));
  return abs(magn - HOME_MAGN);
}

float readIMU_GForce() {
  BMX160::Accl_Value accel;
  bmx160.getAllData(); 
  accel = bmx160.getAcceleration();
  
  float acc_x = accel.x / 16384.0; 
  float acc_y = accel.y / 16384.0; 
  float acc_z = accel.z / 16384.0; 
  
  return sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));
}

float readIMU_Tilt() {
  BMX160::Accl_Value accel;
  bmx160.getAllData(); 
  accel = bmx160.getAcceleration();
  
  float acc_y = accel.y / 16384.0; 
  float acc_z = accel.z / 16384.0; 
  float roll = atan2(acc_y, acc_z) * 180.0 / M_PI; 
  
  return abs(roll); 
}

float readAHT20_Temp() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); 
  return temp.temperature; 
}

// ----------------------------------------------------------------
// --- CORE LOGIC: Autonomous Threat Level Assessment (ATLA) ---
// ----------------------------------------------------------------
void runATLA(float gForce, float tiltAngle, float internalTemp, int irReading, int rotaryValue, float magDiff) {
  // RENAMED currentThreatScore to currentHazardScore
  currentHazardScore = 0;
  if (currentHazardScore < HAZARD_LEVEL_WARNING) { primaryAnomaly = "NONE"; }

  // 1. CRASH / IMPACT CHECK (Accel)
  if (gForce > G_FORCE_CRITICAL) { currentHazardScore = 10; primaryAnomaly = "CRASH! DANGER!"; return; }

  // 2. CONTAINER SECURITY/ROLLOVER (Mag/Tilt Combined)
  if (magDiff > MAG_THRESHOLD || tiltAngle > TILT_CRITICAL) {
    currentHazardScore += 5;
    if (primaryAnomaly == "NONE") {
      if (magDiff > MAG_THRESHOLD) { primaryAnomaly = "CONTAINER MOVED"; } 
      else { primaryAnomaly = "TILT/ROLL"; }
    }
  } else if (gForce > G_FORCE_WARNING) { currentHazardScore += 1; if (primaryAnomaly == "NONE") primaryAnomaly = "ROUGH ROAD"; }

  // 3. RADIATION LEAK CHECK
  if (irReading < IR_CRITICAL_LOW) { currentHazardScore += 4; if (primaryAnomaly == "NONE") primaryAnomaly = "RAD. LEAK! PULL!"; } 
  else if (irReading < IR_WARNING_LOW) { currentHazardScore += 2; if (primaryAnomaly == "NONE") primaryAnomaly = "Rad Warning"; }

  // 4. FIRE / OVERHEAT CHECK
  if (internalTemp > TEMP_CRITICAL) { currentHazardScore += 3; if (primaryAnomaly == "NONE") primaryAnomaly = "FIRE! CONTAIN!"; }

  // 5. LOAD SECURITY CHECK (Rotary Sensor)
  if (rotaryValue < 50) { currentHazardScore += 1; if (primaryAnomaly == "NONE") primaryAnomaly = "LOAD SHIFTED"; }
}


// ----------------------------------------------------------------
// --- DRIVER INTERFACE UPDATE (Idea 2) ---
// ----------------------------------------------------------------
void updateDriverInterface() {
  lcd.clear();
  lcd.home();
  String instruction = "ALL SYSTEMS OK"; 

  // RENAMED THREAT_LEVEL_CRITICAL to HAZARD_LEVEL_CRITICAL
  if (currentHazardScore >= HAZARD_LEVEL_CRITICAL) {
    // CRITICAL STATE
    if (millis() % 200 < 100) { lcd.setRGB(255, 0, 0); } else { lcd.setRGB(0, 0, 0); }
    
    // Idea 2: Actionable Instruction Logic (Concise Text)
    if (primaryAnomaly == "RAD. LEAK! PULL!") instruction = "SEEK SHELTER ASAP";
    else if (primaryAnomaly == "CRASH! DANGER!") instruction = "STAY PUT CALL CNL";
    else if (primaryAnomaly == "FIRE! CONTAIN!") instruction = "PULL OVER EXTING";
    else if (primaryAnomaly == "TILT/ROLL") instruction = "CRITICAL TILT!";
    else if (primaryAnomaly == "CONTAINER MOVED") instruction = "PULL OVER CHECK";
    else instruction = "EMERGENCY! CALL CNL"; 

    lcd.print(primaryAnomaly);
    lcd.setCursor(0, 1);
    lcd.print(instruction);

    playCodeAlarm(primaryAnomaly);
    
  // RENAMED THREAT_LEVEL_WARNING to HAZARD_LEVEL_WARNING
  } else if (currentHazardScore >= HAZARD_LEVEL_WARNING) {
    // WARNING STATE
    lcd.setRGB(255, 255, 0); 
    lcd.print("WARNING! Score: "); 
    // RENAMED currentThreatScore to currentHazardScore
    lcd.print(currentHazardScore);
    lcd.setCursor(0, 1);
    lcd.print(primaryAnomaly);

    if (millis() % 1000 < 500) { digitalWrite(BUZZER_PIN, HIGH); } else { digitalWrite(BUZZER_PIN, LOW); }
    
  } else {
    // NOMINAL STATE (Displaying GPS Placeholders)
    lcd.setRGB(0, 255, 0);
    
    // Line 1: Show Fixed Location (Demonstrates GPS Integration)
    lcd.print("Lat:"); lcd.print(latitude, 3);
    
    // Line 2: Show Fixed Location
    lcd.setCursor(0, 1);
    lcd.print("Lon:"); lcd.print(longitude, 3);

    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ----------------------------------------------------------------
// --- REMOTE COMMUNICATION (Idea 4) ---
// ----------------------------------------------------------------

void transmitData(int score, String anomaly, float gForce, float temp, String status) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    // Construct the data payload (JSON) - includes Location
    String jsonPayload = "{\"score\":" + String(score) + 
                         ",\"anomaly\":\"" + anomaly + 
                         "\",\"gForce\":" + String(gForce, 2) + 
                         ",\"temp\":" + String(temp, 1) + 
                         ",\"latitude\":" + String(latitude, 4) +   
                         ",\"longitude\":" + String(longitude, 4) + 
                         ",\"status\":\"" + status + "\"}"; 

    http.begin(serverName); 
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(jsonPayload);
    
    if (httpCode > 0) {
      lastTransmitTime = millis(); 
    }
    
    http.end();
  }
}

void checkAndTransmit() {
    float gForce = readIMU_GForce(); 
    float internalTemp = readAHT20_Temp();
    String status = "NOMINAL_CHECK";
    bool transmitted = false;

    // A. CRITICAL/WARNING STATE (Priority Packet Pushing)
    if (currentHazardScore >= HAZARD_LEVEL_WARNING) { // RENAMED
        criticalStateEndTime = millis(); 
        status = (currentHazardScore >= HAZARD_LEVEL_CRITICAL) ? "CRITICAL_ALERT" : "WARNING_ALERT"; // RENAMED

        if (millis() - lastTransmitTime > criticalInterval) {
             transmitData(currentHazardScore, primaryAnomaly, gForce, internalTemp, status); // RENAMED
             transmitted = true;
        }
    }
    
    // B. AUTOMATED "ALL-CLEAR" BURST
    else if (criticalStateEndTime != 0) { 
        if (millis() - criticalStateEndTime >= allClearHoldTime) {
            status = "ALL_CLEAR_CONFIRMED";
            transmitData(currentHazardScore, primaryAnomaly, gForce, internalTemp, status); // RENAMED
            criticalStateEndTime = 0; 
            transmitted = true;
        }
    }
    
    // C. NOMINAL STATE (Low-Frequency Check-in)
    if (!transmitted && currentHazardScore == 0) { // RENAMED
        if (millis() - lastTransmitTime > nominalInterval) {
            transmitData(currentHazardScore, primaryAnomaly, gForce, internalTemp, "NOMINAL_CHECK"); // RENAMED
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

  // --- WiFi Setup ---
  lcd.begin(16, 2);
  lcd.setRGB(0, 0, 255); 
  lcd.print("Connecting to WiFi");

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, password);
    delay(1000);
    lcd.print(".");
  }

  // Pin Setup
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // LCD Final Message
  lcd.clear();
  lcd.setRGB(0, 255, 0); 
  lcd.print("Guardian Node v2.0");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP()); 
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
  float magDiff = readMagnetometerDiff(); 

  // 2. RUN AUTONOMOUS THREAT LEVEL ASSESSMENT (ATLA)
  runATLA(gForce, tiltAngle, internalTemp, irReading, rotaryValue, magDiff);

  // 3. UPDATE DRIVER INTERFACE (LCD and Buzzer)
  updateDriverInterface();

  // 4. REMOTE COMMUNICATION (Idea 4)
  checkAndTransmit();

  delay(500); 
}
