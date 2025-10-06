# 🚛 Canadian Nuclear Laboratories - Vehicle State Monitoring Challenge

Telligence is an Arduino-based platform designed for real-time monitoring of vehicle and container conditions during nuclear waste transport. Built on the Arduino Uno R4 WiFi, it integrates multiple sensors to detect crashes, vibrations, temperature, humidity, and simulated radiation leaks to help ensure safety and environmental protection during high-risk transport operations.

## 🚀 Features
🧠 Hazard Level Check (HLC) System — Calculates safety status using sensors like accelerometer, magnetometer, and vibration data (IMU-based)    
☢️ Radiation & Fire Detection — Detects abnormal IR readings indicating radiation leaks or fire hazards    
📡 Live Telemetry Uploads — Sends sensor data to ThingSpeak every few seconds for remote visualization & communication    
💡 LCD & Buzzer Alerts — Displays system state (Normal → Warning → Critical) and emits an SOS alarm pattern for emergencies    
🌡️ Environmental Monitoring — Measures real-time temperature and humidity using the DHT22 sensor    
⚙️ Sensor Fusion & Real-Time Processing — Combines multiple inputs for accurate, immediate hazard assessment    

## ⚙️ How It Works

1. Data Collection:    
The IMU captures vibration, acceleration, and tilt data, while the AHT22 monitors ambient temperature and humidity.     

2. Event Detection:    
The IR sensor detects radiation or fire simulation signals based on IR intensity thresholds.    

3. Hazard Scoring:    
A weighted algorithm evaluates vibration intensity, tilt, and environmental readings to determine overall system status.    

4. Alerts & Telemetry:    
The LCD displays current state (Normal / Warning / Critical) with color coordination. The buzzer provides tiered audio feedback with an SOS alarm pattern for critical alerts.    

5. Cloud Upload:    
Sensor data is transmitted via WiFi to ThingSpeak for live data tracking and analysis.    
  
## 🧰 Hardware & Components    

- Arduino Uno R4 WiFi
- Grove IMU (Accelerometer, Gyroscope, Magnetometer)    
- DHT22 Temperature & Humidity Sensor    
- IR Sensor and Emitter (Radiation Simulation)    
- LCD Display    
- Piezo Buzzer    
- Breadboard, Wires, Resistors, Power Supply    

- https://thingspeak.mathworks.com/channels/3100192

 <img width="1882" height="1244" alt="image" src="https://github.com/user-attachments/assets/1b89552a-b46b-4d6c-8ae9-c7834bbda45e" />
