# 🚛 Canadian Nuclear Laboratories - Vehicle State Monitoring Challenge (Telligence)

Telligence is an Arduino-based platform designed for real-time monitoring of vehicle and container conditions during nuclear waste transport. Built on the Arduino Uno R4 WiFi, it integrates multiple sensors to detect crashes, vibrations, temperature, humidity, and simulated radiation leaks to help ensure safety and environmental protection during high-risk transport operations.

<img width="853" height="593" alt="image" src="https://github.com/user-attachments/assets/74eadc10-89ca-4df7-b54d-99b5b0909b9f" />


## 🚀 Features
- 🧠 Hazard Level Check (HLC) System — Calculates safety status using sensors like accelerometer, magnetometer, & vibration data (IMU-based)    
- ☢️ Radiation & Fire Detection — Detects abnormal IR readings indicating radiation leaks or fire hazards    
- 📡 Live Telemetry Uploads — Sends sensor data to ThingSpeak every few seconds for remote visualization & communication    
- 💡 LCD & Buzzer Alerts — Displays system state (Normal → Warning → Critical) and emits an SOS alarm pattern for emergencies    
- 🌡️ Environmental Monitoring — Measures real-time temperature and humidity using the DHT22 sensor    
- ⚙️ Sensor Fusion & Real-Time Processing — Combines multiple inputs for accurate, immediate hazard assessment    

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

<img width="891" height="474" alt="image" src="https://github.com/user-attachments/assets/eb1de686-5531-408e-927a-12c2d8bc403e" />

  
## 🧰 Hardware & Components    

- Arduino Uno R4 WiFi
- Grove IMU (Accelerometer, Gyroscope, Magnetometer)    
- AHT22 Temperature & Humidity Sensor    
- IR Sensor and Emitter (Radiation Simulation)    
- LCD Display    
- Buzzer    
- Breadboard, Wires, Resistors, Power Supply

## 🔌 Circuit Overview

- IMU: SDA/SCL → I²C pins on Arduino    
- DHT22: Signal → Digital pin      
- IR Sensor: Analog input for radiation/fire detection    
- LCD: I²C communication for display output    
- Buzzer: Digital output for alerts    
- WiFi Telemetry: Built-in R4 WiFi module connects to ThingSpeak

<img width="492" height="637" alt="image" src="https://github.com/user-attachments/assets/35c330b5-e3ba-469b-a4c7-cbcb01209c06" />


## 🌐 Data Visualization

All system data is transmitted to ThingSpeak, where it can be visualized in real time through charts showing:

- Vibration magnitude
- Temperature & humidity trends
- Radiation detection states
- System status over time

🔗 Interface: https://thingspeak.mathworks.com/channels/3100192

 <img width="470.5" height="361" alt="image" src="https://github.com/user-attachments/assets/1b89552a-b46b-4d6c-8ae9-c7834bbda45e" />

## 🔮 Future Improvements

- Add GPS tracking for full vehicle telemetry    
- Implement MQTT or HTTPS for more secure cloud communication    
- Include sensors for reliability in harsh conditions    
- Use location to alert citizens and stakeholders (using sensor trends too)      
- Use a better processor      
- Connect with a better database for multiple trucks and interface for driver     



