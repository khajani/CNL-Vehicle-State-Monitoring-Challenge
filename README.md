# 🚛 Canadian Nuclear Laboratories - Vehicle State Monitoring Challenge

Telligence is an Arduino-based platform designed for real-time monitoring of vehicle and container conditions during nuclear waste transport. Built on the Arduino Uno R4 WiFi, it integrates multiple sensors to detect crashes, vibrations, temperature, humidity, and simulated radiation leaks to help ensure safety and environmental protection during high-risk transport operations.

## 🚀 Features
🧠 Hazard Level Check (HLC) System — Calculates safety status using sensors like accelerometer, magnetometer, and vibration data (IMU-based)
☢️ Radiation & Fire Detection — Detects abnormal IR readings indicating radiation leaks or fire hazards
📡 Live Telemetry Uploads — Sends sensor data to ThingSpeak every few seconds for remote visualization & communication
💡 LCD & Buzzer Alerts — Displays system state (Normal → Warning → Critical) and emits an SOS alarm pattern for emergencies
🌡️ Environmental Monitoring — Measures real-time temperature and humidity using the DHT22 sensor
⚙️ Sensor Fusion & Real-Time Processing — Combines multiple inputs for accurate, immediate hazard assessment

- Implemented Hazard Level Check system to assign score based on issues detected in sensors

- https://thingspeak.mathworks.com/channels/3100192

 <img width="1882" height="1244" alt="image" src="https://github.com/user-attachments/assets/1b89552a-b46b-4d6c-8ae9-c7834bbda45e" />
