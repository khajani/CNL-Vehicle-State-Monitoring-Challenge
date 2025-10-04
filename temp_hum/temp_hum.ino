#include "Arduino_SensorKit.h"
#include <rgb_lcd.h>
#include <Wire.h>
#include <AHT20.h>
AHT20 aht20;

void setup()
{
  Serial.begin(115200);
  Serial.println("AHT20 example");

  Wire.begin();
  if (aht20.begin() == false)
  {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while(true);
  }
}

void loop()
{
  float temperature = aht20.getTemperature();
  float humidity = aht20.getHumidity();

  Serial.print("T: ");
  Serial.print(temperature, 2);
  Serial.print(" C\t H: ");
  Serial.print(humidity, 2);
  Serial.println("% RH");

  delay(2000);
}



rgb_lcd lcd;

// The LCD is already part of the Sensor Kit library as "Display"
void setup() {
  Serial.begin(115200);

  // // Start LCD
  // Display.begin();
  // Display.backlight();

  // Start the temperature/humidity sensor
  //Environment.begin();
}

//void loop() {
  //float temperature = Environment.readTemperature();
  //float humidity = Environment.readHumidity();

  //Serial.print("Temperature = ");
  //Serial.print(Environment.readTemperature());
  //Serial.println(" C");

  //Serial.print("Humidity = ");
  //Serial.print(Environment.readHumidity());
  //Serial.println(" %");

  // Clear LCD and display message if temp > 25
  // Display.clear();
  // if (temperature > 25.0) {
  //   Display.setCursor(0, 0);
  //   Display.print("Temp too high");
  // } else {
  //   Display.setCursor(0, 0);
  //   Display.print("Temp: ");
  //   Display.print(Environment.readTemperature());
  //   Display.print(" C");
    
  //   Display.setCursor(0, 1);
  //   Display.print("Humidity: ");
  //   Display.print(Environment.readHumidity());
  //   Display.print("%");
  // }

  delay(20);
}
