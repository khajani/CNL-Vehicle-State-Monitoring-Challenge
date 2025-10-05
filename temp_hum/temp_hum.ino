#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit AHT20 Example");

  if (!aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // populate temp and humidity objects

  Serial.print("Temperature: "); 
  Serial.print(temp.temperature); 
  Serial.println(" °C");
  Serial.print("Humidity: "); 
  Serial.print(humidity.relative_humidity); 
  Serial.println(" %");
  delay(2000);
}



rgb_lcd lcd;

// The LCD is already part of the Sensor Kit library as "Display"
//void setup() {
  //Serial.begin(115200);

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
