#define ctsPin 3 // Pin for capactitive touch sensor

void setup() {
  Serial.begin(115200);
  pinMode(ctsPin, INPUT);
}

void loop() {
  int ctsValue = digitalRead(ctsPin);
  if (ctsValue == HIGH) {
    Serial.println("TOUCHED");
  }  else{
    Serial.println("not touched");
  }
delay(500);

}

