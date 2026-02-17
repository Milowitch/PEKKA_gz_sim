void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  if (Serial1.available()) {
    String data = Serial1.readString();
    // Process the received string data
    Serial.println(data); // Example: Print to the main serial monitor
  }
}
