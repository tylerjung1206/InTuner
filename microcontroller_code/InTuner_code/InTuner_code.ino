void setup() {
  Serial.begin(115200);
}

void loop() {
  int micValue = analogRead(A0);
  Serial.println(micValue);      
  delayMicroseconds(100);
}
