const int N = 128;
const double sampling_freq = 8475.0;
const unsigned long sampling_period_micros = 1000000.0 / sampling_freq;

void setup() {
  Serial.begin(115200);
}

void loop() {
  for (int i = 0; i < N; i++) {
    unsigned long time = micros();

    int sample = analogRead(A0);
    Serial.println(sample); 
    while (micros() - time < sampling_period_micros);
  }

  //Serial.println("---");
  delay(100);
}
