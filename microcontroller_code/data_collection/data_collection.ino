#include <LiquidCrystal.h>

#include <LinkedList.h>

const int N = 128;
const double sampling_freq = 8475.0;
const unsigned long sampling_period_micros = 1000000.0 / sampling_freq;
LinkedList<int> samples = LinkedList<int>();

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

unsigned long lastLcdUpdate = 0;
const unsigned long lcdInterval = 1000;

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.print("Happy Practicing!");
  delay(1000);
  lcd.clear();
}

void loop() {
  samples.clear();
  for (int i = 0; i < N; i++) {
    unsigned long time = micros();
    int sample = analogRead(A0);
    samples.add(sample);
    Serial.println(sample); 

    unsigned long lcd_display_time = millis();

    if (lcd_display_time - lastLcdUpdate >= lcdInterval) {
      lastLcdUpdate = lcd_display_time;

      lcd.setCursor(0, 0);
      lcd.print("Amplitude:     ");  // pad to clear leftovers
      lcd.setCursor(0, 1);
      lcd.print("                "); // clear row
      lcd.setCursor(0, 1);
      lcd.print(sample);
    }
  
    while (micros() - time < sampling_period_micros) {

    }
  }

  Serial.println("done");
  delay(100);
}
