#include <Arduino.h>
#include <LiquidCrystal.h>
#include <arduinoFFT.h>
#include <vector>
#include <deque>
#include <iostream>
#include <unordered_map>
#include <string>
#include <map>
#include <algorithm>
using namespace std;

std::map<int, int> freqHistogram;
vector<float> recordedPeaks;

#define N 8192
const float sampFreq = 6000;

const int rs = 27, en = 26, d4 = 25, d5 = 33, d6 = 32, d7 = 14;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//feedback/performance mode pin
const int buttonPin = 4; 
bool inFeedbackMode = true;
bool lastButtonState = HIGH;

const int buttonPinLCD = 5;
bool modeOne = true;
bool lastStateLCD = HIGH;

bool inTune;

float vReal[N];
float vImag[N];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, N, sampFreq);

void sampleData() {
  unsigned long muPerSamp = 1000000 / sampFreq;
  unsigned long timer = micros();

  for (int i = 0; i < N; i++) {
    while (micros() - timer < muPerSamp) {
      
    }

    timer += muPerSamp;
    vReal[i] = analogRead(36);
    vImag[i] = 0;
  }
}

void removeDCOffset() {
  float mean = 0;
  for (int i = 0; i < N; i++) {
    mean += vReal[i];
  }
  mean /= N;

  for (int i = 0;i < N; i++) {
    vReal[i] -= mean;
  }
}

//manual standard violin freqs inputs (for reference)----

vector<string> stdViolinNotes = {
    "G3", "G#3", "A3", "A#3", "B3", "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", 
    "G4", "G#4", "A4", "A#4", "B4", "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", 
    "G5", "G#5", "A5", "A#5", "B5", "C6", "C#6", "D6", "D#6", "E6", "F6", "F#6", 
    "G6", "G#6", "A6", "A#6", "B6", "C7", "C#7", "D7", "D#7", "E7"
};

unordered_map<string, int> noteToMidi = {
    {"G3", 55}, {"G#3", 56}, {"A3", 57}, {"A#3", 58}, {"B3", 59},
    {"C4", 60}, {"C#4", 61}, {"D4", 62}, {"D#4", 63}, {"E4", 64},
    {"F4", 65}, {"F#4", 66}, {"G4", 67}, {"G#4", 68}, {"A4", 69}, 
    {"A#4", 70}, {"B4", 71}, {"C5", 72}, {"C#5", 73}, {"D5", 74}, 
    {"D#5", 75}, {"E5", 76}, {"F5", 77}, {"F#5", 78}, {"G5", 79}, 
    {"G#5", 80}, {"A5", 81}, {"A#5", 82}, {"B5", 83}, {"C6", 84},
    {"C#6", 85}, {"D6", 86}, {"D#6", 87}, {"E6", 88}, {"F6", 89}, 
    {"F#6", 90}, {"G6", 91}, {"G#6", 92}, {"A6", 93}, {"A#6", 94}, 
    {"B6", 95}, {"C7", 96}, {"C#7", 97}, {"D7", 98}, {"D#7", 99}, 
    {"E7", 100}
};

//-----

//midi to frequency conversion, dont need the tuning standardization function from python
//this returns single note, iterate through for loop to get a list of
double noteToFreq(const string& note, double tuning) {
    int midi = noteToMidi[note];
    return tuning * pow(2.0, (midi - 69) / 12.0);
}

//can replicate for others if needed
vector<double> freqs_440() {
  vector<double> freqs_in_440_hz;
  
  for (const auto& note : stdViolinNotes) {
    freqs_in_440_hz.push_back(noteToFreq(note, 440));
  }

  return freqs_in_440_hz;
}

//for feedback mode's accuracy feedback
double freqToCents(double f_actual, double f_reference) {
    return 1200.0 * log2(f_actual / f_reference);
}

//"notes" would be freqs_in_440_hz, or other tuning standardizations, for example
int accuracySingleNote(double peak, const vector<double>& notes) {
    double closestFreq = 0.0;
    double minDiff = numeric_limits<double>::max();

    for (const auto& noteFreq : notes) {
        double diff = abs(peak - noteFreq);
        if (diff < minDiff) {
            minDiff = diff;
            closestFreq = noteFreq;
        }
    }

    double differenceCents = freqToCents(closestFreq, peak);

    if (differenceCents > -5 && differenceCents <= 0) {

        //cout << "In tune, " << -differenceCents << " cents sharp\n";
        inTune = true;
        //return "In tune, " + to_string(-differenceCents) + " cents sharp\n";
        return -differenceCents;

    } else if (differenceCents < -5) {
        //cout << "Out of tune, " << -differenceCents << " cents sharp\n";
        inTune = false;
        //return "Out of tune" + to_string(-differenceCents) + " cents sharp\n";
        return -differenceCents;
    } else if (differenceCents < 5 && differenceCents >= 0) {
        inTune = true;
        //cout << "In tune, " << differenceCents << " cents flat\n";
        //return "In tune, " + to_string(differenceCents) + " cents flat\n";
        return -differenceCents;
    } else { 
        //cout << "Out of tune, " << differenceCents << " cents flat\n";
        inTune = false;
        //return "Out of tune, " + to_string(differenceCents) + " cents flat\n";
        return -differenceCents;
    }
}

//performance mode helper methods
bool peakAccuracy(double peakFreq, const vector<double>& violinFreqs, double toleranceCents = 20) {
    int centsDiff = accuracySingleNote(peakFreq, violinFreqs);
    return abs(centsDiff) <= toleranceCents;
}

//this determines the number the player receives regarding their accuracy
void weightedAccuracy(const vector<float>& peaks) {
    vector<double> violinFreqs = freqs_440();
    std::map<float, int> freqCounts;
    for (float f : peaks) {
        float rounded = round(f);
        freqCounts[rounded]++;
    }

    int totalPeaks = peaks.size();
    int accuratePeaks = 0;

    for (std::map<float,int>::const_iterator it = freqCounts.begin(); it != freqCounts.end(); ++it) {
        float freq = it->first;
        int count = it->second;

        if (peakAccuracy(freq, violinFreqs)) {
            accuratePeaks += count;
        }
    }

    float accuracyPercent = (totalPeaks > 0) ?(accuratePeaks * 100.0f / totalPeaks): 0;

    Serial.print("Weighted accuracy: ");
    Serial.print(accuracyPercent);
    Serial.println("%");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Accuracy:");
    lcd.setCursor(0, 1);
    lcd.print(accuracyPercent, 1);
    lcd.print("%");
    delay(6000);
}


//these are for the button toggle logic
void checkModeSwitch() {
  bool buttonState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && buttonState == LOW) {
    inFeedbackMode = !inFeedbackMode;
    lcd.clear();
    delay(50);
  }
  lastButtonState = buttonState;
}

void checkLCDModeSwitch() {
  static bool lastStateLCD = HIGH;
  bool LCDbuttonState = digitalRead(buttonPinLCD);

  if (lastStateLCD == HIGH && LCDbuttonState == LOW) {
    lcd.clear();
    modeOne = !modeOne;
    delay(50);
  }

  lastStateLCD = LCDbuttonState;
}


//for performance mode, identifies the top 10 frequencies during performance
void analyzeTopFrequencies() {
  std::map<float, int> roundedFreqCounts;

  //rounding
  for (float f : recordedPeaks) {
    float rounded = round(f);  
    roundedFreqCounts[rounded]++;
  }

  vector<pair<float, int>> freqVec(roundedFreqCounts.begin(), roundedFreqCounts.end());
  sort(freqVec.begin(), freqVec.end(), [](pair<float,int> &a, pair<float,int> &b) {
    return a.second > b.second;
  });


  Serial.println("Top 10 freqs:");
  for (int i = 0; i < 10 && i < freqVec.size(); i++) {
    Serial.print(freqVec[i].first);
    Serial.print(" Hz - ");
    Serial.print(freqVec[i].second);
    Serial.println(" times");
  }
}



void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);

  pinMode(buttonPin, INPUT);

  lcd.begin(16, 2);
  lcd.print("Happy Practicing");
  delay(2000);
  lcd.clear();
}


void loop() {
  checkModeSwitch();
  checkLCDModeSwitch();

  if (inFeedbackMode) {//feedback mode
    sampleData();
    removeDCOffset();

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    float x = FFT.majorPeak();

    vector<double> frequencies = freqs_440();
    int i = accuracySingleNote(x,frequencies);

    if (modeOne) {
      Serial.print("Peak frequency: ");
      Serial.print(x);
      Serial.println(" Hz");

      lcd.setCursor(0, 0);
      lcd.print("Peak freq (Hz):");
      lcd.setCursor(0, 1);
      lcd.print(x);
    } else {
      if (inTune) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("In tune by:");
        lcd.setCursor(0, 1);
        lcd.print(i);
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Out of tune by:");
        lcd.setCursor(0, 1);
        lcd.print(i);
      }
    }
    delay(100);

  } else { //performance mode
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Analyzing...");
    delay(1000);

    recordedPeaks.clear();

    unsigned long startMillis = millis();
    const unsigned long durationMillis = 10000;

    while (millis() - startMillis < durationMillis) {
      sampleData();
      removeDCOffset();

      FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(FFTDirection::Forward);
      FFT.complexToMagnitude();

      float peak = FFT.majorPeak();
      recordedPeaks.push_back(peak);

      Serial.print("Chunk peak: ");
      Serial.print(peak);
      Serial.println(" Hz");
    }

    analyzeTopFrequencies();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Done");
    delay(3000);
    weightedAccuracy(recordedPeaks);
    inFeedbackMode = true;
  }
}
