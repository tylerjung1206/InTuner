#include <Arduino.h>
#include <LiquidCrystal.h>
#include <iostream>
//#include <fftw3.h>
#include <arduinoFFT.h>

#include <complex>
#include <vector>
#include <unordered_map>
#include <string>
#include <cmath>
#include <map>
#include <queue>

#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

//I chose linked-list data structure because it queues and dequeue the fastest
//in my situation
class Node {
  public: 
    int value;
    Node* next;
    Node(int val) : value(val), next(nullptr) {   
    }
};

class LinkedList {
  private:
    Node* head;
    int size; 
  
  public:

    int getFirstandRemove() {
      //wont happen
      if (!head) {
        return -1;
      }

      int front_val = head->value;
      Node* curr = head;
      head = head->next;
      delete curr;
      size--;
      return front_val;
    }

    void addToEnd(int insert) {
      Node* insert_node = new Node(insert);
      if (!head) {
        head = insert_node;

      } else {
        Node* current = head;

        while (current->next) {
          current = current->next;
        }
        current->next = insert_node;
      }
      size++;
    }

    void clear() {
      while (head) {
        Node* temp = head;
        head = head->next;
        delete temp;
      }
      size = 0;
    }
};

const int N = 512;

vector<double> FFT(LinkedList curr_samples, int N) {

  float vReal[N];
  float vImag[N];
  ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, N, sampling_freq);
  for (int i=0; i < N; i++) {
    vReal[i] = curr_samples.getFirstandRemove();
    vImag[i] = 0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  vector<double> magnitudes;
  for (int i = 0; i < N; i++) {
    magnitudes.push_back(vReal[i]);
  }
  
  return magnitudes;
/*
  fftw_complex *in, *out;

  fftw_plan p;
  in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);

  for (int i=0; i < N; i++) {
    in[i][0] = signal[i];
    in[i][1] = 0.0;
  }


  p = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

  fftw_execute(p);

  vector<complex<double>> result(N);

  for (int i = 0; i < N; ++i) {
    result[i] = complex<double>(out[i][0], out[i][1]);
  }

  fftw_destroy_plan(p);
  fftw_free(in); fftw_free(out);

  return result;
  */
}

//standard violin freqs----

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


struct Peak {
  double freq;
  double amp;
};

//FFTW only gives bins, not freqs, so i have to convert to freqs using the sample rate (defined in ino code)
Peak peakDetector(const vector<double>& fftResult, double sampleRate) { //subject to change
  double topBin = 0;
  double topAmp = 0.0;

  for (int i = 0; i < fftResult.size(); ++i) { 
    double amp = abs(fftResult[i]);
    if (amp > topAmp) {
        topAmp = amp;
        topBin = i;
    }
  }
  double freq = topBin * sampleRate / N;
  return {freq, topAmp};
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

        cout << "In tune, " << -differenceCents << " cents sharp\n";
        return 1;
    } else if (differenceCents < -5) {
        cout << "Out of tune, " << -differenceCents << " cents sharp\n";
        return 1;
    } else if (differenceCents < 5 && differenceCents >= 0) {
        cout << "In tune, " << differenceCents << " cents flat\n";
        return 0;
    } else { 
        cout << "Out of tune, " << differenceCents << " cents flat\n";
        return 0;

    }

    //i might change the returns to smthing else soon
}


//double stops code

//top two peaks
pair<Peak, Peak> peakDetectorTopTwo(const vector<complex<double>>& fftResult, double sampleRate = 8475.0) { //again subject to change
  double topBin = 0;
  double topAmp = 0;
  double secondTopBin = 0;
  double secondTopAmp = 0;


  for (int i = 0; i < N / 2; ++i) { 
    double amp = abs(fftResult[i]);
    if (amp > topAmp) {
      secondTopAmp = topAmp;
      secondTopBin = topBin;

      topAmp = amp;
      topBin = i;

    } else if (amp > secondTopAmp) {
      secondTopAmp = amp;
      secondTopBin = i;
    }
  }

  double topFreq = topBin * sampleRate / N;
  double secondTopFreq = secondTopBin * sampleRate / N;

  return {{topFreq, topAmp}, {secondTopFreq, secondTopAmp}};

}

int overtone_compute(pair<Peak, Peak> topFreqs) {
  return abs(topFreqs.first.freq - topFreqs.second.freq);
}

int accuracyDoubleStops(double peak1, double peak2, const vector<double>& notes) {
    double closestFreq_1 = 0.0;
    double closestFreq_2 = 0.0;
    double minDiff = numeric_limits<double>::max();

    for (const auto& noteFreq : notes) {
        double diff_1 = abs(peak1 - noteFreq);
        if (diff_1 < minDiff) {
            minDiff = diff_1;
            closestFreq_1 = noteFreq;
        }
    }
    minDiff = numeric_limits<double>::max();

    for (const auto& noteFreq : notes) {
        double diff_2 = abs(peak2 - noteFreq);
        if (diff_2 < minDiff) {
            minDiff = diff_2;
            closestFreq_2 = noteFreq;
        }
    }

    double differenceCents_first = freqToCents(closestFreq_1, peak1);
    double differenceCents_second = freqToCents(closestFreq_2, peak2);

    //this logic will be determined at testing

    /*if (differenceCents_first > -5 && differenceCents_first <= 0) {

        cout << "In tune, " << -differenceCents << " cents sharp\n";
        return 1;

    } else if (differenceCents < -5) {
        cout << "Out of tune, " << -differenceCents << " cents sharp\n";
        return 1;
    } else if (differenceCents < 5 && differenceCents >= 0) {
        cout << "In tune, " << differenceCents << " cents flat\n";
        return 0;
    } else { 
        cout << "Out of tune, " << differenceCents << " cents flat\n";
        return 0;

    }

    //i might change the returns to smthing else soon*/
    return 0;
}



const double sampling_freq = 8475.0;
const unsigned long sampling_period_micros = 1000000.0 / sampling_freq;
LinkedList samples;

//const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
const int rs = 27, en = 26, d4 = 25, d5 = 33, d6 = 32, d7 = 14;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

unsigned long lastLcdUpdate = 0;
const unsigned long lcdInterval = 1000;

void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);

  lcd.begin(16, 2);
  lcd.print("Happy Practicing!");
  delay(1000);
  lcd.clear();
}

void loop() {
  samples.clear();
  for (int i = 0; i < N; i++) {
    unsigned long time = micros();
    int sample = analogRead(36);

    samples.addToEnd(sample);
    Serial.println(sample);

    unsigned long lcd_display_time = millis();

    if (lcd_display_time - lastLcdUpdate >= lcdInterval) {
      lastLcdUpdate = lcd_display_time;

      lcd.setCursor(0, 0);
      lcd.print("Amplitude:     ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0, 1);
      lcd.print(sample);
    }
  
    while (micros() - time < sampling_period_micros) {

    }
  }

  vector<double> freq_domain = FFT(samples, N);
  Peak peak = peakDetector(freq_domain, 8475.0);

  vector<double> frequencies = freqs_440();
  int accuracy = accuracySingleNote(peak.freq, frequencies);


  //Serial.println("done");
  delay(100);
}
