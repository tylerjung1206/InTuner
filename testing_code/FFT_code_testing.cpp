#include <iostream>
#include <fftw3.h>
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

const int N = 128;

vector<complex<double>> FFT(const double* signal, int N) {

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
}

//standard violin frequencies----

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
Peak peakDetector(const vector<complex<double>>& fftResult, double sampleRate = 8475.0) { //subject to change
  double topBin = 0;
  double topAmp = 0.0;

  for (int i = 0; i < N / 2; ++i) { 
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


int main() {
  cout << "Program started..." << endl;

  //data pipeline from the microcontroller
  const char* portname = "/dev/cu.usbmodem101";
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

  if (fd < 0) {
    cerr << "can't open serial port\n"; //can only use one serial port at a time
    return 1;
  }

  struct termios tty;
  tcgetattr(fd, &tty);
  cfsetospeed(&tty, B115200); //from .ino
  cfsetispeed(&tty, B115200);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 1;
  tty.c_cc[VTIME] = 1;
  tcsetattr(fd, TCSANOW, &tty);

  char c;
  vector<int> currentBatch;
  //queue<vector<int>> batchQueue;
  string currentLine;

  while (true) {
    if (read(fd, &c, 1) > 0) {
      if (c == '\n') {
        try {
          int sample = stoi(currentLine);
          currentBatch.push_back(sample);
          cout << "Sample [" << currentBatch.size() << "] = " << sample << endl;

          /*if (currentBatch.size() == N) {
            batchQueue.push(currentBatch);
            cout << "Batch size: " << batchQueue.size() << endl;
            currentBatch.clear();
          }*/

        } catch (...) {
          cerr << "invalid input : " << currentLine << endl;
        }
        currentLine.clear();
      } else {
      currentLine += c;
      }
    }

    if (currentBatch.size() == N) {
      vector<double> batchAsDouble;
      vector<int> curr = currentBatch;

      for(int sample : curr) {
        batchAsDouble.push_back(static_cast<double>(sample));
      }

      const double* curr_signal = batchAsDouble.data();
      vector<complex<double>> curr_freq_domain_result = FFT(curr_signal, N);
      Peak curr_peak = peakDetector(curr_freq_domain_result, 8475); //calculated sample rate
      cout << "peak: " << curr_peak.freq << endl;
      currentBatch.clear(); 
    }
  }


  close(fd);

  //call FFT method and other needed methods in some way
  return 0;
}

