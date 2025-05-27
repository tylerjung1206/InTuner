#include <iostream>
using namespace std;
#include <fftw3.h>
#include <complex>
#include <vector>
#include <unordered_map>
#include <string>
#include <cmath>
#include <map>


const int N = 128;

int main() {

  //recieving code from wifi signal from microcontroller
  //add once i get microcontroller

  //implement a queue to make sure data is received in order and not lost

  //call FFT method and other needed methods in some way
}



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
    "G3", "G#3", "A3", "A#3", "B3",
    "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4",
    "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5", "A5", "A#5", "B5",
    "C6", "C#6", "D6", "D#6", "E6", "F6", "F#6", "G6", "G#6", "A6", "A#6", "B6",
    "C7", "C#7", "D7", "D#7", "E7"
};

unordered_map<string, int> noteToMidi = {
    {"G3", 55}, {"G#3", 56}, {"A3", 57}, {"A#3", 58}, {"B3", 59},
    {"C4", 60}, {"C#4", 61}, {"D4", 62}, {"D#4", 63}, {"E4", 64},
    {"F4", 65}, {"F#4", 66}, {"G4", 67}, {"G#4", 68}, {"A4", 69}, {"A#4", 70}, {"B4", 71},
    {"C5", 72}, {"C#5", 73}, {"D5", 74}, {"D#5", 75}, {"E5", 76},
    {"F5", 77}, {"F#5", 78}, {"G5", 79}, {"G#5", 80}, {"A5", 81}, {"A#5", 82}, {"B5", 83},
    {"C6", 84}, {"C#6", 85}, {"D6", 86}, {"D#6", 87}, {"E6", 88},
    {"F6", 89}, {"F#6", 90}, {"G6", 91}, {"G#6", 92}, {"A6", 93}, {"A#6", 94}, {"B6", 95},
    {"C7", 96}, {"C#7", 97}, {"D7", 98}, {"D#7", 99}, {"E7", 100}
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
