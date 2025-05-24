#include <arduinoFFT.h>


//const int N = 256; use this if using board with more RAM
const int N = 128;
//int samples[N];
const double sampling_freq = 8475.0; //originally 10000.0 but due to unexpectly higher analog read time, i made it lower (allows  loop to be longer than 100microsecs)
//it appears analog read is around 116 to 120 so this is in adjustment to that
const double sampling_period = 1.0 / sampling_freq;
const unsigned long sampling_period_micros = sampling_period * 1000000;

float lastPeaks[3] = {0, 0, 0};
int peakIndex = 0;

float vReal[N];
float vImag[N];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, N, sampling_freq);

void setup() {
  Serial.begin(115200);
}

/*initial code, memory inefficient, but easier to read, use if I get a better board
void loop() {
  for (int i=0; i < N; i++) {
    unsigned long time = micros();
    samples[i] = analogRead(A0);

    //Serial.println(samples[i]); //remove later this takes a lot of time
    while (micros() - time < sampling_period_micros) {

    }
  }

  for (int i=0; i < N; i++) {
    vReal[i] = (float)samples[i]; 
    vImag[i] = 0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude(); 
  float x = FFT.majorPeak();

  Serial.print("Peak frequency: ");
  Serial.println(x);
  delay(1000);
}*/

void loop() {
  for (int i=0; i < N; i++) {
    unsigned long time = micros();
    vReal[i] = analogRead(A0);
    unsigned long time_after = micros();
    vImag[i] = 0;

    if (i == 0) { //only trust the first value (used for checking analog time)
      //Serial.println(time_after - time);
    } 

    while (micros() - time < sampling_period_micros) {

    }
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude(); 
  float x = FFT.majorPeak();

  lastPeaks[peakIndex] = x;
  peakIndex = (peakIndex + 1) % 3;

  float smoothedPeak = (lastPeaks[0] + lastPeaks[1] + lastPeaks[2]) / 3.0;

  Serial.print("Peak frequency: ");
  Serial.println(smoothedPeak, 2);
  Serial.println(analogRead(A0));
  delay(1000);
}



