#!/usr/bin/env python
# coding: utf-8

# In[114]:


import numpy as np
from scipy.fft import fft, ifft, fftfreq
from scipy.io import wavfile
import matplotlib.pyplot as plt


# In[154]:


#slow recording (canon)
samplerate_slow, data_slow = wavfile.read('slow.wav')


# In[155]:


N = len(data_slow)
T = 1 / samplerate_slow

#convert to mono from stereo (averaging)
if len(data_slow.shape) == 2:
    data_slow = data_slow.mean(axis=1)

#signal normalization
data_slow = data_slow / np.max(np.abs(data_slow))

yf = fft(data)
xf = fftfreq(N, T)[:N//2]

plt.figure(figsize=(25, 4))
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2])) #scaled, normalized, and positive only
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")
plt.grid()
plt.show()


# In[156]:


#fast recording
samplerate_fast, data_fast = wavfile.read('fast.wav')


# In[157]:


N = len(data_fast)
T = 1.0 / samplerate_fast

if len(data_fast.shape) == 2:
    data_fast = data_fast.mean(axis=1)

data_fast = data_fast / np.max(np.abs(data_fast))

yf = fft(data_fast)
xf = fftfreq(N, T)[:N//2]

plt.figure(figsize=(25, 4))
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2])) #scaled and normalized
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")
plt.grid()
plt.show()


# In[164]:


#note- violin frequency ranges from 196 Hz (Low G) to over 12000 Hz (due to harmonics)

#first check notes on violin range
import librosa

#tuned to 440 hz
violin_possible_notes = [
    "G3", "G#3", "A3", "A#3", "B3",
    "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4",
    "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5", "A5", "A#5", "B5",
    "C6", "C#6", "D6", "D#6", "E6", "F6", "F#6", "G6", "G#6", "A6", "A#6", "B6",
    "C7", "C#7", "D7", "D#7", "E7"
]

notes_as_freq_440 = np.array([])

for note in violin_possible_notes:
    curr_freq = librosa.note_to_hz(note)
    notes_as_freq_440 = np.append(notes_as_freq_440, curr_freq)
    
print(notes_as_freq_440)

#i want to make sure it works for different tuning levels
def tuning_adjustment(freqs, deviation_from_440):
    freqs += deviation_from_440
    return


# In[165]:


#Feedback Mode- realtime feedback on intonation

test_1_rate, data_test_1 = wavfile.read('test_1.wav')

N = len(data_test_1)
T = 1.0 / test_1_rate

if len(data_test_1.shape) == 2:
    data_test_1 = data_test_1.mean(axis=1)

data_test_1 = data_test_1 / np.max(np.abs(data_test_1))

yf = fft(data_test_1)
xf = fftfreq(N, T)[:N//2]

plt.figure(figsize=(25, 4))
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2])) #scaled and normalized
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")
plt.grid()
plt.show()


# In[166]:


#non-double stop/chords peak detection

def convert_to_real(yf):
     return 2.0/N * np.abs(yf[0:N//2])
              
freq_dict = dict(zip(xf, convert_to_real(yf)))
print(list(freq_dict.items())[:10])

#O(n) time
def peak_detector(dict):
    top_freq = 0
    top_amp = 0
    for freq, amp in dict.items():
        if amp > top_amp:
            top_freq = freq
            top_amp = amp
    return {"freq": top_freq, "amp": top_amp}      


# In[167]:


#i played an A (slightly flat) in my test recording, as expected, the peak detector finds 439 hz to be the peak
print(peak_detector(freq_dict))


# In[169]:


#now the primary feature of the feedback system should be to return accuracy
#this can be measured in many ways and will most likely have changeable parameters

#convert to cents (for meaningful interpretation. hertz is not an absolute parameter)
#one cent is 1% of a semitone
def freq_to_cents(f_actual, f_reference):
    return 1200 * np.log2(f_actual / f_reference)


#0(n) runtime
def accuracy_single_note(peak, notes):
    differences = np.abs(peak - notes)
    
    note_diff_pairs = dict(zip(notes, differences))
    #print(note_diff_pairs)
    compare_freq = min(note_diff_pairs, key=note_diff_pairs.get)
    difference_cents = freq_to_cents(compare_freq, peak)
    
    #these are changeable parameters, ill adjust more further into testing
    if difference_cents > -5 and difference_cents <= 0:
        print(f'In tune, {difference_cents * -1} cents sharp')
        return 1
    
    elif difference_cents < -5:
        print(f'Out of tune, {difference_cents * -1} cents sharp')
        return 1
    
    elif difference_cents < 5 and difference_cents >= 0:
        print(f'In tune, {difference_cents} cents flat')
        return 0
    
    elif difference_cents > 5:
        print(f'Out of tune, {difference_cents} cents flat')
        return 0
    #1 indicates sharp, 0 indicates flat
    
accuracy_single_note(443, notes_as_freq_440)


# In[126]:


#Performance Mode- preliminary Python code below
#Objective- run an analysis on a short clip/recording of the violinist, give feedback


# In[ ]:




