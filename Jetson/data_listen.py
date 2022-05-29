import scipy.io.wavfile as wavf
import numpy as np
import os
import time

# Data to WAV translation code 
# The objective of this code is to be able to "hear" what the microphones are hearing

# The code is in its preliminary stages and isn't very useful yet... Hopefully later we can figure out how to slow down the sound to make it audible



nFiles = len([name for name in os.listdir('datasamples') ])
print(nFiles)
fileList = [name for name in os.listdir('datasamples')]
dirName = "datasamples/"
nDataSamples = 50;
print("Loading in data from " + str(nDataSamples) + " data samples and processing.")
tInit = time.perf_counter()

samples = [0]*10000*nDataSamples

for i in range(0, nDataSamples):
    fileName = fileList[i]
    file = open(dirName+fileName, "r")
    data = [0]*10000
    index = 0
    for line in file:
        if index < 10000:
            data[index] = int(line.strip('\n'))
            samples[index] = data[index]
            index = index + 1
        
            
    
fs = 465454
out_f = 'out.wav'
waveform = np.int16(samples)
wavf.write(out_f, fs, waveform)
