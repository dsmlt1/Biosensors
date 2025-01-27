import serial
import matplotlib.pyplot as plt
import numpy as np
import time

s = serial.Serial('com10', 9600) # podesite odgovarajuci COM port
samples = []
k = 0
N = 720
Fs = 360
print("Reading starts...")
t1 = time.time()
while (k < N):
    if s.inWaiting() > 0:
        m = s.readline()
        try:
            r = float(m)
            samples.append(r)
            k = k + 1
        except:
            pass
print(f'{time.time()-t1:.2f}')
print("Reading ends...")
t = np.arange(N)/Fs
fig = plt.figure()
plt.title("Potentiometer voltage")
plt.xlabel('t[s]')
plt.ylabel('Amplitude[V]')
plt.grid();plt.ylim([-0.2,3.6])
plt.plot(t,samples)
plt.show()

