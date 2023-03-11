import numpy as np
import matplotlib.pyplot as plt

def db2pow(x):
    return 10**(x/10)
R = 110 # target's initial position [m]
v = -20 # target's initial velocity [m/s]
c = 3e8 # speed of light [m/s]
d_res = 1 # range resolution [m]
B = c / (2 * d_res)
R_max = 200 # maximum range [m]
Tchirp = 5.5 * 2 * (R_max) / c
slope = B / Tchirp
fc = 77e9 # carrier freq
Nr = 1024 # for length of time OR # of range cells
Nd = 128 # #of doppler cells OR #of sent periods % number of chirps
t = np.linspace(0, Nd * Tchirp, Nr * Nd) #total time for samples
Tx = np.zeros((len(t),)) # transmitted signal
Rx = np.zeros((len(t),)) # received signal
Mix = np.zeros((len(t),)) # beat signal
r_t = np.zeros((len(t),))
td = np.zeros((len(t),))
for i in range(len(t)):
    r_t[i] = R + v * t[i]
    td[i] = (2 * r_t[i]) / c
    Tx[i] = np.cos(2 * np.pi * (fc * t[i] + (slope * t[i]**2) / 2))
    Rx[i] = np.cos(2 * np.pi * (fc * (t[i] - td[i]) + (slope * (t[i] - td[i])**2) / 2))

Mix = Tx * Rx
Mix = np.reshape(Mix, (Nr, Nd))
Y = np.fft.fft(Mix, axis=0)
P = Y / Nr
P2 = np.abs(P)
P1 = P2[0:Nr // 2 + 1]
plt.figure('Range from First FFT')
plt.plot(P1)
plt.axis([0, 200, 0, 0.5])

Mix = np.reshape(Mix, (Nr, Nd))
sig_fft2 = np.fft.fft2(Mix, s=(Nr, Nd))
sig_fft2 = sig_fft2[0:Nr // 2, 0:Nd]
sig_fft2 = np.fft.fftshift(sig_fft2)
RDM = np.abs(sig_fft2)
RDM = 10 * np.log10(RDM)
doppler_axis = np.linspace(-100, 100, Nd)
range_axis = np.linspace(-200, 200, Nr // 2) * ((Nr // 2) / 400)
plt.figure('2D FFT output - Range Doppler Map')
plt.pcolormesh(doppler_axis, range_axis, RDM)
plt.colorbar()

Tr = 7 # number of training cells for range
Td = 7 # number of training cells for doppler
Gr = 2 # number of guard cells for range
Gd = 2 # number of guard cells for doppler
offset = 5 # offset the threshold by SNR value in [dB]
range_cells = 2 * (Tr + Gr) + 1
doppler_cells = 2 * (Td + Gd) + 1
noise_level = np.ones((doppler_cells, range_cells))
for i in range(1 + Td, 1 + Td + Gd * 2):
    for j in range(1 + Tr, 1 + Tr + Gr * 2):
        noise_level[i, j] =0
noise_level = noise_level / np.sum(noise_level)

threshold_CFAR = 10*np.log10(np.convolve(np.ones(range_cells), np.ones(doppler_cells)[:,np.newaxis]*db2pow(RDM), mode='same'))
threshold_CFAR = threshold_CFAR + offset
threshold_CFAR[0:Tr+Gr,:] = 0
threshold_CFAR[-(Tr+Gr):,:] = 0
threshold_CFAR[:,0:Td+Gd] = 0
threshold_CFAR[:,-(Td+Gd):] = 0

RDM = RDM > threshold_CFAR

plt.figure('Output of the 2D CFAR process')
plt.pcolormesh(doppler_axis, range_axis, RDM)
plt.colorbar()
plt.show()

