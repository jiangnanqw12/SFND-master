import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

target_start_position = 110    # target range = 100m
target_velocity = -20          # target doppler velocity = -20m/s
fc = 77e9                      # carrier freq
max_range = 200
range_resolution = 1          # range resolution can determine bandwidth
max_velocity = 70             # max velocity can determine Ts_x-axis
c = 3e8
wavelength = c / fc           # wavelength
bandwidth = c / (2 * range_resolution)    # d_res = c/2*Bsweep
Tchirp = 5.5 * 2 * max_range / c
slope = bandwidth / Tchirp
Nd = 128                       # # of doppler cells OR # of sent periods % number of chirps
Nr = 1024                      # for length of time OR # of range cells
t = np.linspace(0, Nd * Tchirp, Nr * Nd)    # total time for samples
Tx = np.zeros_like(t)         # transmitted signal
Rx = np.zeros_like(t)         # received signal
Mix = np.zeros_like(t)        # beat signal
r_t = np.zeros_like(t)
td = np.zeros_like(t)
target_position = target_start_position

for i in range(len(t)):
    target_position = target_start_position + target_velocity * t[i]
    time_delay = (2 * target_position) / c    # compute time_delay.
    Tx[i] = np.cos(2 * np.pi * (fc * t[i] + slope * t[i]**2 / 2))
    Rx[i] = np.cos(2 * np.pi * (fc * (t[i] - time_delay) + slope * (t[i] - time_delay)**2 / 2))
    Mix[i] = Tx[i] * Rx[i]

Mix = np.reshape(Mix, [Nr, Nd])
Y = np.fft.fft(Mix, Nr, axis=0)
P2 = np.abs(Y / Nr)
P1 = P2[:Nr // 2 + 1, :]
P1T = P1

plt.figure('Range from First FFT')
plt.subplot(3, 1, 1)
plt.plot(P1)
plt.axis([0, 200, 0, 1])
plt.title('first')

P1M = P2[:Nr // 2 + 1, :]
plt.subplot(3, 1, 2)
plt.plot(P1M[:, 0])
plt.axis([0, 200, 0, 1])
plt.title('first')

plt.subplot(3, 1, 3)
plt.plot(P1M[:, 1])
plt.axis([0, 200, 0, 1])
plt.title('second')

Mix = np.reshape(Mix, [Nr, Nd])
sig_fft2 = np.fft.fft2(Mix, (Nr, Nd))
sig_fft2 = sig_fft2[:Nr // 2, :Nd]
sig_fft2 = np.fft.fftshift(sig_fft2)
RDM = np.abs(sig_fft2)
RDM = 10 * np.log10(RDM)
doppler_axis = np.linspace(-100, 100, Nd)
range_axis = np.linspace(-200,200, Nr // 2) * ((Nr // 2) / 400)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
D, R = np.meshgrid(doppler_axis, range_axis)
ax.plot_surface(D, R, RDM)
ax.set_xlabel('Doppler Velocity')
ax.set_ylabel('Range')
ax.set_zlabel('Signal Strength')
ax.set_title('2D FFT Range Doppler Map')

Tr = 10
Td = 8
Gr = 4
Gd = 4
offset = 6
noise_level = np.zeros((1, 1))
RESULT = np.zeros((Nr // 2, Nd))

for i in range(Tr + Gr + 1, Nr // 2 - (Gr + Tr)):
    for j in range(Td + Gd + 1, Nd - (Gd + Td)):
        noise_level = 0
        train_cell = 0
        for p in range(i - (Tr + Gr), i + Tr + Gr):
            for q in range(j - (Td + Gd), j + Td + Gd):
                if abs(i - p) > Gr or abs(j - q) > Gd:
                    noise_level += 10**(RDM[p, q] / 10)
                    train_cell += 1
        noise_average = noise_level / train_cell
        noise_threshold = 10 * np.log10(noise_average)
        noise_threshold += offset
        CUT = RDM[i, j]
        if CUT < noise_threshold:
            RESULT[i, j] = 0
        else:
            RESULT[i, j] = 1
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
D, R = np.meshgrid(doppler_axis, range_axis[Tr + Gr:Nr // 2 - (Gr + Tr)])
ax.plot_surface(D, R, RESULT[Tr + Gr:Nr // 2 - (Gr + Tr), :])
ax.set_xlabel('Doppler Velocity')
ax.set_ylabel('Range')
ax.set_zlabel('Detection')
ax.set_title('2D CFAR on Range Doppler Map')
plt.show()