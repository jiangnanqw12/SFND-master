import numpy as np
import matplotlib.pyplot as plt
target_start_position = 60 # target range = 100m
target_velocity = -50 # target doppler velocity = -20m/s
fc = 77e9 # carrier freq
max_range = 200
range_resolution = 1 # range resolution can determine bandwidth
max_velocity = 100 # max velocity can determine Ts_x-axis
c = 3e8
wavelength = c / fc # wavelength
bandwidth = c / (2 * range_resolution) # d_res = c/2*Bsweep
Tchirp = 5.5 * 2 * max_range / c
slope = bandwidth / Tchirp
Nd = 128 # #of doppler cells OR #of sent periods % number of chirps
Nr = 1024 # for length of time OR # of range cells
t = np.linspace(0, Nd * Tchirp, Nr * Nd) # total time for samples
Tx = np.zeros(len(t)) # transmitted signal
Rx = np.zeros(len(t)) # received signal
Mix = np.zeros(len(t)) # beat signal
r_t = np.zeros(len(t))
td = np.zeros(len(t))
target_position = target_start_position
for i in range(len(t)):
    target_position = target_start_position + target_velocity * t[i]
    time_delay = (2 * target_position) / c # compute time_delay.
    Tx[i] = np.cos(2 * np.pi * (fc * t[i] + slope * t[i] * t[i] / 2))
    Rx[i] = np.cos(2 * np.pi * (fc * (t[i] - time_delay) + slope * (t[i] - time_delay) * (t[i] - time_delay) / 2))
    Mix[i] = Tx[i] * Rx[i]

Mix = np.reshape(Mix, [Nr, Nd])
Y = np.fft.fft(Mix, Nr, 0)
P2 = np.abs(Y / Nr)
P1 = P2[:Nr // 2]
P1 = np.fft.fftshift(P1) # important
fig, ax = plt.subplots(figsize=(8, 6), nrows=2, ncols=1, sharex=True, gridspec_kw={"hspace": 0.3})
ax[0].plot(np.linspace(0, max_range, Nr // 2), P1)
ax[0].set_xlim([0, 200])
ax[0].set_ylim([0, 1])
ax[0].set_title("Range from First FFT")
sig_fft2 = np.fft.fft2(Mix, s=(Nr, Nd))
sig_fft2 = sig_fft2[:Nr // 2, :]
sig_fft2 = np.fft.fftshift(sig_fft2)
RDM = np.abs(sig_fft2)
RDM = 10 * np.log10(RDM)
doppler_axis = np.linspace(-100, 100, Nd)
range_axis = np.linspace(-200, 200, Nr // 2) * ((Nr // 2) / 400)
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(projection='3d')
X, Y = np.meshgrid(doppler_axis, range_axis)
ax.plot_surface(X, Y, RDM, cmap='viridis')
ax.set_xlabel('Doppler Velocity')
ax.set_ylabel('Range')
ax.set_zlabel('Amplitude')
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
                    noise_level = noise_level + np.power(10, RDM[p, q] / 10)
                    train_cell = train_cell + 1
        noise_average = noise_level / train_cell
        noise_threshold = 10 * np.log10(noise_average) + offset
        CUT = RDM[i, j]

    if CUT < noise_threshold:
        RESULT[i, j] = 0
    else:
        RESULT[i, j] = 1
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(projection='3d')
X, Y = np.meshgrid(doppler_axis, range_axis)
ax.plot_surface(X, Y, RESULT, cmap='viridis')
ax.set_xlabel('Doppler Velocity')
ax.set_ylabel('Range')
ax.set_zlabel('Amplitude')
plt.show()