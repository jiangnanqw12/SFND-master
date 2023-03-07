
import numpy as np
import matplotlib.pyplot as plt

Fs = 1000  # Sampling frequency
T = 1 / Fs  # Sampling period
L = 1500  # Length of signal
t = np.arange(0, L) * T  # Time vector

# TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
S = 0.7 * np.sin(77 * 2 * np.pi * t) + 2 * np.sin(43 * 2 * np.pi * t)

# Corrupt the signal with noise
X = S + 2 * np.random.randn(len(t))

# Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t).
plt.plot(1000 * t[:50], X[:50])
plt.title('Signal Corrupted with Zero-Mean Random Noise')
plt.xlabel('t (milliseconds)')
plt.ylabel('X(t)')

# TODO : Compute the Fourier transform of the signal.
signal_fft = np.fft.fft(X)

# TODO : Compute the two-sided spectrum P2.
# Then compute the single-sided spectrum P1 based on
# P2 and the even-valued signal length L.
# first take the amplitude of the normalized signal
P2 = np.abs(signal_fft / L)
# then we just compute the single-sided spectrum as we reject the mirror image
P1 = P2[:L // 2 + 1]

# Plotting
f = Fs * np.arange(0, L // 2 + 1) / L
plt.figure()
plt.plot(f, P1)
plt.title('Single-Sided Amplitude Spectrum of X(t)')
plt.xlabel('f (Hz)')
plt.ylabel('|P1(f)|')
plt.show()
