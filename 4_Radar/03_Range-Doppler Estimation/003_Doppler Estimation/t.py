import numpy as np
import matplotlib.pyplot as plt

# Set up parameters
c = 3e8  # Speed of light in m/s
fc = 77e9  # Carrier frequency in Hz
R = 200  # Maximum range in meters
B = 40e6  # Chirp bandwidth in Hz
Tchirp = 5.6e-6  # Chirp duration in s

# Calculate range frequency and beat frequency
f_r = np.linspace(-B/2, B/2, 1000)  # Range frequency in Hz
tau = 2*R/c  # Two-way time delay in seconds
f_b = 2*B*tau  # Beat frequency in Hz

# Plot range frequency and beat frequency
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
ax1.plot(f_r, np.ones_like(f_r)*fc, 'b', label='Carrier frequency')
ax1.plot(f_r, np.ones_like(f_r)*(fc + f_r), 'r', label='Range frequency')
ax1.set_xlabel('Frequency (Hz)')
ax1.set_ylabel('Magnitude (Hz)')
ax1.legend()
ax1.set_title('Range frequency vs Carrier frequency')

ax2.plot(f_r, np.ones_like(f_r)*f_b, 'g', label='Beat frequency')
ax2.set_xlabel('Frequency (Hz)')
ax2.set_ylabel('Magnitude (Hz)')
ax2.legend()
ax2.set_title('Beat frequency vs Range frequency')

plt.tight_layout()
plt.show()
