import numpy as np
import matplotlib.pyplot as plt

# Data_points
Ns = 1000

# Generate random noise
s = np.abs(np.random.randn(Ns, 1))

idx_list=[100, 200, 300, 700]
val_list=[8, 9, 4, 11]
# Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
for i in range(len(idx_list)):
    s[idx_list[i]] = val_list[i]


# plot the output
plt.plot(s)

# TODO: Apply CFAR to detect the targets by filtering the noise.

# 1. Define the following:
# 1a. Training Cells
# 1b. Guard Cells
T = 12  # number of training cell
G = 4  # number of guard cell

# Offset : Adding room above noise threshold for desired SNR
offset = 5

# Vector to hold threshold values
threshold_cfar = []

# Vector to hold final signal after thresholding
signal_cfar = []

# 2. Slide window across the signal length
for i in range(Ns - (G + T + 1)):

    # 2. - 5. Determine the noise threshold by measuring it within the training cells
    # compute noise of training cell
    noise_level = np.sum(s[i:i + T - 1])
    threshold = (noise_level / T) * offset
    threshold_cfar.append(threshold)

    # 6. Measuring the signal within the CUT
    signal = float(s[i + T + G])

    if signal < threshold:
        signal = 0

    # 8. Filter the signal above the threshold
    signal_cfar.append(signal)
print(signal_cfar)
# plot the filtered signal
plt.plot(signal_cfar, 'g--')

# plot original sig, threshold and filtered signal within the same figure.
plt.figure()
plt.plot(s)
plt.plot(np.roll(threshold_cfar, G), 'r--', linewidth=2)
plt.plot(np.roll(signal_cfar, (T + G)), 'g--', linewidth=4)
plt.legend(['Signal', 'CFAR Threshold', 'Detection'])
plt.show()
