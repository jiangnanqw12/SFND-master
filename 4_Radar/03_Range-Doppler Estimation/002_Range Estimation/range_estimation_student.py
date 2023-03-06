import numpy as np

# Define constants
c = 3 * 10**8
resolution = 1
range_max = 300

# Calculate the bandwidth of the chirp signal
Bsweep = c / (2 * resolution)

# Calculate the chirp time
Ts = 5.5 * (2 * range_max / c)

# Define the frequency shifts
fb = np.array([0, 1.1e6, 13e6, 24e6])

# Calculate the range
calculated_range = c * Ts * fb / (2 * Bsweep)

# Display the calculated range
print(calculated_range)
