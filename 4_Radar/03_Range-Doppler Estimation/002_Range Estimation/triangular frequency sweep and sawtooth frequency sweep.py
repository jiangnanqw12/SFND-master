import numpy as np
import matplotlib.pyplot as plt

# Generate a triangular frequency sweep
t = np.linspace(0, 10, 1000)  # time axis
f_start = 10e3                # starting frequency
f_stop = 50e3                 # stopping frequency
T_chirp = 5e-3                # chirp time

f_tri = np.linspace(f_start, f_stop, len(t))
f_tri[:len(t)//2] = np.linspace(f_start, f_stop, len(t)//2)
f_tri[len(t)//2:] = np.linspace(f_stop, f_start, len(t)//2)

# Generate a sawtooth frequency sweep
f_saw = np.mod(np.linspace(f_start, f_stop, len(t)), f_stop)

# Plot the frequency sweeps
fig, ax = plt.subplots(2, 1, figsize=(8, 6))

ax[0].plot(t, f_tri / 1e3)
ax[0].set_xlabel('Time (s)')
ax[0].set_ylabel('Frequency (kHz)')
ax[0].set_title('Triangular frequency sweep')

ax[1].plot(t, f_saw / 1e3)
ax[1].set_xlabel('Time (s)')
ax[1].set_ylabel('Frequency (kHz)')
ax[1].set_title('Sawtooth frequency sweep')

plt.get_current_fig_manager().window.showMaximized()
plt.tight_layout()
plt.show()

plt.savefig('frequency_sweeps.svg', format='svg')
