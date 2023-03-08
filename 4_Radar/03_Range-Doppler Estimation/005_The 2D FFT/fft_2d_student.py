import numpy as np
import matplotlib.pyplot as plt

def test1():
    # Generate 2 - D data with repeated blocks
    P = np.array(np.meshgrid(np.arange(20), np.arange(20))).T
    P = np.sin(np.sqrt(P[:, :, 0] ** 2 + P[:, :, 1] ** 2)) * 10
    X = np.tile(P, (5, 10))

    # Display the data

    plt.imshow(X, cmap = 'gray')
    plt.show()

    # Compute the 2 - D Fourier transform of the data
    Y = np.fft.fft2(X)

    # Shift the zero - frequency component to the center of the output
    Y = np.fft.fftshift(Y)

    # Plot the magnitude of the resulting matrix
    plt.imshow(np.abs(Y), cmap = 'gray')
    plt.show()
def test2():

    # Create and plot 2-D data with repeated blocks.
    P = np.array(np.mat('''
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
            0     0     0    0     0     0     0    0    0    0
        '''))
    P = P + 3
    X = np.tile(P, (5, 10))
    plt.imshow(X, cmap='gray')

    # Compute the 2-D Fourier transform of the data.
    # Shift the zero-frequency component to the center of the output, and
    # plot the resulting 100-by-200 matrix, which is the same size as X.
    Y = np.fft.fft2(X)
    Y = np.fft.fftshift(Y)
    plt.figure()
    plt.imshow(np.abs(Y), cmap='gray')
    plt.show()

if __name__ == '__main__':
    #test1()
    test2()