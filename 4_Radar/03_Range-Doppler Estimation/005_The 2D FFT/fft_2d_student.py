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
        4     4     4     4     5     6     7     7     6     5;
        4     4     4     5     7     8     9     9     8     6;
        4     4     5     7     9    10    10    10     9     7;
        5     5     7     9    10    10    10    10     9     7;
        6     7     9    10    10    10    10    10     9     7;
        7     8    10    10    10    10    10    10     9     7;
        7     9    10    10    10    10    10    10     9     7;
        6     9    10    10    10    10    10    10     9     7;
        5     8     9     9     9     9     9     9     8     6;
        4     6     7     7     7     7     7     7     6     5
    '''))

    X = np.tile(P, (5, 10))

    # Plot the data
    plt.figure(1)
    plt.imshow(X)
    plt.title('Original Image')
    plt.show()

    # Compute the 2-D Fourier transform of the data.
    Y = np.fft.fft2(X)

    # Shift the zero-frequency component to the center of the output, and
    # plot the resulting 100-by-200 matrix, which is the same size as X.
    Yshift = np.fft.fftshift(Y)
    plt.figure(2)
    plt.imshow(abs(Yshift))
    plt.title('2-D Fourier Transform')
    plt.show()

def test3():
    import math
    import cmath

    # Create and plot 2-D data with repeated blocks.
    P = [[math.sin(x) for x in range(20)] for y in range(20)]
    X = [P[i % 20] * 5 for i in range(100)]
    #X = [P for i in range(5)]
    #X = [P[i % 20] * 5 for i in range(50) for j in range(10)]
    #X = [P[i % 20] * 5 for i in range(50) for j in range(20)]
    for row in X:
        print(row)

    # Compute the 2-D Fourier transform of the data.
    # Shift the zero-frequency component to the center of the output, and
    # plot the resulting 100-by-200 matrix, which is the same size as X.
    Y = [[0 for x in range(200)] for y in range(100)]
    N = len(X)
    M = len(X[0])
    for u in range(N):
        for v in range(M):
            sum = 0
            for x in range(N):
                for y in range(M):
                    e = cmath.exp(-2j * math.pi * ((u * x / N) + (v * y / M)))
                    sum += X[x][y] * e
            Y[u][v] = sum

    for row in Y:
        print(row)

    # Display the result
    import matplotlib.pyplot as plt
    plt.imshow(abs(Y), cmap='gray', origin='lower')
    plt.show()



if __name__ == '__main__':
    #test1()
    test3()