import cmath
import numpy as np

def fft(x):
    N = len(x)
    if N <= 1:
        return x
    even = fft(x[0::2])
    odd = fft(x[1::2])
    T = [cmath.exp(-2j * cmath.pi * k / N) * odd[k] for k in range(N // 2)]
    return [even[k] + T[k] for k in range(N // 2)] + [even[k] - T[k] for k in range(N // 2)]

def fft2(x):
    N, M = x.shape
    if N <= 1 and M <= 1:
        return x
    elif N > 1 and M <= 1:
        return np.vstack([fft(x[i, :]) for i in range(N)])
    elif N <= 1 and M > 1:
        return np.vstack([fft(x[:, i]) for i in range(M)]).T
    else:
        even = fft2(x[0:N:2, 0:M:2])
        odd1 = fft2(x[0:N:2, 1:M:2])
        odd2 = fft2(x[1:N:2, 0:M:2])
        odd3 = fft2(x[1:N:2, 1:M:2])
        T1 = np.exp(-2j * np.pi * np.arange(N // 2) / N)[:, np.newaxis]
        T2 = np.exp(-2j * np.pi * np.arange(M // 2) / M)[np.newaxis, :]
        T = T1 * odd3 + T2 * odd2
        return np.vstack([np.hstack([even[i, j] + T[i, j], even[i, j] - T[i, j]]) for i in range(N // 2) for j in range(M // 2)])
def main():
    x = [0, 1, 2, 3, 4, 5, 6, 7]
    y = fft(x)
    print(y)
    x = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    y = fft2(x)
    #print(y)
if __name__ == '__main__':
    main()
