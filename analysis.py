import numpy as np
import matplotlib.pyplot as plt


def getDiff(data):
    arr = [data[i] - data[i - 1] for i in range(1, len(data))]
    arr.append(arr[-1])
    return arr

def plotData(uwb, ser, fig):
    serx = ser[:-1]
    uwbx = uwb[:-1]

    diff_ser = getDiff(serx)
    diff_uwb = getDiff(uwbx)

    plt.figure(fig)
    plt.subplot(2, 1, 1)
    plt.scatter(serx, diff_ser, c = 'k', s = 7)
    plt.plot(serx, diff_ser, c = 'k', label = 'data')
    xs = np.linspace(serx[0], serx[-1], 3)
    ys = [ser[-1] for i in range(3)]
    plt.plot(xs, ys, c = 'r', label = 'average')
    plt.title('Serial communication interval')
    plt.grid(axis = 'both')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.scatter(uwbx, diff_uwb, c = 'k', s = 7)
    plt.plot(uwbx, diff_uwb, c = 'k', label = 'data')
    xs = np.linspace(uwbx[0], uwbx[-1], 3)
    ys = [uwb[-1] for i in range(3)]
    plt.plot(xs, ys, c = 'r', label = 'average')
    plt.title('UWB communication interval')
    plt.grid(axis = 'both')
    plt.legend()

    return fig + 1

def plotComprehensive(uwb, ser, fig):
    serx = ser[:-1]
    uwbx = ser[:-1]
    ys = (0, 1)

    plt.figure(fig)
    sery = np.ones_like(serx)
    uwby = np.ones_like(uwbx)
    plt.scatter(serx, sery, c = 'b', label = 'serial')
    plt.scatter(uwbx, uwby, c = 'r', label = 'uwb', s = 1)
    plt.title('Comparison UWB and serial')

    return fig + 1

if __name__ == "__main__":
    fig = 0
    uwb = np.loadtxt("./uwb.txt", dtype = float, delimiter = ',')
    ser = np.loadtxt("./ser.txt", dtype = float, delimiter = ',')
    print("Here1")
    fig = plotData(uwb, ser, fig)
    print("Here2")
    fig = plotComprehensive(uwb, ser, fig)
    print("Here3")
    plt.show()