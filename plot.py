import numpy as np
import matplotlib.pyplot as plt

def originAccel(origin, filtered, fig):
    plt.figure(fig)
    xa = origin[:, 1]
    ya = origin[:, 2]
    za = origin[:, 3]
    xaf = filtered[:, 1]
    yaf = filtered[:, 2]
    zaf = filtered[:, 3]

    xs = origin[:, 0]
    xfs = filtered[:, 0]
    plt.subplot(3, 1, 1)
    plt.scatter(xs, xa, c = 'b', s = 3)
    plt.plot(xs, xa, c = 'b', label = 'origin')
    plt.plot(xfs, xaf, c = 'r', label = 'filtered')
    plt.grid(axis = 'both')


    plt.subplot(3, 1, 2)
    plt.scatter(xs, ya, c = 'b', s = 3)
    plt.plot(xs, ya, c = 'b', label = 'origin')
    plt.plot(xfs, yaf, c = 'r', label = 'filtered')
    plt.grid(axis = 'both')

    plt.subplot(3, 1, 3)
    plt.scatter(xs, za, c = 'b', s = 3)
    plt.plot(xs, za, c = 'b', label = 'origin')
    plt.plot(xfs, zaf, c = 'r', label = 'filtered')
    plt.grid(axis = 'both')
    return fig + 1

def makeIntegral(ts, accel):
    length = len(ts)
    dts = [ts[1] - ts[0]]               
    for i in range(length - 1):
        dts.append(ts[i+1] - ts[i])
    v = [0.0, ]
    for i in range(length):
        old_v = v[-1]
        if i == 0:
            dv = accel[0] / 2 * dts[0]
        else:
            dv = (accel[i] + accel[i - 1]) / 2. * dts[i]
        v.append(dv + old_v)
    return v[1:]

def speedPlot(origin, filtered, fig):
    plt.figure(fig)
    xa = origin[:, 1]
    ya = origin[:, 2]
    xaf = filtered[:, 1]
    yaf = filtered[:, 2]
    xs = origin[:, 0]
    xfs = filtered[:, 0]
    lx = len(xs)
    lf = len(xfs)

    vx = makeIntegral(xs, xa)
    vy = makeIntegral(xs, ya)
    vxf = makeIntegral(xfs, xaf)
    vyf = makeIntegral(xfs, yaf)

    plt.subplot(2, 1, 1)
    plt.scatter(xs, vx, c = 'b', s = 3)
    plt.plot(xs, vx, c = 'b', label = 'origin')
    plt.plot(xfs, vxf, c = 'r', label = 'filtered')
    plt.grid(axis = 'both')


    plt.subplot(2, 1, 2)
    plt.scatter(xs, vy, c = 'b', s = 3)
    plt.plot(xs, vy, c = 'b', label = 'origin')
    plt.plot(xfs, vyf, c = 'r', label = 'filtered')
    plt.grid(axis = 'both')
    return fig + 1


if __name__ == "__main__":
    fig = 0
    origin = np.loadtxt("./origin.txt", dtype = float, delimiter = ",")
    filtered = np.loadtxt("./filtered.txt", dtype = float, delimiter = ",")
    fig = speedPlot(origin, filtered, fig)
    fig = originAccel(origin, filtered, fig)
    plt.show()