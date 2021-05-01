import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d


class LinearSpline:
    
    
    def __init__(self):
        self.ts = []
        self.xs = []

    def add_entry(self, t, x):
        self.xs.append(x)
        self.ts.append(t)

    def interpolate(self, t):
        if t < self.ts[0]:
            return self.xs[0]
        if t > self.ts[-1]:
            return self.xs[-1]
        return np.interp(t, self.ts, self.xs)
        
        


class LinearSpline3D:
    def __init__(self):
        self.splinex = LinearSpline()
        self.spliney = LinearSpline()
        self.splinez = LinearSpline()

    def add_entry(self, t, x, y ,z):
        self.splinex.add_entry(t, x)
        self.spliney.add_entry(t, y)
        self.splinez.add_entry(t, z)

    def interpolate(self, t):

        return self.splinex.interpolate(t), self.spliney.interpolate(t), self.splinez.interpolate(t)


if __name__ == "__main__":
    spline = LinearSpline()
    spline.add_entry(0., 0.)
    spline.add_entry(0.5, 0.2)
    spline.add_entry(1.5, -0.4)
    spline.add_entry(2.3, 0.6)

    xs = np.arange(-0.1, 2.5, 0.1)
    ys = []
    for x in xs:
        ys.append(spline.interpolate(x))

    plt.plot(xs, ys)
    plt.show()
