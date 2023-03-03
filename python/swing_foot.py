import placo
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt

swing = placo.SwingFootQuintic.make_trajectory(
    0., 2.5, 0.07, np.array([0., 0., 0.]), np.array([.2, 0., 0.]))

ts = np.linspace(0., 2.5, 1000)
pos = np.array([swing.pos(t) for t in ts])

print(swing.pos(0.25*2.5))
print(swing.pos(0.75*2.5))

plt.plot(ts, pos.T[0])
plt.plot(ts, pos.T[1])
plt.plot(ts, pos.T[2])

plt.grid()
plt.show()

plt.plot(pos.T[0], pos.T[2])

for i in range(1000):
    if (abs(pos.T[2][i]-0.07) < 10e-5):
        plt.plot(pos.T[0][i], pos.T[2][i], 'ro')

plt.xlim((-0.1, 0.3))
plt.ylim((-0.01, 0.15))
plt.title("Cubic spline followed during a step")
plt.grid()
plt.show()
