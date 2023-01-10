import placo
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt

swing = placo.SwingFoot(2.5, 0., np.array([0., 0., 0.]), np.array([1., 0., 0.]))

ts = np.linspace(0., 2.5, 1000)
pos = np.array([swing.pos(t) for t in ts])

plt.plot(ts, pos.T[0])
plt.plot(ts, pos.T[1])
plt.plot(ts, pos.T[2])
plt.grid()
plt.show()

plt.plot(pos.T[0], pos.T[2])
plt.grid()
plt.show()