import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

x = np.linspace(0, 10*np.pi, 100)
y = np.sin(x)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

a = np.array([0.4, 0.84, 1.56])

i = 0
while True:
    ax.scatter(a[0]+ 0.1*i, a[1]+ 0.1*i, a[2]+ 0.1*i)
    ax.text(a[0]+ 0.1*i, a[1]+ 0.1*i, a[2]+ 0.1*i, str(i))
    plt.pause(0.01)
    i += 1
plt.show()