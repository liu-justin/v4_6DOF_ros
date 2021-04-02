import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

depths = np.load("depths.npy")
diameters = np.load("diameters.npy")

fig = plt.figure()
ax = fig.add_subplot(111)

for i in range(len(depths)):
    ax.scatter(depths[i], diameters[i])

b2, b1, b0 = np.polyfit(depths, diameters, 2)

x = np.arange(min(depths), max(depths),0.1)
y = b2*x**2 + b1*x + b0

ax.plot(x,y)

# a = np.array([0.4, 0.84, 1.56])

# i = 0
# while True:
#     ax.scatter(a[0]+ 0.1*i, a[1]+ 0.1*i, a[2]+ 0.1*i)
#     ax.text(a[0]+ 0.1*i, a[1]+ 0.1*i, a[2]+ 0.1*i, str(i))
#     plt.pause(0.01)
#     i += 1
plt.show()