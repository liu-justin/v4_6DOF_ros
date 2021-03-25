import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def linear(x_1, x_2, t_1, t_2, x_error, t_error):
    x1_low = x_1*(1-x_error)
    x1_high = x_1*(1+x_error)
    t1_low = t_1*(1-t_error)
    t1_high = t_1*(1+t_error)
    x2_low = x_2*(1-x_error)
    x2_high = x_2*(1+x_error)
    t2_low = t_2*(1-t_error)
    t2_high = t_2*(1+t_error)
    # full negative slope
    if (x2_high - x1_low) <= 0:
        min_m = (x2_low - x1_high)/(t2_low - t1_high) # BL2_TR1_m
        max_m = (x2_high - x1_low)/(t2_high - t1_low) # TR2_BL1_m
        min_b = x1_high - min_m*t1_high
        max_b = x1_low - max_m*t1_low
        print("full negative")

    
    # mixed slope
    elif (x2_low - x1_high) <= 0:
        min_m = (x2_low - x1_high)/(t2_low - t1_high) # BL2_TR1_m
        max_m = (x2_high - x1_low)/(t2_low - t1_high) # TL2_BR1_m
        min_b = x1_high - min_m*t1_high
        max_b = x1_low - max_m*t1_high
        print("mixed")

    # all positive slope
    else:
        min_m = (x2_low - x1_high)/(t2_high - t1_low) # BR2_TL1_m
        max_m = (x2_high - x1_low)/(t2_low - t1_high) # TL2_BR1_m
        min_b = x1_high - min_m*t1_low
        max_b = x1_low - max_m*t1_high

        print("full positive")

    return min_m, min_b, max_m, max_b


    # TL2_BR1_m = (x2_high - x1_low)/(t2_low - t1_high)
    # BR2_TL1_m = (x2_low - x1_high)/(t2_high - t1_low)
    # TR2_BL1_m = (x2_high - x1_low)/(t2_high - t1_low)
    # BL2_TR1_m = (x2_low - x1_high)/(t2_low - t1_high)

# https://stackoverflow.com/questions/21565994/method-to-return-the-equation-of-a-straight-line-given-two-points
# polyfit uses least squares fit, lets see which one is faster

x1 = 4
x2 = 2
t1 = 2
t2 = 4

x = np.array(range(10))

m1, b1, m2, b2 = linear(x1, x2, t1, t2, 0.02, 0.02)
print(f"{m1}, {b1}, {m2}, {b2}")

y1 = m1*x+b1
y2 = m2*x + b2

plt.scatter(t1, x1)
plt.scatter(t2, x2)
plt.plot(x,y1)
plt.plot(x,y2)
plt.show()