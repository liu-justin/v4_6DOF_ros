import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def linear(x1, x2, t1, t2, x_error, t_error):
    x1_low = x1*(1-x_error)
    x1_high = x1*(1+x_error)
    t1_low = t1*(1-t_error)
    t1_high = t1*(1+t_error)
    x2_low = x2*(1-x_error)
    x2_high = x2*(1+x_error)
    t2_low = t2*(1-t_error)
    t2_high = t2*(1+t_error)
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

def poly_b_c(x2, t2, x1, t1):
    g = -9.81
    b = (x2-x1)/(t2-t1) - g*(t2+t1)
    c = c = x1 - g*(t1**2) - b*t1
    return b,c

def poly(x1, x2, t1, t2, x_error, t_error):
    x1_low = x1*(1-x_error)
    x1_high = x1*(1+x_error)
    t1_low = t1*(1-t_error)
    t1_high = t1*(1+t_error)
    x2_low = x2*(1-x_error)
    x2_high = x2*(1+x_error)
    t2_low = t2*(1-t_error)
    t2_high = t2*(1+t_error)

    g = -9.81

    # b = (x2-x1)/(t2-t1) - g*(t2+t1)
    # c = x1 - g*(t1**2) - b*t1
    b_list = [0,0,0,0]
    c_list = [0,0,0,0]
    
    b_list[0], c_list[0] = poly_b_c(x2_high, t2_high, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_b_c(x2_high, t2_high, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_b_c(x2_high, t2_high, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_b_c(x2_high, t2_high, x1_low, t1_low) # TR2_BL1

    # find min/max of each corner in 2, then somehow figure out all the other stuff

    b_list = [0,0,0,0]
    c_list = [0,0,0,0]
    
    b_list[0], c_list[0] = poly_b_c(x2_high, t2_low, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_b_c(x2_high, t2_low, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_b_c(x2_high, t2_low, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_b_c(x2_high, t2_low, x1_low, t1_low) # TR2_BL1

    b_list = [0,0,0,0]
    c_list = [0,0,0,0]
    
    b_list[0], c_list[0] = poly_b_c(x2_low, t2_low, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_b_c(x2_low, t2_low, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_b_c(x2_low, t2_low, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_b_c(x2_low, t2_low, x1_low, t1_low) # TR2_BL1

    b_list = [0,0,0,0]
    c_list = [0,0,0,0]
    
    b_list[0], c_list[0] = poly_b_c(x2_low, t2_high, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_b_c(x2_low, t2_high, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_b_c(x2_low, t2_high, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_b_c(x2_low, t2_high, x1_low, t1_low) # TR2_BL1

    x = np.arange(1,6,0.1)
    z1 = g*(x**2) + b*x + c
    plt.scatter(t1, x1)
    plt.scatter(t2, x2)
    plt.plot(x,z1)
    # y = []
    for i in range(0,4):
        plt.plot(x, g*(x**2) + b_list[i]*x + c_list[i])

    plt.show()

    return b,c

# https://stackoverflow.com/questions/21565994/method-to-return-the-equation-of-a-straight-line-given-two-points
# polyfit uses least squares fit, lets see which one is faster

x1 = 4
x2 = 2
t1 = 2
t2 = 4

x = np.array(range(10))

m1, b1, m2, b2 = linear(x1, x2, t1, t2, 0.02, 0.02)
print(f"{m1}, {b1}, {m2}, {b2}")
b,c = poly(x1,x2,t1,t2,0.1, 0.2)

y1 = m1*x+b1
y2 = m2*x + b2
z1 = -9.81*(x**2) + b*x + c

plt.scatter(t1, x1)
plt.scatter(t2, x2)
plt.plot(x,y1)
plt.plot(x,y2)
plt.plot(x,z1)
plt.show()