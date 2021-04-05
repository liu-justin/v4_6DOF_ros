import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def linear_simple(x1, x2, t1, t2):
    beta1 = (x2 - x1)/(t2 - t1) # BL2_TR1_m
    beta0 = x1 - beta1*t1
    return beta1,beta0

def poly_simple(x1, x2, t1, t2):
    beta2 = -9.81
    beta1 = (x2-x1)/(t2-t1) - beta2*(t2+t1)
    beta0 = x1 - beta2*(t1**2) - beta1*t1
    return beta1,beta0

def linear_least_squared(x, y):
    n = len(x)
    x_bar = sum(x)/n
    y_bar = sum(y)/n
    Sxy = sum([(yi-y_bar)*xi for xi,yi in zip(x,y)])
    Sxx = sum([(xi-x_bar)*xi for xi in x])
    beta1 = Sxy/Sxx
    beta0 = y_bar - beta1*x_bar

    return beta1, beta0

def poly_least_squared_beta2const(x, y): # derivation in google drive
    n = len(x)
    g = -9.81
    x_bar = sum(x)/n
    x2_bar = sum([xi**2 for xi in x])/n
    y_bar = sum(y)/n
    Sxy = sum([(yi-y_bar)*xi for xi,yi in zip(x,y)])
    Sxx = sum([(xi-x_bar)*xi for xi in x])
    g_sum = g * sum([xi*(xi**2-x2_bar) for xi in x])
    beta1 = (Sxy - g_sum)/Sxx
    beta0 = y_bar - beta1*x_bar - g*x2_bar

    return beta1, beta0

def poly_least_squared_beta0const(x, y, beta0):
    sum_xi = sum(x)
    sum_xi2 = sum([xi**2 for xi in x])
    sum_xi3 = sum([xi**3 for xi in x])
    sum_xi4 = sum([xi**4 for xi in x])
    sum_xiyi = sum([xi*yi for xi,yi in zip(x,y)])

    numerator_1 = sum([(xi**2)*yi for xi,yi in zip(x,y)])
    numerator_2 = beta0 * sum_xi2
    numerator_3 = sum_xiyi * sum_xi3 / sum_xi2
    numerator_4 = beta0 * (sum_xi/sum_xi2) * sum_xi3

    denom = -1*(sum_xi3*sum_xi3/sum_xi2 - sum_xi4)

    beta2 = (numerator_1 - numerator_2 - numerator_3 + numerator_4)/denom
    beta1 = (sum_xiyi - beta0*sum_xi - beta2*sum_xi3)/sum_xi2

    return beta2, beta1

def linear_errored(x1, x2, t1, t2, x_error, t_error):
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
        beta1_low = (x2_low - x1_high)/(t2_low - t1_high) # BL2_TR1_m
        beta1_high = (x2_high - x1_low)/(t2_high - t1_low) # TR2_BL1_m
        beta0_low = x1_high - beta1_low*t1_high
        beta0_high = x1_low - beta1_high*t1_low
        print("full negative")
    
    # mixed slope
    elif (x2_low - x1_high) <= 0:
        beta1_low = (x2_low - x1_high)/(t2_low - t1_high) # BL2_TR1_m
        beta1_high = (x2_high - x1_low)/(t2_low - t1_high) # TL2_BR1_m
        beta0_low = x1_high - beta1_low*t1_high
        beta0_high = x1_low - beta1_high*t1_high
        print("mixed")

    # all positive slope
    else:
        beta1_low = (x2_low - x1_high)/(t2_high - t1_low) # BR2_TL1_m
        beta1_high = (x2_high - x1_low)/(t2_low - t1_high) # TL2_BR1_m
        beta0_low = x1_high - beta1_low*t1_low
        beta0_high = x1_low - beta1_high*t1_high

        print("full positive")

    return beta1_low, beta0_low, beta1_high, beta0_high


    # TL2_BR1_m = (x2_high - x1_low)/(t2_low - t1_high)
    # BR2_TL1_m = (x2_low - x1_high)/(t2_high - t1_low)
    # TR2_BL1_m = (x2_high - x1_low)/(t2_high - t1_low)
    # BL2_TR1_m = (x2_low - x1_high)/(t2_low - t1_high)

def poly_errored(x1, x2, t1, t2, x1_error, x2_error, t_error):
    radius1 = np.sqrt(x1_error**2 + t_error**2)
    radius2 = np.sqrt(x2_error**2 + t_error**2)

    

def poly_errored_old(x1, x2, t1, t2, x_error, t_error):
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
    
    b_list[0], c_list[0] = poly_simple(x2_high, t2_high, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_simple(x2_high, t2_high, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_simple(x2_high, t2_high, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_simple(x2_high, t2_high, x1_low, t1_low) # TR2_BL1

    # find min/max of each corner in 2, then somehow figure out all the other stuff

    b_list = [0,0,0,0]
    c_list = [0,0,0,0]
    
    b_list[0], c_list[0] = poly_simple(x2_high, t2_low, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_simple(x2_high, t2_low, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_simple(x2_high, t2_low, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_simple(x2_high, t2_low, x1_low, t1_low) # TR2_BL1

    b_list = [0,0,0,0]
    c_list = [0,0,0,0]
    
    b_list[0], c_list[0] = poly_simple(x2_low, t2_low, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_simple(x2_low, t2_low, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_simple(x2_low, t2_low, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_simple(x2_low, t2_low, x1_low, t1_low) # TR2_BL1

    b_list = [0,0,0,0]
    c_list = [0,0,0,0]
    
    b_list[0], c_list[0] = poly_simple(x2_low, t2_high, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_simple(x2_low, t2_high, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_simple(x2_low, t2_high, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_simple(x2_low, t2_high, x1_low, t1_low) # TR2_BL1

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

"""
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
"""

"""
    t = [0, 0.0333399772644043, 0.06669783592224121]
    y = [0.03290366, 0.06382939, 0.08356368]

    beta1, beta0 = poly_least_squared_beta2const(t,y)
    beta1_a, beta0_a = poly_init_beta2const(y[0], y[1], t[0], t[1])
    beta2_b, beta1_b = poly_least_squared_beta0const(t,y,y[0])
    print(f"{beta1}, {beta0}")
    print(f"{beta1_a}, {beta0_a}")
    print(f"{beta2_b}, {beta1_b}")

    fig = plt.figure()
    ax = fig.add_subplot(111)
    for yi,ti in zip(y, t):
        ax.scatter(ti, yi)
    x = np.arange(0,t[-1],0.01)
    y= y[0] + beta1_b*x + beta2_b*(x**2)
    ax.plot(x,y)
    plt.show()
"""
