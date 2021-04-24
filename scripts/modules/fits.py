import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def linear_simple(x2, t2, x1, t1):
    beta1 = (x2 - x1)/(t2 - t1) # BL2_TR1_m
    beta0 = x1 - beta1*t1
    return [beta0,beta1,0]

def poly_simple(x2, t2, x1, t1):
    beta2 = -9.81
    beta1 = (x2-x1)/(t2-t1) - beta2*(t2+t1)
    beta0 = x1 - beta2*(t1**2) - beta1*t1
    return [beta0,beta1,beta2]

def linear_least_squares(x, y):
    beta2 = 0
    n = len(x)
    x_bar = sum(x)/n
    y_bar = sum(y)/n
    Sxy = sum([(yi-y_bar)*xi for xi,yi in zip(x,y)])
    Sxx = sum([(xi-x_bar)*xi for xi in x])
    beta1 = Sxy/Sxx
    beta0 = y_bar - beta1*x_bar

    y_predicted = [beta0 + beta1*xi + beta2*xi**2 for xi in x]
    avg_residuals = sum([abs(yi - yi_p) for yi, yi_p in zip(y, y_predicted)])

    return [beta0, beta1, beta2], avg_residuals

def poly_least_squares_beta2const(x, y): # derivation in google drive
    n = len(x)
    beta2 = -9.81
    x_bar = sum(x)/n
    x2_bar = sum([xi**2 for xi in x])/n
    y_bar = sum(y)/n
    Sxy = sum([(yi-y_bar)*xi for xi,yi in zip(x,y)])
    Sxx = sum([(xi-x_bar)*xi for xi in x])
    g_sum = beta2 * sum([xi*(xi**2-x2_bar) for xi in x])
    beta1 = (Sxy - g_sum)/Sxx
    beta0 = y_bar - beta1*x_bar - beta2*x2_bar

    y_predicted = [beta0 + beta1*xi + beta2*xi**2 for xi in x]
    avg_residuals = sum([abs(yi - yi_p) for yi, yi_p in zip(y, y_predicted)])

    return [beta0, beta1, beta2], avg_residuals

def poly_least_squares_beta0const(x, y, beta0):
    n = len(x)
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

    y_predicted = [beta0 + beta1*xi + beta2*xi**2 for xi in x]
    avg_residuals = sum([abs(yi - yi_p) for yi, yi_p in zip(y, y_predicted)])

    return [beta0, beta1, beta2], avg_residuals

def linear_errored(x2, t2, x1, t1, x_error, t_error):
    x1_low = x1 - x_error
    x1_high = x1 + x_error
    t1_low = t1 - t_error
    t1_high = t1 + t_error
    x2_low = x2 - x_error
    x2_high = x2 + x_error
    t2_low = t2 - t_error
    t2_high = t2 + t_error

    # full negative slope
    if (x2_high - x1_low) <= 0:
        beta_low = linear_simple(x2_low, t2_low, x1_high, t1_high)
        beta_high = linear_simple(x2_high, t2_high, x1_low, t1_low)
    
    # mixed slope
    elif (x2_low - x1_high) <= 0:
        beta_low = linear_simple(x2_low, t2_low, x1_high, t1_high)
        beta_high = linear_simple(x2_high, t2_low, x1_low, t1_high)

    # all positive slope
    else:
        beta_low = linear_simple(x2_low, t2_high, x1_high, t1_low)
        beta_high = linear_simple(x2_high, t2_low, x1_low, t1_high)

    return beta_low, beta_high

def poly_errored(x2, t2, x1, t1, x_error, t_error):
    x1_low = x1 - x_error
    x1_high = x1 + x_error
    t1_low = t1 - t_error
    t1_high = t1 + t_error
    x2_low = x2 - x_error
    x2_high = x2 + x_error
    t2_low = t2 - t_error
    t2_high = t2 + t_error

    beta2 = -9.81

    beta_simple = poly_simple(x2, t2, x1, t1)
    # find which t is associated with the peak of the poly, differentiate and equate to 0
    t_max = beta_simple[1]/(2*beta2)

    # full negative slope
    if (x2_high - x1_low) <= 0:
        if (t1 < t_max < t2): # if the peak is inbtwn the points
            betas_low = poly_simple(x2_low, t2_low, x1_high, t1_high)
            betas_high = poly_simple(x2_high, t2_high, x1_low, t1_low)
        else: # both points are to the right of the peak
            betas_low = poly_simple(x2_low, t2_low, x1_high, t1_high)
            betas_high = poly_simple(x2_high, t2_high, x1_low, t1_low)

    # full positive slope
    elif(x2_low - x1_high) >= 0:
        if (t1 < t_max < t2):
            betas_low = poly_simple(x2_low, t2_low, x1_high, t1_low)
            betas_high = poly_simple(x2_high, t2_low, x1_low, t1_high)
        else: # both points are to the left of the peak
            betas_low = poly_simple(x2_low, t2_high, x1_high, t1_low)
            betas_high = poly_simple(x2_high, t2_low, x1_low, t1_high)

    # pretty equal
    else:
        betas_low = poly_simple(x2_low, t2_low, x1_high, t1_high)
        betas_high = poly_simple(x2_high, t2_low, x1_low, t1_high)

    return betas_low, betas_high

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

    beta1, beta0 = poly_least_squares_beta2const(t,y)
    beta1_a, beta0_a = poly_init_beta2const(y[0], y[1], t[0], t[1])
    beta2_b, beta1_b = poly_least_squares_beta0const(t,y,y[0])
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

# list of all 16 combinations
"""
    b_list = [0] * 16
    c_list = [0] * 16
    
    b_list[0], c_list[0] = poly_simple(x2_high, t2_high, x1_high, t1_low) # TR2_TL1
    b_list[1], c_list[1] = poly_simple(x2_high, t2_high, x1_high, t1_high) # TR2_TR1
    b_list[2], c_list[2] = poly_simple(x2_high, t2_high, x1_low, t1_high) # TR2_BR1
    b_list[3], c_list[3] = poly_simple(x2_high, t2_high, x1_low, t1_low) # TR2_BL1
    
    b_list[4], c_list[4] = poly_simple(x2_high, t2_low, x1_high, t1_low) # TR2_TL1
    b_list[5], c_list[5] = poly_simple(x2_high, t2_low, x1_high, t1_high) # TR2_TR1
    b_list[6], c_list[6] = poly_simple(x2_high, t2_low, x1_low, t1_high) # TR2_BR1
    b_list[7], c_list[7] = poly_simple(x2_high, t2_low, x1_low, t1_low) # TR2_BL1
    
    b_list[8], c_list[8] = poly_simple(x2_low, t2_low, x1_high, t1_low) # TR2_TL1
    b_list[9], c_list[9] = poly_simple(x2_low, t2_low, x1_high, t1_high) # TR2_TR1
    b_list[10], c_list[10] = poly_simple(x2_low, t2_low, x1_low, t1_high) # TR2_BR1
    b_list[11], c_list[11] = poly_simple(x2_low, t2_low, x1_low, t1_low) # TR2_BL1
    
    b_list[12], c_list[12] = poly_simple(x2_low, t2_high, x1_high, t1_low) # TR2_TL1
    b_list[13], c_list[13] = poly_simple(x2_low, t2_high, x1_high, t1_high) # TR2_TR1
    b_list[14], c_list[14] = poly_simple(x2_low, t2_high, x1_low, t1_high) # TR2_BR1
    b_list[15], c_list[15] = poly_simple(x2_low, t2_high, x1_low, t1_low) # TR2_BL1
"""
