import numpy as np
import matplotlib.pyplot as plt

def linear_init(x1, x2, t1, t2):
    m = (x2 - x1)/(t2 - t1) # BL2_TR1_m
    b = x1 - m*t1
    return m,b

def poly_init(x1, x2, t1, t2):
    g = -9.81
    b = (x2-x1)/(t2-t1) - g*(t2+t1)
    c = x1 - g*(t1**2) - b*t1
    return b,c

def linear_least_squared(x, y):
    # n = len(x)
    # Sx = sum(x)
    # Sy = sum(y)
    # Sxx = [ xi**3 for xi in x ]
    # Syy = [ yi**3 for yi in y ]
    # Sxy = [xi*yi for xi,yi in zip(x,y)]
    # delta = n*Sxx - Sx**2

    # m = (n*Sxy - Sx*Sy) / delta
    # b = (Sxx*Sy - Sx*Sxy) / delta
    # return m,b
    n = len(x)
    x_bar = sum(x)/n
    y_bar = sum(y)/n
    # Sxy = sum([(xi-x_bar)*(yi-y_bar) for xi,yi in zip(x,y)])
    # Sxx = sum([(xi - x_bar)**2 for xi in x])
    Sxy = sum([(yi-y_bar)*xi for xi,yi in zip(x,y)])
    Sxx = sum([(xi-x_bar)*xi for xi in x])

    beta1 = Sxy/Sxx
    beta0 = y_bar - beta1*x_bar

    return beta1, beta0

def poly_least_squared(x, y): # derivation in google drive
    n = len(x)
    g = -9.81
    x_bar = sum(x)/n
    x_squared_bar = sum([xi**2 for xi in x])/n
    y_bar = sum(y)/n
    Sxy = sum([(yi-y_bar)*xi for xi,yi in zip(x,y)])
    Sxx = sum([(xi-x_bar)*xi for xi in x])
    g_sum = g * sum([xi*(xi**2-x_squared_bar) for xi in x])
    beta1 = (Sxy - g_sum)/Sxx
    beta0 = y_bar - beta1*x_bar - g*x_squared_bar

    print(f"{beta1}, {beta0}")

    return beta1, beta0

class Trajectory():
    def __init__(self, time, point):
        self.init_time = time
        self.times = [0]
        self.points = [point]
        self.beta1_x = 5
        self.beta0_x = 0
        self.beta1_y = 0
        self.beta0_y = 0
        self.beta1_z = 5
        self.beta0_z = 0

        self.constants_established = False

    def appendFirst(self, new_time, new_point):
        time_delta = new_time - self.init_time
        m_x, b_x = linear_init(self.points[-1][0], new_point[0], self.times[-1], time_delta)
        b_y, c_y = poly_init(self.points[-1][1], new_point[1], self.times[-1], time_delta)
        m_z, b_z = linear_init(self.points[-1][2], new_point[2], self.times[-1], time_delta)

        if m_x <= 5 and m_z <= 5:
            self.times.append(time_delta)
            self.points.append(new_point)
            self.beta1_x = m_x
            self.beta0_x = b_x
            self.beta1_y = b_y
            self.beta0_y = c_y
            self.beta1_z = m_z
            self.beta0_z = b_z
            self.constants_established = True
            print("suceeded in first append")
        else:
            print("failed in first append")
            
    
    def append(self, new_time, new_point):
        if not self.constants_established:
            self.appendFirst(new_time, new_point)
            return True
        else:
            time_delta = new_time - self.init_time
            predicted_x = self.beta0_x + self.beta1_x*time_delta
            predicted_y = self.beta0_y + self.beta1_y*time_delta + -9.81*(time_delta**2)
            predicted_z = self.beta0_z + self.beta1_z*time_delta

            print(f"appending into traj w/ length {len(self.times)}: predicted {predicted_x},{predicted_y},{predicted_z}")
            print(f"appending into traj w/ length {len(self.times)}: actual {new_point[0]}, {new_point[1]}, {new_point[2]}")

            if abs((new_point[0]-predicted_x)/predicted_x) < 0.1 and abs((new_point[1]-predicted_y)/predicted_y) < 0.1 and abs((new_point[2]-predicted_z)/predicted_z) < 0.1:
                self.times.append(time_delta)
                self.points.append(new_point)

                self.beta1_x, self.beta0_x = linear_least_squared(self.times, self.points[0]) # X
                self.beta1_y, self.beta0_y = poly_least_squared(self.times, self.points[1]) # Y
                self.beta1_z, self.beta0_z = linear_least_squared(self.times, self.points[2]) # Z

                print(f"succeeded in into traj w/ length {len(self.times)}")
                return True

            # need to figure out least squared fitting https://www.mathsisfun.com/data/least-squares-regression.html
            # https://mathworld.wolfram.com/LeastSquaresFitting.html
            else:
                print(f"failed in appending into traj w/ length {len(self.times)}")
                return False